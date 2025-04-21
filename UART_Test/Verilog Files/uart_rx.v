`timescale 1ns/1ps

/*
 * Custom UART RX design, with all requisite customization options
 * Written by Shubhayu Das, November 2021
 * Version: 1.2.0
 * Last updated: 27th November, 2021
 * 
 * The clock frequency and the baud rate have to be calculated and set manually.
 * As it stands now, this module will automatically calculate the prescalar needed.
 * This division will be done in software, which is inefficient, and wastes power
 * 
 ***************************
 *
 * THEORY OF WORKING of this module:
 *
 * CLOCKS:
 * - An input clock of 25MHz is expected.
 * - Internally, a "slow_clk_en" pulse is generated, depending on baud rate.
 *
 * UART:
 * An active low signal starts receiving.
 * I am implementing oversampling. Each RX bit is sampled 15 times,
 * then the "obtained_bit" function is used to calculate the detected bit
 *
 * An FSM is used to handle the operation of the module.
 * FSM states:
 *  - IDLE          : Waiting for start bit(0)
 *  - DETECT_START  : Possible start bit detected. Oversample and decide
 *  - STARTED       : Receiving started, oversample and decide individual bits
 *  - CONVERSION    : RX complete, run parity check, retain data until flushed 
 *
 * Oversampling:
 * The incoming RX signal is oversampled, and the oversampled bits are stored in
 * the "oversampled_bit" buffer. Once the buffer is full, the "obtained_bit"
 * function is used to find the accurate bit. This bit is stored in "sampled_rx_data",
 * at the correct location
 *
 * RX:
 * Oversampling is continued until all bits are received. The "sampled_rx_data"
 * contains the final data. This data must be read only AFTER the converted flag goes high.
 *
 ***************************
 * Parameters in the module:
 *
 * FRAME_BITS   - The number of bits in the data frame. Allowed values: 5-9 bits/frame
 * PARITY_BIT   - Set to "2" to disable parity check. Set to "0/1" for odd/even parity resp.
 * STOP_BITS    - The number of stop bits, Allowed values: 1-2 bits
 *
 ***************************
 * Inputs to the module:
 *
 * rx       - The input rx signal
 * i_clk    - The clock of the local system, used for sampling the bits and FSM sync
 * flush    - Clear the converted data from the register
 * 
 ***************************
 * Outputs of the module
 *
 * data         - The final converted data, of appropriate bit-width
 * converted    - Active high signal indicating successfully received frame
 * data_valid   - Active high signal indicating if received data is valid(if parity is enabled)
 * busy         - Indicates if data reception has started and system is busy
 * 
 ***************************
 * In this version, the oversampling factor is 15.
 * 
 * Note that due to small mismatches, between the input clock frequency and the target baud rate,
 * there will a drift over time. This needs to be taken care of in the parent module.
 *
 ***************************
 *
 * Sample input clock setup for Basys 3:
 * Set the clock to 100MHz in the constraints file. 
 * Then divide the clock to get a time period of 540ns
 *
 ***************************
 * TODO: handling lost bits, using timeout
 * Actually looking at the stop bit(s)
*/

//`define DEBUG


module uart_rx #(
    parameter FRAME_BITS = 8,
    parameter PARITY_BIT = 2,
    parameter STOP_BITS = 1
)(
    input rx,
    input i_clk,
    input flush,
    output reg [FRAME_BITS-1:0] data,
    output reg converted,
    output reg data_valid,
    output reg busy
);

// UART oversampling config
localparam OVERSAMPLE_FACTOR = 15;
localparam SAMPLE_CLK_DIV = 100_000_000 / (115200 * OVERSAMPLE_FACTOR); // ~58

// FSM states
localparam IDLE = 0, DETECT_START = 1, STARTED = 2, CONVERSION = 3;
localparam TOTAL_RX_BITS = (PARITY_BIT < 2) ? FRAME_BITS + STOP_BITS + 1 : FRAME_BITS + STOP_BITS;

localparam SAMPLE_LOC_1 = (OVERSAMPLE_FACTOR / 2) - 1;
localparam SAMPLE_LOC_2 = (OVERSAMPLE_FACTOR / 2);
localparam SAMPLE_LOC_3 = (OVERSAMPLE_FACTOR / 2) + 1;

reg [1:0] state;
reg [3:0] bit_location;
reg [3:0] insert_location;

reg [OVERSAMPLE_FACTOR-1:0] oversampled_bit;
reg [TOTAL_RX_BITS-1:0] sampled_rx_data;

// === Clock divider for oversampling ===
reg [7:0] clk_div;
wire sample_en = (clk_div == 0);

always @(posedge i_clk) begin
    if (clk_div >= SAMPLE_CLK_DIV - 1)
        clk_div <= 0;
    else
        clk_div <= clk_div + 1;
end

// === Main UART RX FSM ===
always @(posedge i_clk) begin
    if (sample_en) begin
        case (state)
            IDLE: begin
                converted <= 0;
                data_valid <= 0;
                busy <= 0;
                bit_location <= 0;
                insert_location <= 0;
                oversampled_bit <= 0;
                sampled_rx_data <= 0;
                data <= 0;
                if (!rx)
                    state <= DETECT_START;
            end

            DETECT_START: begin
                oversampled_bit[insert_location] <= rx;
                insert_location <= insert_location + 1;
                if (insert_location == OVERSAMPLE_FACTOR - 5) begin
                    busy <= ~obtained_bit(oversampled_bit);
                    insert_location <= 0;
                    oversampled_bit <= 0;
                    state <= busy ? STARTED : IDLE;
                end
            end

            STARTED: begin
                oversampled_bit[insert_location] <= rx;
                insert_location <= insert_location + 1;
                if (insert_location == OVERSAMPLE_FACTOR - 1) begin
                    sampled_rx_data[bit_location] <= obtained_bit(oversampled_bit);
                    oversampled_bit <= 0;
                    insert_location <= 0;
                    bit_location <= bit_location + 1;
                    if (bit_location == TOTAL_RX_BITS - 1)
                        state <= CONVERSION;
                end
            end

            CONVERSION: begin
                if (!converted) begin
                    converted <= 1;
                    data <= sampled_rx_data[FRAME_BITS-1:0];
                    if (PARITY_BIT < 2)
                        data_valid <= (^data == PARITY_BIT);
                    else
                        data_valid <= 1;
                end
                if (flush)
                    state <= IDLE;
            end
        endcase
    end
end

// === Bit majority function ===
function obtained_bit;
    input [OVERSAMPLE_FACTOR-1:0] oversampled_vector;
    begin
        obtained_bit = (oversampled_vector[SAMPLE_LOC_1] & oversampled_vector[SAMPLE_LOC_2]) |
                       (oversampled_vector[SAMPLE_LOC_2] & oversampled_vector[SAMPLE_LOC_3]) |
                       (oversampled_vector[SAMPLE_LOC_1] & oversampled_vector[SAMPLE_LOC_3]);
    end
endfunction

endmodule

