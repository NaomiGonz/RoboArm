`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.04.2025 21:07:29
// Design Name: 
// Module Name: uart_echo
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module uart_echo (
    input clk,           // 100 MHz FPGA clock
    input rst,           // active-high reset
    input rx,            // UART RX from BeagleBone
    output tx,            // UART TX to BeagleBone
    output reg led_dbg,    // Debug LED output (connect this to any available LED)
    output reg led_dbg1    // Debug LED output (connect this to any available LED)
);

wire [7:0] rx_data;
wire rx_converted;
wire rx_valid;
reg flush;

wire tx_busy;
reg tx_enable;
reg [7:0] tx_data;

// === Instantiate Receiver ===
uart_rx #(
    .FRAME_BITS(8),
    .PARITY_BIT(2),  // 2 = disabled
    .STOP_BITS(1)
) uart_rx_inst (
    .rx(rx),
    .i_clk(clk),
    .flush(flush),
    .data(rx_data),
    .converted(rx_converted),
    .data_valid(rx_valid),
    .busy()
);

// === Instantiate Transmitter ===
uart_tx #(
    .FRAME_BITS(8),
    .PARITY_BIT(2),
    .STOP_BITS(1),
    .RETENTION_DURATION(15)
) uart_tx_inst (
    .clk(clk),
    .tx_enable(tx_enable),
    .data(tx_data),
    .tx(tx),
    .tx_busy(tx_busy)
);

// === Echo Control ===
reg [1:0] state;
localparam IDLE = 0, SEND = 1, FLUSH = 2;

always @(posedge clk) begin
    if (rst) begin
        state <= IDLE;
        tx_enable <= 0;
        flush <= 0;
    end else begin
        case (state)
            IDLE: begin
                tx_enable <= 0;
                flush <= 0;
                if (rx_converted && rx_valid && !tx_busy) begin
                    tx_data <= rx_data;
                    tx_enable <= 1;
                    state <= SEND;
                end
            end

            SEND: begin
                tx_enable <= 0;  // pulse-style
                state <= FLUSH;
            end

            FLUSH: begin
                flush <= 1;
                state <= IDLE;
            end
        endcase
    end
end

// === Debug LED toggles briefly when UART RX sees a valid byte ===
reg [23:0] led_timer;

always @(posedge clk) begin
    if (rst) begin
        led_dbg <= 0;
        led_timer <= 0;
    end else begin
        if (rx_converted && rx_valid) begin
            led_dbg <= 1;
            led_timer <= 0;
        end else if (led_timer < 24'd8_000_000) begin
            led_timer <= led_timer + 1;
        end else begin
            led_dbg <= 0;
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        led_dbg1 <= 0;
    end else begin
        led_dbg1 <= ~led_dbg1; // Blink constantly
    end
end


endmodule
