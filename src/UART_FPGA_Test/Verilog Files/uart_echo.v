`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// uart_test - test link for RoboArm
//   • Receive 20 bytes (5 joint angles) from BeagleBone
//   • Transmit 60 bytes (5×{x,y,z}) back - hard-coded to 1.0f
//   • Heart-beat LED toggles every 0.5 s
//////////////////////////////////////////////////////////////////////////////////
module uart_test
(
    input  wire clk,          // 100 MHz Basys-3 clock (W5)
    input  wire rst,          // active-high reset      (BTN-C)
    input  wire rx,           // UART RX  (P9-24 ? JA1)
    output wire tx,           // UART TX  (P9-26 ? JA2)
    output reg  led_hb        // heart-beat LED (E19)
);

// -----------------------------------------------------------------------------
// UART instances (115 200 baud)
// -----------------------------------------------------------------------------
localparam integer CLKS_PER_BIT = 868;   // 100 MHz / 115 200 ? 868.1

wire       rx_dv;
wire [7:0] rx_byte;

uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) U_RX (
    .i_Clock    (clk),
    .i_Rx_Serial(rx),
    .o_Rx_DV    (rx_dv),
    .o_Rx_Byte  (rx_byte)
);

reg        tx_dv_reg  = 1'b0;
reg  [7:0] tx_byte_reg = 8'h00;
wire       tx_busy;
wire       tx_done;

uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) U_TX (
    .i_Clock    (clk),
    .i_Tx_DV    (tx_dv_reg),
    .i_Tx_Byte  (tx_byte_reg),
    .o_Tx_Active(tx_busy),
    .o_Tx_Serial(tx),
    .o_Tx_Done  (tx_done)
);

// give top-level port the short name Vivado expects
assign tx = tx;   // (keeps tx as top-level)

// -----------------------------------------------------------------------------
// RX buffer (20 bytes) & TX ROM (60 bytes)
// -----------------------------------------------------------------------------
reg [7:0] rx_buf [0:19];
reg [4:0] rx_cnt = 5'd0;              // 0 … 19

reg [7:0] tx_rom [0:59];
reg [5:0] tx_cnt = 6'd0;              // 0 … 59

reg       sending   = 1'b0;           // we are in the middle of a reply
reg       load_next = 1'b0;           // request to load next byte into UART

// -----------------------------------------------------------------------------
// ROM initialisation - 5 points (1.0,1.0,1.0) each
// IEEE-754 1.0f little-endian = 00 00 80 3F
// -----------------------------------------------------------------------------
integer k;
initial begin
    for (k = 0; k < 60; k = k + 1) begin
        case (k % 4)
            0,1 : tx_rom[k] = 8'h00;
            2   : tx_rom[k] = 8'h80;
            3   : tx_rom[k] = 8'h3F;
        endcase
    end
end

// -----------------------------------------------------------------------------
// Main control FSM
// -----------------------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        //---------------- Global reset ----------------
        rx_cnt     <= 0;
        sending    <= 1'b0;
        tx_cnt     <= 0;
        tx_dv_reg  <= 1'b0;
        load_next  <= 1'b0;

    end else begin
        //---------------- Reception phase ----------------
        if (rx_dv && !sending) begin
            rx_buf[rx_cnt] <= rx_byte;
            if (rx_cnt == 5'd19) begin           // 20th byte just arrived
                sending  <= 1'b0;                // will be set to 1 below
                load_next <= 1'b1;               // trigger first TX byte
                sending  <= 1'b1;
                rx_cnt   <= 0;
            end else begin
                rx_cnt <= rx_cnt + 1'b1;
            end
        end

        //---------------- Transmission phase ----------------
        // load_next gets asserted either at start (above) or when tx_done fires
        if (load_next && !tx_busy) begin         // UART is idle ? load byte
            tx_byte_reg <= tx_rom[tx_cnt];
            tx_dv_reg   <= 1'b1;                 // keep high until busy goes high
            load_next   <= 1'b0;                 // clear request
        end

        // drop dv **after** transmitter has latched the byte
        if (tx_dv_reg && tx_busy)
            tx_dv_reg <= 1'b0;

        // when current byte completely sent, prepare the next
        if (sending && tx_done) begin
            if (tx_cnt == 6'd59) begin           // last byte done
                tx_cnt  <= 0;
                sending <= 1'b0;
            end else begin
                tx_cnt   <= tx_cnt + 1'b1;
                load_next <= 1'b1;               // request next byte
            end
        end
    end
end

// -----------------------------------------------------------------------------
// 0.5-second heartbeat (@100 MHz)
// -----------------------------------------------------------------------------
reg [26:0] hb_ctr = 0;      // width for up to ~1.3 s

always @(posedge clk) begin
    if (rst) begin
        hb_ctr <= 0;
        led_hb <= 1'b0;
    end else if (hb_ctr == 27'd49_999_999) begin   // 0.5 s
        hb_ctr <= 0;
        led_hb <= ~led_hb;
    end else begin
        hb_ctr <= hb_ctr + 1'b1;
    end
end

endmodule