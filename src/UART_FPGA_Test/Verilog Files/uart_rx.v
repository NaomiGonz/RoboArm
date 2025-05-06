`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////
// Simple UART Receiver (8N1)
// From Nandland, slightly cleaned and configured
//////////////////////////////////////////////////////////////////////

module uart_rx #(
    parameter CLKS_PER_BIT = 868  // for 100MHz clk, 115200 baud
)(
    input        i_Clock,
    input        i_Rx_Serial,
    output reg   o_Rx_DV,
    output reg [7:0] o_Rx_Byte
);

    localparam s_IDLE         = 3'b000;
    localparam s_RX_START_BIT = 3'b001;
    localparam s_RX_DATA_BITS = 3'b010;
    localparam s_RX_STOP_BIT  = 3'b011;
    localparam s_CLEANUP      = 3'b100;
   
    reg [2:0] r_SM_Main = 0;
    reg [11:0] r_Clock_Count = 0;
    reg [2:0] r_Bit_Index = 0;
    reg [7:0] r_Rx_Shift = 0;

    reg r_Rx_Data_R = 1'b1;
    reg r_Rx_Data = 1'b1;

    always @(posedge i_Clock) begin
        r_Rx_Data_R <= i_Rx_Serial;
        r_Rx_Data   <= r_Rx_Data_R;
    end

    always @(posedge i_Clock) begin
        case (r_SM_Main)
            s_IDLE: begin
                o_Rx_DV <= 1'b0;
                r_Clock_Count <= 0;
                r_Bit_Index <= 0;

                if (r_Rx_Data == 1'b0)  // Start bit detected
                    r_SM_Main <= s_RX_START_BIT;
            end

            s_RX_START_BIT: begin
                if (r_Clock_Count == (CLKS_PER_BIT-1)/2) begin
                    if (r_Rx_Data == 1'b0) begin
                        r_Clock_Count <= 0;
                        r_SM_Main <= s_RX_DATA_BITS;
                    end else begin
                        r_SM_Main <= s_IDLE;
                    end
                end else begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end
            end

            s_RX_DATA_BITS: begin
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    r_Rx_Shift[r_Bit_Index] <= r_Rx_Data;
                    if (r_Bit_Index < 7)
                        r_Bit_Index <= r_Bit_Index + 1;
                    else
                        r_SM_Main <= s_RX_STOP_BIT;
                end
            end

            s_RX_STOP_BIT: begin
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    o_Rx_DV <= 1'b1;
                    o_Rx_Byte <= r_Rx_Shift;
                    r_Clock_Count <= 0;
                    r_SM_Main <= s_CLEANUP;
                end
            end

            s_CLEANUP: begin
                r_SM_Main <= s_IDLE;
                o_Rx_DV <= 1'b0;
            end

            default: r_SM_Main <= s_IDLE;
        endcase
    end
endmodule
