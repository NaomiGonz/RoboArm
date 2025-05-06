`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////
// Simple UART Transmitter (8N1)
// From Nandland, slightly cleaned and configured
//////////////////////////////////////////////////////////////////////

module uart_tx #(
    parameter CLKS_PER_BIT = 868
)(
    input        i_Clock,
    input        i_Tx_DV,
    input  [7:0] i_Tx_Byte,
    output reg   o_Tx_Serial,
    output       o_Tx_Active,
    output reg   o_Tx_Done
);

    localparam s_IDLE         = 3'b000;
    localparam s_TX_START_BIT = 3'b001;
    localparam s_TX_DATA_BITS = 3'b010;
    localparam s_TX_STOP_BIT  = 3'b011;
    localparam s_CLEANUP      = 3'b100;

    reg [2:0] r_SM_Main = 0;
    reg [11:0] r_Clock_Count = 0;
    reg [2:0] r_Bit_Index = 0;
    reg [7:0] r_Tx_Data = 0;
    reg r_Tx_Active = 0;

    always @(posedge i_Clock) begin
        case (r_SM_Main)
            s_IDLE: begin
                o_Tx_Serial <= 1'b1;  // Idle high
                o_Tx_Done <= 1'b0;
                r_Clock_Count <= 0;
                r_Bit_Index <= 0;

                if (i_Tx_DV == 1'b1) begin
                    r_Tx_Active <= 1'b1;
                    r_Tx_Data <= i_Tx_Byte;
                    r_SM_Main <= s_TX_START_BIT;
                end
            end

            s_TX_START_BIT: begin
                o_Tx_Serial <= 1'b0;
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    r_SM_Main <= s_TX_DATA_BITS;
                end
            end

            s_TX_DATA_BITS: begin
                o_Tx_Serial <= r_Tx_Data[r_Bit_Index];
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Clock_Count <= 0;
                    if (r_Bit_Index < 7)
                        r_Bit_Index <= r_Bit_Index + 1;
                    else
                        r_SM_Main <= s_TX_STOP_BIT;
                end
            end

            s_TX_STOP_BIT: begin
                o_Tx_Serial <= 1'b1;
                if (r_Clock_Count < CLKS_PER_BIT-1) begin
                    r_Clock_Count <= r_Clock_Count + 1;
                end else begin
                    r_Tx_Active <= 1'b0;
                    r_SM_Main <= s_CLEANUP;
                    r_Clock_Count <= 0;
                end
            end

            s_CLEANUP: begin
                o_Tx_Done <= 1'b1;
                r_SM_Main <= s_IDLE;
            end

            default: r_SM_Main <= s_IDLE;
        endcase
    end

    assign o_Tx_Active = r_Tx_Active;

endmodule
