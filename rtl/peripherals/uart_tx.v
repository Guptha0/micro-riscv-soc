//==============================================================================
// Module Name: uart_tx
// Description: Production-grade 8-bit Asynchronous Serial UART Transmitter
//              with configurable clock divider and MMIO register interface.
//
// Protocol:
//   - 1 Start Bit (logic LOW)
//   - 8 Data Bits (LSB first)
//   - 1 Stop Bit (logic HIGH)
//   - Idle Line: HIGH
//
// MMIO Mapping:
//   - Data Register: Writing byte triggers serial transmission.
//   - Status: Read bit 0 indicates busy status (1 = busy, 0 = ready/idle).
//==============================================================================

`timescale 1ns / 1ps

module uart_tx #(
    parameter DEFAULT_CLKS_PER_BIT = 8 // Clock divider for simulation (e.g., 8 cycles/bit)
)(
    input  wire       clk,       // System clock
    input  wire       rst_n,     // Active-low reset
    input  wire       tx_start,  // Trigger transmission (write enable pulse)
    input  wire [7:0] tx_data,   // 8-bit data byte to transmit
    output reg        tx_pin,    // Serial TX physical line
    output reg        tx_busy,   // Busy indicator (1: transmitting, 0: idle)
    output reg        tx_done    // Pulse asserted upon byte transmission completion
);

    // State Encoding
    localparam [2:0] STATE_IDLE  = 3'b000;
    localparam [2:0] STATE_START = 3'b001;
    localparam [2:0] STATE_DATA  = 3'b010;
    localparam [2:0] STATE_STOP  = 3'b011;

    reg [2:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_buf;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= STATE_IDLE;
            clk_cnt  <= 16'd0;
            bit_idx  <= 3'd0;
            data_buf <= 8'd0;
            tx_pin   <= 1'b1; // Idle high
            tx_busy  <= 1'b0;
            tx_done  <= 1'b0;
        end else begin
            tx_done <= 1'b0; // Default pulse low

            case (state)
                STATE_IDLE: begin
                    tx_pin  <= 1'b1;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;

                    if (tx_start) begin
                        data_buf <= tx_data;
                        tx_busy  <= 1'b1;
                        state    <= STATE_START;
                    end else begin
                        tx_busy  <= 1'b0;
                    end
                end

                STATE_START: begin
                    tx_pin <= 1'b0; // Start bit

                    if (clk_cnt < (DEFAULT_CLKS_PER_BIT - 1)) begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end else begin
                        clk_cnt <= 16'd0;
                        state   <= STATE_DATA;
                    end
                end

                STATE_DATA: begin
                    tx_pin <= data_buf[bit_idx]; // Transmit LSB first

                    if (clk_cnt < (DEFAULT_CLKS_PER_BIT - 1)) begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end else begin
                        clk_cnt <= 16'd0;
                        if (bit_idx < 3'd7) begin
                            bit_idx <= bit_idx + 3'd1;
                        end else begin
                            bit_idx <= 3'd0;
                            state   <= STATE_STOP;
                        end
                    end
                end

                STATE_STOP: begin
                    tx_pin <= 1'b1; // Stop bit

                    if (clk_cnt < (DEFAULT_CLKS_PER_BIT - 1)) begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end else begin
                        clk_cnt <= 16'd0;
                        tx_busy <= 1'b0;
                        tx_done <= 1'b1;
                        state   <= STATE_IDLE;
                    end
                end

                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
