//==============================================================================
// Module Name: dmem
// Description: Synchronous 4KB Main Data Memory (RAM) for RV32I / RV32IM SoC.
//
// Supports:
//   - Synchronous write (sw) on posedge clk when mem_write is asserted.
//   - Combinational/synchronous read (lw) based on word-aligned address.
//   - Word address space: 1024 words = 4096 bytes (4 KB).
//==============================================================================

`timescale 1ns / 1ps

module dmem #(
    parameter RAM_WORDS = 1024 // 1024 words = 4096 bytes (4 KB)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        mem_write,
    input  wire        mem_read,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output wire [31:0] rdata
);

    reg [31:0] ram [0:RAM_WORDS-1];
    integer i;

    initial begin
        for (i = 0; i < RAM_WORDS; i = i + 1) begin
            ram[i] = 32'h0000_0000;
        end
    end

    wire [9:0] word_idx = addr[11:2];

    always @(posedge clk) begin
        if (mem_write && (addr[31:12] != 20'h80000) && (addr[31:24] != 8'h02)) begin
            ram[word_idx] <= wdata;
        end
    end

    assign rdata = (mem_read && (addr[31:12] != 20'h80000) && (addr[31:24] != 8'h02)) ? ram[word_idx] : 32'h0000_0000;

endmodule
