//==============================================================================
// Module Name: regfile
// Description: Production-grade 32-bit RISC-V (RV32I) Register File
//
// Specifications:
//   - 32 general-purpose registers (x0 to x31), each 32 bits wide.
//   - Register x0 is permanently hardwired to 32'h00000000.
//     Any write attempt to x0 is strictly ignored.
//   - Dual asynchronous read ports:
//       * rs1_data asynchronously outputs the value of register rs1.
//       * rs2_data asynchronously outputs the value of register rs2.
//       * Read address 5'd0 unconditionally yields 32'h00000000.
//   - Single synchronous write port:
//       * Writes occur on posedge clk when write_enable is asserted.
//       * Writes to rd = 5'd0 are suppressed.
//   - Configurable WRITE_FIRST parameter:
//       * 0 (default): Standard dual-port asynchronous read (classic RISC-V).
//       * 1: Internal write-through bypass forwarding if reading the same
//            register being written in the active clock cycle.
//==============================================================================

`timescale 1ns / 1ps

module regfile #(
    parameter WRITE_FIRST = 0 // 0: standard async read, 1: write-through bypass
)(
    input  wire        clk,          // System clock
    input  wire        rst_n,        // Active-low asynchronous reset
    input  wire        write_enable, // Synchronous write enable (RegWrite)
    input  wire [4:0]  rs1,          // Source register 1 address (Read Port 1)
    input  wire [4:0]  rs2,          // Source register 2 address (Read Port 2)
    input  wire [4:0]  rd,           // Destination register address (Write Port)
    input  wire [31:0] rd_data,      // Write data input
    output wire [31:0] rs1_data,     // Read data output 1
    output wire [31:0] rs2_data      // Read data output 2
);

    // 32 registers, 32 bits wide
    reg [31:0] registers [31:0];

    integer i;

    //--------------------------------------------------------------------------
    // Initial Power-On State (Clears simulation 'x' states)
    //--------------------------------------------------------------------------
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'h0000_0000;
        end
    end

    //--------------------------------------------------------------------------
    // Synchronous Write Port with x0 Protection
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all architectural registers x1-x31 to 0
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'h0000_0000;
            end
        end else begin
            // Synchronous write only when write_enable is high AND target rd is not x0
            if (write_enable && (rd != 5'd0)) begin
                registers[rd] <= rd_data;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Dual Asynchronous Read Ports with Permanent Hardwired x0 Protection
    //--------------------------------------------------------------------------
    // Read Port 1 (rs1)
    assign rs1_data = (rs1 == 5'd0) ? 32'h0000_0000 :
                      (WRITE_FIRST && write_enable && (rs1 == rd)) ? rd_data :
                      registers[rs1];

    // Read Port 2 (rs2)
    assign rs2_data = (rs2 == 5'd0) ? 32'h0000_0000 :
                      (WRITE_FIRST && write_enable && (rs2 == rd)) ? rd_data :
                      registers[rs2];

endmodule
