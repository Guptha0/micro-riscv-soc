//==============================================================================
// Module Name: clint_timer
// Description: RISC-V Core Local Interruptor (CLINT) Hardware Machine Timer.
//
// Specifications:
//   - 64-bit real-time cycle counter (mtime) incrementing on every clock cycle.
//   - 64-bit machine timer compare register (mtimecmp).
//   - Hardware interrupt signal: timer_irq is asserted whenever mtime >= mtimecmp.
//
// Memory-Mapped Address Registers:
//   - 0x02004000 / 0x80000010 : mtimecmp (lower 32-bit word)
//   - 0x02004004 / 0x80000014 : mtimecmp (upper 32-bit word)
//   - 0x0200BFF8 / 0x80000018 : mtime    (lower 32-bit word)
//   - 0x0200BFFC / 0x8000001C : mtime    (upper 32-bit word)
//==============================================================================

`timescale 1ns / 1ps

module clint_timer (
    input  wire        clk,        // System clock
    input  wire        rst_n,      // Asynchronous active-low reset
    input  wire        mem_we,     // MMIO write enable
    input  wire        mem_re,     // MMIO read enable
    input  wire [31:0] mem_addr,   // 32-bit MMIO byte address
    input  wire [31:0] mem_wdata,  // 32-bit MMIO write data
    output reg  [31:0] mem_rdata,  // 32-bit MMIO read data
    output wire        timer_irq,  // Hardware Timer Interrupt output
    output wire [63:0] mtime_out,  // Real-time counter probe
    output wire [63:0] mtimecmp_out// Compare threshold probe
);

    reg [63:0] mtime;
    reg [63:0] mtimecmp;

    // Timer Interrupt is active-high whenever mtime >= mtimecmp
    assign timer_irq    = (mtime >= mtimecmp);
    assign mtime_out    = mtime;
    assign mtimecmp_out = mtimecmp;

    // Address Decoding (supports RISC-V standard CLINT offsets and 0x8000001X alias)
    wire is_mtimecmp_lo = (mem_addr == 32'h0200_4000) || (mem_addr == 32'h8000_0010);
    wire is_mtimecmp_hi = (mem_addr == 32'h0200_4004) || (mem_addr == 32'h8000_0014);
    wire is_mtime_lo    = (mem_addr == 32'h0200_BFF8) || (mem_addr == 32'h8000_0018);
    wire is_mtime_hi    = (mem_addr == 32'h0200_BFFC) || (mem_addr == 32'h8000_001C);

    // Synchronous Update & MMIO Writes
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtime    <= 64'd0;
            mtimecmp <= 64'h0000_0000_FFFF_FFFF; // Set to 32-bit max initially to avoid early IRQ
        end else begin
            // mtime increments every clock cycle
            mtime <= mtime + 64'd1;

            // Software writes override counter or compare values
            if (mem_we) begin
                if (is_mtimecmp_lo) mtimecmp[31:0]  <= mem_wdata;
                if (is_mtimecmp_hi) mtimecmp[63:32] <= mem_wdata;
                if (is_mtime_lo)    mtime[31:0]     <= mem_wdata;
                if (is_mtime_hi)    mtime[63:32]    <= mem_wdata;
            end
        end
    end

    // Combinational MMIO Reads
    always @(*) begin
        if (mem_re) begin
            if (is_mtimecmp_lo)      mem_rdata = mtimecmp[31:0];
            else if (is_mtimecmp_hi) mem_rdata = mtimecmp[63:32];
            else if (is_mtime_lo)    mem_rdata = mtime[31:0];
            else if (is_mtime_hi)    mem_rdata = mtime[63:32];
            else                     mem_rdata = 32'h0000_0000;
        end else begin
            mem_rdata = 32'h0000_0000;
        end
    end

endmodule
