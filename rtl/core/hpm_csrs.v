//==============================================================================
// Module Name: hpm_csrs
// Description: Privileged Architecture M-Mode Trap Controller & Hardware
//              Performance Monitor (HPM) CSRs compliant with RISC-V v1.11/v1.12.
//
// Supported M-Mode Control & Status Registers (CSRs):
//   - 0x300: mstatus        (Machine Status: MIE bit 3, MPIE bit 7, MPP bits 12:11)
//   - 0x304: mie            (Machine Interrupt Enable: MTIE bit 7)
//   - 0x305: mtvec          (Machine Trap Vector Base Address)
//   - 0x320: mcountinhibit  (Bit 0: CY, 2: IR, 3: HPM3, 4: HPM4)
//   - 0x341: mepc           (Machine Exception Program Counter)
//   - 0x342: mcause         (Machine Trap Cause Register)
//   - 0x344: mip            (Machine Interrupt Pending: MTIP bit 7)
//   - 0xC00 / 0xB00: mcycle         (Cycles elapsed [31:0])
//   - 0xC80 / 0xB80: mcycleh        (Cycles elapsed [63:32])
//   - 0xC02 / 0xB02: minstret       (Instructions retired [31:0])
//   - 0xC82 / 0xB82: minstreth      (Instructions retired [63:32])
//   - 0xC03 / 0xB03: mhpmcounter3   (Hardware Event 3: Branches [31:0])
//   - 0xC83 / 0xB83: mhpmcounter3h  (Hardware Event 3: Branches [63:32])
//   - 0xC04 / 0xB04: mhpmcounter4   (Hardware Event 4: Mispredictions [31:0])
//   - 0xC84 / 0xB84: mhpmcounter4h  (Hardware Event 4: Mispredictions [63:32])
//
// Instructions Handled:
//   - CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI, MRET
//==============================================================================

`timescale 1ns / 1ps

module hpm_csrs (
    input  wire        clk,
    input  wire        rst_n,

    // Pipeline CSR Access Interface
    input  wire        csr_en,          // 1 if CSR operation requested
    input  wire [2:0]  csr_funct3,      // CSR operation mode
    input  wire [11:0] csr_addr,        // 12-bit CSR address
    input  wire [31:0] csr_wdata,       // Write operand (from rs1 or zimm)
    output reg  [31:0] csr_rdata,       // Read value returned to rd

    // Telemetry Event Strobes
    input  wire        event_cycle,     // Active clock cycle
    input  wire        event_instret,   // Instruction committed in WB stage
    input  wire        event_branch,    // Branch/jump resolved in EX stage
    input  wire        event_mispredict,// Branch misprediction occurred

    // M-Mode Trap & Interrupt Interface
    input  wire        timer_irq_in,    // Physical timer interrupt line from CLINT
    output wire        irq_pending,     // Qualified interrupt request to pipeline
    input  wire        trap_taken,      // 1-cycle strobe when pipeline takes trap
    input  wire [31:0] trap_pc,         // Current PC latched into mepc on trap
    input  wire [31:0] trap_cause,      // Cause code latched into mcause
    input  wire        mret_exec,       // 1-cycle strobe when mret executes in EX
    output wire [31:0] mtvec_out,       // Vector jump target for pipeline redirect
    output wire [31:0] mepc_out,        // Return jump target for mret redirect
    output wire [31:0] mstatus_out,     // Exported mstatus for diagnostics
    output wire [31:0] mcause_out       // Exported mcause for diagnostics
);

    // M-Mode Architectural Registers
    reg        mstatus_mie;             // Bit 3: Global interrupt enable
    reg        mstatus_mpie;            // Bit 7: Prior interrupt enable
    reg        mie_mtie;                // Bit 7: Timer interrupt enable
    reg [31:0] mtvec;                   // Trap vector base
    reg [31:0] mepc;                    // Exception PC
    reg [31:0] mcause;                  // Exception cause

    // 64-bit Performance Counters
    reg [63:0] mcycle;
    reg [63:0] minstret;
    reg [63:0] mhpmcounter3;
    reg [63:0] mhpmcounter4;
    reg [31:0] mcountinhibit;

    // Full mstatus composition (MPP = 2'b11 for Machine mode)
    wire [31:0] mstatus_val = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};
    wire [31:0] mie_val     = {24'b0, mie_mtie, 7'b0};
    wire [31:0] mip_val     = {24'b0, timer_irq_in, 7'b0};

    // Diagnostic & Redirection Outputs
    assign mtvec_out   = mtvec;
    assign mepc_out    = mepc;
    assign mstatus_out = mstatus_val;
    assign mcause_out  = mcause;

    // Qualified Timer Interrupt: physical IRQ asserted AND global MIE AND timer MTIE
    assign irq_pending = timer_irq_in && mstatus_mie && mie_mtie;

    // Combinational CSR Readout Multiplexer
    always @(*) begin
        case (csr_addr)
            12'h300:          csr_rdata = mstatus_val;
            12'h304:          csr_rdata = mie_val;
            12'h305:          csr_rdata = mtvec;
            12'h320:          csr_rdata = mcountinhibit;
            12'h341:          csr_rdata = mepc;
            12'h342:          csr_rdata = mcause;
            12'h344:          csr_rdata = mip_val;
            12'hC00, 12'hB00: csr_rdata = mcycle[31:0];
            12'hC80, 12'hB80: csr_rdata = mcycle[63:32];
            12'hC02, 12'hB02: csr_rdata = minstret[31:0];
            12'hC82, 12'hB82: csr_rdata = minstret[63:32];
            12'hC03, 12'hB03: csr_rdata = mhpmcounter3[31:0];
            12'hC83, 12'hB83: csr_rdata = mhpmcounter3[63:32];
            12'hC04, 12'hB04: csr_rdata = mhpmcounter4[31:0];
            12'hC84, 12'hB84: csr_rdata = mhpmcounter4[63:32];
            default:          csr_rdata = 32'h0000_0000;
        endcase
    end

    // Calculated CSR Write Value
    reg [31:0] next_csr_val;
    always @(*) begin
        case (csr_funct3[1:0])
            2'b01:   next_csr_val = csr_wdata;                  // CSRRW / CSRRWI
            2'b10:   next_csr_val = csr_rdata | csr_wdata;      // CSRRS / CSRRSI
            2'b11:   next_csr_val = csr_rdata & ~csr_wdata;     // CSRRC / CSRRCI
            default: next_csr_val = csr_rdata;
        endcase
    end

    // Synchronous Counter & CSR Management
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstatus_mie   <= 1'b0;
            mstatus_mpie  <= 1'b0;
            mie_mtie      <= 1'b0;
            mtvec         <= 32'h0000_0000;
            mepc          <= 32'h0000_0000;
            mcause        <= 32'h0000_0000;
            mcycle        <= 64'd0;
            minstret      <= 64'd0;
            mhpmcounter3  <= 64'd0;
            mhpmcounter4  <= 64'd0;
            mcountinhibit <= 32'd0;
        end else begin
            // 1. Hardware Event Incrementers
            if (event_cycle && !mcountinhibit[0]) begin
                mcycle <= mcycle + 64'd1;
            end

            if (event_instret && !mcountinhibit[2]) begin
                minstret <= minstret + 64'd1;
            end

            if (event_branch && !mcountinhibit[3]) begin
                mhpmcounter3 <= mhpmcounter3 + 64'd1;
            end

            if (event_mispredict && !mcountinhibit[4]) begin
                mhpmcounter4 <= mhpmcounter4 + 64'd1;
            end

            // 2. Hardware Trap Entry & MRET Execution (Priority over software writes)
            if (trap_taken) begin
                mepc         <= trap_pc;
                mcause       <= trap_cause;
                mstatus_mpie <= mstatus_mie; // Preserve current MIE in MPIE
                mstatus_mie  <= 1'b0;        // Disable interrupts during trap handler
            end else if (mret_exec) begin
                mstatus_mie  <= mstatus_mpie; // Restore MIE from MPIE
                mstatus_mpie <= 1'b1;         // Set MPIE to 1
            end else if (csr_en) begin
                // 3. Explicit Software CSR Writes
                case (csr_addr)
                    12'h300: begin
                        mstatus_mie  <= next_csr_val[3];
                        mstatus_mpie <= next_csr_val[7];
                    end
                    12'h304:          mie_mtie             <= next_csr_val[7];
                    12'h305:          mtvec                <= next_csr_val;
                    12'h320:          mcountinhibit        <= next_csr_val;
                    12'h341:          mepc                 <= next_csr_val;
                    12'h342:          mcause               <= next_csr_val;
                    12'hC00, 12'hB00: mcycle[31:0]        <= next_csr_val;
                    12'hC80, 12'hB80: mcycle[63:32]       <= next_csr_val;
                    12'hC02, 12'hB02: minstret[31:0]      <= next_csr_val;
                    12'hC82, 12'hB82: minstret[63:32]     <= next_csr_val;
                    12'hC03, 12'hB03: mhpmcounter3[31:0]  <= next_csr_val;
                    12'hC83, 12'hB83: mhpmcounter3[63:32] <= next_csr_val;
                    12'hC04, 12'hB04: mhpmcounter4[31:0]  <= next_csr_val;
                    12'hC84, 12'hB84: mhpmcounter4[63:32] <= next_csr_val;
                    default: ;
                endcase
            end
        end
    end

endmodule
