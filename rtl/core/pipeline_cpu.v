//==============================================================================
// Module Name: pipeline_cpu
// Description: Advanced 5-Stage Pipelined 32-bit RISC-V Processor Core
//              supporting RV32IMZicsr_Zalrsc + Dynamic Branch Prediction + HPM CSRs.
//
// Pipeline Stages:
//   1. IF  : Instruction Fetch (PC, I-Cache, 64-entry BTB/BHT Branch Predictor)
//   2. ID  : Instruction Decode & Register Fetch (imm_gen, regfile, hazard unit)
//   3. EX  : Execute & Address Calculation (ALU, M-Ext, Forwarding, CSRs, Traps)
//   4. MEM : Memory Access, D-Cache, Byte Masking & RV32A Atomic Reservation Controller
//   5. WB  : Write-Back to Register File (formatted sub-word loads)
//
// Architectural Features:
//   - Complete RV32I Base Integer Instruction Set (Shifts, Compares, Sub-word Load/Store)
//   - RV32M Hardware Multiplier and Hardware Restoring Divider
//   - RV32A Atomic Memory Extension (LR.W / SC.W) with bus-snooping invalidation
//   - Dynamic Branch Prediction with 64-entry BTB and 2-bit Saturating BHT
//   - Privileged Architecture HPM Telemetry Counters (mcycle, minstret, mhpmcounter3/4)
//   - Privileged M-Mode Trap & Exception Controller (timer interrupt, ecall, ebreak, illegal)
//   - Robust Pipeline Valid Bits preventing bubble overcounting
//==============================================================================

`timescale 1ns / 1ps

module pipeline_cpu (
    input  wire        clk,
    input  wire        rst_n,

    // Instruction Cache Interface (IF stage)
    output wire [31:0] icache_pc,
    output wire        icache_read_en,
    input  wire [31:0] icache_instruction,
    input  wire        icache_stall,

    // Data Memory / MMIO Bus Interface (MEM stage)
    output wire [31:0] dmem_addr,
    output wire [31:0] dmem_wdata,
    output wire [3:0]  dmem_wstrb,
    output wire        dmem_we,
    output wire        dmem_re,
    input  wire [31:0] dmem_rdata,
    input  wire        dmem_stall,

    // Bus-Snooping Invalidation (RV32A)
    input  wire        snoop_we,
    input  wire [31:0] snoop_addr,

    // Real-Time System Tracing & Debug Ports
    output wire [31:0] debug_if_pc,
    output wire [31:0] debug_id_pc,
    output wire [31:0] debug_ex_pc,
    output wire [31:0] debug_mem_pc,
    output wire [31:0] debug_wb_pc,
    output wire [31:0] debug_wb_data,
    output wire [4:0]  debug_wb_rd,
    output wire        debug_wb_reg_write,
    output wire        debug_stall,
    output wire        debug_flush,

    // Advanced Telemetry Probes
    output wire        trace_pred_taken,
    output wire        trace_mispredict,
    output wire        trace_sc_success,
    output wire [63:0] trace_csr_mcycle,
    output wire [63:0] trace_csr_minstret,

    // M-Mode Trap & Interrupt Ports
    input  wire        timer_irq_in,
    output wire        trace_trap_taken,
    output wire        trace_mret_exec,
    output wire [31:0] trace_mtvec,
    output wire [31:0] trace_mepc,
    output wire [31:0] trace_mcause,
    output wire [31:0] trace_mstatus
);

    // Opcodes per RISC-V RV32I / RV32M / RV32A / Privileged specifications
    localparam [6:0] OPCODE_R_TYPE = 7'b0110011; // 7'h33
    localparam [6:0] OPCODE_I_TYPE = 7'b0010011; // 7'h13
    localparam [6:0] OPCODE_LOAD   = 7'b0000011; // 7'h03
    localparam [6:0] OPCODE_STORE  = 7'b0100011; // 7'h23
    localparam [6:0] OPCODE_BRANCH = 7'b1100011; // 7'h63
    localparam [6:0] OPCODE_JAL    = 7'b1101111; // 7'h6F
    localparam [6:0] OPCODE_JALR   = 7'b1100111; // 7'h67
    localparam [6:0] OPCODE_LUI    = 7'b0110111; // 7'h37
    localparam [6:0] OPCODE_AUIPC  = 7'b0010111; // 7'h17
    localparam [6:0] OPCODE_AMO    = 7'b0101111; // 7'h2F (RV32A)
    localparam [6:0] OPCODE_SYSTEM = 7'b1110011; // 7'h73 (CSR / Privileged)

    // ALU operation encodings matching alu.v
    localparam [3:0] ALU_ADD  = 4'b0000;
    localparam [3:0] ALU_SUB  = 4'b1000;
    localparam [3:0] ALU_SLL  = 4'b0001;
    localparam [3:0] ALU_SLT  = 4'b0010;
    localparam [3:0] ALU_SLTU = 4'b0011;
    localparam [3:0] ALU_XOR  = 4'b0100;
    localparam [3:0] ALU_SRL  = 4'b0101;
    localparam [3:0] ALU_SRA  = 4'b1101;
    localparam [3:0] ALU_OR   = 4'b0110;
    localparam [3:0] ALU_AND  = 4'b0111;

    // Write-back source selection (3-bit)
    localparam [2:0] WB_ALU = 3'b000;
    localparam [2:0] WB_MEM = 3'b001;
    localparam [2:0] WB_PC4 = 3'b010;
    localparam [2:0] WB_CSR = 3'b011;
    localparam [2:0] WB_SC  = 3'b100;

    // Forward declarations
    wire        branch_mispredict_ex;
    wire [31:0] branch_redirect_pc;
    wire        branch_taken_ex;
    wire [31:0] branch_target_ex;
    wire        load_use_hazard;
    wire        stall_pipeline;
    wire        stall_m_ext;
    wire        dcache_stall = dmem_stall;
    wire        take_trap;
    wire        ex_is_mret;
    wire [31:0] mtvec_target;
    wire [31:0] mepc_target;

    //==========================================================================
    // 1. INSTRUCTION FETCH (IF) STAGE & DYNAMIC BRANCH PREDICTOR
    //==========================================================================
    reg  [31:0] if_pc;
    wire [31:0] if_pc_next;
    wire        pred_taken;
    wire [31:0] pred_target;
    wire        pred_hit;

    // Dynamic Branch Predictor Instance (BTB + BHT)
    wire update_btb;
    wire ex_is_jump;
    reg  [31:0] id_ex_pc;

    btb_bht #(
        .NUM_ENTRIES(64)
    ) u_btb_bht (
        .clk            (clk),
        .rst_n          (rst_n),
        .if_pc          (if_pc),
        .pred_taken     (pred_taken),
        .pred_target    (pred_target),
        .pred_hit       (pred_hit),
        .update_en      (update_btb),
        .update_pc      (id_ex_pc),
        .update_target  (branch_target_ex),
        .actual_taken   (branch_taken_ex),
        .actual_is_jump (ex_is_jump),
        .pred_count     (),
        .pred_hit_count (),
        .mispred_count  ()
    );

    assign stall_pipeline = load_use_hazard || icache_stall || stall_m_ext || dcache_stall;
    assign icache_pc      = if_pc;
    assign icache_read_en = rst_n;

    // Next PC Logic: Trap Vector > MRET Return > Misprediction redirect > Pipeline Stall > Speculative BTB Target > PC + 4
    assign if_pc_next = take_trap            ? mtvec_target :
                        ex_is_mret           ? mepc_target :
                        branch_mispredict_ex ? branch_redirect_pc :
                        stall_pipeline       ? if_pc :
                        pred_taken           ? pred_target :
                        (if_pc + 32'd4);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if_pc <= 32'h0000_0000;
        end else if (!stall_pipeline || branch_mispredict_ex || take_trap || ex_is_mret) begin
            if_pc <= if_pc_next;
        end
    end

    // IF/ID Pipeline Register (Carrying Speculative Prediction Data & Valid Bit)
    reg [31:0] if_id_pc;
    reg [31:0] if_id_instruction;
    reg        if_id_pred_taken;
    reg [31:0] if_id_pred_target;
    reg        if_id_valid;

    wire flush_if_id = branch_mispredict_ex || take_trap || ex_is_mret;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if_id_pc          <= 32'h0000_0000;
            if_id_instruction <= 32'h0000_0013; // NOP
            if_id_pred_taken  <= 1'b0;
            if_id_pred_target <= 32'h0000_0000;
            if_id_valid       <= 1'b0;
        end else if (flush_if_id) begin
            if_id_pc          <= 32'h0000_0000;
            if_id_instruction <= 32'h0000_0013; // Flush with NOP
            if_id_pred_taken  <= 1'b0;
            if_id_pred_target <= 32'h0000_0000;
            if_id_valid       <= 1'b0;
        end else if (dcache_stall || stall_m_ext) begin
            // Hold IF/ID frozen when downstream stages are stalled
        end else if (icache_stall) begin
            // Insert NOP bubble into ID stage while IF refills from ROM
            if_id_pc          <= 32'h0000_0000;
            if_id_instruction <= 32'h0000_0013;
            if_id_pred_taken  <= 1'b0;
            if_id_pred_target <= 32'h0000_0000;
            if_id_valid       <= 1'b0; // Bubble has valid = 0!
        end else if (!load_use_hazard) begin
            if_id_pc          <= if_pc;
            if_id_instruction <= icache_instruction;
            if_id_pred_taken  <= pred_taken;
            if_id_pred_target <= pred_target;
            if_id_valid       <= 1'b1;
        end
    end

    //==========================================================================
    // 2. INSTRUCTION DECODE (ID) STAGE
    //==========================================================================
    wire [6:0]  id_opcode = if_id_instruction[6:0];
    wire [4:0]  id_rd     = if_id_instruction[11:7];
    wire [2:0]  id_funct3 = if_id_instruction[14:12];
    wire [4:0]  id_rs1    = if_id_instruction[19:15];
    wire [4:0]  id_rs2    = if_id_instruction[24:20];
    wire [6:0]  id_funct7 = if_id_instruction[31:25];

    // Immediate Extraction
    wire [31:0] id_imm_i = {{20{if_id_instruction[31]}}, if_id_instruction[31:20]};
    wire [31:0] id_imm_s = {{20{if_id_instruction[31]}}, if_id_instruction[31:25], if_id_instruction[11:7]};
    wire [31:0] id_imm_b = {{20{if_id_instruction[31]}}, if_id_instruction[7], if_id_instruction[30:25], if_id_instruction[11:8], 1'b0};
    wire [31:0] id_imm_u = {if_id_instruction[31:12], 12'h000};
    wire [31:0] id_imm_j = {{12{if_id_instruction[31]}}, if_id_instruction[19:12], if_id_instruction[20], if_id_instruction[30:21], 1'b0};

    reg [31:0] id_imm;
    always @(*) begin
        case (id_opcode)
            OPCODE_I_TYPE, OPCODE_LOAD, OPCODE_JALR: id_imm = id_imm_i;
            OPCODE_STORE:                            id_imm = id_imm_s;
            OPCODE_BRANCH:                           id_imm = id_imm_b;
            OPCODE_LUI, OPCODE_AUIPC:                id_imm = id_imm_u;
            OPCODE_JAL:                              id_imm = id_imm_j;
            OPCODE_SYSTEM:                           id_imm = id_imm_i; // CSR 12-bit address in [31:20]
            default:                                 id_imm = id_imm_i;
        endcase
    end

    // Register File Instance (with internal write-first bypass)
    wire [31:0] id_rs1_data;
    wire [31:0] id_rs2_data;
    wire [31:0] wb_write_data;
    reg  [4:0]  mem_wb_rd;
    reg         mem_wb_reg_write;
    reg         mem_wb_valid;
    wire        regfile_we = mem_wb_valid && mem_wb_reg_write && (mem_wb_rd != 5'd0);

    regfile #(
        .WRITE_FIRST(1) // Transparent forwarding for ID/WB collisions
    ) u_regfile (
        .clk          (clk),
        .rst_n        (rst_n),
        .write_enable (regfile_we),
        .rs1          (id_rs1),
        .rs2          (id_rs2),
        .rd           (mem_wb_rd),
        .rd_data      (wb_write_data),
        .rs1_data     (id_rs1_data),
        .rs2_data     (id_rs2_data)
    );

    // Opcode Legality Check
    wire id_valid_opcode = (id_opcode == OPCODE_R_TYPE) ||
                           (id_opcode == OPCODE_I_TYPE) ||
                           (id_opcode == OPCODE_LOAD)   ||
                           (id_opcode == OPCODE_STORE)  ||
                           (id_opcode == OPCODE_BRANCH) ||
                           (id_opcode == OPCODE_JAL)    ||
                           (id_opcode == OPCODE_JALR)   ||
                           (id_opcode == OPCODE_LUI)    ||
                           (id_opcode == OPCODE_AUIPC)  ||
                           (id_opcode == OPCODE_AMO)    ||
                           (id_opcode == OPCODE_SYSTEM);

    // Synchronous Exception Detection in ID
    wire id_is_ecall   = if_id_valid && (id_opcode == OPCODE_SYSTEM) && (if_id_instruction[31:20] == 12'h000) && (id_funct3 == 3'b000);
    wire id_is_ebreak  = if_id_valid && (id_opcode == OPCODE_SYSTEM) && (if_id_instruction[31:20] == 12'h001) && (id_funct3 == 3'b000);
    wire id_is_mret_op = if_id_valid && (id_opcode == OPCODE_SYSTEM) && (if_id_instruction[31:20] == 12'h302) && (id_funct3 == 3'b000);
    wire id_is_illegal = if_id_valid && !id_valid_opcode;

    // Control Unit Decoder
    reg        id_reg_write;
    reg        id_mem_read;
    reg        id_mem_write;
    reg        id_alu_src_a_sel;
    reg        id_alu_src_b_sel;
    reg [3:0]  id_alu_ctrl;
    reg [2:0]  id_wb_sel;
    reg        id_is_branch;
    reg        id_is_jal;
    reg        id_is_jalr;
    reg        id_is_lr;
    reg        id_is_sc;
    reg        id_is_csr;
    reg        id_is_mret;

    always @(*) begin
        id_reg_write     = 1'b0;
        id_mem_read      = 1'b0;
        id_mem_write     = 1'b0;
        id_alu_src_a_sel = 1'b0; // 0: rs1, 1: pc
        id_alu_src_b_sel = 1'b0; // 0: rs2, 1: imm
        id_alu_ctrl      = ALU_ADD;
        id_wb_sel        = WB_ALU;
        id_is_branch     = 1'b0;
        id_is_jal        = 1'b0;
        id_is_jalr       = 1'b0;
        id_is_lr         = 1'b0;
        id_is_sc         = 1'b0;
        id_is_csr        = 1'b0;
        id_is_mret       = 1'b0;

        if (if_id_valid) begin
            case (id_opcode)
                OPCODE_R_TYPE: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_alu_src_a_sel = 1'b0;
                    id_alu_src_b_sel = 1'b0;
                    id_wb_sel        = WB_ALU;
                    case (id_funct3)
                        3'b000:  id_alu_ctrl = id_funct7[5] ? ALU_SUB : ALU_ADD;
                        3'b001:  id_alu_ctrl = ALU_SLL;
                        3'b010:  id_alu_ctrl = ALU_SLT;
                        3'b011:  id_alu_ctrl = ALU_SLTU;
                        3'b100:  id_alu_ctrl = ALU_XOR;
                        3'b101:  id_alu_ctrl = id_funct7[5] ? ALU_SRA : ALU_SRL;
                        3'b110:  id_alu_ctrl = ALU_OR;
                        3'b111:  id_alu_ctrl = ALU_AND;
                        default: id_alu_ctrl = ALU_ADD;
                    endcase
                end

                OPCODE_I_TYPE: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_alu_src_a_sel = 1'b0;
                    id_alu_src_b_sel = 1'b1;
                    id_wb_sel        = WB_ALU;
                    case (id_funct3)
                        3'b000:  id_alu_ctrl = ALU_ADD;
                        3'b001:  id_alu_ctrl = ALU_SLL;                          // SLLI
                        3'b010:  id_alu_ctrl = ALU_SLT;                          // SLTI
                        3'b011:  id_alu_ctrl = ALU_SLTU;                         // SLTIU
                        3'b100:  id_alu_ctrl = ALU_XOR;                          // XORI
                        3'b101:  id_alu_ctrl = id_funct7[5] ? ALU_SRA : ALU_SRL; // SRAI / SRLI
                        3'b110:  id_alu_ctrl = ALU_OR;                           // ORI
                        3'b111:  id_alu_ctrl = ALU_AND;                          // ANDI
                        default: id_alu_ctrl = ALU_ADD;
                    endcase
                end

                OPCODE_LOAD: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_mem_read      = 1'b1;
                    id_alu_src_a_sel = 1'b0;
                    id_alu_src_b_sel = 1'b1;
                    id_alu_ctrl      = ALU_ADD;
                    id_wb_sel        = WB_MEM;
                end

                OPCODE_STORE: begin
                    id_mem_write     = 1'b1;
                    id_alu_src_a_sel = 1'b0;
                    id_alu_src_b_sel = 1'b1;
                    id_alu_ctrl      = ALU_ADD;
                end

                OPCODE_BRANCH: begin
                    id_is_branch     = 1'b1;
                    id_alu_src_a_sel = 1'b0;
                    id_alu_src_b_sel = 1'b0;
                    id_alu_ctrl      = ALU_SUB;
                end

                OPCODE_JAL: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_is_jal        = 1'b1;
                    id_alu_src_a_sel = 1'b1; // pc
                    id_alu_src_b_sel = 1'b1; // imm
                    id_alu_ctrl      = ALU_ADD;
                    id_wb_sel        = WB_PC4;
                end

                OPCODE_JALR: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_is_jalr       = 1'b1;
                    id_alu_src_a_sel = 1'b0; // rs1
                    id_alu_src_b_sel = 1'b1; // imm
                    id_alu_ctrl      = ALU_ADD;
                    id_wb_sel        = WB_PC4;
                end

                OPCODE_LUI: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_alu_src_a_sel = 1'b0; // 0
                    id_alu_src_b_sel = 1'b1; // imm
                    id_alu_ctrl      = ALU_ADD;
                    id_wb_sel        = WB_ALU;
                end

                OPCODE_AUIPC: begin
                    id_reg_write     = (id_rd != 5'd0);
                    id_alu_src_a_sel = 1'b1; // pc
                    id_alu_src_b_sel = 1'b1; // imm
                    id_alu_ctrl      = ALU_ADD;
                    id_wb_sel        = WB_ALU;
                end

                OPCODE_AMO: begin
                    // RV32A Atomic Instructions (LR.W, SC.W)
                    if (id_funct3 == 3'b010) begin
                        case (id_funct7[6:2])
                            5'b00010: begin // LR.W
                                id_is_lr         = 1'b1;
                                id_reg_write     = (id_rd != 5'd0);
                                id_mem_read      = 1'b1;
                                id_alu_src_a_sel = 1'b0;
                                id_alu_src_b_sel = 1'b0; // offset is 0
                                id_alu_ctrl      = ALU_ADD;
                                id_wb_sel        = WB_MEM;
                            end
                            5'b00011: begin // SC.W
                                id_is_sc         = 1'b1;
                                id_reg_write     = (id_rd != 5'd0);
                                id_mem_write     = 1'b1;
                                id_alu_src_a_sel = 1'b0;
                                id_alu_src_b_sel = 1'b0;
                                id_alu_ctrl      = ALU_ADD;
                                id_wb_sel        = WB_SC; // writes sc_result (0/1) to rd
                            end
                            default: ;
                        endcase
                    end
                end

                OPCODE_SYSTEM: begin
                    // Privileged CSR Operations (CSRRW, CSRRS, CSRRC, etc.)
                    if (id_funct3 != 3'b000) begin
                        id_is_csr        = 1'b1;
                        id_reg_write     = (id_rd != 5'd0);
                        id_alu_src_a_sel = 1'b0;
                        id_alu_src_b_sel = 1'b0;
                        id_alu_ctrl      = ALU_ADD;
                        id_wb_sel        = WB_CSR; // writes previous CSR value to rd
                    end else if (id_is_mret_op) begin
                        // MRET (Machine-Mode Return from Trap)
                        id_is_mret       = 1'b1;
                        id_reg_write     = 1'b0;
                    end
                end

                default: begin
                    id_reg_write     = 1'b0;
                    id_alu_ctrl      = ALU_ADD;
                    id_wb_sel        = WB_ALU;
                end
            endcase
        end
    end

    // RV32M Extension Detection
    wire id_is_m_ext = if_id_valid && (id_opcode == OPCODE_R_TYPE) && (id_funct7 == 7'b0000001);
    wire id_is_mul   = id_is_m_ext && (!id_funct3[2]); // MUL, MULH, MULHSU, MULHU
    wire id_is_div   = id_is_m_ext && (id_funct3[2]);  // DIV, DIVU, REM, REMU

    // Load-Use Hazard Detection Unit
    reg [4:0] id_ex_rd;
    reg       id_ex_mem_read;
    reg       id_ex_valid;

    assign load_use_hazard = id_ex_valid && id_ex_mem_read && (id_ex_rd != 5'd0) &&
                             ((id_ex_rd == id_rs1) || (id_ex_rd == id_rs2));

    wire flush_id_ex = branch_mispredict_ex || load_use_hazard || take_trap || ex_is_mret;

    // ID/EX Pipeline Register
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_imm;
    reg [4:0]  id_ex_rs1;
    reg [4:0]  id_ex_rs2;
    reg [2:0]  id_ex_funct3;
    reg [6:0]  id_ex_opcode;
    reg        id_ex_reg_write;
    reg        id_ex_mem_write;
    reg        id_ex_alu_src_a_sel;
    reg        id_ex_alu_src_b_sel;
    reg [3:0]  id_ex_alu_ctrl;
    reg [2:0]  id_ex_wb_sel;
    reg        id_ex_is_branch;
    reg        id_ex_is_jal;
    reg        id_ex_is_jalr;
    reg        id_ex_is_mul;
    reg        id_ex_is_div;
    reg        id_ex_is_lr;
    reg        id_ex_is_sc;
    reg        id_ex_is_csr;
    reg        id_ex_is_mret;
    reg        id_ex_is_ecall;
    reg        id_ex_is_ebreak;
    reg        id_ex_is_illegal;
    reg        id_ex_pred_taken;
    reg [31:0] id_ex_pred_target;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_pc            <= 32'h0000_0000;
            id_ex_rs1_data      <= 32'h0000_0000;
            id_ex_rs2_data      <= 32'h0000_0000;
            id_ex_imm           <= 32'h0000_0000;
            id_ex_rs1           <= 5'd0;
            id_ex_rs2           <= 5'd0;
            id_ex_rd            <= 5'd0;
            id_ex_funct3        <= 3'd0;
            id_ex_opcode        <= 7'd0;
            id_ex_reg_write     <= 1'b0;
            id_ex_mem_read      <= 1'b0;
            id_ex_mem_write     <= 1'b0;
            id_ex_alu_src_a_sel <= 1'b0;
            id_ex_alu_src_b_sel <= 1'b0;
            id_ex_alu_ctrl      <= ALU_ADD;
            id_ex_wb_sel        <= WB_ALU;
            id_ex_is_branch     <= 1'b0;
            id_ex_is_jal        <= 1'b0;
            id_ex_is_jalr       <= 1'b0;
            id_ex_is_mul        <= 1'b0;
            id_ex_is_div        <= 1'b0;
            id_ex_is_lr         <= 1'b0;
            id_ex_is_sc         <= 1'b0;
            id_ex_is_csr        <= 1'b0;
            id_ex_is_mret       <= 1'b0;
            id_ex_is_ecall      <= 1'b0;
            id_ex_is_ebreak     <= 1'b0;
            id_ex_is_illegal    <= 1'b0;
            id_ex_pred_taken    <= 1'b0;
            id_ex_pred_target   <= 32'h0000_0000;
            id_ex_valid         <= 1'b0;
        end else if (dcache_stall) begin
            // Hold ID/EX state frozen during D-Cache miss
        end else if (flush_id_ex) begin
            id_ex_pc            <= 32'h0000_0000;
            id_ex_rs1_data      <= 32'h0000_0000;
            id_ex_rs2_data      <= 32'h0000_0000;
            id_ex_imm           <= 32'h0000_0000;
            id_ex_rs1           <= 5'd0;
            id_ex_rs2           <= 5'd0;
            id_ex_rd            <= 5'd0;
            id_ex_funct3        <= 3'd0;
            id_ex_opcode        <= 7'd0;
            id_ex_reg_write     <= 1'b0;
            id_ex_mem_read      <= 1'b0;
            id_ex_mem_write     <= 1'b0;
            id_ex_alu_src_a_sel <= 1'b0;
            id_ex_alu_src_b_sel <= 1'b0;
            id_ex_alu_ctrl      <= ALU_ADD;
            id_ex_wb_sel        <= WB_ALU;
            id_ex_is_branch     <= 1'b0;
            id_ex_is_jal        <= 1'b0;
            id_ex_is_jalr       <= 1'b0;
            id_ex_is_mul        <= 1'b0;
            id_ex_is_div        <= 1'b0;
            id_ex_is_lr         <= 1'b0;
            id_ex_is_sc         <= 1'b0;
            id_ex_is_csr        <= 1'b0;
            id_ex_is_mret       <= 1'b0;
            id_ex_is_ecall      <= 1'b0;
            id_ex_is_ebreak     <= 1'b0;
            id_ex_is_illegal    <= 1'b0;
            id_ex_pred_taken    <= 1'b0;
            id_ex_pred_target   <= 32'h0000_0000;
            id_ex_valid         <= 1'b0;
        end else if (!stall_m_ext) begin
            id_ex_pc            <= if_id_pc;
            id_ex_rs1_data      <= id_rs1_data;
            id_ex_rs2_data      <= id_rs2_data;
            id_ex_imm           <= id_imm;
            id_ex_rs1           <= id_rs1;
            id_ex_rs2           <= id_rs2;
            id_ex_rd            <= id_rd;
            id_ex_funct3        <= id_funct3;
            id_ex_opcode        <= id_opcode;
            id_ex_reg_write     <= id_reg_write;
            id_ex_mem_read      <= id_mem_read;
            id_ex_mem_write     <= id_mem_write;
            id_ex_alu_src_a_sel <= id_alu_src_a_sel;
            id_ex_alu_src_b_sel <= id_alu_src_b_sel;
            id_ex_alu_ctrl      <= id_alu_ctrl;
            id_ex_wb_sel        <= id_wb_sel;
            id_ex_is_branch     <= id_is_branch;
            id_ex_is_jal        <= id_is_jal;
            id_ex_is_jalr       <= id_is_jalr;
            id_ex_is_mul        <= id_is_mul;
            id_ex_is_div        <= id_is_div;
            id_ex_is_lr         <= id_is_lr;
            id_ex_is_sc         <= id_is_sc;
            id_ex_is_csr        <= id_is_csr;
            id_ex_is_mret       <= id_is_mret;
            id_ex_is_ecall      <= id_is_ecall;
            id_ex_is_ebreak     <= id_is_ebreak;
            id_ex_is_illegal    <= id_is_illegal;
            id_ex_pred_taken    <= if_id_pred_taken;
            id_ex_pred_target   <= if_id_pred_target;
            id_ex_valid         <= if_id_valid;
        end
    end

    //==========================================================================
    // 3. EXECUTE (EX) STAGE & FORWARDING UNIT
    //==========================================================================
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_reg_write;
    reg        ex_mem_valid;
    reg [31:0] ex_mem_alu_result;

    // Forwarding Unit Multiplexer Selects
    reg [1:0] forward_a;
    reg [1:0] forward_b;

    always @(*) begin
        if (ex_mem_valid && ex_mem_reg_write && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs1)) begin
            forward_a = 2'b10; // Forward from EX/MEM stage
        end else if (mem_wb_valid && mem_wb_reg_write && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs1)) begin
            forward_a = 2'b01; // Forward from MEM/WB stage
        end else begin
            forward_a = 2'b00; // No forwarding
        end

        if (ex_mem_valid && ex_mem_reg_write && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs2)) begin
            forward_b = 2'b10; // Forward from EX/MEM stage
        end else if (mem_wb_valid && mem_wb_reg_write && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs2)) begin
            forward_b = 2'b01; // Forward from MEM/WB stage
        end else begin
            forward_b = 2'b00; // No forwarding
        end
    end

    // Resolve Forwarded Register Values
    wire [31:0] fwd_val_a = (forward_a == 2'b10) ? ex_mem_alu_result :
                            (forward_a == 2'b01) ? wb_write_data :
                            id_ex_rs1_data;

    wire [31:0] fwd_val_b = (forward_b == 2'b10) ? ex_mem_alu_result :
                            (forward_b == 2'b01) ? wb_write_data :
                            id_ex_rs2_data;

    // ALU Input Operand Selection
    wire [31:0] ex_alu_in_a = (id_ex_opcode == OPCODE_LUI) ? 32'h0000_0000 :
                              id_ex_alu_src_a_sel          ? id_ex_pc     : fwd_val_a;
    wire [31:0] ex_alu_in_b = (id_ex_is_lr || id_ex_is_sc) ? 32'h0000_0000 :
                              id_ex_alu_src_b_sel          ? id_ex_imm     : fwd_val_b;

    // ALU Instantiation
    wire [31:0] ex_alu_result;
    wire        ex_alu_zero;
    wire        ex_alu_overflow;

    alu u_alu (
        .a        (ex_alu_in_a),
        .b        (ex_alu_in_b),
        .alu_ctrl (id_ex_alu_ctrl),
        .result   (ex_alu_result),
        .zero     (ex_alu_zero),
        .overflow (ex_alu_overflow)
    );

    // Branch Condition Evaluation
    reg ex_branch_cond;
    always @(*) begin
        case (id_ex_funct3)
            3'b000:  ex_branch_cond = (fwd_val_a == fwd_val_b);                  // BEQ
            3'b001:  ex_branch_cond = (fwd_val_a != fwd_val_b);                  // BNE
            3'b100:  ex_branch_cond = ($signed(fwd_val_a) < $signed(fwd_val_b)); // BLT
            3'b101:  ex_branch_cond = ($signed(fwd_val_a) >= $signed(fwd_val_b));// BGE
            3'b110:  ex_branch_cond = (fwd_val_a < fwd_val_b);                  // BLTU
            3'b111:  ex_branch_cond = (fwd_val_a >= fwd_val_b);                 // BGEU
            default: ex_branch_cond = 1'b0;
        endcase
    end

    assign branch_taken_ex  = id_ex_valid && ((id_ex_is_branch && ex_branch_cond) || id_ex_is_jal || id_ex_is_jalr);
    assign branch_target_ex = id_ex_is_jalr ? ((fwd_val_a + id_ex_imm) & ~32'd1) :
                              (id_ex_pc + id_ex_imm);

    // Dynamic Branch Predictor Misprediction Evaluation
    wire ex_is_ctrl = id_ex_valid && (id_ex_is_branch || id_ex_is_jal || id_ex_is_jalr);
    assign ex_is_jump = id_ex_valid && (id_ex_is_jal || id_ex_is_jalr);

    // Branch Misprediction Occurred IF:
    //   1. Predicted Taken != Actual Taken
    //   2. Both Taken, but Predicted Target != Actual Target
    assign branch_mispredict_ex = ex_is_ctrl && (
        (id_ex_pred_taken != branch_taken_ex) ||
        (branch_taken_ex && (id_ex_pred_target != branch_target_ex))
    );

    assign branch_redirect_pc = branch_taken_ex ? branch_target_ex : (id_ex_pc + 32'd4);
    assign update_btb = ex_is_ctrl && !dcache_stall && !stall_m_ext;

    // RV32M Multiplier and Divider Subsystems
    reg mul_running;
    reg div_running;

    wire mul_ready;
    wire mul_busy;
    wire [31:0] mul_result;

    wire div_ready;
    wire div_busy;
    wire [31:0] div_result;

    wire mul_start = id_ex_valid && id_ex_is_mul && !mul_running && !mul_ready;
    wire div_start = id_ex_valid && id_ex_is_div && !div_running && !div_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_running <= 1'b0;
            div_running <= 1'b0;
        end else begin
            if (mul_start) mul_running <= 1'b1;
            else if (mul_ready) mul_running <= 1'b0;

            if (div_start) div_running <= 1'b1;
            else if (div_ready) div_running <= 1'b0;
        end
    end

    assign stall_m_ext = id_ex_valid && ((id_ex_is_mul && !mul_ready) || (id_ex_is_div && !div_ready));

    multiplier u_multiplier (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (mul_start),
        .funct3 (id_ex_funct3),
        .a      (fwd_val_a),
        .b      (fwd_val_b),
        .result (mul_result),
        .ready  (mul_ready),
        .busy   (mul_busy)
    );

    divider u_divider (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (div_start),
        .funct3 (id_ex_funct3),
        .a      (fwd_val_a),
        .b      (fwd_val_b),
        .result (div_result),
        .ready  (div_ready),
        .busy   (div_busy)
    );

    // Privileged HPM CSR Block
    wire [31:0] ex_csr_rdata;
    wire [31:0] csr_operand = id_ex_funct3[2] ? {27'b0, id_ex_rs1} : fwd_val_a;
    wire        csr_op_en   = id_ex_valid && id_ex_is_csr && !dcache_stall && !stall_m_ext && !branch_mispredict_ex && !take_trap && !ex_is_mret;

    assign ex_is_mret = id_ex_valid && id_ex_is_mret && !dcache_stall && !stall_m_ext;

    wire        irq_pending;
    wire [31:0] mstatus_diag;
    wire [31:0] mcause_diag;

    // Trap & Exception Handling
    wire ex_has_exception = id_ex_valid && (id_ex_is_ecall || id_ex_is_ebreak || id_ex_is_illegal);
    assign take_trap = (irq_pending || ex_has_exception) && !dcache_stall && !stall_m_ext;

    reg [31:0] trap_cause;
    always @(*) begin
        if (irq_pending) begin
            trap_cause = 32'h8000_0007; // Machine Timer Interrupt
        end else if (id_ex_is_illegal) begin
            trap_cause = 32'h0000_0002; // Illegal Instruction
        end else if (id_ex_is_ebreak) begin
            trap_cause = 32'h0000_0003; // Breakpoint
        end else if (id_ex_is_ecall) begin
            trap_cause = 32'h0000_000B; // Environment Call from M-Mode
        end else begin
            trap_cause = 32'h8000_0007;
        end
    end

    // Trap PC: save PC of the faulting/interrupted instruction
    wire [31:0] trap_saved_pc = ex_has_exception ? id_ex_pc :
                                (id_ex_valid && (id_ex_pc != 32'd0)) ? id_ex_pc :
                                (if_id_valid && (if_id_pc != 32'd0)) ? if_id_pc : if_pc;

    hpm_csrs u_hpm_csrs (
        .clk              (clk),
        .rst_n            (rst_n),
        .csr_en           (csr_op_en),
        .csr_funct3       (id_ex_funct3),
        .csr_addr         (id_ex_imm[11:0]),
        .csr_wdata        (csr_operand),
        .csr_rdata        (ex_csr_rdata),
        .event_cycle      (rst_n),                                  // Architectural mcycle counts all active clocks!
        .event_instret    (mem_wb_valid && !dcache_stall),           // minstret counts ONLY true retired instructions!
        .event_branch     (ex_is_ctrl && !dcache_stall && !stall_m_ext),
        .event_mispredict (branch_mispredict_ex && !dcache_stall && !stall_m_ext),
        .timer_irq_in     (timer_irq_in),
        .irq_pending      (irq_pending),
        .trap_taken       (take_trap),
        .trap_pc          (trap_saved_pc),
        .trap_cause       (trap_cause),
        .mret_exec        (ex_is_mret),
        .mtvec_out        (mtvec_target),
        .mepc_out         (mepc_target),
        .mstatus_out      (mstatus_diag),
        .mcause_out       (mcause_diag)
    );

    assign trace_csr_mcycle   = u_hpm_csrs.mcycle;
    assign trace_csr_minstret = u_hpm_csrs.minstret;

    wire [31:0] ex_stage_result = id_ex_is_mul ? mul_result :
                                  id_ex_is_div ? div_result :
                                  ex_alu_result;

    // EX/MEM Pipeline Register
    reg [31:0] ex_mem_pc;
    reg [31:0] ex_mem_store_data;
    reg [2:0]  ex_mem_funct3;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg [2:0]  ex_mem_wb_sel;
    reg        ex_mem_is_lr;
    reg        ex_mem_is_sc;
    reg [31:0] ex_mem_csr_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_mem_pc         <= 32'h0000_0000;
            ex_mem_alu_result <= 32'h0000_0000;
            ex_mem_store_data <= 32'h0000_0000;
            ex_mem_funct3     <= 3'd0;
            ex_mem_rd         <= 5'd0;
            ex_mem_reg_write  <= 1'b0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_wb_sel     <= WB_ALU;
            ex_mem_is_lr      <= 1'b0;
            ex_mem_is_sc      <= 1'b0;
            ex_mem_csr_rdata  <= 32'h0000_0000;
            ex_mem_valid      <= 1'b0;
        end else if (take_trap) begin
            // Trap flush: prevent interrupted/faulting instruction from committing
            ex_mem_pc         <= 32'h0000_0000;
            ex_mem_alu_result <= 32'h0000_0000;
            ex_mem_store_data <= 32'h0000_0000;
            ex_mem_funct3     <= 3'd0;
            ex_mem_rd         <= 5'd0;
            ex_mem_reg_write  <= 1'b0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_wb_sel     <= WB_ALU;
            ex_mem_is_lr      <= 1'b0;
            ex_mem_is_sc      <= 1'b0;
            ex_mem_csr_rdata  <= 32'h0000_0000;
            ex_mem_valid      <= 1'b0;
        end else if (dcache_stall) begin
            // Hold EX/MEM state frozen during D-Cache miss
        end else if (stall_m_ext) begin
            // Insert bubble into EX/MEM while M-extension is computing
            ex_mem_reg_write  <= 1'b0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_is_lr      <= 1'b0;
            ex_mem_is_sc      <= 1'b0;
            ex_mem_valid      <= 1'b0;
        end else begin
            ex_mem_pc         <= id_ex_pc;
            ex_mem_alu_result <= ex_stage_result;
            ex_mem_store_data <= fwd_val_b;
            ex_mem_funct3     <= id_ex_funct3;
            ex_mem_rd         <= id_ex_rd;
            ex_mem_reg_write  <= id_ex_reg_write;
            ex_mem_mem_read   <= id_ex_mem_read;
            ex_mem_mem_write  <= id_ex_mem_write;
            ex_mem_wb_sel     <= id_ex_wb_sel;
            ex_mem_is_lr      <= id_ex_is_lr;
            ex_mem_is_sc      <= id_ex_is_sc;
            ex_mem_csr_rdata  <= ex_csr_rdata;
            ex_mem_valid      <= id_ex_valid;
        end
    end

    //==========================================================================
    // 4. MEMORY ACCESS (MEM) STAGE & RV32A ATOMIC CONTROLLER
    //==========================================================================
    wire        sc_success;
    wire [31:0] sc_result;
    wire        effective_mem_we;
    wire        effective_mem_re;

    atomics u_atomics (
        .clk              (clk),
        .rst_n            (rst_n),
        .is_lr            (ex_mem_valid && ex_mem_is_lr),
        .is_sc            (ex_mem_valid && ex_mem_is_sc),
        .is_store         (ex_mem_valid && ex_mem_mem_write && !ex_mem_is_sc),
        .mem_addr         (ex_mem_alu_result),
        .mem_wdata        (ex_mem_store_data),
        .effective_mem_we (effective_mem_we),
        .effective_mem_re (effective_mem_re),
        .sc_success       (sc_success),
        .sc_result        (sc_result),
        .snoop_we         (snoop_we),
        .snoop_addr       (snoop_addr),
        .res_valid_out    (),
        .res_addr_out     (),
        .lr_count         (),
        .sc_success_count (),
        .sc_fail_count    ()
    );

    // Byte write mask generation for SB, SH, SW
    wire [1:0] mem_byte_offset = ex_mem_alu_result[1:0];
    reg [3:0]  mem_wstrb;
    reg [31:0] mem_aligned_wdata;

    always @(*) begin
        case (ex_mem_funct3)
            3'b000: begin // SB (Store Byte)
                case (mem_byte_offset)
                    2'b00: begin mem_wstrb = 4'b0001; mem_aligned_wdata = {24'b0, ex_mem_store_data[7:0]}; end
                    2'b01: begin mem_wstrb = 4'b0010; mem_aligned_wdata = {16'b0, ex_mem_store_data[7:0], 8'b0}; end
                    2'b10: begin mem_wstrb = 4'b0100; mem_aligned_wdata = {8'b0, ex_mem_store_data[7:0], 16'b0}; end
                    2'b11: begin mem_wstrb = 4'b1000; mem_aligned_wdata = {ex_mem_store_data[7:0], 24'b0}; end
                endcase
            end
            3'b001: begin // SH (Store Halfword)
                case (mem_byte_offset[1])
                    1'b0: begin mem_wstrb = 4'b0011; mem_aligned_wdata = {16'b0, ex_mem_store_data[15:0]}; end
                    1'b1: begin mem_wstrb = 4'b1100; mem_aligned_wdata = {ex_mem_store_data[15:0], 16'b0}; end
                endcase
            end
            default: begin // SW (Store Word), SC.W
                mem_wstrb         = 4'b1111;
                mem_aligned_wdata = ex_mem_store_data;
            end
        endcase
    end

    assign dmem_addr  = ex_mem_alu_result;
    assign dmem_wdata = mem_aligned_wdata;
    assign dmem_wstrb = mem_wstrb;
    assign dmem_we    = ex_mem_valid && effective_mem_we;
    assign dmem_re    = ex_mem_valid && (ex_mem_mem_read || effective_mem_re);

    assign trace_sc_success = sc_success;

    // MEM/WB Pipeline Register
    reg [31:0] mem_wb_pc;
    reg [31:0] mem_wb_alu_result;
    reg [31:0] mem_wb_mem_rdata;
    reg [2:0]  mem_wb_funct3;
    reg [2:0]  mem_wb_wb_sel;
    reg [31:0] mem_wb_sc_result;
    reg [31:0] mem_wb_csr_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wb_pc         <= 32'h0000_0000;
            mem_wb_alu_result <= 32'h0000_0000;
            mem_wb_mem_rdata  <= 32'h0000_0000;
            mem_wb_funct3     <= 3'd0;
            mem_wb_rd         <= 5'd0;
            mem_wb_reg_write  <= 1'b0;
            mem_wb_wb_sel     <= WB_ALU;
            mem_wb_sc_result  <= 32'h0000_0000;
            mem_wb_csr_rdata  <= 32'h0000_0000;
            mem_wb_valid      <= 1'b0;
        end else if (dcache_stall) begin
            // Bubble MEM/WB stage during D-Cache miss
            mem_wb_reg_write  <= 1'b0;
            mem_wb_valid      <= 1'b0;
        end else begin
            mem_wb_pc         <= ex_mem_pc;
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_mem_rdata  <= dmem_rdata;
            mem_wb_funct3     <= ex_mem_funct3;
            mem_wb_rd         <= ex_mem_rd;
            mem_wb_reg_write  <= ex_mem_reg_write;
            mem_wb_wb_sel     <= ex_mem_wb_sel;
            mem_wb_sc_result  <= sc_result;
            mem_wb_csr_rdata  <= ex_mem_csr_rdata;
            mem_wb_valid      <= ex_mem_valid;
        end
    end

    //==========================================================================
    // 5. WRITE-BACK (WB) STAGE
    //==========================================================================
    // Sub-word formatted load data for LB, LH, LW, LBU, LHU
    wire [1:0] wb_byte_offset = mem_wb_alu_result[1:0];
    reg [31:0] formatted_load_data;

    always @(*) begin
        case (mem_wb_funct3)
            3'b000: begin // LB (Signed Byte)
                case (wb_byte_offset)
                    2'b00: formatted_load_data = {{24{mem_wb_mem_rdata[7]}},  mem_wb_mem_rdata[7:0]};
                    2'b01: formatted_load_data = {{24{mem_wb_mem_rdata[15]}}, mem_wb_mem_rdata[15:8]};
                    2'b10: formatted_load_data = {{24{mem_wb_mem_rdata[23]}}, mem_wb_mem_rdata[23:16]};
                    2'b11: formatted_load_data = {{24{mem_wb_mem_rdata[31]}}, mem_wb_mem_rdata[31:24]};
                endcase
            end
            3'b001: begin // LH (Signed Halfword)
                case (wb_byte_offset[1])
                    1'b0: formatted_load_data = {{16{mem_wb_mem_rdata[15]}}, mem_wb_mem_rdata[15:0]};
                    1'b1: formatted_load_data = {{16{mem_wb_mem_rdata[31]}}, mem_wb_mem_rdata[31:16]};
                endcase
            end
            3'b010: begin // LW (Word)
                formatted_load_data = mem_wb_mem_rdata;
            end
            3'b100: begin // LBU (Unsigned Byte)
                case (wb_byte_offset)
                    2'b00: formatted_load_data = {24'b0, mem_wb_mem_rdata[7:0]};
                    2'b01: formatted_load_data = {24'b0, mem_wb_mem_rdata[15:8]};
                    2'b10: formatted_load_data = {24'b0, mem_wb_mem_rdata[23:16]};
                    2'b11: formatted_load_data = {24'b0, mem_wb_mem_rdata[31:24]};
                endcase
            end
            3'b101: begin // LHU (Unsigned Halfword)
                case (wb_byte_offset[1])
                    1'b0: formatted_load_data = {16'b0, mem_wb_mem_rdata[15:0]};
                    1'b1: formatted_load_data = {16'b0, mem_wb_mem_rdata[31:16]};
                endcase
            end
            default: begin
                formatted_load_data = mem_wb_mem_rdata;
            end
        endcase
    end

    assign wb_write_data = (mem_wb_wb_sel == WB_MEM) ? formatted_load_data :
                           (mem_wb_wb_sel == WB_PC4) ? (mem_wb_pc + 32'd4) :
                           (mem_wb_wb_sel == WB_CSR) ? mem_wb_csr_rdata :
                           (mem_wb_wb_sel == WB_SC)  ? mem_wb_sc_result :
                           mem_wb_alu_result;

    // Diagnostic Tracing Outputs
    assign debug_if_pc        = if_pc;
    assign debug_id_pc        = if_id_valid ? if_id_pc : 32'd0;
    assign debug_ex_pc        = id_ex_valid ? id_ex_pc : 32'd0;
    assign debug_mem_pc       = ex_mem_valid ? ex_mem_pc : 32'd0;
    assign debug_wb_pc        = mem_wb_valid ? mem_wb_pc : 32'd0;
    assign debug_wb_data      = wb_write_data;
    assign debug_wb_rd        = mem_wb_rd;
    assign debug_wb_reg_write = regfile_we;
    assign debug_stall        = stall_pipeline;
    assign debug_flush        = branch_mispredict_ex || take_trap || ex_is_mret;
    assign trace_pred_taken   = pred_taken;
    assign trace_mispredict   = branch_mispredict_ex;
    assign trace_trap_taken   = take_trap;
    assign trace_mret_exec    = ex_is_mret;
    assign trace_mtvec        = mtvec_target;
    assign trace_mepc         = mepc_target;
    assign trace_mcause       = mcause_diag;
    assign trace_mstatus      = mstatus_diag;

endmodule
