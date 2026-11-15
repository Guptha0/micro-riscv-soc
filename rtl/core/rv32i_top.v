//==============================================================================
// Module Name: rv32i_top
// Description: Production-grade Single-Cycle 32-bit RISC-V (RV32I) Processor Core
//
// Submodules Integrated:
//   1. pc_fetch      : Program Counter & Embedded Instruction Memory (ROM)
//   2. regfile       : 32x32 General Purpose Register File (with x0 hardwired)
//   3. alu           : 32-bit Arithmetic Logic Unit (ADD, SUB, AND, OR, SLT)
//   4. control_unit  : Instruction decoder and datapath control generator
//   5. imm_gen       : Immediate value extractor (I, S, B, U, J types)
//==============================================================================

`timescale 1ns / 1ps

//------------------------------------------------------------------------------
// Immediate Generator Submodule
//------------------------------------------------------------------------------
module imm_gen (
    input  wire [31:0] instruction,
    output reg  [31:0] imm_i,
    output reg  [31:0] imm_s,
    output reg  [31:0] imm_b,
    output reg  [31:0] imm_u,
    output reg  [31:0] imm_j
);
    always @(*) begin
        // I-type: 12-bit signed immediate [31:20]
        imm_i = {{20{instruction[31]}}, instruction[31:20]};

        // S-type: 12-bit signed store immediate [31:25], [11:7]
        imm_s = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};

        // B-type: 13-bit signed branch offset [31], [7], [30:25], [11:8], 1'b0
        imm_b = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};

        // U-type: 20-bit upper immediate [31:12] << 12
        imm_u = {instruction[31:12], 12'h000};

        // J-type: 21-bit signed jump target [31], [19:12], [20], [30:21], 1'b0
        imm_j = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
    end
endmodule

//------------------------------------------------------------------------------
// Control Unit & Decoder Submodule
//------------------------------------------------------------------------------
module control_unit (
    input  wire [6:0]  opcode,
    input  wire [2:0]  funct3,
    input  wire [6:0]  funct7,
    input  wire [31:0] rs1_data,
    input  wire [31:0] rs2_data,
    output reg         reg_write,
    output reg         alu_src_a_sel, // 0: rs1_data, 1: pc
    output reg         alu_src_b_sel, // 0: rs2_data, 1: imm
    output reg  [3:0]  alu_ctrl,
    output reg         branch_taken,
    output reg         is_jalr
);

    // Opcodes per RISC-V RV32I specification
    localparam [6:0] OPCODE_R_TYPE = 7'b0110011; // 7'h33: add, sub, and, or, slt
    localparam [6:0] OPCODE_I_TYPE = 7'b0010011; // 7'h13: addi, andi, ori, slti
    localparam [6:0] OPCODE_LOAD   = 7'b0000011; // 7'h03: lw
    localparam [6:0] OPCODE_STORE  = 7'b0100011; // 7'h23: sw
    localparam [6:0] OPCODE_BRANCH = 7'b1100011; // 7'h63: beq, bne, blt, bge
    localparam [6:0] OPCODE_JAL    = 7'b1101111; // 7'h6F: jal
    localparam [6:0] OPCODE_JALR   = 7'b1100111; // 7'h67: jalr
    localparam [6:0] OPCODE_LUI    = 7'b0110111; // 7'h37: lui
    localparam [6:0] OPCODE_AUIPC  = 7'b0010111; // 7'h17: auipc

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

    // Branch condition evaluation
    reg branch_cond_met;
    always @(*) begin
        case (funct3)
            3'b000:  branch_cond_met = (rs1_data == rs2_data);                  // BEQ
            3'b001:  branch_cond_met = (rs1_data != rs2_data);                  // BNE
            3'b100:  branch_cond_met = ($signed(rs1_data) < $signed(rs2_data)); // BLT
            3'b101:  branch_cond_met = ($signed(rs1_data) >= $signed(rs2_data));// BGE
            3'b110:  branch_cond_met = (rs1_data < rs2_data);                  // BLTU
            3'b111:  branch_cond_met = (rs1_data >= rs2_data);                 // BGEU
            default: branch_cond_met = 1'b0;
        endcase
    end

    // Main Control Decoder
    always @(*) begin
        // Defaults
        reg_write     = 1'b0;
        alu_src_a_sel = 1'b0; // rs1_data
        alu_src_b_sel = 1'b0; // rs2_data
        alu_ctrl      = ALU_ADD;
        branch_taken  = 1'b0;
        is_jalr       = 1'b0;

        case (opcode)
            OPCODE_R_TYPE: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 1'b0; // rs1_data
                alu_src_b_sel = 1'b0; // rs2_data
                case (funct3)
                    3'b000:  alu_ctrl = funct7[5] ? ALU_SUB : ALU_ADD;
                    3'b001:  alu_ctrl = ALU_SLL;
                    3'b010:  alu_ctrl = ALU_SLT;
                    3'b011:  alu_ctrl = ALU_SLTU;
                    3'b100:  alu_ctrl = ALU_XOR;
                    3'b101:  alu_ctrl = funct7[5] ? ALU_SRA : ALU_SRL;
                    3'b110:  alu_ctrl = ALU_OR;
                    3'b111:  alu_ctrl = ALU_AND;
                    default: alu_ctrl = ALU_ADD;
                endcase
            end

            OPCODE_I_TYPE: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 1'b0; // rs1_data
                alu_src_b_sel = 1'b1; // imm
                case (funct3)
                    3'b000:  alu_ctrl = ALU_ADD; // ADDI
                    3'b001:  alu_ctrl = ALU_SLL; // SLLI
                    3'b010:  alu_ctrl = ALU_SLT; // SLTI
                    3'b011:  alu_ctrl = ALU_SLTU;// SLTIU
                    3'b100:  alu_ctrl = ALU_XOR; // XORI
                    3'b101:  alu_ctrl = funct7[5] ? ALU_SRA : ALU_SRL; // SRAI / SRLI
                    3'b110:  alu_ctrl = ALU_OR;  // ORI
                    3'b111:  alu_ctrl = ALU_AND; // ANDI
                    default: alu_ctrl = ALU_ADD;
                endcase
            end

            OPCODE_BRANCH: begin
                reg_write     = 1'b0;
                alu_src_a_sel = 1'b0;
                alu_src_b_sel = 1'b0;
                alu_ctrl      = ALU_SUB;
                branch_taken  = branch_cond_met;
            end

            OPCODE_JAL: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 1'b1; // pc
                alu_src_b_sel = 1'b1; // imm
                alu_ctrl      = ALU_ADD;
                branch_taken  = 1'b1;
            end

            OPCODE_JALR: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 1'b0; // rs1_data
                alu_src_b_sel = 1'b1; // imm
                alu_ctrl      = ALU_ADD;
                branch_taken  = 1'b1;
                is_jalr       = 1'b1;
            end

            OPCODE_LUI: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 1'b0; // 0
                alu_src_b_sel = 1'b1; // imm_u
                alu_ctrl      = ALU_ADD;
            end

            OPCODE_AUIPC: begin
                reg_write     = 1'b1;
                alu_src_a_sel = 1'b1; // pc
                alu_src_b_sel = 1'b1; // imm_u
                alu_ctrl      = ALU_ADD;
            end

            default: begin
                reg_write     = 1'b0;
                alu_src_a_sel = 1'b0;
                alu_src_b_sel = 1'b0;
                alu_ctrl      = ALU_ADD;
                branch_taken  = 1'b0;
                is_jalr       = 1'b0;
            end
        endcase
    end

endmodule

//------------------------------------------------------------------------------
// Top-Level Processor Integration
//------------------------------------------------------------------------------
module rv32i_top #(
    parameter MEM_DEPTH = 64,
    parameter HEX_FILE  = ""
)(
    input  wire        clk,
    input  wire        rst_n,

    // Real-Time System Monitoring & Debug Ports
    output wire [31:0] pc_out,
    output wire [31:0] instruction_out,
    output wire [31:0] alu_result_out,
    output wire [31:0] rs1_data_out,
    output wire [31:0] rs2_data_out,
    output wire [31:0] rd_data_out,
    output wire [31:0] imm_out,
    output wire [3:0]  alu_ctrl_out,
    output wire        reg_write_out,
    output wire        branch_taken_out
);

    // Internal Datapath Wires
    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] instruction;
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    wire [31:0] rd_data;
    wire [31:0] alu_in_a;
    wire [31:0] alu_in_b;
    wire [31:0] alu_result;
    wire        alu_zero;
    wire        alu_overflow;

    // Instruction Field Decomposition
    wire [6:0]  opcode = instruction[6:0];
    wire [4:0]  rd     = instruction[11:7];
    wire [2:0]  funct3 = instruction[14:12];
    wire [4:0]  rs1    = instruction[19:15];
    wire [4:0]  rs2    = instruction[24:20];
    wire [6:0]  funct7 = instruction[31:25];

    // Immediate Wires
    wire [31:0] imm_i;
    wire [31:0] imm_s;
    wire [31:0] imm_b;
    wire [31:0] imm_u;
    wire [31:0] imm_j;
    reg  [31:0] imm;

    // Control Signals
    wire        reg_write;
    wire        alu_src_a_sel;
    wire        alu_src_b_sel;
    wire [3:0]  alu_ctrl;
    wire        branch_taken;
    wire        is_jalr;

    // Target calculation for branches and jumps
    wire [31:0] branch_target = pc + imm_b;
    wire [31:0] jal_target    = pc + imm_j;
    wire [31:0] jalr_target   = (rs1_data + imm_i) & ~32'd1;

    assign next_pc = is_jalr                  ? jalr_target   :
                     (opcode == 7'b1101111)   ? jal_target    :
                     branch_target;

    // Register Writeback Multiplexer: JAL/JALR write return address (pc + 4)
    assign rd_data = (opcode == 7'b1101111 || opcode == 7'b1100111) ? (pc + 32'd4) : alu_result;

    // ALU Operand Multiplexers
    assign alu_in_a = (opcode == 7'b0110111) ? 32'h0000_0000 : // LUI
                      alu_src_a_sel          ? pc            : rs1_data;
    assign alu_in_b = alu_src_b_sel          ? imm           : rs2_data;

    // Immediate Selection based on opcode
    always @(*) begin
        case (opcode)
            7'b0010011, 7'b0000011, 7'b1100111: imm = imm_i; // I-type, Load, JALR
            7'b0100011:                         imm = imm_s; // S-type (store)
            7'b1100011:                         imm = imm_b; // B-type (branch)
            7'b0110111, 7'b0010111:             imm = imm_u; // LUI, AUIPC
            7'b1101111:                         imm = imm_j; // JAL
            default:                            imm = imm_i;
        endcase
    end

    //--------------------------------------------------------------------------
    // 1. Program Counter & Instruction Memory (ROM)
    //--------------------------------------------------------------------------
    pc_fetch #(
        .MEM_DEPTH(MEM_DEPTH),
        .HEX_FILE(HEX_FILE)
    ) u_pc_fetch (
        .clk          (clk),
        .rst_n        (rst_n),
        .branch_taken (branch_taken),
        .next_pc      (next_pc),
        .pc           (pc),
        .instruction  (instruction)
    );

    //--------------------------------------------------------------------------
    // 2. Immediate Generator
    //--------------------------------------------------------------------------
    imm_gen u_imm_gen (
        .instruction (instruction),
        .imm_i       (imm_i),
        .imm_s       (imm_s),
        .imm_b       (imm_b),
        .imm_u       (imm_u),
        .imm_j       (imm_j)
    );

    //--------------------------------------------------------------------------
    // 3. Register File (32 x 32-bit with x0 hardwired to 0)
    //--------------------------------------------------------------------------
    regfile #(
        .WRITE_FIRST(0)
    ) u_regfile (
        .clk          (clk),
        .rst_n        (rst_n),
        .write_enable (reg_write),
        .rs1          (rs1),
        .rs2          (rs2),
        .rd           (rd),
        .rd_data      (rd_data),
        .rs1_data     (rs1_data),
        .rs2_data     (rs2_data)
    );

    //--------------------------------------------------------------------------
    // 4. Control Unit & Decoder
    //--------------------------------------------------------------------------
    control_unit u_control_unit (
        .opcode        (opcode),
        .funct3        (funct3),
        .funct7        (funct7),
        .rs1_data      (rs1_data),
        .rs2_data      (rs2_data),
        .reg_write     (reg_write),
        .alu_src_a_sel (alu_src_a_sel),
        .alu_src_b_sel (alu_src_b_sel),
        .alu_ctrl      (alu_ctrl),
        .branch_taken  (branch_taken),
        .is_jalr       (is_jalr)
    );

    //--------------------------------------------------------------------------
    // 5. Arithmetic Logic Unit (ALU)
    //--------------------------------------------------------------------------
    alu u_alu (
        .a        (alu_in_a),
        .b        (alu_in_b),
        .alu_ctrl (alu_ctrl),
        .result   (alu_result),
        .zero     (alu_zero),
        .overflow (alu_overflow)
    );

    // Debug Outputs
    assign pc_out           = pc;
    assign instruction_out  = instruction;
    assign alu_result_out   = alu_result;
    assign rs1_data_out     = rs1_data;
    assign rs2_data_out     = rs2_data;
    assign rd_data_out      = rd_data;
    assign imm_out          = imm;
    assign alu_ctrl_out     = alu_ctrl;
    assign reg_write_out    = reg_write;
    assign branch_taken_out = branch_taken;

endmodule
