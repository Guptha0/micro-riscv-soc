//==============================================================================
// Module Name: btb_bht
// Description: Dynamic Branch Predictor featuring:
//   - 64-entry Direct-Mapped Branch Target Buffer (BTB)
//   - 2-bit Saturating Counter Branch History Table (BHT)
//
// 2-Bit Saturating Counter States:
//   - 2'b00: Strongly Not Taken (SNT)
//   - 2'b01: Weakly Not Taken   (WNT)
//   - 2'b10: Weakly Taken       (WT)
//   - 2'b11: Strongly Taken     (ST)
//
// Prediction Rule:
//   Predict Taken if BTB Hit (valid & tag match) AND counter[1] == 1'b1.
//==============================================================================

`timescale 1ns / 1ps

module btb_bht #(
    parameter NUM_ENTRIES = 64
)(
    input  wire        clk,
    input  wire        rst_n,

    // Instruction Fetch (IF) Stage Lookup
    input  wire [31:0] if_pc,
    output wire        pred_taken,
    output wire [31:0] pred_target,
    output wire        pred_hit,

    // Execution (EX) Stage Resolution & Training
    input  wire        update_en,
    input  wire [31:0] update_pc,
    input  wire [31:0] update_target,
    input  wire        actual_taken,
    input  wire        actual_is_jump,

    // Telemetry Statistics
    output reg  [31:0] pred_count,
    output reg  [31:0] pred_hit_count,
    output reg  [31:0] mispred_count
);

    // 64 Entries -> 6-bit index [7:2], 24-bit tag [31:8]
    wire [5:0]  if_index = if_pc[7:2];
    wire [23:0] if_tag   = if_pc[31:8];

    wire [5:0]  ex_index = update_pc[7:2];
    wire [23:0] ex_tag   = update_pc[31:8];

    // Storage Structures
    reg [63:0]  valid_array;
    reg [23:0]  tag_array    [0:NUM_ENTRIES-1];
    reg [31:0]  target_array [0:NUM_ENTRIES-1];
    reg [1:0]   bht_array    [0:NUM_ENTRIES-1];

    // IF Stage Combinational Prediction
    wire entry_valid = valid_array[if_index];
    wire tag_matches = (tag_array[if_index] == if_tag);
    wire btb_hit     = entry_valid && tag_matches;
    wire counter_tk  = bht_array[if_index][1]; // 2'b10 or 2'b11

    assign pred_hit    = btb_hit;
    assign pred_taken  = btb_hit && counter_tk;
    assign pred_target = btb_hit ? target_array[if_index] : (if_pc + 32'd4);

    // Synchronous Update / Training from EX Stage
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_array     <= 64'd0;
            pred_count      <= 32'd0;
            pred_hit_count  <= 32'd0;
            mispred_count   <= 32'd0;
            for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
                tag_array[i]    <= 24'd0;
                target_array[i] <= 32'd0;
                bht_array[i]    <= 2'b01; // Initialize to Weakly Not Taken
            end
        end else begin
            // Track prediction telemetry in IF stage
            if (pred_taken) begin
                pred_count <= pred_count + 32'd1;
            end

            // EX Stage Training
            if (update_en) begin
                valid_array[ex_index]  <= 1'b1;
                tag_array[ex_index]    <= ex_tag;
                target_array[ex_index] <= update_target;

                if (actual_is_jump) begin
                    // Unconditional jumps are always strongly taken
                    bht_array[ex_index] <= 2'b11;
                end else if (actual_taken) begin
                    // Saturating increment towards 2'b11 (ST)
                    if (bht_array[ex_index] != 2'b11) begin
                        bht_array[ex_index] <= bht_array[ex_index] + 2'b01;
                    end
                end else begin
                    // Saturating decrement towards 2'b00 (SNT)
                    if (bht_array[ex_index] != 2'b00) begin
                        bht_array[ex_index] <= bht_array[ex_index] - 2'b01;
                    end
                end
            end
        end
    end

endmodule
