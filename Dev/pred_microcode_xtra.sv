// Temporary prediction ROM fragment. Copy the localparams and case rows into
// pred_microcode.sv once the main program's PC and scratch allocations are set.
//
// Caller contract:
//   1. Copy src_a into XTRA_ANGLE (for example, multiply by XTRA_ONE).
//   2. SEQ_CALL XTRA_WRAP for bounded, finite prediction angles. It wraps to
//      [-pi, pi). If this is UOP_WRAP, copy XTRA_ANGLE to the original dest.
//   3. For UOP_TRIG_FIX, SEQ_CALL XTRA_SIN (neg_b=0) or XTRA_COS (neg_b=1).
//      Copy XTRA_POLY to the original dest when the routine returns.
//      The two calls are sequential and can reuse one return PC.
//
// Executor contract:
//   SEQ_FP_LT jumps to targ_pc when FP32(src_a) < FP32(src_b), otherwise PC+1.
//   It uses the two operand reads. Neither arithmetic nor writeback is issued.
//   MAC rows wait for completion. A row with wb_en=0 still updates the MAC
//   accumulator. neg_b flips the sign of operand B before issuing the MAC.
//   SEQ_JUMP and SEQ_RET on a MAC row take effect after its writeback.
//
// All data temporaries are in shared scratch. The MAC must see both operands
// for a scratch-to-scratch multiply; operand routing belongs to the executor.
module pred_microcode_xtra #(
    parameter logic [6:0] XTRA_PC_BASE = 7'd64
) (
    input  logic [6:0] pc,
    output pred_ukf_uop uop
);

    // Scratchpad ends at 63. The first PROM word is at 64.
    localparam logic [6:0] XTRA_ANGLE = 7'd60;
    localparam logic [6:0] XTRA_Z     = 7'd61;
    localparam logic [6:0] XTRA_POLY  = 7'd62;
    localparam logic [6:0] XTRA_SIGN  = 7'd63;

    // Draft PROM addresses follow the order in pred_microcode.sv.
    localparam logic [6:0] XTRA_MATH_BASE = 7'd93;
    localparam logic [6:0] XTRA_ONE      = XTRA_MATH_BASE + 7'd1;
    localparam logic [6:0] XTRA_PI       = XTRA_MATH_BASE + 7'd4;
    localparam logic [6:0] XTRA_PI_2     = XTRA_MATH_BASE + 7'd5;
    localparam logic [6:0] XTRA_NEG_PI   = XTRA_MATH_BASE + 7'd6;
    localparam logic [6:0] XTRA_PI_4     = XTRA_MATH_BASE + 7'd8;
    localparam logic [6:0] XTRA_NEG_PI_4 = XTRA_MATH_BASE + 7'd9;
    localparam logic [6:0] XTRA_3PI_4    = XTRA_MATH_BASE + 7'd10;
    localparam logic [6:0] XTRA_NEG_3PI_4 = XTRA_MATH_BASE + 7'd11;

    // Assume the Taylor coefficients occupy the end of the constants table.
    localparam logic [6:0] XTRA_SIN_C3 = XTRA_MATH_BASE + 7'd14;
    localparam logic [6:0] XTRA_SIN_C5 = XTRA_MATH_BASE + 7'd15;
    localparam logic [6:0] XTRA_SIN_C7 = XTRA_MATH_BASE + 7'd16;
    localparam logic [6:0] XTRA_COS_C2 = XTRA_MATH_BASE + 7'd17;
    localparam logic [6:0] XTRA_COS_C4 = XTRA_MATH_BASE + 7'd18;
    localparam logic [6:0] XTRA_COS_C6 = XTRA_MATH_BASE + 7'd19;
    localparam logic [6:0] XTRA_TWO_PI = XTRA_MATH_BASE + 7'd20;

    // Use an unassigned pc_uop encoding until the executor defines it.
    localparam pc_uop SEQ_FP_LT = pc_uop'(3'd6);

    localparam logic [6:0] XTRA_WRAP = XTRA_PC_BASE + 7'd0;
    localparam logic [6:0] XTRA_WRAP_LOW = XTRA_PC_BASE + 7'd4;
    localparam logic [6:0] XTRA_WRAP_ADD = XTRA_PC_BASE + 7'd6;
    localparam logic [6:0] XTRA_SIN = XTRA_PC_BASE + 7'd9;
    localparam logic [6:0] XTRA_SIN_POS = XTRA_PC_BASE + 7'd13;
    localparam logic [6:0] XTRA_SIN_POS_MID = XTRA_PC_BASE + 7'd14;
    localparam logic [6:0] XTRA_SIN_POS_OUTER = XTRA_PC_BASE + 7'd17;
    localparam logic [6:0] XTRA_SIN_NEG_OUTER = XTRA_PC_BASE + 7'd20;
    localparam logic [6:0] XTRA_SIN_NEG_MID = XTRA_PC_BASE + 7'd23;
    localparam logic [6:0] XTRA_SIN_CENTRAL = XTRA_PC_BASE + 7'd26;
    localparam logic [6:0] XTRA_COS = XTRA_PC_BASE + 7'd27;
    localparam logic [6:0] XTRA_COS_POS = XTRA_PC_BASE + 7'd31;
    localparam logic [6:0] XTRA_COS_POS_MID = XTRA_PC_BASE + 7'd32;
    localparam logic [6:0] XTRA_COS_POS_OUTER = XTRA_PC_BASE + 7'd35;
    localparam logic [6:0] XTRA_COS_NEG_OUTER = XTRA_PC_BASE + 7'd38;
    localparam logic [6:0] XTRA_COS_NEG_MID = XTRA_PC_BASE + 7'd41;
    localparam logic [6:0] XTRA_COS_CENTRAL = XTRA_PC_BASE + 7'd44;
    localparam logic [6:0] XTRA_EVAL_SIN = XTRA_PC_BASE + 7'd45;
    localparam logic [6:0] XTRA_EVAL_COS = XTRA_PC_BASE + 7'd54;

    always_comb begin
        uop = '0;
        uop.dp = UOP_NOP;
        case (pc)

        // Wrap in place. Each MAC pair seeds the accumulator, then adds one turn.
        XTRA_WRAP: begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_PI; uop.targ_pc = XTRA_WRAP_LOW; end
        (XTRA_PC_BASE + 7'd1): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd2): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_TWO_PI; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd3): begin uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_WRAP; end
        XTRA_WRAP_LOW: begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_NEG_PI; uop.targ_pc = XTRA_WRAP_ADD; end
        (XTRA_PC_BASE + 7'd5): begin uop.seq = SEQ_RET; end
        XTRA_WRAP_ADD: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd7): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_TWO_PI; uop.src_b = XTRA_ONE; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd8): begin uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_WRAP; end

        // Reduce the wrapped angle for a sine request.
        XTRA_SIN: begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_NEG_3PI_4; uop.targ_pc = XTRA_SIN_NEG_OUTER; end
        (XTRA_PC_BASE + 7'd10): begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_NEG_PI_4; uop.targ_pc = XTRA_SIN_NEG_MID; end
        (XTRA_PC_BASE + 7'd11): begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_PI_4; uop.src_b = XTRA_ANGLE; uop.targ_pc = XTRA_SIN_POS; end
        (XTRA_PC_BASE + 7'd12): begin uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_SIN_CENTRAL; end
        XTRA_SIN_POS: begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_3PI_4; uop.src_b = XTRA_ANGLE; uop.targ_pc = XTRA_SIN_POS_OUTER; end
        XTRA_SIN_POS_MID: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd15): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI_2; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd16): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_SIN_POS_OUTER: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd18): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd19): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_SIN_NEG_OUTER: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd21): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI; uop.src_b = XTRA_ONE; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd22): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_SIN_NEG_MID: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd24): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI_2; uop.src_b = XTRA_ONE; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd25): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_SIN_CENTRAL: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end

        // Reduce the wrapped angle for a cosine request.
        XTRA_COS: begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_NEG_3PI_4; uop.targ_pc = XTRA_COS_NEG_OUTER; end
        (XTRA_PC_BASE + 7'd28): begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_NEG_PI_4; uop.targ_pc = XTRA_COS_NEG_MID; end
        (XTRA_PC_BASE + 7'd29): begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_PI_4; uop.src_b = XTRA_ANGLE; uop.targ_pc = XTRA_COS_POS; end
        (XTRA_PC_BASE + 7'd30): begin uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_COS_CENTRAL; end
        XTRA_COS_POS: begin uop.seq = SEQ_FP_LT; uop.src_a = XTRA_3PI_4; uop.src_b = XTRA_ANGLE; uop.targ_pc = XTRA_COS_POS_OUTER; end
        XTRA_COS_POS_MID: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd33): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI_2; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd34): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_COS_POS_OUTER: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd36): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd37): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_COS_NEG_OUTER: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd39): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI; uop.src_b = XTRA_ONE; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd40): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_COS_NEG_MID: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd42): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_PI_2; uop.src_b = XTRA_ONE; uop.dest = XTRA_ANGLE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd43): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_COS_CENTRAL: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end

        // Sine Horner evaluation, then multiply by the reduced angle and sign.
        XTRA_EVAL_SIN: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ANGLE; uop.dest = XTRA_Z; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd46): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_SIN_C5; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd47): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = XTRA_SIN_C7; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd48): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_SIN_C3; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd49): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = XTRA_POLY; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd50): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd51): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = XTRA_POLY; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd52): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_POLY; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd53): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_POLY; uop.src_b = XTRA_SIGN; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_RET; end

        // Cosine Horner evaluation, then apply the stored sign.
        XTRA_EVAL_COS: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ANGLE; uop.src_b = XTRA_ANGLE; uop.dest = XTRA_Z; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd55): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_COS_C4; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd56): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = XTRA_COS_C6; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd57): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_COS_C2; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd58): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = XTRA_POLY; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd59): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_ONE; uop.src_b = XTRA_ONE; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd60): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = XTRA_POLY; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        (XTRA_PC_BASE + 7'd61): begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = XTRA_POLY; uop.src_b = XTRA_SIGN; uop.dest = XTRA_POLY; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = SEQ_RET; end
        default: begin end
        endcase
    end
endmodule
