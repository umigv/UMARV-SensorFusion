 module pred_microcode_prog (
        input logic [8:0] pc,
        input logic [3:0] sigma_idx,
        input logic chol_downdate, wheel_valid,

        output ukf_defs::pred_ukf_uop uop
    );
        localparam logic [8:0] PROPAGATE_PC        = 9'd228;
        localparam logic [8:0] ACC_MEAN_PC         = 9'd241;
        localparam logic [8:0] SUB_MEAN_PC         = 9'd251;
        localparam logic [8:0] SCALE_U_AND_CHOL_PC = 9'd257;
        localparam logic [8:0] CHOL_UPDATE_PC      = 9'd262;
        localparam logic [6:0] REG_X          = 7'd0;
        localparam logic [6:0] REG_Y          = 7'd1;
        localparam logic [6:0] REG_THETA      = 7'd2;
        localparam logic [6:0] REG_OMEGA      = 7'd3;
        localparam logic [6:0] REG_WHEEL_BIAS = 7'd4;
        localparam logic [6:0] REG_V_WHEEL    = 7'd5;
        localparam logic [6:0] REG_S_BASE     = 7'd6;
        localparam logic [6:0] SPAD_BASE      = 7'd21;
        localparam logic [6:0] PROM_WEIGHT_SQRT_MC = 7'd37;
        localparam logic [6:0] PROM_SIGMAPT_CONST  = 7'd38;
        localparam logic [6:0] PROM_WEIGHT_MC2     = 7'd39;
        localparam logic [6:0] PROM_WEIGHT_MEAN1   = 7'd40;
        localparam logic [6:0] PROM_WEIGHT_SQRT_C0 = 7'd41;
        localparam logic [6:0] REG_SQ_BASE         = ukf_defs::PROM_SR_BASE;

        localparam logic [8:0] ANGLE_WRAP_COMPARE = 9'd417;
        localparam logic [6:0] XTRA_Z = SPAD_BASE + 7'd13;
        localparam logic [6:0] XTRA_SIGN = SPAD_BASE + 7'd15;
        localparam logic [8:0] XTRA_WRAP_LOW = 9'd421;
        localparam logic [8:0] XTRA_WRAP_ADD = 9'd423;
        localparam logic [8:0] XTRA_SIN_POS = 9'd430;
        localparam logic [8:0] XTRA_SIN_POS_MID = 9'd431;
        localparam logic [8:0] XTRA_SIN_POS_OUTER = 9'd434;
        localparam logic [8:0] XTRA_SIN_NEG_OUTER = 9'd437;
        localparam logic [8:0] XTRA_SIN_NEG_MID = 9'd440;
        localparam logic [8:0] XTRA_SIN_CENTRAL = 9'd443;
        localparam logic [8:0] XTRA_COS_POS = 9'd448;
        localparam logic [8:0] XTRA_COS_POS_MID = 9'd449;
        localparam logic [8:0] XTRA_COS_POS_OUTER = 9'd452;
        localparam logic [8:0] XTRA_COS_NEG_OUTER = 9'd455;
        localparam logic [8:0] XTRA_COS_NEG_MID = 9'd458;
        localparam logic [8:0] XTRA_COS_CENTRAL = 9'd461;
        localparam logic [8:0] XTRA_EVAL_SIN = 9'd462;
        localparam logic [8:0] XTRA_EVAL_COS = 9'd471;

        logic [2:0] col_idx;
        logic neg_sigma;
        logic [6:0] s_src0, s_src1, s_src2, s_src3, s_src4;
        assign neg_sigma = sigma_idx >= 4'd5;
        assign col_idx = neg_sigma ? (sigma_idx - 4'd5) : sigma_idx[2:0];
        assign s_src0 = (col_idx == 3'd0) ? REG_S_BASE : ukf_defs::PROM_MATH_ZERO;
        assign s_src1 = (col_idx == 3'd0) ? (REG_S_BASE + 7'd1) :
                        (col_idx == 3'd1) ? (REG_S_BASE + 7'd5) : ukf_defs::PROM_MATH_ZERO;
        assign s_src2 = (col_idx == 3'd0) ? (REG_S_BASE + 7'd2) :
                        (col_idx == 3'd1) ? (REG_S_BASE + 7'd6) :
                        (col_idx == 3'd2) ? (REG_S_BASE + 7'd9) : ukf_defs::PROM_MATH_ZERO;
        assign s_src3 = (col_idx == 3'd0) ? (REG_S_BASE + 7'd3) :
                        (col_idx == 3'd1) ? (REG_S_BASE + 7'd7) :
                        (col_idx == 3'd2) ? (REG_S_BASE + 7'd10) :
                        (col_idx == 3'd3) ? (REG_S_BASE + 7'd12) : ukf_defs::PROM_MATH_ZERO;
        assign s_src4 = (col_idx == 3'd0) ? (REG_S_BASE + 7'd4) :
                        (col_idx == 3'd1) ? (REG_S_BASE + 7'd8) :
                        (col_idx == 3'd2) ? (REG_S_BASE + 7'd11) :
                        (col_idx == 3'd3) ? (REG_S_BASE + 7'd13) : (REG_S_BASE + 7'd14);

    always_comb begin
        uop = '0;
        uop.dp = ukf_defs::UOP_NOP;
        case (pc)
        // =========================================================================
        // PASS 1: MEAN (x0, s1+..s5+, s1-..s5- -> SPAD_BASE + 7..11)
        // =========================================================================

        // SIGMA PT 0 (x0: no S offset, initializes SPAD[7..11] with W_mean1)
            9'd0:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd1:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd2:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd3:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd4:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
            // mean init (PROM_WEIGHT_MEAN1 * x0_prop)
            9'd5:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE;             uop.dest = SPAD_BASE + 7'd7;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd6:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd1;      uop.dest = SPAD_BASE + 7'd8;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd7:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd2;      uop.dest = SPAD_BASE + 7'd9;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd8:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd3;      uop.dest = SPAD_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
            9'd9:   begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd4;      uop.dest = SPAD_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

        // SIGMA PT 1 (+ row 0: S[0..4])
    // PASS 1 LOOP (sigma_idx 0..9 -> 10..21)
        9'd10: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;          uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd11: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src0; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd12: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;          uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd13: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd14: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;      uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd15: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd16: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;      uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd17: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd18: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd19: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd20: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = ACC_MEAN_PC; uop.instr_done = 1'b1; end
        9'd21: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_LOOP; uop.targ_pc = 9'd10; uop.instr_done = 1'b1; end

    // Wrap mean THETA (SPAD[9])
        9'd22: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd9; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd23: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = ukf_defs::PRED_ANGLE_WRAP; end
        9'd24: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd9; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // PASS 2 LOOP (sigma_idx 0..9 -> 25..37)
        9'd25: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;          uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd26: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src0; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd27: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;          uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd28: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd29: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;      uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd30: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd31: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;      uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd32: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd33: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd34: begin uop.dp = ukf_defs::UOP_MAC; uop.neg_b = neg_sigma; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = s_src4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd35: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = SUB_MEAN_PC; uop.instr_done = 1'b1; end
        9'd36: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end
        9'd37: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_LOOP; uop.targ_pc = 9'd25; uop.instr_done = 1'b1; end

    // d0 (38..48)
        9'd38: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;          uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd39: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;          uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd40: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;      uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd41: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;      uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd42: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd43: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = SUB_MEAN_PC; uop.instr_done = 1'b1; end
        9'd44: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_C0; uop.src_b = SPAD_BASE;        uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd45: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_C0; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd46: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_C0; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd47: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_C0; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd48: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_C0; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = CHOL_UPDATE_PC; uop.instr_done = 1'b1; end

    // FINAL HANDOFF: SQ -> S (49..63), then wrap SPAD[9] & halt (64)
        9'd49: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE;         uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE;         uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd50: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd1;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd1;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd51: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd2;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd2;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd52: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd3;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd3;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd53: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd4;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd4;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd54: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd5;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd5;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd55: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd6;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd6;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd56: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd7;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd7;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd57: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd8;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd8;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd58: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd9;  uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd9;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd59: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd10; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd60: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd11; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd61: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd12; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd62: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd13; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd63: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd14; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_S_BASE + 7'd14; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd64: begin uop.dp = ukf_defs::UOP_NOP; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = 9'd495; end

    // SUBROUTINE: PROPAGATE (9'd228..9'd240)
    // =========================================================================
        9'd228: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd229: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = SPAD_BASE + 7'd2; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; if (wheel_valid) begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = 9'd488; end end
        9'd230: begin if (!wheel_valid) begin uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = 9'd238; end else begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = REG_V_WHEEL; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end end
        9'd231: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd232: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = ukf_defs::PRED_ANGLE_COS; end
        9'd233: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.instr_done = 1'b1;  end
        9'd234: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE;        uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd235: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = 9'd482; end
        9'd236: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.instr_done = 1'b1;  end
        9'd237: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd1; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd238: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd2; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1;  end
        9'd239: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd240: begin uop.seq = ukf_defs::SEQ_RET; end

    // =========================================================================
    // SUBROUTINE: ACC_MEAN (9'd241..9'd250)
    // =========================================================================
        9'd241: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd7;  uop.instr_done = 1'b1; end
        9'd242: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE;        uop.dest = SPAD_BASE + 7'd7;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd243: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd8;  uop.instr_done = 1'b1; end
        9'd244: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd8;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd245: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd9;  uop.instr_done = 1'b1; end
        9'd246: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd9;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd247: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd10; uop.instr_done = 1'b1; end
        9'd248: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd249: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd11; uop.instr_done = 1'b1; end
        9'd250: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd11; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: SUB_MEAN (9'd251..9'd256)
    // =========================================================================
        9'd251: begin uop.dp = ukf_defs::UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE;        uop.src_b = SPAD_BASE + 7'd7;  uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd252: begin uop.dp = ukf_defs::UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd8;  uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd253: begin uop.dp = ukf_defs::UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd2; uop.src_b = SPAD_BASE + 7'd9;  uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd254: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = 9'd485; end
        9'd255: begin uop.dp = ukf_defs::UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd256: begin uop.dp = ukf_defs::UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd4; uop.src_b = SPAD_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: SCALE_U_AND_CHOL (9'd257..9'd261, tail-jumps to CHOL_UPDATE)
    // =========================================================================
        9'd257: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE;        uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd258: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd259: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd260: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd261: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = CHOL_UPDATE_PC; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: CHOL_UPDATE (9'd262..9'd338)
    //   In-place on REG_SQ_BASE[0..14]; SPAD[0..4]=u, [5]=c, [6]=s, [12]=r, [13]=tmp
    // =========================================================================
    // Stage k=0: r00 (SQ+0), off-diags r01..r04 (SQ+1..4)
        9'd262: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE;        uop.src_b = REG_SQ_BASE;        uop.instr_done = 1'b1; end
        9'd263: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE;           uop.src_b = SPAD_BASE;           uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd264: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd265: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = REG_SQ_BASE; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd266: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE;         uop.src_b = REG_SQ_BASE; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd267: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = REG_SQ_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=1 (r01, u1)
        9'd268: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd1; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd269: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd270: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd271: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd272: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=2 (r02, u2)
        9'd273: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd2; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd274: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd275: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd276: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd2; uop.instr_done = 1'b1; end
        9'd277: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=3 (r03, u3)
        9'd278: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd3; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd279: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd280: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd281: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd282: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r04, u4)
        9'd283: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd4; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd284: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd285: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd286: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd287: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=1: r11 (SQ+5), off-diags r12..r14 (SQ+6..8)
        9'd288: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd5; uop.src_b = REG_SQ_BASE + 7'd5; uop.instr_done = 1'b1; end
        9'd289: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd1;    uop.src_b = SPAD_BASE + 7'd1;    uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd290: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd291: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = REG_SQ_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd292: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd1;  uop.src_b = REG_SQ_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd293: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = REG_SQ_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=2 (r12, u2)
        9'd294: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd6; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd295: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd296: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd297: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd2; uop.instr_done = 1'b1; end
        9'd298: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd6; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=3 (r13, u3)
        9'd299: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd7; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd300: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd301: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd7; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd302: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd303: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd7; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r14, u4)
        9'd304: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd8; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd305: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd306: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd8; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd307: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd308: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd8; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=2: r22 (SQ+9), off-diags r23..r24 (SQ+10..11)
        9'd309: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd9; uop.src_b = REG_SQ_BASE + 7'd9; uop.instr_done = 1'b1; end
        9'd310: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd2;    uop.src_b = SPAD_BASE + 7'd2;    uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd311: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd312: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = REG_SQ_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd313: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd2;  uop.src_b = REG_SQ_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd314: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = REG_SQ_BASE + 7'd9; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=3 (r23, u3)
        9'd315: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd10; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd316: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd317: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd318: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd319: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r24, u4)
        9'd320: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd11; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd321: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd322: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd323: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd324: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=3: r33 (SQ+12), off-diag r34 (SQ+13)
        9'd325: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd12; uop.src_b = REG_SQ_BASE + 7'd12; uop.instr_done = 1'b1; end
        9'd326: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd3;     uop.src_b = SPAD_BASE + 7'd3;     uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd327: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd328: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = REG_SQ_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd329: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd3;  uop.src_b = REG_SQ_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd330: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.dest = REG_SQ_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r34, u4)
        9'd331: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd13; uop.src_b = ukf_defs::PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd332: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd333: begin uop.dp = ukf_defs::UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = REG_SQ_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd334: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd335: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = REG_SQ_BASE + 7'd13; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=4: r44 (SQ+14)
        9'd336: begin uop.dp = ukf_defs::UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_SQ_BASE + 7'd14; uop.src_b = REG_SQ_BASE + 7'd14; uop.instr_done = 1'b1; end
        9'd337: begin uop.dp = ukf_defs::UOP_MAC;  uop.neg_b = chol_downdate; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd4;     uop.src_b = SPAD_BASE + 7'd4;     uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd338: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = REG_SQ_BASE + 7'd14; uop.wb_en = 1'b1; uop.seq = ukf_defs::SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
        ukf_defs::PRED_ANGLE_WRAP: begin uop.seq = ukf_defs::SEQ_ANGLE_CHECK; uop.src_a = ukf_defs::PRED_ANGLE_ARG; end
        // Wrap in place. Each MAC pair seeds the accumulator, then adds one turn.
        ANGLE_WRAP_COMPARE: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_PI; uop.targ_pc = XTRA_WRAP_LOW; end
        9'd418: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd419: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_WRAP_STEP; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd420: begin uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = ANGLE_WRAP_COMPARE; end
        XTRA_WRAP_LOW: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_NEG_PI; uop.targ_pc = XTRA_WRAP_ADD; end
        9'd422: begin uop.seq = ukf_defs::SEQ_RET; end
        XTRA_WRAP_ADD: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd424: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_WRAP_STEP; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd425: begin uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = ANGLE_WRAP_COMPARE; end

        // Reduce the wrapped angle for a sine request.
        ukf_defs::PRED_ANGLE_SIN: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_NEG_3PI_4; uop.targ_pc = XTRA_SIN_NEG_OUTER; end
        9'd427: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_NEG_PI_4; uop.targ_pc = XTRA_SIN_NEG_MID; end
        9'd428: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PROM_MATH_PI_4; uop.src_b = ukf_defs::PRED_ANGLE_ARG; uop.targ_pc = XTRA_SIN_POS; end
        9'd429: begin uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_SIN_CENTRAL; end
        XTRA_SIN_POS: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PROM_MATH_3PI_4; uop.src_b = ukf_defs::PRED_ANGLE_ARG; uop.targ_pc = XTRA_SIN_POS_OUTER; end
        XTRA_SIN_POS_MID: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd432: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI_2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd433: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_SIN_POS_OUTER: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd435: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd436: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_SIN_NEG_OUTER: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd438: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd439: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_SIN_NEG_MID: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd441: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI_2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd442: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_SIN_CENTRAL: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end

        // Reduce the wrapped angle for a cosine request.
        ukf_defs::PRED_ANGLE_COS: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_NEG_3PI_4; uop.targ_pc = XTRA_COS_NEG_OUTER; end
        9'd445: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_NEG_PI_4; uop.targ_pc = XTRA_COS_NEG_MID; end
        9'd446: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PROM_MATH_PI_4; uop.src_b = ukf_defs::PRED_ANGLE_ARG; uop.targ_pc = XTRA_COS_POS; end
        9'd447: begin uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_COS_CENTRAL; end
        XTRA_COS_POS: begin uop.seq = ukf_defs::SEQ_FP_LT; uop.src_a = ukf_defs::PROM_MATH_3PI_4; uop.src_b = ukf_defs::PRED_ANGLE_ARG; uop.targ_pc = XTRA_COS_POS_OUTER; end
        XTRA_COS_POS_MID: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd450: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI_2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd451: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_COS_POS_OUTER: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd453: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd454: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_COS_NEG_OUTER: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd456: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd457: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.neg_b = 1'b1; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end
        XTRA_COS_NEG_MID: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd459: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = ukf_defs::PROM_MATH_PI_2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd460: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_SIN; end
        XTRA_COS_CENTRAL: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = XTRA_SIGN; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = XTRA_EVAL_COS; end

        // Sine Horner evaluation, then multiply by the reduced angle and sign.
        XTRA_EVAL_SIN: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PRED_ANGLE_ARG; uop.dest = XTRA_Z; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd463: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_SIN_C5; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd464: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = ukf_defs::PROM_MATH_SIN_C7; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd465: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_SIN_C3; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd466: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd467: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd468: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd469: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd470: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_RESULT; uop.src_b = XTRA_SIGN; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_RET; end

        // Cosine Horner evaluation, then apply the stored sign.
        XTRA_EVAL_COS: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PRED_ANGLE_ARG; uop.dest = XTRA_Z; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd472: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_COS_C4; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd473: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = ukf_defs::PROM_MATH_COS_C6; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd474: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_COS_C2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd475: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd476: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.instr_done = 1'b1; end
        9'd477: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = XTRA_Z; uop.src_b = ukf_defs::PRED_ANGLE_RESULT; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd478: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_RESULT; uop.src_b = XTRA_SIGN; uop.dest = ukf_defs::PRED_ANGLE_RESULT; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_RET; end
        9'd482: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd14; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_JUMP; uop.targ_pc = ukf_defs::PRED_ANGLE_SIN; end
        9'd485: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1;  end
        9'd486: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = ukf_defs::PRED_ANGLE_WRAP; end
        9'd487: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_RET; end
        // The midpoint is wrapped once and reused for both trig calls.
        9'd488: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = ukf_defs::PRED_ANGLE_WRAP; end
        9'd489: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd14; uop.wb_en = 1'b1; uop.instr_done = 1'b1; uop.seq = ukf_defs::SEQ_RET; end
        9'd495: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd9; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = ukf_defs::PRED_ANGLE_ARG; uop.wb_en = 1'b1; uop.instr_done = 1'b1;  end
        9'd496: begin uop.seq = ukf_defs::SEQ_CALL; uop.targ_pc = ukf_defs::PRED_ANGLE_WRAP; end
        9'd497: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = ukf_defs::PRED_ANGLE_ARG; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = SPAD_BASE + 7'd9; uop.wb_en = 1'b1; uop.instr_done = 1'b1;  end
        9'd498: begin uop.dp = ukf_defs::UOP_DONE; uop.seq = ukf_defs::SEQ_HALT; end
        default: begin uop.seq = ukf_defs::SEQ_HALT; end
        endcase
    end

    endmodule
