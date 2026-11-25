 module pred_microcode_prog (
        input logic [9:0] pc,

        output pred_ukf_uop uop 
    );
        // Subroutine entry PCs
        localparam logic [8:0] PROPAGATE_PC        = 9'd228;
        localparam logic [8:0] ACC_MEAN_PC         = 9'd241;
        localparam logic [8:0] SUB_MEAN_PC         = 9'd251;
        localparam logic [8:0] SCALE_U_AND_CHOL_PC = 9'd257;
        localparam logic [8:0] CHOL_UPDATE_PC      = 9'd262;
        localparam logic [8:0] CHOL_DOWNDATE_PC    = 9'd339;

        // USE THESE FOR MEM ACCESS IN UOPS
        // state regs
        localparam logic [6:0] REG_X          = 7'd0;
        localparam logic [6:0] REG_Y          = 7'd1;
        localparam logic [6:0] REG_THETA      = 7'd2;
        localparam logic [6:0] REG_OMEGA      = 7'd3;
        localparam logic [6:0] REG_WHEEL_BIAS = 7'd4;

        // velo calc from dw (1): 5
        localparam logic [6:0] REG_V_WHEEL    = 7'd5;

        // S mat (15): 6..20  -> access as (REG_S_BASE + i), i = 0..14
        localparam logic [6:0] REG_S_BASE     = 7'd6;

     // state (5): 0..4, v_wheel (1): 5, S (15): 6..20
        localparam logic [6:0] SPAD_BASE           = 7'd21; // 16 regs: 21..36

        localparam logic [6:0] PROM_WEIGHT_SQRT_MC = 7'd37;
        localparam logic [6:0] PROM_SIGMAPT_CONST  = 7'd38;
        localparam logic [6:0] PROM_WEIGHT_MC2     = 7'd39;
        localparam logic [6:0] PROM_WEIGHT_MEAN1   = 7'd40;
        localparam logic [6:0] PROM_WEIGHT_COVAR1  = 7'd41;

        // SQ mat (15): 42..56
        localparam logic [6:0] PROM_SQ_BASE        = 7'd42;

        // (Optional SR mat 10: 57..66 -> PROM_MATH_BASE = 7'd67)
        localparam logic [6:0] PROM_MATH_BASE      = 7'd57;

    always_comb begin
        uop = '0;
        uop.dp = UOP_NOP;
        case (pc)
    // =========================================================================
    // PASS 1: MEAN (x0, s1+..s5+, s1-..s5- -> SPAD_BASE + 7..11)
    // =========================================================================

    // SIGMA PT 0 (x0: no S offset, initializes SPAD[7..11] with W_mean1)
        9'd0:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd1:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd2:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd3:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd4:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        // mean init (PROM_WEIGHT_MEAN1 * x0_prop)
        9'd5:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE;             uop.dest = SPAD_BASE + 7'd7;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd6:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd1;      uop.dest = SPAD_BASE + 7'd8;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd7:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd2;      uop.dest = SPAD_BASE + 7'd9;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd8:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd3;      uop.dest = SPAD_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd9:   begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_MEAN1;  uop.src_b = SPAD_BASE + 7'd4;      uop.dest = SPAD_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // SIGMA PT 1 (+ col 0: S[0..4])
        9'd10:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd11:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE;            uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd12:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd13:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd1;     uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd14:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd15:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd2;     uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd16:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd17:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd3;     uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd18:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd19:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd4;     uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd20:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 2 (+ col 1: S[5..8], X unchanged)
        9'd21:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd22:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd23:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd5;     uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd24:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd25:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd6;     uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd26:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd27:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd7;     uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd28:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd29:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd8;     uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd30:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 3 (+ col 2: S[9..11], X,Y unchanged)
        9'd31:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd32:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd33:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd34:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd9;     uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd35:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd36:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd10;    uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd37:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd38:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd11;    uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd39:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 4 (+ col 3: S[12..13], X,Y,THETA unchanged)
        9'd40:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd41:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd42:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd43:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd44:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd12;    uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd45:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd46:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd13;    uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd47:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 5 (+ col 4: S[14], X,Y,THETA,OMEGA unchanged)
        9'd48:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd49:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd50:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd51:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd52:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd53:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd14;    uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd54:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 6 (- col 0: S[0..4], neg_b = 1)
        9'd55:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd56:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE;        uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd57:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd58:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd59:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd60:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd61:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd62:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd63:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd64:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd65:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 7 (- col 1: S[5..8], neg_b = 1, X unchanged)
        9'd66:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd67:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd68:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd69:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd70:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd6; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd71:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd72:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd7; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd73:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd74:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd8; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd75:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 8 (- col 2: S[9..11], neg_b = 1, X,Y unchanged)
        9'd76:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd77:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd78:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd79:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd9;  uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd80:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd81:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd82:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd83:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd84:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 9 (- col 3: S[12..13], neg_b = 1, X,Y,THETA unchanged)
        9'd85:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd86:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd87:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd88:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd89:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd90:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd91:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd13; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd92:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // SIGMA PT 10 (- col 4: S[14], neg_b = 1, X,Y,THETA,OMEGA unchanged)
        9'd93:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd94:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd95:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd96:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd97:  begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd98:  begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd14; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd99:  begin uop.dp = UOP_NOP;  uop.seq = SEQ_CALL; uop.targ_pc = ACC_MEAN_PC;      uop.instr_done = 1'b1; end

    // Wrap final THETAmean (SPAD[9])
        9'd100: begin uop.dp = UOP_WRAP; uop.src_a = SPAD_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd9; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // =========================================================================
    // PASS 2: d0..d10 (X_prop - X_mean -> SPAD_BASE + 0..4)
    // Insert/call covar/QR step after each d_i before the next d overwrites SPAD[0..4]
    // =========================================================================

       
    // d1 (+ row 0: S+[0..4])
        9'd101: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd102: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE;            uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd103: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd104: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd1;     uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd105: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd106: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd2;     uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd107: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd108: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd3;     uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd109: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd110: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd4;     uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd111: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd112: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d2 (+ row 1: S+[5..8])
        9'd113: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd114: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd115: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd5;     uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd116: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd117: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd6;     uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd118: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd119: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd7;     uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd120: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd121: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd8;     uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd122: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd123: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d3 (+ row 2: S+[9..11])
        9'd124: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd125: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd126: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd127: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd9;     uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd128: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd129: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd10;    uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd130: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd131: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd11;    uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd132: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd133: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d4 (+ row 3: S+[12..13])
        9'd134: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd135: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd136: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd137: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd138: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd12;    uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd139: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd140: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd13;    uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd141: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd142: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d5 (+ row 4: S+[14])
        9'd143: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd144: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd145: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd146: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd147: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd148: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd14;    uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd149: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd150: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d6 (- row 0: S+[0..4], neg_b = 1)
        9'd151: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd152: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE;        uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd153: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd154: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd155: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd156: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd157: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd158: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd159: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd160: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd161: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd162: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d7 (- row 1: S+[5..8], neg_b = 1)
        9'd163: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd164: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd165: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd166: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd167: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd6; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd168: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd169: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd7; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd170: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd171: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd8; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd172: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd173: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d8 (- row 2: S+[9..11], neg_b = 1)
        9'd174: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd175: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd176: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd177: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd9;  uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd178: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd179: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd180: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd181: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd182: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd183: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d9 (- row 3: S+[12..13], neg_b = 1)
        9'd184: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd185: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd186: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd187: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd188: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd189: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd190: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd13; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd191: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd192: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d10 (- row 4: S+[14], neg_b = 1)
        9'd193: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd194: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd195: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd196: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd197: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd198: begin uop.dp = UOP_MAC; uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd14; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd199: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;         uop.instr_done = 1'b1; end
        9'd200: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SCALE_U_AND_CHOL_PC; uop.instr_done = 1'b1; end

    // d0 (central sigma point x0 -> scale by sqrt(|W_c0|) -> CHOL_UPDATE or CHOL_DOWNDATE)
        9'd201: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd202: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y;              uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd203: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd204: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA;          uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd205: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS;     uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = PROPAGATE_PC; uop.instr_done = 1'b1; end
        9'd206: begin uop.dp = UOP_NOP; uop.seq = SEQ_CALL; uop.targ_pc = SUB_MEAN_PC;      uop.instr_done = 1'b1; end
        9'd207: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_COVAR1; uop.src_b = SPAD_BASE;             uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd208: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_COVAR1; uop.src_b = SPAD_BASE + 7'd1;      uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd209: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_COVAR1; uop.src_b = SPAD_BASE + 7'd2;      uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd210: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_COVAR1; uop.src_b = SPAD_BASE + 7'd3;      uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd211: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_COVAR1; uop.src_b = SPAD_BASE + 7'd4;      uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_CALL; uop.targ_pc = CHOL_UPDATE_PC; /* use CHOL_DOWNDATE_PC if W_c0 < 0 */ uop.instr_done =
        1'b1; end

    // =========================================================================
    // FINAL HANDOFF: Copy S_Q_working (PROM_SQ_BASE) -> S^- (REG_S_BASE)
    // =========================================================================
        9'd212: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE;         uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE;         uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd213: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd1;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd1;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd214: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd2;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd2;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd215: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd3;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd3;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd216: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd4;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd4;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd217: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd5;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd5;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd218: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd6;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd6;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd219: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd7;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd7;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd220: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd8;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd8;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd221: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd9;  uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd9;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd222: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd10; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd223: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd11; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd224: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd225: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd13; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd226: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd14; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = REG_S_BASE + 7'd14; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // DONE (x^- in SPAD[7..11], S^- in REG_S_BASE[0..14])
        9'd227: begin uop.dp = UOP_DONE; uop.seq = SEQ_HALT; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: PROPAGATE (9'd228..9'd240)
    // =========================================================================
        9'd228: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd229: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd230: begin uop.dp = UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = REG_V_WHEEL; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd231: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = PROM_MATH_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd232: begin uop.dp = UOP_TRIG; uop.neg_b = 1'b1; uop.src_a = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd233: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd6; uop.instr_done = 1'b1; end
        9'd234: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE;        uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd235: begin uop.dp = UOP_TRIG; uop.neg_b = 1'b0; uop.src_a = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd236: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd6; uop.instr_done = 1'b1; end
        9'd237: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd1; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd238: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd239: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd240: begin uop.dp = UOP_WRAP; uop.src_a = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.seq = SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: ACC_MEAN (9'd241..9'd250)
    // =========================================================================
        9'd241: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd7;  uop.instr_done = 1'b1; end
        9'd242: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE;        uop.dest = SPAD_BASE + 7'd7;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd243: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd8;  uop.instr_done = 1'b1; end
        9'd244: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd8;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd245: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd9;  uop.instr_done = 1'b1; end
        9'd246: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd9;  uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd247: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd10; uop.instr_done = 1'b1; end
        9'd248: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd249: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd11; uop.instr_done = 1'b1; end
        9'd250: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_WEIGHT_MC2; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd11; uop.wb_en = 1'b1; uop.seq = SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: SUB_MEAN (9'd251..9'd256)
    // =========================================================================
        9'd251: begin uop.dp = UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE;        uop.src_b = SPAD_BASE + 7'd7;  uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd252: begin uop.dp = UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd8;  uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd253: begin uop.dp = UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd2; uop.src_b = SPAD_BASE + 7'd9;  uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd254: begin uop.dp = UOP_WRAP; uop.src_a = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd255: begin uop.dp = UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd256: begin uop.dp = UOP_ADD;  uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd4; uop.src_b = SPAD_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: SCALE_U_AND_CHOL (9'd257..9'd261, tail-jumps to CHOL_UPDATE)
    // =========================================================================
        9'd257: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE;        uop.dest = SPAD_BASE;        uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd258: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd259: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd260: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd261: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_WEIGHT_SQRT_MC; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.seq = SEQ_JUMP; uop.targ_pc = CHOL_UPDATE_PC; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: CHOL_UPDATE (9'd262..9'd338)
    //   In-place on PROM_SQ_BASE[0..14]; SPAD[0..4]=u, [5]=c, [6]=s, [12]=r, [13]=tmp
    // =========================================================================
    // Stage k=0: r00 (SQ+0), off-diags r01..r04 (SQ+1..4)
        9'd262: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE;        uop.src_b = PROM_SQ_BASE;        uop.instr_done = 1'b1; end
        9'd263: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE;           uop.src_b = SPAD_BASE;           uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd264: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd265: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd266: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE;         uop.src_b = PROM_SQ_BASE; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd267: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=1 (r01, u1)
        9'd268: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd1; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd269: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd270: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd271: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd272: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=2 (r02, u2)
        9'd273: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd2; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd274: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd275: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd276: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd2; uop.instr_done = 1'b1; end
        9'd277: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=3 (r03, u3)
        9'd278: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd3; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd279: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd280: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd281: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd282: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r04, u4)
        9'd283: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd4; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd284: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd285: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd286: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd287: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=1: r11 (SQ+5), off-diags r12..r14 (SQ+6..8)
        9'd288: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd5; uop.src_b = PROM_SQ_BASE + 7'd5; uop.instr_done = 1'b1; end
        9'd289: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd1;    uop.src_b = SPAD_BASE + 7'd1;    uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd290: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd291: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd292: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd1;  uop.src_b = PROM_SQ_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd293: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=2 (r12, u2)
        9'd294: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd6; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd295: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd296: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd297: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd2; uop.instr_done = 1'b1; end
        9'd298: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd6; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=3 (r13, u3)
        9'd299: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd7; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd300: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd301: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd7; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd302: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd303: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd7; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r14, u4)
        9'd304: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd8; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd305: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd306: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd8; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd307: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd308: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd8; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=2: r22 (SQ+9), off-diags r23..r24 (SQ+10..11)
        9'd309: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd9; uop.src_b = PROM_SQ_BASE + 7'd9; uop.instr_done = 1'b1; end
        9'd310: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd2;    uop.src_b = SPAD_BASE + 7'd2;    uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd311: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd312: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd313: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd2;  uop.src_b = PROM_SQ_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd314: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE + 7'd9; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=3 (r23, u3)
        9'd315: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd10; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd316: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd317: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd318: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd319: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r24, u4)
        9'd320: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd11; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd321: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd322: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd323: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd324: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=3: r33 (SQ+12), off-diag r34 (SQ+13)
        9'd325: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd12; uop.instr_done = 1'b1; end
        9'd326: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd3;     uop.src_b = SPAD_BASE + 7'd3;     uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd327: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd328: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd329: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd3;  uop.src_b = PROM_SQ_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd330: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        // j=4 (r34, u4)
        9'd331: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd13; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd332: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd333: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd334: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd335: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd13; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    // Stage k=4: r44 (SQ+14)
        9'd336: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd14; uop.src_b = PROM_SQ_BASE + 7'd14; uop.instr_done = 1'b1; end
        9'd337: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd4;     uop.src_b = SPAD_BASE + 7'd4;     uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd338: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = PROM_SQ_BASE + 7'd14; uop.wb_en = 1'b1; uop.seq = SEQ_RET; uop.instr_done = 1'b1; end

    // =========================================================================
    // SUBROUTINE: CHOL_DOWNDATE (9'd339..9'd415, for d0 if W_c0 < 0)
    //   Identical to CHOL_UPDATE except neg_b=1 on u_k^2 and s*u_j
    // =========================================================================
    // Stage k=0
        9'd339: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE;        uop.src_b = PROM_SQ_BASE;        uop.instr_done = 1'b1; end
        9'd340: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE; uop.src_b = SPAD_BASE; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd341: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd342: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd343: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE;         uop.src_b = PROM_SQ_BASE; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd344: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd345: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd1; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd346: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd347: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd348: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd349: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd350: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd2; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd351: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd352: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd353: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd2; uop.instr_done = 1'b1; end
        9'd354: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd355: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd3; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd356: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd357: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd358: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd359: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd360: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd4; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd361: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd362: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd363: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd364: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    // Stage k=1
        9'd365: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd5; uop.src_b = PROM_SQ_BASE + 7'd5; uop.instr_done = 1'b1; end
        9'd366: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd367: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd368: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd369: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd1;  uop.src_b = PROM_SQ_BASE + 7'd5; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd370: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd371: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd6; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd372: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd373: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd374: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd2; uop.instr_done = 1'b1; end
        9'd375: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd6; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd376: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd7; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd377: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd378: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd7; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd379: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd380: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd7; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd381: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd8; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd382: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd383: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd8; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd384: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd385: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd8; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    // Stage k=2
        9'd386: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd9; uop.src_b = PROM_SQ_BASE + 7'd9; uop.instr_done = 1'b1; end
        9'd387: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd2; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd388: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd389: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd390: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd2;  uop.src_b = PROM_SQ_BASE + 7'd9; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd391: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE + 7'd9; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd392: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd10; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd393: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd394: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd10; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd395: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        9'd396: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd10; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd397: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd11; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd398: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd399: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd11; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd400: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd401: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd11; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    // Stage k=3
        9'd402: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd12; uop.instr_done = 1'b1; end
        9'd403: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd404: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd405: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_SQ_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd406: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd3;  uop.src_b = PROM_SQ_BASE + 7'd12; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd407: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd12; uop.src_b = PROM_MATH_BASE + 7'd1; uop.dest = PROM_SQ_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd408: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd13; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        9'd409: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd410: begin uop.dp = UOP_DIV;  uop.src_a = SPAD_BASE + 7'd13; uop.src_b = SPAD_BASE + 7'd5; uop.dest = PROM_SQ_BASE + 7'd13; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd411: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE + 7'd4; uop.instr_done = 1'b1; end
        9'd412: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd6; uop.src_b = PROM_SQ_BASE + 7'd13; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    // Stage k=4
        9'd413: begin uop.dp = UOP_MAC;  uop.clr_acc = 1'b1; uop.src_a = PROM_SQ_BASE + 7'd14; uop.src_b = PROM_SQ_BASE + 7'd14; uop.instr_done = 1'b1; end
        9'd414: begin uop.dp = UOP_MAC;  uop.neg_b = 1'b1; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE + 7'd4; uop.src_b = SPAD_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd12; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        9'd415: begin uop.dp = UOP_SQRT; uop.src_a = SPAD_BASE + 7'd12; uop.dest = PROM_SQ_BASE + 7'd14; uop.wb_en = 1'b1; uop.seq = SEQ_RET; uop.instr_done = 1'b1; end
        
        endcase
    end

    endmodule
