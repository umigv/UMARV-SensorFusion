module pred_microcode_prog (
    input logic [6:0] pc,

    output pred_ukf_uop uop 
);

 //USE THESE FOR MEM ACCESS IN UOPS
    //state regs
    localparam logic [6:0] REG_X          = 7'd0;
    localparam logic [6:0] REG_Y          = 7'd1;
    localparam logic [6:0] REG_THETA      = 7'd2;
    localparam logic [6:0] REG_OMEGA      = 7'd3;
    localparam logic [6:0] REG_WHEEL_BIAS = 7'd4;

    // velo calc from dw (1): 5
    localparam logic [6:0] REG_V_WHEEL    = 7'd5;

    // S mat (15): 6..20  -> access as (REG_S_BASE + i), i = 0..14
    // S mat (15): 6..20  -> access as (REG_S_BASE + i), i = 0..14
    localparam logic [6:0] REG_S_BASE     = 7'd6;


    // SPAD (generic scratchpad): starts at 21 -> access as (SPAD_BASE + i)
    // (e.g., 21..63 for 43 regs, or 21..75 if you use 55 regs)
    localparam logic [6:0] SPAD_BASE      = 7'd21;

    // =========================================================
    // PROM (Combinational Constants, 0 DFFs)
    // =========================================================
    // constants: 64..67 (put at 80+ if SPAD is 55)
    localparam logic [6:0] PROM_SIGMAPT_CONST    = 7'd64;
    localparam logic [6:0] PROM_WEIGHT_MC2       = 7'd65;
    localparam logic [6:0] PROM_WEIGHT_MEAN1     = 7'd66;
    localparam logic [6:0] PROM_WEIGHT_COVAR1    = 7'd67;


    // SQ mat (15): 68..82 -> access as (PROM_SQ_BASE + i), i = 0..14
    localparam logic [6:0] PROM_SQ_BASE          = 7'd68;


    // Extra math/trig constants (0.0, 1.0, dt, dt/2, pi, pi/2, -pi, -pi/2, pi/4, -pi/4, 3pi/4, -3pi/4, pi/180, earth's radius, sin/cos coeffs): 93+
    // PROM_MATH_BASE and the shared math decoder are defined in sys_defs.sv.

always_comb begin
    uop = '0;
    uop.dp  = UOP_NOP;
    uop.dp  = UOP_NOP;
    case(pc)
    //sigma pt 1
        7'd0: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd1: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE; uop.dest = SPAD_BASE; uop.instr_done = 1'b1; end

        7'd2: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd3: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        
        7'd4: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd5: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        
        7'd6: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd7: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        
        7'd8: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd9: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    //propogated sigma pt 1 theta M
        7'd10: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        7'd11: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    //propogated v using bias
        7'd12: begin uop.dp = UOP_ADD; uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = REG_V_WHEEL; uop.src_b = SPAD_BASE + 7'd4;uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    //propogated x  using theta M
        7'd13: begin uop.dp = UOP_TRIG; uop.src_a = SPAD_BASE + 7'd2; uop.neg_b = 1'b1; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        7'd14: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE+7'd6; uop.instr_done = 1'b1; end
        7'd15: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE; uop.src_b = PROM_MATH_BASE+7'd1; uop.dest = SPAD_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    //propogated y using theta M
        7'd13: begin uop.dp = UOP_TRIG; uop.src_a = SPAD_BASE + 7'd2; uop.neg_b = 1'b0; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        7'd14: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE+7'd6; uop.instr_done = 1'b1; end
        7'd15: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE+7'd1; uop.src_b = PROM_MATH_BASE+7'd1; uop.dest = SPAD_BASE+7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    //v calc
    //sigma pt 1
        7'd0: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_X; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd1: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE; uop.dest = SPAD_BASE; uop.instr_done = 1'b1; end

        7'd2: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_Y; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd3: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd1; uop.dest = SPAD_BASE + 7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        
        7'd4: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_THETA; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd5: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        
        7'd6: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_OMEGA; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd7: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd3; uop.dest = SPAD_BASE + 7'd3; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        
        7'd8: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = REG_WHEEL_BIAS; uop.src_b = PROM_MATH_BASE + 7'd1; uop.instr_done = 1'b1; end
        7'd9: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_SIGMAPT_CONST; uop.src_b = REG_S_BASE + 7'd4; uop.dest = SPAD_BASE + 7'd4; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    //propogated sigma pt 1 theta M
        7'd10: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = PROM_MATH_BASE + 7'd3; uop.src_b = SPAD_BASE + 7'd3; uop.instr_done = 1'b1; end
        7'd11: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = PROM_MATH_BASE + 7'd1; uop.src_b = SPAD_BASE + 7'd2; uop.dest = SPAD_BASE + 7'd2; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    //propogated v using bias
        7'd12: begin uop.dp = UOP_ADD; uop.neg_b = 1'b1; uop.clr_acc = 1'b1; uop.src_a = REG_V_WHEEL; uop.src_b = SPAD_BASE + 7'd4;uop.dest = SPAD_BASE + 7'd5; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    //propogated x  using theta M
        7'd13: begin uop.dp = UOP_TRIG; uop.src_a = SPAD_BASE + 7'd2; uop.neg_b = 1'b1; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        7'd14: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE+7'd6; uop.instr_done = 1'b1; end
        7'd15: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE; uop.src_b = PROM_MATH_BASE+7'd1; uop.dest = SPAD_BASE; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
    //propogated y using theta M
        7'd13: begin uop.dp = UOP_TRIG; uop.src_a = SPAD_BASE + 7'd2; uop.neg_b = 1'b0; uop.dest = SPAD_BASE + 7'd6; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end
        7'd14: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b1; uop.src_a = SPAD_BASE + 7'd5; uop.src_b = SPAD_BASE+7'd6; uop.instr_done = 1'b1; end
        7'd15: begin uop.dp = UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SPAD_BASE+7'd1; uop.src_b = PROM_MATH_BASE+7'd1; uop.dest = SPAD_BASE+7'd1; uop.wb_en = 1'b1; uop.instr_done = 1'b1; end

    //v calc
    endcase
end

endmodule