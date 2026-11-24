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

    // S+ mat (15): 6..20  -> access as (REG_S_BASE + i), i = 0..14
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


    // Extra math/trig constants (0.0, 1.0, dt, pi, sin/cos coeffs): 93+
    localparam logic [6:0] PROM_MATH_BASE        = 7'd93;

always_comb begin
    uop = '0;
    uop.dp  = UOP_NONE;
    
    case(pc)

    endcase
end

endmodule