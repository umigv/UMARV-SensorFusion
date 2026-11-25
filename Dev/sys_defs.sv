// Shared read-only address map. The measurement-noise factor occupies 83..92.
localparam logic [6:0] PROM_SR_BASE   = 7'd83;
localparam logic [6:0] PROM_MATH_BASE = 7'd93;
localparam logic [6:0] PROM_MATH_ZERO = PROM_MATH_BASE;
localparam logic [6:0] PROM_MATH_ONE  = PROM_MATH_BASE + 7'd1;

localparam logic [6:0] PROM_MATH_PI = PROM_MATH_BASE + 7'd4;
localparam logic [6:0] PROM_MATH_PI_2 = PROM_MATH_BASE + 7'd5;
localparam logic [6:0] PROM_MATH_NEG_PI = PROM_MATH_BASE + 7'd6;
localparam logic [6:0] PROM_MATH_NEG_PI_2 = PROM_MATH_BASE + 7'd7;
localparam logic [6:0] PROM_MATH_PI_4 = PROM_MATH_BASE + 7'd8;
localparam logic [6:0] PROM_MATH_NEG_PI_4 = PROM_MATH_BASE + 7'd9;
localparam logic [6:0] PROM_MATH_3PI_4 = PROM_MATH_BASE + 7'd10;
localparam logic [6:0] PROM_MATH_NEG_3PI_4 = PROM_MATH_BASE + 7'd11;
localparam logic [6:0] PROM_MATH_SIN_C3 = PROM_MATH_BASE + 7'd14;
localparam logic [6:0] PROM_MATH_SIN_C5 = PROM_MATH_BASE + 7'd15;
localparam logic [6:0] PROM_MATH_SIN_C7 = PROM_MATH_BASE + 7'd16;
localparam logic [6:0] PROM_MATH_COS_C2 = PROM_MATH_BASE + 7'd17;
localparam logic [6:0] PROM_MATH_COS_C4 = PROM_MATH_BASE + 7'd18;
localparam logic [6:0] PROM_MATH_COS_C6 = PROM_MATH_BASE + 7'd19;
localparam logic [6:0] PROM_MATH_TWO_PI = PROM_MATH_BASE + 7'd20;
// Dynamic operand address used by the wrap microcode.
localparam logic [6:0] PROM_WRAP_STEP = 7'd114;

function automatic logic [31:0] prom_math_read(input logic [6:0] addr);
    case (addr)
        PROM_MATH_ZERO: prom_math_read = 32'h00000000;
        PROM_MATH_ONE:  prom_math_read = 32'h3f800000;
        PROM_MATH_PI: prom_math_read = 32'h40490fdb;
        PROM_MATH_PI_2: prom_math_read = 32'h3fc90fdb;
        PROM_MATH_NEG_PI: prom_math_read = 32'hc0490fdb;
        PROM_MATH_NEG_PI_2: prom_math_read = 32'hbfc90fdb;
        PROM_MATH_PI_4: prom_math_read = 32'h3f490fdb;
        PROM_MATH_NEG_PI_4: prom_math_read = 32'hbf490fdb;
        PROM_MATH_3PI_4: prom_math_read = 32'h4016cbe4;
        PROM_MATH_NEG_3PI_4: prom_math_read = 32'hc016cbe4;
        PROM_MATH_SIN_C3: prom_math_read = 32'hbe2aaaab;
        PROM_MATH_SIN_C5: prom_math_read = 32'h3c088889;
        PROM_MATH_SIN_C7: prom_math_read = 32'hb9500d01;
        PROM_MATH_COS_C2: prom_math_read = 32'hbf000000;
        PROM_MATH_COS_C4: prom_math_read = 32'h3d2aaaab;
        PROM_MATH_COS_C6: prom_math_read = 32'hbab60b61;
        PROM_MATH_TWO_PI: prom_math_read = 32'h40c90fdb;
        default:        prom_math_read = 32'h00000000;
    endcase
endfunction

// These helpers operate on finite FP32 words. The range limit bounds the
// number of single-turn MAC reductions and rejects NaN and infinity.
function automatic logic angle_supported(input logic [31:0] angle);
    angle_supported = angle[30:0] <= 31'h43c90fdb; // 128*pi
endfunction

function automatic logic angle_needs_wrap(input logic [31:0] angle);
    angle_needs_wrap = angle[31]
                     ? angle[30:0] > 31'h40490fdb
                     : angle[30:0] >= 31'h40490fdb;
endfunction

// Largest binary multiple of FP32 2*pi no greater than |angle|, or
// one turn for the final reduction from [pi, 2*pi).  Only the exponent
// changes. Subtracting this operand with the MAC is exact by Sterbenz's
// lemma, and clears one binary digit of the remaining number of turns.
function automatic logic [31:0] angle_wrap_step(input logic [31:0] angle);
    logic [7:0] exponent;
    exponent = angle[30:23];
    if (exponent <= 8'd129)
        exponent = 8'd129;
    else if (angle[22:0] < 23'h490fdb)
        exponent = exponent - 8'd1;
    angle_wrap_step = {1'b0, exponent, 23'h490fdb};
endfunction

function automatic logic fp32_lt(input logic [31:0] a, b);
    if (a[30:0] == 0 && b[30:0] == 0)
        fp32_lt = 1'b0;
    else if (a[31] != b[31])
        fp32_lt = a[31];
    else
        fp32_lt = a[31] ? a[30:0] > b[30:0] : a[30:0] < b[30:0];
endfunction

typedef struct packed {
    logic ukf_pulse;        // measurement snapshot strobe
    logic [31:0] ukf_beat_ts;      // time of the beat this snapshot belongs to, us
    logic ukf_vn_valid;     // one-clock correction strobe for a good VN message
    logic [31:0] ukf_vn_yaw_f32;   // raw IEEE float: yaw, degrees, clockwise from north
    logic [31:0] ukf_vn_rate_f32;  // raw IEEE float: body Z turn rate, rad/s
    logic        [63:0] ukf_vn_lat_f64;   // raw IEEE double: latitude, degrees
    logic        [63:0] ukf_vn_lon_f64;   // raw IEEE double: longitude, degrees
    logic ukf_dw_valid;    // one-clock prediction strobe for a good wheel delta
    logic signed [27:0] ukf_dw_delta;     // counts moved since the last good reading
} meas_regfile_t;

// FP32 measurements in the same local frame and units as the prediction.
typedef struct packed {
    logic [31:0] x;
    logic [31:0] y;
    logic [31:0] theta;
    logic [31:0] omega;
} corr_measurement_t;

// FP32 upper triangle in row-major order: S11..S15, S22..S25, ..., S55.
typedef logic [14:0][31:0] pred_s_factor_t;

// FP32 upper triangle of the full 4x4 noise factor: SR11..SR14, ..., SR44.
typedef logic [9:0][31:0] meas_s_factor_t;

typedef struct packed{
    logic [31:0] ukf_x;
    logic [31:0] ukf_y;
    logic [31:0] ukf_theta;
    logic [31:0] ukf_omega;
    logic [31:0] ukf_wheelB;
    logic ukf_valid;
}UKF_OUT_PACKET;

typedef struct packed{
    logic [31:0] pred_x;
    logic [31:0] pred_y;
    logic [31:0] pred_theta;
    logic [31:0] pred_omega;
    logic [31:0] pred_wheelB;
    logic pred_valid;
}PRED_PACKET;

// Prediction angle microcode entry points and scratch addresses.
localparam logic [8:0] PRED_ANGLE_WRAP   = 9'd416;
localparam logic [8:0] PRED_ANGLE_SIN    = 9'd426;
localparam logic [8:0] PRED_ANGLE_COS    = 9'd444;
localparam logic [6:0] PRED_ANGLE_ARG    = 7'd33;
localparam logic [6:0] PRED_ANGLE_RESULT = 7'd27;

typedef enum logic [2:0] { 
    UOP_MAC, UOP_ADD, UOP_DIV, UOP_DONE, UOP_NOP, UOP_SQRT, UOP_TRIG, UOP_WRAP
} dp_uop;

typedef enum logic [2:0] { 
    SEQ_NEXT, SEQ_JUMP, SEQ_LOOP, SEQ_HALT, SEQ_CALL, SEQ_RET, SEQ_FP_LT, SEQ_ANGLE_CHECK
 } pc_uop;


typedef struct packed {
    dp_uop dp; //datapath unit accessing
    pc_uop seq; //pc changing
    logic clr_acc; //clear MAC accumulator
    logic wb_en;  
    logic neg_b;  //write to regfile
    logic [6:0] src_a;   // register or PROM address
    logic [6:0] src_b; 
    logic [6:0] dest;
    logic [6:0] targ_pc; //size of correction engine program
    logic instr_done; //wait until uop finishes
} corr_ukf_uop;

typedef struct packed {
    dp_uop dp; //datapath unit accessing
    pc_uop seq; //pc changing
    logic clr_acc; //clear MAC accumulator
    logic wb_en;    //write to regfile
    logic neg_b; 
    logic [6:0] src_a;   //change depedning on size of regfile
    logic [6:0] src_b; 
    logic [6:0] dest;
    logic [8:0] targ_pc;
    //logic [3:0] sigma_idx; //indexing into sigma point block
    logic sig_pt_idx; //offset address by sigma_idx
    logic instr_done; //wait until uop finishes
} pred_ukf_uop;

typedef struct packed {
    logic start, clear, last;
    logic [31:0] a, b;
} ukf_mac_request_t;

typedef struct packed {
    logic busy, valid;
    logic [31:0] result;
} ukf_mac_response_t;

typedef struct packed {
    logic start, sqrt_mode;
    logic [31:0] a, b;
} ukf_divsqrt_request_t;

typedef struct packed {
    logic busy, done, invalid, divide_by_zero, overflow;
    logic [31:0] result;
} ukf_divsqrt_response_t;

typedef struct packed {
    logic active, write_enable;
    logic [6:0] read_addr_a, read_addr_b, write_addr;
    logic [31:0] write_data;
} ukf_spad_request_t;

typedef struct packed {
    logic [31:0] a, b;
    logic heading_negative;
} ukf_spad_response_t;
