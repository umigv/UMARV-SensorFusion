// Shared read-only address map. The measurement-noise factor occupies 83..92.
localparam logic [6:0] PROM_SR_BASE   = 7'd83;
localparam logic [6:0] PROM_MATH_BASE = 7'd93;
localparam logic [6:0] PROM_MATH_ZERO = PROM_MATH_BASE;
localparam logic [6:0] PROM_MATH_ONE  = PROM_MATH_BASE + 7'd1;

// Prediction currently declares the rest of the math layout without values.
// Reserve those slots until their FP32 encodings/configuration are defined.
function automatic logic [31:0] prom_math_read(input logic [6:0] addr);
    case (addr)
        PROM_MATH_ZERO: prom_math_read = 32'h00000000;
        PROM_MATH_ONE:  prom_math_read = 32'h3f800000;
        default:        prom_math_read = 32'h00000000;
    endcase
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

typedef enum logic [2:0] { 
    UOP_MAC, UOP_ADD, UOP_DIV, UOP_DONE, UOP_NOP, UOP_SQRT, UOP_TRIG, UOP_WRAP
} dp_uop;

typedef enum logic [2:0] { 
    SEQ_NEXT, SEQ_JUMP, SEQ_LOOP, SEQ_HALT, SEQ_CALL, SEQ_RET
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
    logic [6:0] targ_pc; //size of correction engine program
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
