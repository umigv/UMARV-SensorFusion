typedef struct packed {
    logic ukf_pulse;        // one clock: the measurements below are ready
    logic [31:0] ukf_beat_ts;      // time of the beat this snapshot belongs to, us
    logic ukf_vn_valid;     // new, CRC-good, status-good message this beat
    logic [31:0] ukf_vn_yaw_f32;   // raw IEEE float: yaw, degrees, clockwise from north
    logic [31:0] ukf_vn_rate_f32;  // raw IEEE float: body Z turn rate, rad/s
    logic        [63:0] ukf_vn_lat_f64;   // raw IEEE double: latitude, degrees
    logic        [63:0] ukf_vn_lon_f64;   // raw IEEE double: longitude, degrees
    logic ukf_dw_valid;    // good wheel reading this beat, with a previous one to compare against
    logic signed [27:0] ukf_dw_delta;     // counts moved since the last good reading
} meas_regfile_t;

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
    UOP_MAC, UOP_ADD, UOP_DIV, UOP_DONE, UOP_NOP, UOP_SQRT, UOP_TRIG_FIX, UOP_WRAP
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
    logic [6:0] src_a;   //change depedning on size of regfile
    logic [6:0] src_b; 
    logic [6:0] dest;
    logic [5:0] targ_pc; //size of correction engine program
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