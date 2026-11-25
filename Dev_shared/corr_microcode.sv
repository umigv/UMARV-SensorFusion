module corr_microcode #(
    parameter bit ENABLE_SECOND_UPDATE = 1'b1,
    parameter bit ENABLE_THIRD_UPDATE = 1'b1,
    parameter bit ENABLE_FOURTH_UPDATE = 1'b1,
    parameter bit SHARE_MAC = 1'b0,
    parameter bit SHARED_STORAGE = 1'b0,
    parameter logic [31:0] ORIGIN_LATITUDE = 32'b0,
    parameter logic [31:0] ORIGIN_LONGITUDE = 32'b0,
    parameter logic [31:0] EARTH_RADIUS = 32'h4ac26d70,
    parameter logic [160:0] INITIAL_STATE = '0,
    parameter logic [479:0] INITIAL_S = '0
) (
    input  logic clock, reset,
    input  logic start,
    input  logic measurement_load,
    input  ukf_defs::corr_measurement_t measurement,
    input  ukf_defs::PRED_PACKET prediction,
    input  ukf_defs::pred_s_factor_t prediction_s,
    input  ukf_defs::meas_s_factor_t measurement_s,
    input  logic prediction_load,
    output ukf_defs::ukf_spad_request_t spad_request,
    input ukf_defs::ukf_spad_response_t spad_response,
    output logic done, x_update_valid, failed,
    output ukf_defs::UKF_OUT_PACKET result,
    output ukf_defs::pred_s_factor_t corrected_s,
    output logic xy_update_valid, xyz_update_valid,
    input logic sensor_mac_owner,
    input ukf_defs::ukf_mac_request_t sensor_mac_request,
    input logic prediction_div_owner,
    input ukf_defs::ukf_divsqrt_request_t prediction_div_request,
    output ukf_defs::ukf_divsqrt_response_t prediction_div_response,
    output ukf_defs::ukf_mac_response_t sensor_mac_response
);

    localparam logic [6:0] REG_X          = 7'd0;
    localparam logic [6:0] REG_Y          = 7'd1;
    localparam logic [6:0] REG_THETA      = 7'd2;
    localparam logic [6:0] REG_OMEGA      = 7'd3;
    localparam logic [6:0] REG_WHEEL_BIAS = 7'd4;
    // Address 5 is reserved for prediction's wheel velocity.
    localparam logic [6:0] REG_S_BASE     = 7'd6;
    localparam int S_WORDS = 15;
    localparam logic [6:0] REG_MEAS_X     = 7'd21;
    localparam logic [6:0] REG_MEAS_Y     = 7'd22;
    localparam logic [6:0] REG_MEAS_THETA = 7'd23;
    localparam logic [6:0] REG_MEAS_OMEGA = 7'd24;

    localparam logic [6:0] SPAD_BASE    = 7'd25;
    localparam logic [6:0] REG_D        = SPAD_BASE;
    localparam logic [6:0] REG_Q        = SPAD_BASE + 7'd1;
    localparam logic [6:0] REG_G        = SPAD_BASE + 7'd2;
    localparam logic [6:0] REG_TEMP     = SPAD_BASE + 7'd3;
    localparam logic [6:0] REG_U_BASE   = SPAD_BASE + 7'd4;
    localparam logic [6:0] REG_RADICAND = SPAD_BASE + 7'd9;
    localparam logic [6:0] REG_T        = SPAD_BASE + 7'd10;
    localparam logic [6:0] REG_C        = SPAD_BASE + 7'd11;

    // Reuse the downdate temporaries while preparing the whitened row.
    localparam logic [6:0] REG_A   = REG_RADICAND;
    localparam logic [6:0] REG_T1  = REG_A;
    localparam logic [6:0] REG_B   = REG_T;
    localparam logic [6:0] REG_T2  = REG_B;
    localparam logic [6:0] REG_EPS = REG_C;
    // Column 4 consumes t3 before replacing its slot with the last u word.
    localparam logic [6:0] REG_T3 = REG_U_BASE + 7'd4;
    // The last row keeps t4 at 25 and reuses the variance/root slot at 28.
    localparam logic [6:0] REG_T4  = REG_D;
    localparam logic [6:0] REG_S14 = REG_S_BASE + 7'd3;
    localparam logic [6:0] REG_S24 = REG_S_BASE + 7'd7;
    localparam logic [6:0] REG_S34 = REG_S_BASE + 7'd10;
    localparam logic [6:0] REG_S44 = REG_S_BASE + 7'd12;
    localparam logic [6:0] REG_S13 = REG_S_BASE + 7'd2;
    localparam logic [6:0] REG_S23 = REG_S_BASE + 7'd6;
    localparam logic [6:0] REG_S33 = REG_S_BASE + 7'd9;
    localparam logic [6:0] REG_S12 = REG_S_BASE + 7'd1;
    localparam logic [6:0] REG_S22 = REG_S_BASE + 7'd5;
    localparam logic [6:0] SRC_R11 = ukf_defs::PROM_SR_BASE;
    localparam logic [6:0] SRC_R12 = ukf_defs::PROM_SR_BASE + 7'd1;
    localparam logic [6:0] SRC_R22 = ukf_defs::PROM_SR_BASE + 7'd4;
    localparam logic [6:0] SRC_R13 = ukf_defs::PROM_SR_BASE + 7'd2;
    localparam logic [6:0] SRC_R23 = ukf_defs::PROM_SR_BASE + 7'd5;
    localparam logic [6:0] SRC_R33 = ukf_defs::PROM_SR_BASE + 7'd7;

    localparam logic [6:0] SRC_R14 = ukf_defs::PROM_SR_BASE + 7'd3;
    localparam logic [6:0] SRC_R24 = ukf_defs::PROM_SR_BASE + 7'd6;
    localparam logic [6:0] SRC_R34 = ukf_defs::PROM_SR_BASE + 7'd8;
    localparam logic [6:0] SRC_R44 = ukf_defs::PROM_SR_BASE + 7'd9;

    localparam logic [6:0] PC_VARIANCE = 7'd4;
    localparam logic [6:0] PC_VECTOR = 7'd9;
    localparam logic [6:0] PC_VECTOR_NEXT = 7'd10;
    localparam logic [6:0] PC_STATE = 7'd11;
    localparam logic [6:0] PC_STATE_NEXT = 7'd13;
    localparam logic [6:0] PC_DOWNDATE = 7'd14;
    localparam logic [6:0] PC_DIAG_WRITE = 7'd19;
    localparam logic [6:0] PC_OFFDIAG = 7'd20;
    localparam logic [6:0] PC_OFFDIAG_NEXT = 7'd24;
    localparam logic [6:0] PC_X_DONE = 7'd25;
    localparam logic [6:0] PC_WHITEN = 7'd26;
    localparam logic [6:0] PC_XY_VECTOR = 7'd40;
    localparam logic [6:0] PC_XY_VECTOR_NEXT = 7'd42;
    localparam logic [6:0] PC_XY_DONE = 7'd43;
    localparam logic [6:0] PC_HEADING_WHITEN = 7'd44;
    localparam logic [6:0] PC_HEADING_VARIANCE = 7'd59;
    localparam logic [6:0] PC_HEADING_VECTOR = 7'd60;
    localparam logic [6:0] PC_XYZ_DONE = 7'd61;

    localparam logic [6:0] PC_RATE_WHITEN = 7'd62;
    localparam logic [6:0] PC_RATE_VARIANCE = 7'd86;
    localparam logic [6:0] PC_RATE_VECTOR = 7'd87;
    localparam logic [6:0] PC_FULL_DONE = 7'd88;
    localparam logic [6:0] PC_WRAP_INNOVATION = 7'd89;
    localparam logic [6:0] PC_WRAP_HEADING = 7'd90;
    localparam logic [6:0] PC_WRAP_RESIDUAL = 7'd91;
    localparam logic [6:0] PC_CONVERT = 7'd92;
    localparam logic [6:0] SRC_ORIGIN_LON = 7'd115;
    localparam logic [6:0] SRC_ORIGIN_LAT = 7'd116;
    localparam logic [6:0] SRC_DEG_TO_RAD = 7'd117;
    localparam logic [6:0] SRC_EARTH_RADIUS = 7'd118;
    localparam logic [6:0] SRC_COS_COEFF = 7'd119;
    localparam logic [6:0] SRC_COS_SEED = 7'd120;
    logic [6:0] wrap_return_pc;
    logic wrap_step, wrap_needed;

    typedef enum logic [1:0] {UPDATE_X, UPDATE_XY, UPDATE_XYZ, UPDATE_XYZW} update_stage_t;

    typedef enum logic [2:0] {
        IDLE, ISSUE_A, WAIT_A, ISSUE_B, WAIT_B, WAIT_DIVSQRT
    } corr_state_t;

    corr_state_t state;
    logic [6:0] pc;
    logic [2:0] row, col;
    update_stage_t update_stage;
    logic third_term, fourth_term, full_update_valid;
    logic [6:0] diag_addr, factor_addr, row_u, col_u, state_addr, meas_addr;
    logic [6:0] second_row_addr, third_row_addr, fourth_row_addr;
    localparam logic [6:0] RF_LAST = SHARED_STORAGE ? REG_S_BASE + 7'd14 : REG_C;
    logic [31:0] regfile [RF_LAST:REG_X];
    logic residual_heading_negative;
    ukf_defs::corr_ukf_uop uop;
    logic [31:0] operand_a, operand_b, mac_a, mac_b, mac_result, div_result;
    logic mac_start, mac_busy, mac_done, mac_result_valid;
    logic div_start, div_busy, div_done, div_invalid, div_zero, div_overflow;
    logic bad_operands, arithmetic_complete, arithmetic_failed;
    logic [31:0] arithmetic_result;
    logic [S_WORDS*32-1:0] prediction_words, corrected_words;
    logic [319:0] measurement_words;
    assign prediction_words = prediction_s;
    assign measurement_words = measurement_s;
    assign corrected_s = corrected_words;

    // Packed upper-triangular rows start at offsets 0, 5, 9, 12, 14.
    always_comb begin
        case (row)
            3'd0: diag_addr = REG_S_BASE;
            3'd1: diag_addr = REG_S_BASE + 7'd5;
            3'd2: diag_addr = REG_S_BASE + 7'd9;
            3'd3: diag_addr = REG_S_BASE + 7'd12;
            default: diag_addr = REG_S_BASE + 7'd14;
        endcase
    end
    assign factor_addr = diag_addr + {4'b0, (col - row)};
    assign row_u = REG_U_BASE + {4'b0, row};
    assign col_u = REG_U_BASE + {4'b0, col};
    assign state_addr = REG_X + {4'b0, col};
    assign meas_addr = REG_MEAS_X + {4'b0, col};
    assign second_row_addr = REG_S22 - 7'd1 + {4'b0, col};
    assign third_row_addr = REG_S33 - 7'd2 + {4'b0, col};
    assign fourth_row_addr = REG_S44 - 7'd3 + {4'b0, col};
    assign third_term = (update_stage == UPDATE_XYZ || update_stage == UPDATE_XYZW) && col >= 3'd2;
    assign fourth_term = update_stage == UPDATE_XYZW && col >= 3'd3;

    always_comb begin
        uop = '0;
        uop.seq = ukf_defs::SEQ_NEXT;
        uop.clr_acc = 1'b1;
        uop.wb_en = 1'b1;
        case (pc)
            // Consume the normalized measurements in place.
            7'd0: begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_X; uop.src_b = REG_X; uop.dest = REG_MEAS_X; end
            7'd1: begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_Y; uop.src_b = REG_Y; uop.dest = REG_MEAS_Y; end
            7'd2: begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_THETA; uop.src_b = REG_THETA; uop.dest = REG_MEAS_THETA; end
            7'd3: begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_OMEGA; uop.src_b = REG_OMEGA; uop.dest = REG_MEAS_OMEGA; end

            PC_WRAP_INNOVATION, PC_WRAP_HEADING, PC_WRAP_RESIDUAL: begin
                uop.dp = ukf_defs::UOP_ADD;
                uop.src_a = pc == PC_WRAP_HEADING ? REG_THETA : REG_MEAS_THETA;
                uop.src_b = ukf_defs::PROM_MATH_TWO_PI;
                uop.dest = uop.src_a;
                uop.neg_b = pc == PC_WRAP_HEADING ? !regfile[REG_THETA][31]
                                                  : !residual_heading_negative;
            end

            // d, sqrt(d), q, g. The root in REG_TEMP dies when q is written.
            PC_VARIANCE: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S_BASE; uop.src_b = REG_S_BASE; uop.wb_en = 1'b0; end
            7'd5: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SRC_R11; uop.src_b = SRC_R11; uop.dest = REG_D; end
            7'd6: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = REG_D; uop.dest = REG_TEMP; end
            7'd7: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = REG_TEMP; uop.dest = REG_Q; end
            7'd8: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_MEAS_X; uop.src_b = REG_Q; uop.dest = REG_G; end

            // Form one component of u at a time, using the original row 0.
            PC_VECTOR: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S_BASE; uop.src_b = factor_addr; uop.dest = REG_TEMP; end
            PC_VECTOR_NEXT: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_TEMP; uop.src_b = REG_Q; uop.dest = col_u; uop.seq = ukf_defs::SEQ_LOOP; uop.targ_pc = PC_VECTOR; end

            // Reuse the same rounded delta for the state and its innovation.
            PC_STATE: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = col_u; uop.src_b = REG_G; uop.dest = REG_TEMP; end
            7'd12: begin uop.dp = ukf_defs::UOP_ADD; uop.src_a = state_addr; uop.src_b = REG_TEMP; uop.dest = state_addr; end
            PC_STATE_NEXT: begin
                if (col == 3'd4)
                    uop.dp = ukf_defs::UOP_NOP;
                else
                    uop.dp = ukf_defs::UOP_ADD;
                uop.neg_b = 1'b1;
                uop.src_a = meas_addr;
                uop.src_b = REG_TEMP;
                uop.dest = meas_addr;
                uop.seq = ukf_defs::SEQ_LOOP;
                uop.targ_pc = PC_STATE;
            end

            // Upper-triangular downdate: t=u[k]/S[k,k], r=sqrt(S[k,k]^2-u[k]^2), c=r/S[k,k].
            PC_DOWNDATE: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = row_u; uop.src_b = diag_addr; uop.dest = REG_T; end
            7'd15: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = diag_addr; uop.src_b = diag_addr; uop.wb_en = 1'b0; end
            7'd16: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.neg_b = 1'b1; uop.src_a = row_u; uop.src_b = row_u; uop.dest = REG_RADICAND; end
            7'd17: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = REG_RADICAND; uop.dest = REG_TEMP; end
            7'd18: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = REG_TEMP; uop.src_b = diag_addr; uop.dest = REG_C; end
            PC_DIAG_WRITE: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_TEMP; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = diag_addr; end

            // S[k,j]=(S[k,j]-t*u[j])/c, then u[j]=c*u[j]-t*S[k,j].
            PC_OFFDIAG: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = factor_addr; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.wb_en = 1'b0; end
            7'd21: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.neg_b = 1'b1; uop.src_a = REG_T; uop.src_b = col_u; uop.dest = REG_TEMP; end
            7'd22: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = REG_TEMP; uop.src_b = REG_C; uop.dest = factor_addr; end
            7'd23: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_C; uop.src_b = col_u; uop.wb_en = 1'b0; end
            PC_OFFDIAG_NEXT: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.neg_b = 1'b1; uop.src_a = REG_T; uop.src_b = factor_addr; uop.dest = col_u; uop.seq = ukf_defs::SEQ_LOOP; uop.targ_pc = PC_OFFDIAG; end

            PC_X_DONE, PC_XY_DONE, PC_XYZ_DONE, PC_FULL_DONE: begin uop.dp = ukf_defs::UOP_DONE; uop.seq = ukf_defs::SEQ_HALT; uop.wb_en = 1'b0; end

            // Whiten row 2 of SR^T against the posterior from the x stage.
            PC_WHITEN: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_R11; uop.src_b = SRC_R22; uop.dest = REG_TEMP; end
            7'd27: begin uop.dp = ukf_defs::UOP_DIV; uop.neg_b = 1'b1; uop.src_a = SRC_R12; uop.src_b = REG_TEMP; uop.dest = REG_A; end
            7'd28: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = SRC_R22; uop.dest = REG_B; end
            7'd29: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_A; uop.src_b = REG_MEAS_X; uop.wb_en = 1'b0; end
            7'd30: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_B; uop.src_b = REG_MEAS_Y; uop.dest = REG_EPS; end
            7'd31: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S_BASE; uop.src_b = REG_A; uop.wb_en = 1'b0; end
            7'd32: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S12; uop.src_b = REG_B; uop.dest = REG_T1; end
            7'd33: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S22; uop.src_b = REG_B; uop.dest = REG_T2; end

            // The whitened row has unit noise variance.
            7'd34: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.wb_en = 1'b0; end
            7'd35: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T1; uop.src_b = REG_T1; uop.wb_en = 1'b0; end
            7'd36: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T2; uop.src_b = REG_T2; uop.dest = REG_D; uop.wb_en = update_stage == UPDATE_XY; end
            7'd37: begin uop.dp = ukf_defs::UOP_SQRT; uop.src_a = update_stage == UPDATE_XYZW ? REG_TEMP : REG_D; uop.dest = REG_TEMP; end
            7'd38: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = REG_TEMP; uop.dest = REG_Q; end
            7'd39: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_EPS; uop.src_b = REG_Q; uop.dest = REG_G; end

            // Row 2 has no entry for column 0. For other columns the first
            // product stays in the MAC until the second product is accumulated.
            PC_XY_VECTOR: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = factor_addr; uop.src_b = REG_T1; uop.wb_en = col == 3'd0; uop.dest = REG_TEMP; end
            7'd41: begin
                if (col == 3'd0)
                    uop.dp = ukf_defs::UOP_NOP;
                else
                    uop.dp = ukf_defs::UOP_MAC;
                uop.clr_acc = 1'b0;
                uop.src_a = second_row_addr;
                uop.src_b = REG_T2;
                uop.dest = REG_TEMP;
                uop.wb_en = !third_term;
            end
            PC_XY_VECTOR_NEXT: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_TEMP; uop.src_b = REG_Q; uop.dest = col_u; uop.seq = ukf_defs::SEQ_LOOP; uop.targ_pc = PC_XY_VECTOR; end
            // Back-substitute SR against the third unit vector. REG_T3 holds c
            // until the last projection that uses it.
            PC_HEADING_WHITEN: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = SRC_R33; uop.dest = REG_T3; end
            7'd45: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_R23; uop.src_b = REG_T3; uop.dest = REG_TEMP; end
            7'd46: begin uop.dp = ukf_defs::UOP_DIV; uop.neg_b = 1'b1; uop.src_a = REG_TEMP; uop.src_b = SRC_R22; uop.dest = REG_B; end
            7'd47: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_R12; uop.src_b = REG_B; uop.wb_en = 1'b0; end
            7'd48: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SRC_R13; uop.src_b = REG_T3; uop.dest = REG_TEMP; end
            7'd49: begin uop.dp = ukf_defs::UOP_DIV; uop.neg_b = 1'b1; uop.src_a = REG_TEMP; uop.src_b = SRC_R11; uop.dest = REG_A; end

            // Form epsilon before overwriting the whitening coefficients with Sh.
            7'd50: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_A; uop.src_b = REG_MEAS_X; uop.wb_en = 1'b0; end
            7'd51: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_B; uop.src_b = REG_MEAS_Y; uop.wb_en = 1'b0; end
            7'd52: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T3; uop.src_b = REG_MEAS_THETA; uop.dest = REG_EPS; end
            7'd53: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S_BASE; uop.src_b = REG_A; uop.wb_en = 1'b0; end
            7'd54: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S12; uop.src_b = REG_B; uop.wb_en = 1'b0; end
            7'd55: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S13; uop.src_b = REG_T3; uop.dest = REG_T1; end
            7'd56: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S22; uop.src_b = REG_B; uop.wb_en = 1'b0; end
            7'd57: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S23; uop.src_b = REG_T3; uop.dest = REG_T2; end
            7'd58: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S33; uop.src_b = REG_T3; uop.dest = REG_T3; end
            PC_HEADING_VARIANCE: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T3; uop.src_b = REG_T3; uop.dest = REG_D; uop.wb_en = update_stage != UPDATE_XYZW; end
            PC_HEADING_VECTOR: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = third_row_addr; uop.src_b = REG_T3; uop.dest = REG_TEMP; uop.wb_en = !fourth_term; end

            // Solve SR*h = e4 by backward substitution using one MAC and divider.
            PC_RATE_WHITEN: begin uop.dp = ukf_defs::UOP_DIV; uop.src_a = ukf_defs::PROM_MATH_ONE; uop.src_b = SRC_R44; uop.dest = REG_T4; end
            7'd63: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_R34; uop.src_b = REG_T4; uop.dest = REG_TEMP; end
            7'd64: begin uop.dp = ukf_defs::UOP_DIV; uop.neg_b = 1'b1; uop.src_a = REG_TEMP; uop.src_b = SRC_R33; uop.dest = REG_T3; end
            7'd65: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_R23; uop.src_b = REG_T3; uop.wb_en = 1'b0; end
            7'd66: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SRC_R24; uop.src_b = REG_T4; uop.dest = REG_TEMP; end
            7'd67: begin uop.dp = ukf_defs::UOP_DIV; uop.neg_b = 1'b1; uop.src_a = REG_TEMP; uop.src_b = SRC_R22; uop.dest = REG_B; end
            7'd68: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_R12; uop.src_b = REG_B; uop.wb_en = 1'b0; end
            7'd69: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SRC_R13; uop.src_b = REG_T3; uop.wb_en = 1'b0; end
            7'd70: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = SRC_R14; uop.src_b = REG_T4; uop.dest = REG_TEMP; end
            7'd71: begin uop.dp = ukf_defs::UOP_DIV; uop.neg_b = 1'b1; uop.src_a = REG_TEMP; uop.src_b = SRC_R11; uop.dest = REG_A; end

            // Compute the innovation, then replace each coefficient after its last use.
            7'd72: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_A; uop.src_b = REG_MEAS_X; uop.wb_en = 1'b0; end
            7'd73: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_B; uop.src_b = REG_MEAS_Y; uop.wb_en = 1'b0; end
            7'd74: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T3; uop.src_b = REG_MEAS_THETA; uop.wb_en = 1'b0; end
            7'd75: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T4; uop.src_b = REG_MEAS_OMEGA; uop.dest = REG_EPS; end
            7'd76: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S_BASE; uop.src_b = REG_A; uop.wb_en = 1'b0; end
            7'd77: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S12; uop.src_b = REG_B; uop.wb_en = 1'b0; end
            7'd78: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S13; uop.src_b = REG_T3; uop.wb_en = 1'b0; end
            7'd79: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S14; uop.src_b = REG_T4; uop.dest = REG_T1; end
            7'd80: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S22; uop.src_b = REG_B; uop.wb_en = 1'b0; end
            7'd81: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S23; uop.src_b = REG_T3; uop.wb_en = 1'b0; end
            7'd82: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S24; uop.src_b = REG_T4; uop.dest = REG_T2; end
            7'd83: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S33; uop.src_b = REG_T3; uop.wb_en = 1'b0; end
            7'd84: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_S34; uop.src_b = REG_T4; uop.dest = REG_T3; end
            7'd85: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_S44; uop.src_b = REG_T4; uop.dest = REG_T4; end
            PC_RATE_VARIANCE: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T4; uop.src_b = REG_T4; uop.dest = REG_TEMP; end
            PC_RATE_VECTOR: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = fourth_row_addr; uop.src_b = REG_T4; uop.dest = REG_TEMP; end

            // Convert SPAD[0..3] from (lat_f32, lon_f32, yaw_f32, -rate_f32) to (x, y, theta, omega).
            PC_CONVERT: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_ORIGIN_LAT; uop.src_b = SRC_DEG_TO_RAD; uop.dest = REG_T1; end
            7'd93: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_T1; uop.src_b = REG_T1; uop.dest = REG_T1; end
            7'd94: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_COS_SEED; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_T2; end
            7'd95: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_COS_COEFF; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.wb_en = 1'b0; end
            7'd96: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.src_a = REG_T2; uop.src_b = REG_T1; uop.dest = REG_T2; end
            7'd97: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = SRC_EARTH_RADIUS; uop.src_b = SRC_DEG_TO_RAD; uop.dest = REG_T1; end
            7'd98: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_T2; uop.src_b = REG_T1; uop.dest = REG_T2; end
            7'd99: begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_Y; uop.src_b = SRC_ORIGIN_LON; uop.dest = REG_T3; end
            7'd100: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_T3; uop.src_b = REG_T2; uop.dest = REG_T3; end
            7'd101: begin uop.dp = ukf_defs::UOP_ADD; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_X; uop.src_b = SRC_ORIGIN_LAT; uop.dest = REG_MEAS_Y; end
            7'd102: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_MEAS_Y; uop.src_b = REG_T1; uop.dest = REG_MEAS_Y; end
            7'd103: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = REG_T3; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.dest = REG_MEAS_X; end
            7'd104: begin uop.dp = ukf_defs::UOP_MAC; uop.src_a = ukf_defs::PROM_MATH_PI_2; uop.src_b = ukf_defs::PROM_MATH_ONE; uop.wb_en = 1'b0; end
            7'd105: begin uop.dp = ukf_defs::UOP_MAC; uop.clr_acc = 1'b0; uop.neg_b = 1'b1; uop.src_a = REG_MEAS_THETA; uop.src_b = SRC_DEG_TO_RAD; uop.dest = REG_MEAS_THETA; end
            default: begin uop.dp = ukf_defs::UOP_NOP; uop.wb_en = 1'b0; end
        endcase
    end

    // Noise words are packed in upper-triangular row order. Other words
    // stay available at the interface for the later whitened measurements.
    function automatic logic [31:0] read_operand(
        input logic [6:0] addr,
        input logic [31:0] register_word,
        input logic [319:0] noise_words
    );
        case (addr)
            SRC_R11: read_operand = noise_words[0*32 +: 32];
            SRC_R12: read_operand = noise_words[1*32 +: 32];
            SRC_R22: read_operand = noise_words[4*32 +: 32];
            SRC_R13: read_operand = noise_words[2*32 +: 32];
            SRC_R23: read_operand = noise_words[5*32 +: 32];
            SRC_R33: read_operand = noise_words[7*32 +: 32];
            SRC_R14: read_operand = noise_words[3*32 +: 32];
            SRC_R24: read_operand = noise_words[6*32 +: 32];
            SRC_R34: read_operand = noise_words[8*32 +: 32];
            SRC_R44: read_operand = noise_words[9*32 +: 32];
            SRC_ORIGIN_LON: read_operand = ORIGIN_LONGITUDE;
            SRC_ORIGIN_LAT: read_operand = ORIGIN_LATITUDE;
            SRC_DEG_TO_RAD: read_operand = 32'h3c8efa35;
            SRC_EARTH_RADIUS: read_operand = EARTH_RADIUS;
            SRC_COS_SEED: read_operand = 32'h310f76c7;
            SRC_COS_COEFF: case (col)
                3'd0: read_operand = 32'hb493f27e;
                3'd1: read_operand = 32'h37d00d01;
                3'd2: read_operand = 32'hbab60b61;
                3'd3: read_operand = 32'h3d2aaaab;
                3'd4: read_operand = 32'hbf000000;
                default: read_operand = 32'h3f800000;
            endcase
            default: read_operand = addr >= ukf_defs::PROM_MATH_BASE ? ukf_defs::prom_math_read(addr) : register_word;
        endcase
    endfunction
    generate
        if (SHARED_STORAGE)
            assign residual_heading_negative = spad_response.heading_negative;
        else
            assign residual_heading_negative = regfile[REG_MEAS_THETA][31];
    endgenerate
    // Correction addresses 21..36 map to the same sixteen words as
    // prediction's scratchpad. The standalone build keeps local storage.
    assign spad_request = {
        SHARED_STORAGE && state != IDLE,
        SHARED_STORAGE && arithmetic_complete && !arithmetic_failed && uop.wb_en &&
            uop.dest >= REG_MEAS_X && uop.dest <= REG_C,
        uop.src_a[3:0] - REG_MEAS_X[3:0],
        uop.src_b[3:0] - REG_MEAS_X[3:0],
        uop.dest[3:0] - REG_MEAS_X[3:0], arithmetic_result
    };
    assign operand_a = read_operand(uop.src_a,
                                    SHARED_STORAGE && uop.src_a >= REG_MEAS_X ? spad_response.a :
                                    uop.src_a <= RF_LAST ? regfile[uop.src_a] : 32'b0,
                                    measurement_words);
    assign operand_b = wrap_step ? ukf_defs::angle_wrap_step(operand_a) : read_operand(uop.src_b,
                                    SHARED_STORAGE && uop.src_b >= REG_MEAS_X ? spad_response.b :
                                    uop.src_b <= RF_LAST ? regfile[uop.src_b] : 32'b0,
                                    measurement_words);

    function automatic logic positive_finite(input logic [31:0] word);
        positive_finite = !word[31] && (|word[30:0]) && word[30:23] != 8'hff;
    endfunction

    assign wrap_step = pc == PC_WRAP_INNOVATION || pc == PC_WRAP_HEADING ||
                       pc == PC_WRAP_RESIDUAL;
    assign wrap_needed = ukf_defs::angle_needs_wrap(operand_a);

    // Reject unsupported angles and invalid downdates before issuing arithmetic.
    assign bad_operands = (wrap_step && !ukf_defs::angle_supported(operand_a)) ||
                          (uop.dp == ukf_defs::UOP_SQRT && !positive_finite(operand_a)) ||
                          (uop.dp == ukf_defs::UOP_DIV && !positive_finite(operand_b)) ||
                          (pc == PC_DOWNDATE && operand_a[30:0] >= operand_b[30:0]) ||
                          (pc == PC_VARIANCE && !positive_finite(operand_a)) ||
                          (pc == 7'd5 && !positive_finite(operand_a)) ||
                          (pc == PC_WHITEN && !positive_finite(operand_b));

    assign mac_start = !mac_busy && !bad_operands && (!wrap_step || wrap_needed) &&
                       ((state == ISSUE_A && (uop.dp == ukf_defs::UOP_ADD || uop.dp == ukf_defs::UOP_MAC)) ||
                        state == ISSUE_B);
    assign mac_a = state == ISSUE_B ? {operand_b[31] ^ uop.neg_b, operand_b[30:0]}
                 : (pc == 7'd98 && operand_a[31] ? 32'b0
                 : {operand_a[31] ^ (uop.dp == ukf_defs::UOP_MAC && uop.neg_b), operand_a[30:0]});
    assign mac_b = uop.dp == ukf_defs::UOP_ADD ? ukf_defs::prom_math_read(ukf_defs::PROM_MATH_ONE) : operand_b;

    ukf_defs::ukf_mac_request_t correction_mac_request, active_mac_request;
    assign correction_mac_request = {mac_start, state == ISSUE_A && uop.clr_acc,
                                     state == ISSUE_B || (uop.dp == ukf_defs::UOP_MAC && uop.wb_en),
                                     mac_a, mac_b};
    assign active_mac_request = SHARE_MAC && sensor_mac_owner
                              ? sensor_mac_request : correction_mac_request;
    assign sensor_mac_response = {mac_busy, mac_result_valid, mac_result};

    // The merged MAC reset input is active low.
    fp32_mac residual_mac (
        .clock(clock), .reset(!reset), .start(active_mac_request.start),
        .clear(active_mac_request.clear), .last(active_mac_request.last),
        .a(active_mac_request.a), .b(active_mac_request.b), .busy(mac_busy), .done(mac_done),
        .result_valid(mac_result_valid), .result(mac_result), .exception_flags()
    );

    assign div_start = state == ISSUE_A && !div_busy && !bad_operands &&
                       (uop.dp == ukf_defs::UOP_DIV || uop.dp == ukf_defs::UOP_SQRT);
    ukf_defs::ukf_divsqrt_request_t active_div_request;
    assign active_div_request = SHARE_MAC && prediction_div_owner ? prediction_div_request :
        {div_start, uop.dp == ukf_defs::UOP_SQRT,
         operand_a[31] ^ (uop.dp == ukf_defs::UOP_DIV && uop.neg_b), operand_a[30:0], operand_b};
    assign prediction_div_response = {div_busy, div_done, div_invalid, div_zero,
                                      div_overflow, div_result};
    fp32_divsqrt correction_divsqrt (
        .clk(clock), .rst_n(!reset), .start(active_div_request.start),
        .mode(active_div_request.sqrt_mode), .a(active_div_request.a), .b(active_div_request.b),
        .busy(div_busy), .done(div_done), .result(div_result),
        .invalid(div_invalid), .divide_by_zero(div_zero), .overflow(div_overflow),
        .underflow(), .inexact()
    );

    assign arithmetic_complete = (state == WAIT_A && uop.dp == ukf_defs::UOP_MAC &&
                                   (uop.wb_en ? mac_result_valid : mac_done)) ||
                                 (state == WAIT_B && mac_result_valid) ||
                                 (state == WAIT_DIVSQRT && div_done);
    assign arithmetic_result = state == WAIT_DIVSQRT ? div_result : mac_result;
    assign arithmetic_failed = (uop.wb_en && arithmetic_result[30:23] == 8'hff) ||
                               (state == WAIT_DIVSQRT && (div_invalid || div_zero || div_overflow));

    // Sample the state and factor together on the corresponding stage pulse.
    // UKF validity pulses only on successful completion of the fourth row.
    assign result = {regfile[REG_X], regfile[REG_Y], regfile[REG_THETA],
                     regfile[REG_OMEGA], regfile[REG_WHEEL_BIAS], full_update_valid};
    for (genvar i = 0; i < S_WORDS; i++) begin : export_factor
        assign corrected_words[i*32 +: 32] = regfile[REG_S_BASE + i];
    end

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            for (int addr = REG_X; addr <= RF_LAST; addr++)
                regfile[addr] <= '0;
            if (SHARED_STORAGE) begin
                {regfile[REG_X], regfile[REG_Y], regfile[REG_THETA],
                 regfile[REG_OMEGA], regfile[REG_WHEEL_BIAS]} <= INITIAL_STATE[160:1];
                for (int i = 0; i < S_WORDS; i++)
                    regfile[REG_S_BASE + i] <= INITIAL_S[32*i +: 32];
            end
            state <= IDLE;
            pc <= '0;
            wrap_return_pc <= '0;
            row <= '0;
            col <= '0;
            done <= 1'b0;
            x_update_valid <= 1'b0;
            xy_update_valid <= 1'b0;
            xyz_update_valid <= 1'b0;
            full_update_valid <= 1'b0;
            update_stage <= UPDATE_X;
            failed <= 1'b0;
        end else begin
            done <= 1'b0;
            x_update_valid <= 1'b0;
            xy_update_valid <= 1'b0;
            xyz_update_valid <= 1'b0;
            full_update_valid <= 1'b0;
            failed <= 1'b0;
            // Prediction owns the main bank until done. Load its completed
            // pair on that edge, even on a wheel-only beat.
            if (SHARED_STORAGE && state == IDLE && prediction_load) begin
                {regfile[REG_X], regfile[REG_Y], regfile[REG_THETA],
                 regfile[REG_OMEGA], regfile[REG_WHEEL_BIAS]} <= prediction[160:1];
                for (int i = 0; i < S_WORDS; i++)
                    regfile[REG_S_BASE + i] <= prediction_words[i*32 +: 32];
            end
            if (!SHARED_STORAGE && state == IDLE && measurement_load) begin
                regfile[REG_MEAS_X] <= measurement.x;
                regfile[REG_MEAS_Y] <= measurement.y;
                regfile[REG_MEAS_THETA] <= measurement.theta;
                regfile[REG_MEAS_OMEGA] <= measurement.omega;
            end
            if ((arithmetic_complete && arithmetic_failed) || (state == ISSUE_A && bad_operands)) begin
                // No rollback bank. A failed in-place update is never published.
                failed <= 1'b1;
                done <= 1'b1;
                state <= IDLE;
            end else if (arithmetic_complete || (state == ISSUE_A && uop.dp == ukf_defs::UOP_NOP)) begin
                if (arithmetic_complete && uop.wb_en && uop.dest <= RF_LAST)
                    regfile[uop.dest] <= arithmetic_result;
                state <= ISSUE_A;
                pc <= pc + 7'd1;
                if (uop.seq == ukf_defs::SEQ_LOOP) begin
                    if (col == 3'd4) begin
                        col <= '0;
                        if (pc == PC_OFFDIAG_NEXT) begin
                            row <= row + 3'd1;
                            pc <= PC_DOWNDATE;
                        end else if (pc == PC_XY_VECTOR_NEXT) begin
                            pc <= PC_STATE;
                        end
                    end else begin
                        col <= col + 3'd1;
                        pc <= uop.targ_pc;
                    end
                end
                case (pc)
                    7'd2: begin
                        wrap_return_pc <= 7'd3;
                        pc <= PC_WRAP_INNOVATION;
                    end
                    7'd12: if (col == 3'd2) begin
                        wrap_return_pc <= PC_STATE_NEXT;
                        pc <= PC_WRAP_HEADING;
                    end
                    PC_STATE_NEXT: if (col == 3'd2) begin
                        wrap_return_pc <= PC_STATE;
                        pc <= PC_WRAP_RESIDUAL;
                    end
                    PC_WRAP_INNOVATION, PC_WRAP_HEADING, PC_WRAP_RESIDUAL: pc <= pc;
                    PC_DIAG_WRITE: begin
                        if (row == 3'd4) begin
                            case (update_stage)
                                UPDATE_XY:  pc <= PC_XY_DONE;
                                UPDATE_XYZ: pc <= PC_XYZ_DONE;
                                UPDATE_XYZW: pc <= PC_FULL_DONE;
                                default:    pc <= PC_X_DONE;
                            endcase
                        end else
                            col <= row + 3'd1;
                    end
                    7'd58, 7'd85: pc <= 7'd34;
                    7'd36: if (update_stage != UPDATE_XY)
                        pc <= PC_HEADING_VARIANCE;
                    PC_HEADING_VARIANCE: pc <= update_stage == UPDATE_XYZW ? PC_RATE_VARIANCE : 7'd37;
                    PC_RATE_VARIANCE: pc <= 7'd37;
                    7'd41: if (third_term)
                        pc <= PC_HEADING_VECTOR;
                    PC_HEADING_VECTOR: pc <= fourth_term ? PC_RATE_VECTOR : PC_XY_VECTOR_NEXT;
                    PC_RATE_VECTOR: pc <= PC_XY_VECTOR_NEXT;
                    7'd96: begin
                        if (col == 3'd5)
                            col <= 3'd0;
                        else begin
                            col <= col + 3'd1;
                            pc <= 7'd95;
                        end
                    end
                    7'd105: pc <= 7'd0;
                    default: begin end
                endcase
            end else begin
                case (state)
                    IDLE: if (start && (SHARED_STORAGE || prediction.pred_valid)) begin
                        if (!SHARED_STORAGE) begin
                            {regfile[REG_X], regfile[REG_Y], regfile[REG_THETA],
                             regfile[REG_OMEGA], regfile[REG_WHEEL_BIAS]} <= prediction[160:1];
                            for (int i = 0; i < S_WORDS; i++)
                                regfile[REG_S_BASE + i] <= prediction_words[i*32 +: 32];
                        end
                        pc <= SHARED_STORAGE ? PC_CONVERT : 7'd0;
                        row <= '0;
                        col <= '0;
                        update_stage <= UPDATE_X;
                        state <= ISSUE_A;
                    end
                    ISSUE_A: begin
                        if (wrap_step && !wrap_needed) begin
                            pc <= wrap_return_pc;
                        end else if (uop.seq == ukf_defs::SEQ_HALT) begin
                            x_update_valid <= pc == PC_X_DONE;
                            xy_update_valid <= pc == PC_XY_DONE;
                            xyz_update_valid <= pc == PC_XYZ_DONE;
                            if (ENABLE_SECOND_UPDATE && pc == PC_X_DONE) begin
                                update_stage <= UPDATE_XY;
                                row <= '0;
                                col <= '0;
                                pc <= PC_WHITEN;
                            end else if (ENABLE_THIRD_UPDATE && pc == PC_XY_DONE) begin
                                update_stage <= UPDATE_XYZ;
                                row <= '0;
                                col <= '0;
                                pc <= PC_HEADING_WHITEN;
                            end else if (ENABLE_FOURTH_UPDATE && pc == PC_XYZ_DONE) begin
                                update_stage <= UPDATE_XYZW;
                                row <= '0;
                                col <= '0;
                                pc <= PC_RATE_WHITEN;
                            end else begin
                                full_update_valid <= pc == PC_FULL_DONE;
                                done <= 1'b1;
                                state <= IDLE;
                            end
                        end else if (mac_start) begin
                            state <= WAIT_A;
                        end else if (div_start) begin
                            state <= WAIT_DIVSQRT;
                        end
                    end
                    WAIT_A: if (uop.dp == ukf_defs::UOP_ADD && mac_done)
                        state <= ISSUE_B;
                    ISSUE_B: if (mac_start)
                        state <= WAIT_B;
                    default: begin end
                endcase
            end
        end
    end
endmodule

