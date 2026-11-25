module UKF_pred_engine (
    input logic clock, reset,
    input UKF_OUT_PACKET corr_packet,
    input pred_s_factor_t corr_s,
    output logic backend_start, backend_active,
    output UKF_OUT_PACKET backend_prior,
    output pred_s_factor_t backend_prior_s,
    output logic [31:0] backend_ts,
    output logic clear_wheel_velocity,
    output logic backend_wheel_valid,
    input logic backend_done,
    input PRED_PACKET backend_prediction,
    input pred_s_factor_t backend_prediction_s,
    input meas_regfile_t timestamp_pack,
    input  logic [6:0]  velocity_scale_addr,
    input  logic [31:0] mem_rdata,
    output logic [6:0]  mem_addr,
    output logic        mem_we,
    output logic [31:0] mem_wdata,
    output logic        mac_start,
    output logic        mac_clear,
    output logic        mac_last,
    output logic [31:0] mac_a,
    output logic [31:0] mac_b,
    input  logic        mac_busy,
    input  logic        mac_result_valid,
    input  logic [31:0] mac_result,
    output logic busy, done,
    output PRED_PACKET pred_pack,
    output pred_s_factor_t pred_s,
    output logic wheel_mac_owner, failed
);

    typedef enum logic [2:0] {
        IDLE, LOAD_INPUTS, CALC_WHEEL_V, START_PRED, WAIT_PRED
    } pred_states;
    pred_states s, ns;
    logic [31:0] dw_delta_f32;
    logic wheel_velocity_done, accept_snapshot;
    logic signed [27:0] wheel_delta;

    int_to_fp32 wheel_delta_converter (
        .int_in(wheel_delta),
        .fp_out(dw_delta_f32)
    );

    deadwheel_velocity wheel_velocity (
        .clk(clock),
        .rst_n(!reset),
        .start(s == LOAD_INPUTS),
        .dw_delta_f32(dw_delta_f32),
        .velocity_scale_addr(velocity_scale_addr),
        .mem_rdata(mem_rdata),
        .mem_addr(mem_addr),
        .mem_we(mem_we),
        .mem_wdata(mem_wdata),
        .mac_start(mac_start),
        .mac_clear(mac_clear),
        .mac_last(mac_last),
        .mac_a(mac_a),
        .mac_b(mac_b),
        .mac_busy(mac_busy),
        .mac_result_valid(mac_result_valid),
        .mac_result(mac_result),
        .busy(),
        .done(wheel_velocity_done)
    );

    assign busy = s != IDLE;
    assign accept_snapshot = s == IDLE && timestamp_pack.ukf_pulse &&
                             (timestamp_pack.ukf_dw_valid || timestamp_pack.ukf_vn_valid);
    assign clear_wheel_velocity = accept_snapshot;
    assign wheel_mac_owner = s == LOAD_INPUTS || s == CALC_WHEEL_V;
    assign backend_start = !reset && s == START_PRED;
    assign backend_active = !reset && (s == START_PRED || s == WAIT_PRED);
    assign backend_prior = corr_packet;
    assign backend_prior_s = corr_s;
    assign done = !reset && backend_done && (s == START_PRED || s == WAIT_PRED);
    assign failed = done && !backend_prediction.pred_valid;
    assign pred_pack = {backend_prediction.pred_x, backend_prediction.pred_y,
                        backend_prediction.pred_theta, backend_prediction.pred_omega,
                        backend_prediction.pred_wheelB, done && backend_prediction.pred_valid};
    assign pred_s = backend_prediction_s;

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            s <= IDLE;
            wheel_delta <= '0;
            backend_ts <= '0;
            backend_wheel_valid <= 1'b0;
        end else begin
            s <= ns;
            if (accept_snapshot) begin
                wheel_delta <= timestamp_pack.ukf_dw_delta;
                backend_ts <= timestamp_pack.ukf_beat_ts;
                backend_wheel_valid <= timestamp_pack.ukf_dw_valid;
            end
        end
    end

    always_comb begin
        ns = s;
        case (s)
            IDLE: if (accept_snapshot) begin
                if (timestamp_pack.ukf_dw_valid)
                    ns = LOAD_INPUTS;
                else
                    ns = START_PRED;
            end
            LOAD_INPUTS: ns = CALC_WHEEL_V;
            CALC_WHEEL_V: if (wheel_velocity_done) ns = START_PRED;
            START_PRED, WAIT_PRED: begin
                if (done)
                    ns = IDLE;
                else
                    ns = WAIT_PRED;
            end
            default: ns = IDLE;
        endcase
    end

endmodule
