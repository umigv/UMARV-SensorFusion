module UKF #(
    parameter logic [31:0] ORIGIN_LATITUDE = 32'b0,
    parameter logic [31:0] ORIGIN_LONGITUDE = 32'b0,
    parameter logic [31:0] EARTH_RADIUS = 32'h4ac26d70,
    parameter logic [31:0] WHEEL_VELOCITY_SCALE = 32'b0,
    parameter logic [160:0] INITIAL_STATE = '0,
    parameter logic [479:0] INITIAL_S = '0
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               ukf_beat,
    input  logic               ukf_pulse,
    input  logic        [31:0] ukf_beat_ts,
    input  logic               ukf_vn_valid,
    input  logic        [31:0] ukf_vn_yaw_f32,
    input  logic        [31:0] ukf_vn_rate_f32,
    input  logic        [63:0] ukf_vn_lat_f64,
    input  logic        [63:0] ukf_vn_lon_f64,
    input  logic               ukf_dw_valid,
    input  logic signed [27:0] ukf_dw_delta,

    input  meas_s_factor_t     measurement_s,

    // Prediction execution backend. Return state and factor on the same done beat.
    output logic               prediction_start,
    output UKF_OUT_PACKET      prediction_prior,
    output pred_s_factor_t     prediction_prior_s,
    output logic        [31:0] prediction_ts, prediction_wheel_velocity,
    output logic               prediction_wheel_valid,
    input  logic               prediction_done,
    input  PRED_PACKET         prediction_result,
    input  pred_s_factor_t     prediction_result_s,

    output logic               ukf_busy,
    output logic               ukf_pred_done,
    output logic               ukf_valid,
    output logic               ukf_failed,
    output logic        [31:0] ukf_x,
    output logic        [31:0] ukf_y,
    output logic        [31:0] ukf_theta,
    output logic        [31:0] ukf_omega,
    output logic        [31:0] ukf_wheelB,
    output pred_s_factor_t     corrected_s
);
    localparam logic [6:0] WHEEL_SCALE_ADDR = 7'd95;

    logic reset, prediction_busy, correction_busy, prediction_failed, correction_failed;
    logic wheel_mac_owner, converter_busy, convert_request, convert_pending, convert_start, convert_done;
    logic [6:0] wheel_address;
    logic [31:0] wheel_read_data;
    meas_regfile_t time_pack, convert_input;
    corr_measurement_t converted;
    PRED_PACKET pred_packet;
    UKF_OUT_PACKET result_packet, latest_state, visible_state;
    pred_s_factor_t pred_s, latest_s, correction_s;
    ukf_mac_request_t wheel_mac_request, converter_mac_request, sensor_mac_request;
    ukf_mac_response_t sensor_mac_response;

    assign reset = ~rst_n;
    assign ukf_busy = prediction_busy || correction_busy || converter_busy || convert_pending;
    assign time_pack.ukf_pulse = rst_n && ukf_pulse && !ukf_busy;
    assign time_pack.ukf_beat_ts = ukf_beat_ts;
    assign time_pack.ukf_dw_valid = ukf_dw_valid;
    assign time_pack.ukf_vn_valid = ukf_vn_valid;
    assign time_pack.ukf_vn_yaw_f32 = ukf_vn_yaw_f32;
    assign time_pack.ukf_vn_rate_f32 = ukf_vn_rate_f32;
    assign time_pack.ukf_vn_lat_f64 = ukf_vn_lat_f64;
    assign time_pack.ukf_vn_lon_f64 = ukf_vn_lon_f64;
    assign time_pack.ukf_dw_delta = ukf_dw_delta;

    assign wheel_read_data = wheel_address == WHEEL_SCALE_ADDR ? WHEEL_VELOCITY_SCALE : '0;
    assign sensor_mac_request = wheel_mac_owner ? wheel_mac_request : converter_mac_request;
    assign convert_start = (convert_request || convert_pending) && !wheel_mac_owner;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            convert_pending <= 1'b0;
        else
            convert_pending <= (convert_request || convert_pending) && !convert_start;
    end

    UKF_pred_engine prediction (
        .clock(clk), .reset(reset),
        .corr_packet(latest_state), .corr_s(latest_s), .timestamp_pack(time_pack),
        .backend_start(prediction_start), .backend_prior(prediction_prior),
        .backend_prior_s(prediction_prior_s), .backend_ts(prediction_ts),
        .backend_wheel_velocity(prediction_wheel_velocity),
        .backend_wheel_valid(prediction_wheel_valid),
        .backend_done(prediction_done), .backend_prediction(prediction_result),
        .backend_prediction_s(prediction_result_s),
        .velocity_scale_addr(WHEEL_SCALE_ADDR), .mem_rdata(wheel_read_data),
        .mem_addr(wheel_address), .mem_we(), .mem_wdata(),
        .mac_start(wheel_mac_request.start), .mac_clear(wheel_mac_request.clear),
        .mac_last(wheel_mac_request.last), .mac_a(wheel_mac_request.a), .mac_b(wheel_mac_request.b),
        .mac_busy(sensor_mac_response.busy), .mac_result_valid(sensor_mac_response.valid),
        .mac_result(sensor_mac_response.result), .wheel_mac_owner(wheel_mac_owner),
        .busy(prediction_busy), .done(ukf_pred_done), .failed(prediction_failed),
        .pred_pack(pred_packet), .pred_s(pred_s)
    );

    vn_to_correction #(
        .ORIGIN_LATITUDE(ORIGIN_LATITUDE), .ORIGIN_LONGITUDE(ORIGIN_LONGITUDE),
        .EARTH_RADIUS(EARTH_RADIUS)
    ) converter (
        .clock(clk), .reset(reset), .start(convert_start), .packet(convert_input),
        .busy(converter_busy), .done(convert_done), .measurement(converted),
        .mac_request(converter_mac_request), .mac_response(sensor_mac_response)
    );

    UKF_corr_engine #(.SHARE_MAC(1'b1), .REPORT_PRED_FAILURE(1'b1)) correction (
        .clock(clk), .reset(reset), .corr_packet(time_pack),
        .pred_packet(pred_packet), .pred_s(pred_s), .measurement_s(measurement_s),
        .convert_start(convert_request), .convert_input(convert_input),
        .convert_done(convert_done), .convert_result(converted),
        .busy(correction_busy), .done(), .failed(correction_failed),
        .x_update_valid(), .xy_update_valid(), .xyz_update_valid(),
        .ukf_output(result_packet), .corrected_s(correction_s),
        .sensor_mac_owner(wheel_mac_owner || converter_busy),
        .sensor_mac_request(sensor_mac_request), .sensor_mac_response(sensor_mac_response),
        .prediction_failed(prediction_failed)
    );

    assign ukf_valid = result_packet.ukf_valid;
    assign visible_state = ukf_valid ? result_packet : latest_state;
    assign {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB} =
        {visible_state.ukf_x, visible_state.ukf_y, visible_state.ukf_theta,
         visible_state.ukf_omega, visible_state.ukf_wheelB};
    assign corrected_s = ukf_valid ? correction_s : latest_s;
    assign ukf_failed = correction_failed || (prediction_failed && !correction_busy);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latest_state <= INITIAL_STATE;
            latest_s <= INITIAL_S;
        end else if (ukf_valid) begin
            latest_state <= result_packet;
            latest_s <= correction_s;
        end else if (pred_packet.pred_valid) begin
            // Retain prediction for a wheel-only beat or a failed correction.
            latest_state <= {pred_packet.pred_x, pred_packet.pred_y, pred_packet.pred_theta,
                             pred_packet.pred_omega, pred_packet.pred_wheelB, 1'b1};
            latest_s <= pred_s;
        end
    end
endmodule
