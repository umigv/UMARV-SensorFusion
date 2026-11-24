module UKF (
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

    output logic               ukf_pred_done,
    output logic               ukf_valid,
    output logic        [31:0] ukf_x,
    output logic        [31:0] ukf_y,
    output logic        [31:0] ukf_theta,
    output logic        [31:0] ukf_omega,
    output logic        [31:0] ukf_wheelB
);

    // SR matrix entries 83 through 92.
    localparam logic [6:0] PROM_SR_BASE = 7'd83;

    logic reset;
    meas_regfile_t time_pack;
    PRED_PACKET pred_packet;
    UKF_OUT_PACKET result_packet, latest_state, next_state;

    assign reset = ~rst_n;
    // The timestamper holds the payload and flags. Only the ready pulse starts work.
    assign time_pack.ukf_beat_ts = ukf_beat_ts;
    assign time_pack.ukf_dw_valid = ukf_pulse && ukf_dw_valid;
    assign time_pack.ukf_vn_valid = ukf_pulse && ukf_vn_valid;
    assign time_pack.ukf_vn_yaw_f32 = ukf_vn_yaw_f32;
    assign time_pack.ukf_vn_rate_f32 = ukf_vn_rate_f32;
    assign time_pack.ukf_vn_lat_f64 = ukf_vn_lat_f64;
    assign time_pack.ukf_vn_lon_f64 = ukf_vn_lon_f64;
    assign time_pack.ukf_dw_delta = ukf_dw_delta;

    UKF_pred_engine prediction (
        .clock(clk),
        .reset(reset),
        .corr_packet(latest_state),
        .timestamp_pack(time_pack),
        .busy(),
        .done(ukf_pred_done),
        .pred_pack(pred_packet)
    );

    UKF_corr_engine correction (
        .clock(clk),
        .reset(reset),
        .corr_packet(time_pack),
        .pred_packet(pred_packet),
        .ukf_output(result_packet)
    );

    always_comb begin
        next_state = latest_state;
        if (result_packet.ukf_valid)
            next_state = result_packet;
        else if (ukf_pred_done && pred_packet.pred_valid)
            next_state = {pred_packet.pred_x, pred_packet.pred_y,
                          pred_packet.pred_theta, pred_packet.pred_omega,
                          pred_packet.pred_wheelB, 1'b1};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latest_state <= '0;
            ukf_valid <= 1'b0;
            {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB} <= '0;
        end else begin
            latest_state <= next_state;
            ukf_valid <= ukf_beat;
            if (ukf_beat)
                {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB} <=
                    {next_state.ukf_x, next_state.ukf_y, next_state.ukf_theta,
                     next_state.ukf_omega, next_state.ukf_wheelB};
        end
    end

endmodule
