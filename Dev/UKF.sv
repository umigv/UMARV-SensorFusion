module UKF (
    input  logic               clk,
    input  logic               rst_n,
    input  logic        [31:0] ukf_beat_ts,
    input  logic               ukf_vn_valid,
    input  logic        [31:0] ukf_vn_yaw_f32,
    input  logic        [31:0] ukf_vn_rate_f32,
    input  logic        [63:0] ukf_vn_lat_f64,
    input  logic        [63:0] ukf_vn_lon_f64,
    input  logic               ukf_dw_valid,
    input  logic signed [27:0] ukf_dw_delta,

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
    UKF_OUT_PACKET result_packet;

    assign reset = ~rst_n;
    assign time_pack.ukf_dw_valid = ukf_dw_valid;
    assign time_pack.ukf_vn_valid = ukf_vn_valid;

    // The timestamper staggers the one-clock valid strobes so the engines run serially.
    // Each payload must settle through its latch before the engine samples the strobe.
    always_latch begin
        if (ukf_dw_valid || ukf_vn_valid)
            time_pack.ukf_beat_ts     <= ukf_beat_ts;
        if (ukf_vn_valid) begin
            time_pack.ukf_vn_yaw_f32  <= ukf_vn_yaw_f32;
            time_pack.ukf_vn_rate_f32 <= ukf_vn_rate_f32;
            time_pack.ukf_vn_lat_f64  <= ukf_vn_lat_f64;
            time_pack.ukf_vn_lon_f64  <= ukf_vn_lon_f64;
        end
        if (ukf_dw_valid)
            time_pack.ukf_dw_delta    <= ukf_dw_delta;
    end

    UKF_pred_engine prediction (
        .clock(clk),
        .reset(reset),
        .corr_packet(result_packet),
        .timestamp_pack(time_pack),
        .busy(),
        .done(),
        .pred_pack(pred_packet)
    );

    UKF_corr_engine correction (
        .clock(clk),
        .reset(reset),
        .corr_packet(time_pack),
        .pred_packet(pred_packet),
        .ukf_output(result_packet)
    );

    assign ukf_valid  = result_packet.ukf_valid;
    assign ukf_x      = result_packet.ukf_x;
    assign ukf_y      = result_packet.ukf_y;
    assign ukf_theta  = result_packet.ukf_theta;
    assign ukf_omega  = result_packet.ukf_omega;
    assign ukf_wheelB = result_packet.ukf_wheelB;

endmodule
