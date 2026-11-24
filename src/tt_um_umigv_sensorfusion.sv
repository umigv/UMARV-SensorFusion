/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny Tapeout top level for the sensor fusion chip.
 *
 * The sensor front end supplies raw samples to the integrated SR-UKF.
 */

module tt_um_umigv_sensorfusion (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // 50 MHz
    input  wire       rst_n     // reset_n - low to reset
);

    // Reset: asserts immediately, releases on a clock edge
    logic [1:0] rst_sync;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) rst_sync <= 2'b00;
        else       rst_sync <= {rst_sync[0], 1'b1};
    end
    logic rst_n_s;
    assign rst_n_s = rst_sync[1];

    // Pins in
    logic vn_rx, dw_miso, enable, dw_check_turns;
    assign vn_rx          = ui_in[0];
    assign dw_miso        = ui_in[1];
    assign enable         = ena && ui_in[2] && noise_count == 0;
    assign dw_check_turns = ui_in[3];

    logic vn_sync, dw_cs_n, dw_sck, dw_mosi, ukf_pulse;
    logic        [31:0] beat_ts;
    logic               vn_valid, dw_valid;
    logic        [31:0] vn_yaw_f32, vn_rate_f32;
    logic        [63:0] vn_lat_f64, vn_lon_f64;
    logic signed [27:0] dw_delta;
    logic [319:0] measurement_s;
    logic [5:0] noise_count;
    logic collecting, busy, corrected, failed, reinit_required, overflow;
    logic [31:0] x, y, heading, rate, wheel_bias;
    logic rejected_snapshot, active_had_vn, sticky_fault;
    localparam [319:0] DEFAULT_NOISE =
        320'h3e0f5c293c9ba5e33db851ecbc23d70a3c8b43963e3851ec3c449ba6bc23d70a3ccccccd3e4ccccd;

    // Load a full runtime noise factor while collection is disabled.
    // A partial factor blocks new beats until its remaining bytes arrive.
    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            measurement_s <= DEFAULT_NOISE;
            noise_count <= 0;
        end else if (ena && !ui_in[2] && ui_in[4] && !busy && !collecting && !ukf_pulse) begin
            measurement_s <= {measurement_s[311:0], uio_in};
            noise_count <= noise_count == 39 ? 0 : noise_count + 1'b1;
        end
    end

    timestamper u_timestamper (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .enable         (enable),
        .incr           (32'd0),        // unused
        .period_us      (16'd2500),     // 400 Hz
        .spi_half_div   (8'd0),         // unused
        .deadline_us    (16'd2000),
        .dw_check_turns (dw_check_turns),
        .vn_sync        (vn_sync),
        .vn_rx          (vn_rx),
        .cs_n           (dw_cs_n),
        .sck            (dw_sck),
        .mosi           (dw_mosi),
        .miso           (dw_miso),
        .ukf_pulse      (ukf_pulse),
        .ukf_beat_ts    (beat_ts),
        .ukf_vn_valid   (vn_valid),
        .ukf_vn_yaw_f32 (vn_yaw_f32),
        .ukf_vn_rate_f32(vn_rate_f32),
        .ukf_vn_lat_f64 (vn_lat_f64),
        .ukf_vn_lon_f64 (vn_lon_f64),
        .ukf_dw_valid   (dw_valid),
        .ukf_dw_delta   (dw_delta),
        .collecting     (collecting),
        .time_us        ()
    );

    UKF u_srukf (
        .clk(clk), .rst_n(rst_n_s), .ukf_beat(1'b0),
        .ukf_pulse(ukf_pulse && !rejected_snapshot), .ukf_beat_ts(beat_ts),
        .ukf_vn_valid(vn_valid), .ukf_vn_yaw_f32(vn_yaw_f32), .ukf_vn_rate_f32(vn_rate_f32),
        .ukf_vn_lat_f64(vn_lat_f64), .ukf_vn_lon_f64(vn_lon_f64),
        .ukf_dw_valid(dw_valid), .ukf_dw_delta(dw_delta), .measurement_s(measurement_s),
        .prediction_done(1'b0), .prediction_result(161'b0), .prediction_result_s(480'b0),
        .prediction_mac_request(67'b0), .prediction_div_request(66'b0),
        .ukf_busy(busy), .ukf_valid(corrected), .ukf_failed(failed),
        .ukf_reinit_required(reinit_required), .ukf_vn_overflow(overflow),
        .ukf_x(x), .ukf_y(y), .ukf_theta(heading), .ukf_omega(rate), .ukf_wheelB(wheel_bias)
    );

    // Each hardware snapshot represents a whole beat. A VN-only beat may
    // queue behind correction, but must not attach to an older wheel-only
    // prediction. Never split a newer wheel/VN pair across UKF beats.
    assign rejected_snapshot = ukf_pulse && (dw_valid || vn_valid) &&
                               (reinit_required || (busy && (dw_valid || !active_had_vn)));
    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) active_had_vn <= 0;
        else if (ukf_pulse && !busy && !rejected_snapshot && (dw_valid || vn_valid))
            active_had_vn <= vn_valid;
    end
    logic dbg_frame;
    logic [167:0] dbg_word;
    logic [7:0] dbg_idx;
    logic [2:0] dbg_div;
    wire [7:0] event_status = {3'b0, rejected_snapshot, overflow, reinit_required, failed, corrected};
    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            sticky_fault <= 0;
            dbg_frame <= 0;
            dbg_idx <= 0;
            dbg_div <= 0;
        end else begin
            if (failed || overflow || rejected_snapshot) sticky_fault <= 1;
            if (!dbg_frame) begin
                if (corrected || failed || overflow || rejected_snapshot) begin
                    dbg_frame <= 1;
                    dbg_idx <= 167;
                    dbg_div <= 0;
                end
            end else begin
                dbg_div <= dbg_div + 1'b1;
                if (dbg_div == 7) begin
                    if (dbg_idx == 0) dbg_frame <= 0;
                    else dbg_idx <= dbg_idx - 1'b1;
                end
            end
        end
    end
    always_ff @(posedge clk)
        if (rst_n_s && !dbg_frame && (corrected || failed || overflow || rejected_snapshot))
            dbg_word <= {event_status, corrected ? {x, y, heading, rate, wheel_bias} : 160'b0};

    wire dbg_data = dbg_frame && dbg_word[dbg_idx];

    // Pins out
    assign uo_out[0] = vn_sync;
    assign uo_out[1] = dw_cs_n;
    assign uo_out[2] = dw_sck;
    assign uo_out[3] = dw_mosi;
    assign uo_out[4] = ukf_pulse;
    assign uo_out[5] = dbg_data;
    assign uo_out[6] = dbg_frame;
    assign uo_out[7] = sticky_fault;

    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // List all unused inputs to prevent warnings
    wire _unused = &{ui_in[7:5], 1'b0};

endmodule
