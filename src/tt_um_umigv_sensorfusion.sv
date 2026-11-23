/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny Tapeout top level for the sensor fusion chip.
 *
 * For now this holds only the timestamper. The settings are tied to constants and the
 * measurements leave on a simple debug serial stream, because the host SPI register block
 * is not written yet. Both of those are stand-ins and will be replaced.
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
    assign enable         = ui_in[2];
    assign dw_check_turns = ui_in[3];

    logic vn_sync, dw_cs_n, dw_sck, dw_mosi, ukf_pulse;
    logic        [31:0] beat_ts;
    logic               vn_valid, dw_valid;
    logic        [31:0] vn_yaw_f32, vn_rate_f32;
    logic        [63:0] vn_lat_f64, vn_lon_f64;
    logic signed [27:0] dw_delta;
    logic        [39:0] time_us;

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
        .time_us        (time_us)
    );

    // Debug stream (stand-in for the register block): after each ukf_pulse, send
    // the 294-bit word below most significant bit first, one bit every 8 clocks,
    // with dbg_frame high for the whole frame.
    localparam DBG_BITS = 294;
    logic [DBG_BITS-1:0] dbg_word;
    assign dbg_word = {time_us,                                  // 40
                       beat_ts,                                  // 32
                       vn_valid,                                 //  1
                       vn_yaw_f32, vn_rate_f32,                  // 32 + 32
                       vn_lat_f64, vn_lon_f64,                   // 64 + 64
                       dw_valid,                                 //  1
                       dw_delta};                                // 28

    logic        dbg_frame;
    logic [8:0]  dbg_idx;   // bits still to send, minus 1
    logic [2:0]  dbg_div;
    always_ff @(posedge clk or negedge rst_n_s) begin
        if(!rst_n_s) begin
            dbg_frame <= 1'b0;
            dbg_idx   <= '0;
            dbg_div   <= '0;
        end else if(ukf_pulse) begin
            dbg_frame <= 1'b1;
            dbg_idx   <= 9'd293;          // DBG_BITS - 1
            dbg_div   <= '0;
        end else if(dbg_frame) begin
            dbg_div <= dbg_div + 1'b1;
            if(dbg_div == 3'd7) begin
                if(dbg_idx == '0) dbg_frame <= 1'b0;
                else              dbg_idx   <= dbg_idx - 1'b1;
            end
        end
    end
    logic dbg_data;
    assign dbg_data = dbg_frame & dbg_word[dbg_idx];

    // Pins out
    assign uo_out[0] = vn_sync;
    assign uo_out[1] = dw_cs_n;
    assign uo_out[2] = dw_sck;
    assign uo_out[3] = dw_mosi;
    assign uo_out[4] = ukf_pulse;
    assign uo_out[5] = dbg_data;
    assign uo_out[6] = dbg_frame;
    assign uo_out[7] = 1'b0;

    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // List all unused inputs to prevent warnings
    wire _unused = &{ena, ui_in[7:4], uio_in, 1'b0};

endmodule
