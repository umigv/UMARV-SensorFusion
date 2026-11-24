// Tiny Tapeout byte interface. All strobes are synchronous to clk.
module tt_um_umarv_srukf (
    input wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input wire ena, clk, rst_n
);
    localparam [2:0] WRITE_BYTE = 1, WHEEL = 2, VN = 3, CLEAR = 4;
    wire [2:0] command = uio_in[2:0];
    wire strobe = ena && uio_in[3] && rst_n;
    reg [511:0] frame;
    reg [6:0] count;
    reg result_ready, failure_seen, overflow_seen, protocol_error;
    wire busy, valid, failed, reinit, overflow;
    wire wheel = strobe && command == WHEEL && count == 4 && !busy && !reinit;
    wire vn = strobe && command == VN && count == 64 && !reinit;
    wire [31:0] x, y, heading, rate, bias;
    wire [479:0] factor;
    wire [639:0] result = {factor, bias, rate, heading, y, x};
    wire [9:0] read_offset = {ui_in[6:0], 3'b000};

    assign uio_oe = 8'b0;
    assign uio_out = 8'b0;
    assign uo_out = !ena ? 8'b0 : uio_in[4] ?
                    (result_ready && !busy && ui_in < 80 ? result[read_offset +: 8] : 8'b0) :
                    {2'b0, protocol_error, overflow_seen, reinit, failure_seen, result_ready, busy};

    always @(posedge clk)
        if (strobe && command == WRITE_BYTE && count < 64)
            frame <= {frame[503:0], ui_in};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 0;
            result_ready <= 0;
            failure_seen <= 0;
            overflow_seen <= 0;
            protocol_error <= 0;
        end else begin
            if (strobe) begin
                case (command)
                    WRITE_BYTE: if (count < 64) count <= count + 1'b1;
                                else protocol_error <= 1;
                    WHEEL, VN: begin
                        count <= 0;
                        if (wheel || vn) result_ready <= 0;
                        else protocol_error <= 1;
                    end
                    CLEAR: begin
                        count <= 0;
                        failure_seen <= 0;
                        overflow_seen <= 0;
                        protocol_error <= 0;
                    end
                    default: protocol_error <= 1;
                endcase
            end
            if (valid) result_ready <= 1;
            if (failed) begin
                failure_seen <= 1;
                result_ready <= 0;
            end
            if (overflow) overflow_seen <= 1;
        end
    end

    UKF #(
        .INITIAL_STATE(161'h7b99999b7c99999a7e6666677e99999a7d147ae3),
        .INITIAL_S(480'h3e6147aebcb439583e3851ec3c449ba63ccccccd3df5c28fbc23d70a3c03126fbc75c28f3e8000003cf5c28fbba3d70a3c23d70a3ca3d70a3e99999a),
        .PROCESS_S(480'h3be560423a83126f3c1374bc00000000000000003c03126f00000000000000003a83126f3c3439580000000000000000000000003b03126f3c449ba6),
        .WHEEL_VELOCITY_SCALE(32'h3e000000)
    ) filter (
        .clk(clk), .rst_n(rst_n), .ukf_beat(1'b0), .ukf_pulse(wheel || vn),
        .ukf_beat_ts(32'b0), .ukf_dw_valid(wheel), .ukf_dw_delta(frame[27:0]),
        .ukf_vn_valid(vn), .ukf_vn_lat_f64(frame[511:448]),
        .ukf_vn_lon_f64(frame[447:384]), .ukf_vn_yaw_f32(frame[383:352]),
        .ukf_vn_rate_f32(frame[351:320]), .measurement_s(frame[319:0]),
        .prediction_done(1'b0), .prediction_result(161'b0), .prediction_result_s(480'b0),
        .prediction_mac_request(67'b0), .prediction_div_request(66'b0),
        .ukf_busy(busy), .ukf_valid(valid), .ukf_failed(failed),
        .ukf_reinit_required(reinit), .ukf_vn_overflow(overflow),
        .ukf_x(x), .ukf_y(y), .ukf_theta(heading), .ukf_omega(rate), .ukf_wheelB(bias),
        .corrected_s(factor)
    );
endmodule
