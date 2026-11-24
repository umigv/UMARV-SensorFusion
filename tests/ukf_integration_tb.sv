module ukf_integration_tb;
    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    logic ukf_beat = 0, ukf_pulse = 0, ukf_vn_valid = 0, ukf_dw_valid = 0;
    logic [31:0] ukf_beat_ts, ukf_vn_yaw_f32, ukf_vn_rate_f32;
    logic [63:0] ukf_vn_lat_f64, ukf_vn_lon_f64;
    logic signed [27:0] ukf_dw_delta;
    meas_s_factor_t measurement_s;
    logic prediction_start, prediction_wheel_valid, prediction_done = 0;
    UKF_OUT_PACKET prediction_prior;
    pred_s_factor_t prediction_prior_s, prediction_result_s;
    logic [31:0] prediction_ts, prediction_wheel_velocity;
    PRED_PACKET prediction_result;
    ukf_mac_request_t prediction_mac_request = '0;
    ukf_mac_response_t prediction_mac_response;
    ukf_divsqrt_request_t prediction_div_request = '0;
    ukf_divsqrt_response_t prediction_div_response;
    logic prediction_mac_grant, prediction_div_grant;
    logic ukf_busy, ukf_pred_done, ukf_valid, ukf_failed, ukf_vn_overflow, ukf_reinit_required;
    logic [31:0] ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB;
    pred_s_factor_t corrected_s;
    PRED_PACKET fixture_prediction;
    pred_s_factor_t fixture_s, previous_s;
    UKF_OUT_PACKET previous_state;
    integer starts = 0, conversions = 0, results = 0, errors = 0;
    integer xy_completions = 0, prediction_completions = 0;
    integer fd, count;
    time conversion_tick, prediction_tick;
    string fixture_path;

    UKF #(.EXTERNAL_PREDICTION(1'b1), .ORIGIN_LATITUDE(32'h42280000), .ORIGIN_LONGITUDE(32'hc2a60000),
          .WHEEL_VELOCITY_SCALE(32'h3f000000)) dut (.*);

    always @(posedge clk) if (rst_n) begin
        if (prediction_mac_grant && (dut.wheel_mac_owner || dut.converter_busy || dut.convert_start))
            $fatal(1, "multiple MAC owners");
        if (prediction_start) starts++;
        if (dut.convert_start) conversions++;
        if (dut.convert_done) begin
            xy_completions++;
            conversion_tick = $time;
        end
        if (ukf_pred_done) begin
            prediction_completions++;
            prediction_tick = $time;
        end
        if (ukf_valid) results++;
        if (ukf_failed) errors++;
        if (ukf_valid && ukf_failed) $fatal(1, "success and failure overlapped");
    end

    task automatic launch(input integer transaction, input bit with_wheel = 1);
        if (ukf_busy) $fatal(1, "launch while busy");
        ukf_beat_ts = transaction;
        ukf_dw_delta = 28'sd12;
        ukf_pulse = 1;
        ukf_vn_valid = 1;
        ukf_dw_valid = with_wheel;
        @(negedge clk);
        ukf_pulse = 0;
        // Deliberately change the live inputs after acceptance.
        ukf_dw_delta = -28'sd8;
        ukf_beat_ts = '1;
        ukf_vn_yaw_f32 = '0;
        ukf_vn_rate_f32 = '0;
        ukf_vn_lat_f64 = '0;
        ukf_vn_lon_f64 = '0;
        if (!prediction_start) begin
            wait (prediction_start);
            @(negedge clk);
        end
        if (prediction_ts !== transaction || prediction_wheel_valid !== with_wheel ||
            prediction_wheel_velocity !== (with_wheel ? 32'h40c00000 : 32'b0))
            $fatal(1, "wheel data or timestamp changed after acceptance");
        if (prediction_prior !== previous_state || prediction_prior_s !== previous_s)
            $fatal(1, "prediction feedback state and factor differ");
    endtask

    task automatic return_prediction(input bit valid_result);
        prediction_result = fixture_prediction;
        prediction_result.pred_valid = valid_result;
        prediction_result_s = fixture_s;
        prediction_done = 1;
        @(negedge clk);
        prediction_done = 0;
        prediction_result = '0;
        prediction_result_s = '0;
    endtask

    task automatic load_fixture(input integer index);
        count = $fscanf(fd, "%h %h %h %h %h %h %h",
                        ukf_vn_lat_f64, ukf_vn_lon_f64, ukf_vn_yaw_f32, ukf_vn_rate_f32,
                        fixture_prediction, fixture_s, measurement_s);
        if (count != 7) $fatal(1, "missing fixture %0d", index);
    endtask

    task automatic reset_filter;
        rst_n = 0;
        repeat (2) @(negedge clk);
        if (ukf_busy || ukf_valid || ukf_failed || corrected_s !== '0)
            $fatal(1, "reset did not cancel the entire transaction");
        previous_state = '0;
        previous_s = '0;
        rst_n = 1;
    endtask

    task automatic finish_valid(input integer index);
        wait (dut.correction.state == dut.correction.LOAD_INPUTS);
        @(negedge clk);
        if (dut.correction.prediction !== fixture_prediction ||
            dut.correction.prediction_s !== fixture_s)
            $fatal(1, "prediction state and factor were sampled separately");
        $display("MEAS %0d %08h%08h%08h%08h", index,
                 dut.correction.correction_program.regfile[21],
                 dut.correction.correction_program.regfile[22],
                 dut.correction.correction_program.regfile[23],
                 dut.correction.correction_program.regfile[24]);

        // A wheel-only pulse during correction must not start prediction.
        // The dedicated queue regression exercises VN acceptance while busy.
        ukf_vn_valid = 0;
        ukf_pulse = 1;
        @(negedge clk);
        ukf_pulse = 0;
        wait (ukf_valid);
        @(negedge clk);
        previous_state = {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB, 1'b1};
        previous_s = corrected_s;
        $display("RESULT %0d %h %h", index, previous_state, previous_s);
        wait (!ukf_busy);
        @(negedge clk);
    endtask

    initial begin
        if (!$value$plusargs("fixtures=%s", fixture_path)) $fatal(1, "fixtures required");
        fd = $fopen(fixture_path, "r");
        if (!fd) $fatal(1, "cannot open fixtures");
        prediction_result = '0;
        prediction_result_s = '0;
        reset_filter();

        for (int i = 0; i < 3; i++) begin
            load_fixture(i);
            launch(i);
            case (i)
                0: return_prediction(1); // Backend can finish on its start beat.
                1: begin
                    wait (dut.convert_done);
                    repeat (5) @(negedge clk);
                    return_prediction(1);
                end
                2: begin
                    wait (dut.convert_done);
                    @(negedge clk);
                    return_prediction(1);
                end
            endcase
            finish_valid(i);
            case (i)
                0: if (prediction_tick >= conversion_tick) $fatal(1, "expected prediction first");
                1: if (prediction_tick <= conversion_tick) $fatal(1, "expected conversion first");
                2: if (prediction_tick != conversion_tick) $fatal(1, "expected simultaneous completion");
            endcase
        end
        if (results != 3 || starts != 3 || conversions != 3 ||
            xy_completions != 3 || prediction_completions != 3)
            $fatal(1, "lost or repeated transaction");

        load_fixture(3);
        launch(3);
        return_prediction(1);
        wait (ukf_failed);
        if (dut.correction.correction_program.row != 3 ||
            dut.correction.correction_program.pc != 14)
            $fatal(1, "expected fourth-stage downdate failure");
        wait (!ukf_busy);
        @(negedge clk);
        if (results != 3 || errors != 1) $fatal(1, "invalid downdate published");

        previous_state = fixture_prediction;
        previous_s = fixture_s;
        load_fixture(4);
        launch(4);
        wait (dut.convert_done);
        repeat (3) @(negedge clk);
        reset_filter(); // Conversion complete, prediction outstanding.

        load_fixture(5);
        launch(5);
        repeat (5) @(negedge clk);
        reset_filter(); // Conversion outstanding.

        load_fixture(6);
        launch(6);
        return_prediction(1);
        wait (dut.correction.state == dut.correction.RUN_CORR_MICROCODE);
        repeat (5) @(negedge clk);
        reset_filter();

        load_fixture(7);
        launch(7);
        return_prediction(0);
        if (!ukf_failed) $fatal(1, "backend failure was not reported");
        wait (!ukf_busy);
        @(negedge clk);
        if (results != 3 || errors != 2) $fatal(1, "aborted transaction published");

        load_fixture(8);
        launch(8, 0);
        return_prediction(1);
        finish_valid(8);
        if (results != 4) $fatal(1, "fresh transaction did not recover after reset");
        $display("PASS");
        $finish;
    end

    initial begin
        repeat (150000) @(posedge clk);
        $fatal(1, "integration timeout");
    end
endmodule
