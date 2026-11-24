module srukf_end_to_end_tb #(
    parameter int INVALID_KIND = 0,
    parameter bit POSITIVE_CENTRAL = 0,
    parameter logic [160:0] START_STATE = '0,
    parameter logic [479:0] START_FACTOR = '0,
    parameter logic [479:0] PROCESS_FACTOR = '0
);
    logic clk = 0, rst_n = 0, ukf_pulse = 0;
    always #5 clk = !clk;
    logic [31:0] ukf_beat_ts = 0, ukf_vn_yaw_f32, ukf_vn_rate_f32;
    logic [63:0] ukf_vn_lat_f64, ukf_vn_lon_f64;
    logic ukf_dw_valid, ukf_vn_valid;
    logic signed [27:0] ukf_dw_delta;
    meas_s_factor_t measurement_s;
    logic ukf_busy, ukf_pred_done, ukf_valid, ukf_failed, ukf_vn_overflow, ukf_reinit_required;
    logic [31:0] ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB;
    pred_s_factor_t corrected_s;

    UKF #(
        .WHEEL_VELOCITY_SCALE(32'h3e000000), .INITIAL_STATE(START_STATE),
        .INITIAL_S(START_FACTOR), .PROCESS_S(PROCESS_FACTOR),
        .CENTRAL_DOWNDATE(!POSITIVE_CENTRAL),
        .SQRT_ABS_W_CENTRAL(INVALID_KIND == 2 ? 32'h49742400 : 32'h3f000000)
    ) dut (
        .ukf_beat(1'b0), .prediction_done(1'b0), .prediction_result(161'b0),
        .prediction_result_s(480'b0), .prediction_mac_request(67'b0),
        .prediction_div_request(66'b0), .prediction_start(), .prediction_prior(),
        .prediction_prior_s(), .prediction_ts(), .prediction_wheel_velocity(),
        .prediction_wheel_valid(), .prediction_mac_grant(), .prediction_mac_response(),
        .prediction_div_grant(), .prediction_div_response(), .*
    );

    integer fd, count, id, mode, tick = 0, accepted, pred_started, pred_finished;
    integer corr_started, conv_started, pred_count, conv_count;
    integer propagated, deviations, updates, downdates;
    wire [8:0] pred_pc = dut.internal_backend.sequencer.pc;
    wire instruction_retired = dut.internal_backend.sequencer.complete;
    UKF_OUT_PACKET held_prior;
    pred_s_factor_t held_prior_s;
    logic [31:0] held_wheel;
    meas_s_factor_t expected_noise;
    corr_measurement_t expected_measurement;
    logic check_shared_handoff = 0;
    logic injected = 0, forced_pivot = 0;
    logic [159:0] completed_prediction;
    pred_s_factor_t completed_prediction_s;
    string fixture_path;
    always @(posedge clk) if (rst_n) begin
        if (check_shared_handoff) begin
            if (dut.active_measurement_s !== expected_noise ||
                {dut.internal_backend.sequencer.spad[0], dut.internal_backend.sequencer.spad[1],
                 dut.internal_backend.sequencer.spad[2], dut.internal_backend.sequencer.spad[3]}
                !== expected_measurement)
                $fatal(1, "SR or converted sample lost during shared-storage handoff");
        end
        check_shared_handoff = dut.load_shared_noise;
        if (dut.vn_dispatch) expected_noise = dut.selected_noise;
        if (dut.convert_done) expected_measurement = dut.converted;
        if (dut.load_shared_noise &&
            (dut.latest_s !== dut.pred_s || dut.noise_snapshot !== expected_noise))
            $fatal(1, "retained S or staged SR incorrect before handoff");
        if (dut.correction_spad_request.active && dut.backend_active)
            $fatal(1, "simultaneous SPAD ownership");
        tick++;
        if (dut.wheel_mac_owner && dut.converter_busy) $fatal(1, "sensor MAC collision");
        if (dut.prediction_mac_grant &&
            (dut.wheel_mac_owner || dut.converter_busy || dut.convert_start))
            $fatal(1, "prediction MAC collision");
        if (dut.backend_mac_request.start && !dut.prediction_mac_grant)
            $fatal(1, "prediction issued MAC without grant");
        if (dut.backend_div_request.start && !dut.prediction_div_grant)
            $fatal(1, "prediction issued DIV without grant");
        if (ukf_pulse && !ukf_busy) accepted = tick;
        if (dut.prediction_start) begin
            held_prior = dut.prediction_prior;
            held_prior_s = dut.prediction_prior_s;
            held_wheel = dut.prediction_wheel_velocity;
            pred_started = tick;
            propagated = 0;
            deviations = 0;
            updates = 0;
            downdates = 0;
        end
        if (dut.backend_active &&
            (held_prior !== dut.prediction_prior || held_prior_s !== dut.prediction_prior_s ||
             held_wheel !== dut.prediction_wheel_velocity))
            $fatal(1, "held prediction input changed during execution");
        if (instruction_retired) begin
            if (dut.internal_backend.sequencer.write_result &&
                dut.internal_backend.sequencer.uop.dest < 21)
                $fatal(1, "original state or factor overwritten at PC %0d", pred_pc);
            if (pred_pc == 0)
                for (int i = 0; i < 15; i++)
                    if (dut.internal_backend.sequencer.working_sq[i] !==
                        PROCESS_FACTOR[32*i +: 32])
                        $fatal(1, "process factor was not reloaded");

            case (pred_pc)
                240: propagated++;
                256: deviations++;
                338: updates++;
                415: downdates++;
                default: begin end
            endcase
        end
        if (dut.convert_start) conv_started = tick;
        if (dut.convert_done) begin
            conv_count++;
            $display("MEAS %0d %032h %0d", id, dut.converted, tick - conv_started);
        end
        if (ukf_pred_done) begin
            pred_count++;
            pred_finished = tick;
            if (dut.pred_packet.pred_valid) begin
                completed_prediction = dut.pred_packet[160:1];
                completed_prediction_s = dut.pred_s;
                if (propagated != 22 || deviations != 11 ||
                    updates != (POSITIVE_CENTRAL ? 11 : 10) ||
                    downdates != (POSITIVE_CENTRAL ? 0 : 1))
                    $fatal(1, "unexpected prediction pass counts %0d %0d %0d %0d",
                           propagated, deviations, updates, downdates);
                $display("PRED %0d %040h %0120h %0d %0d", id, dut.pred_packet[160:1],
                         dut.pred_s, pred_started - accepted, tick - pred_started);
            end
        end
        if (dut.correction.state == dut.correction.LOAD_INPUTS) corr_started = tick;
        if (ukf_valid && (ukf_failed || ukf_reinit_required))
            $fatal(1, "invalid estimate published");
        if ((ukf_reinit_required || dut.correction_update_failed) &&
            dut.prediction_prior.ukf_valid)
            $fatal(1, "corrupted bank marked valid");
        if (ukf_valid)
            $display("CORR %0d %040h %0120h %0d %0d", id,
                     {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB},
                     corrected_s, tick - corr_started, tick - accepted);
    end

    // Inject a negative SQRT radicand during the fourth downdate,
    // after earlier correction stages have already changed the main bank.
    always @(negedge clk) begin
        if (forced_pivot) begin
            release dut.correction.correction_program.operand_a;
            forced_pivot = 0;
        end else if (rst_n && mode == 4 && !injected &&
                     dut.correction.correction_program.state == dut.correction.correction_program.ISSUE_A &&
                     dut.correction.correction_program.update_stage == 3 &&
                     dut.correction.correction_program.row == 3 &&
                     dut.correction.correction_program.pc == 17) begin
            force dut.correction.correction_program.operand_a = 32'hbf800000;
            forced_pivot = 1;
            injected = 1;
        end
    end

    task automatic reset_filter;
        rst_n = 0;
        check_shared_handoff = 0;
        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);
        if (ukf_busy || ukf_valid || ukf_failed || ukf_reinit_required ||
            {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB} !== START_STATE[160:1] ||
            corrected_s !== START_FACTOR)
            $fatal(1, "reset did not restore the configured prior");
    endtask

    initial begin
        if (!$value$plusargs("fixtures=%s", fixture_path)) $fatal(1, "missing fixtures");
        fd = $fopen(fixture_path, "r");
        if (!fd) $fatal(1, "could not open fixtures");
        reset_filter();
        while (!$feof(fd)) begin
            count = $fscanf(fd, "%d %d %d %d %d %h %h %h %h %h\n", id, mode,
                            ukf_dw_valid, ukf_vn_valid, ukf_dw_delta, ukf_vn_lat_f64,
                            ukf_vn_lon_f64, ukf_vn_yaw_f32, ukf_vn_rate_f32, measurement_s);
            if (count != 10) $fatal(1, "bad fixture");
            if (mode == 3) reset_filter();
            injected = 0;
            pred_count = 0;
            conv_count = 0;
            ukf_beat_ts++;
            ukf_pulse = 1;
            @(negedge clk);
            ukf_pulse = 0;
            // Live sensor inputs may change as soon as the beat has been sampled.
            ukf_dw_delta = -13;
            ukf_vn_yaw_f32 = 32'h7fc00000;
            if (mode == 4) begin
                repeat (500) @(negedge clk);
                ukf_pulse = 1;
                @(negedge clk);
                ukf_pulse = 0;
                if (!dut.vn_pending)
                    $fatal(1, "failure fixture did not prefill the VN queue");
            end
            if (mode == 2) begin
                repeat (500) @(negedge clk);
                reset_filter();
                $display("RESET %0d", id);
            end else begin
                for (int wait_cycles = 0; ukf_busy && wait_cycles < 150000; wait_cycles++) begin
                    if (mode == 0 && ukf_failed) $fatal(1, "unexpected failure on %0d at prediction PC %0d", id,
                        dut.internal_backend.sequencer.pc);
                    if ((mode == 1 || mode == 4 || INVALID_KIND != 0) && ukf_valid)
                        $fatal(1, "invalid fixture produced a result");
                    if (ukf_failed) $display("FAIL %0d %0d %0d", id, tick - accepted, pred_pc);
                    @(negedge clk);
                end
                // Terminal valid/failure may share the edge that drops busy.
                if (ukf_failed) $display("FAIL %0d %0d %0d", id, tick - accepted, pred_pc);
                if (ukf_busy || pred_count != 1 || conv_count != int'(ukf_vn_valid && INVALID_KIND == 0))
                    $fatal(1, "missing or repeated completion on %0d pc=%d", id,
                           dut.internal_backend.sequencer.pc);
                if ((mode == 1 || mode == 4) && INVALID_KIND == 0) begin
                    if (!ukf_reinit_required || dut.vn_pending || (mode == 4 && !injected) ||
                        ({ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB} === completed_prediction &&
                         corrected_s === completed_prediction_s))
                        $fatal(1, "partial correction did not require reset");
                    // Try both inputs while the invalid bank is quarantined.
                    ukf_dw_valid = 1;
                    ukf_vn_valid = 1;
                    ukf_pulse = 1;
                    @(negedge clk);
                    ukf_pulse = 0;
                    repeat (6) @(negedge clk);
                    if (!ukf_reinit_required || ukf_busy || ukf_valid || dut.vn_pending ||
                        pred_count != 1 || conv_count != 1)
                        $fatal(1, "work accepted before reset");
                    reset_filter();
                    $display("RESET %0d", id);
                end else if (ukf_reinit_required) begin
                    $fatal(1, "unexpected reset requirement");
                end
            end
        end
        $display("PASS");
        $finish;
    end
    initial begin
        repeat (1500000) @(posedge clk);
        $fatal(1, "watchdog");
    end
endmodule
