module corr_x_tb #(
    parameter bit TEST_SECOND = 1'b0,
    parameter bit TEST_THIRD = 1'b0,
    parameter bit TEST_FOURTH = 1'b0
);
    logic clock = 0, reset = 1;
    always #5 clock = ~clock;

    meas_regfile_t corr_packet = '0, convert_input;
    PRED_PACKET pred_packet = '0;
    pred_s_factor_t pred_s = '0, corrected_s;
    meas_s_factor_t measurement_s = '0;
    corr_measurement_t convert_result = '0;
    UKF_OUT_PACKET ukf_output;
    logic convert_start, convert_done = 0, busy, done, x_update_valid, failed;
    logic xy_update_valid, xyz_update_valid;
    logic sensor_mac_owner = 0, prediction_failed = 0;
    ukf_mac_request_t sensor_mac_request = '0;
    ukf_mac_response_t sensor_mac_response;

    UKF_corr_engine #(
        .ENABLE_SECOND_UPDATE(TEST_SECOND),
        .ENABLE_THIRD_UPDATE(TEST_THIRD),
        .ENABLE_FOURTH_UPDATE(TEST_FOURTH)
    ) dut (.*);

    localparam int LAST_REG = 36;
    logic [31:0] before_write [0:LAST_REG];
    int expected_dest, ticks = 0, request_tick, microcode_tick;
    int x_count = 0, x_tick, xy_count = 0, xy_tick, xyz_count = 0, xyz_tick;
    int third_mac_cycles, third_div_cycles, total_mac_cycles, total_div_cycles;
    int mac_stalls = 0, div_stalls = 0;
    logic loading, retire, last_innov, vector_ready, running;
    logic weights_ready, projection_ready, xy_vector_ready;
    logic heading_weights_ready, heading_projection_ready, heading_vector_ready;

    logic rate_weights_ready, rate_projection_ready, rate_variance_ready, rate_vector_ready;
    logic [31:0] rate_variance;

    always @(posedge clock) begin
        ticks = ticks + 1;
        if (!reset) begin
            if (dut.correction_program.start)
                microcode_tick = ticks;
            if (dut.correction_program.start || running) begin
                running = !dut.correction_program.done;
                loading = dut.correction_program.start;
                retire = dut.correction_program.arithmetic_complete &&
                         !dut.correction_program.arithmetic_failed &&
                         dut.correction_program.uop.wb_en;
                expected_dest = dut.correction_program.uop.dest;
                if (retire && (expected_dest < 0 || expected_dest > LAST_REG || expected_dest == 5))
                    $fatal(1, "illegal write address %0d", expected_dest);
                if (retire && !(dut.correction_program.mac_result_valid ||
                                dut.correction_program.div_done))
                    $fatal(1, "retired without an arithmetic result");
                last_innov = retire && dut.correction_program.pc == 3;
                vector_ready = retire && dut.correction_program.pc == 10 &&
                               dut.correction_program.col == 4;
                weights_ready = retire && dut.correction_program.pc == 28;
                projection_ready = retire && dut.correction_program.pc == 33;
                xy_vector_ready = retire && dut.correction_program.pc == 42 &&
                                  dut.correction_program.col == 4 &&
                                  dut.correction_program.update_stage == 1;
                heading_weights_ready = retire && dut.correction_program.pc == 49;
                heading_projection_ready = retire && dut.correction_program.pc == 58;
                heading_vector_ready = retire && dut.correction_program.pc == 42 &&
                                       dut.correction_program.col == 4 &&
                                       dut.correction_program.update_stage == 2;
                rate_weights_ready = retire && dut.correction_program.pc == 71;
                rate_projection_ready = retire && dut.correction_program.pc == 85;
                rate_variance_ready = retire && dut.correction_program.pc == 86;
                rate_vector_ready = retire && dut.correction_program.pc == 42 &&
                                    dut.correction_program.col == 4 &&
                                    dut.correction_program.update_stage == 3;
                if (dut.correction_program.mac_start || dut.correction_program.mac_busy) begin
                    total_mac_cycles++;
                    if (dut.correction_program.update_stage == 2) third_mac_cycles++;
                end
                if (dut.correction_program.div_start || dut.correction_program.div_busy) begin
                    total_div_cycles++;
                    if (dut.correction_program.update_stage == 2) third_div_cycles++;
                end
                if (dut.correction_program.mac_start && dut.correction_program.mac_busy)
                    $fatal(1, "MAC start while busy");
                if (dut.correction_program.mac_start && dut.correction_program.pc == 34)
                    if (dut.correction_program.uop.src_a != PROM_MATH_ONE ||
                        dut.correction_program.uop.src_b != PROM_MATH_ONE ||
                        dut.correction_program.operand_a !== 32'h3f800000 ||
                        dut.correction_program.operand_b !== 32'h3f800000)
                        $fatal(1, "variance seed must read ROM 1.0 through both ports");
                if (dut.correction_program.div_start && dut.correction_program.div_busy)
                    $fatal(1, "divide/sqrt start while busy");
                if (dut.correction_program.mac_busy && !retire)
                    mac_stalls++;
                if (dut.correction_program.div_busy && !retire)
                    div_stalls++;
                for (int a = 0; a <= LAST_REG; a++)
                    before_write[a] = dut.correction_program.regfile[a];
                #1;
                if (ukf_output.ukf_valid &&
                    (!TEST_FOURTH || !done || failed || x_update_valid || xy_update_valid ||
                     xyz_update_valid || dut.correction_program.pc != 88))
                    $fatal(1, "ukf_valid asserted before a successful fourth stage");
                for (int a = 0; a <= LAST_REG; a++)
                    if (!loading && !(retire && a == expected_dest) &&
                        before_write[a] !== dut.correction_program.regfile[a])
                        $fatal(1, "premature write at %0d", a);
                if (last_innov) begin
                    $write("INNOV");
                    for (int a = 21; a <= 24; a++)
                        $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (vector_ready) begin
                    $write("VECTOR");
                    for (int a = 25; a <= 33; a++)
                        if (a != 28) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (weights_ready)
                    $display("WEIGHTS %08h %08h", dut.correction_program.regfile[34],
                             dut.correction_program.regfile[35]);
                if (projection_ready)
                    $display("PROJECTION %08h %08h %08h", dut.correction_program.regfile[36],
                             dut.correction_program.regfile[34], dut.correction_program.regfile[35]);
                if (xy_vector_ready) begin
                    $write("XY_VECTOR");
                    for (int a = 25; a <= 33; a++)
                        if (a != 28) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (heading_weights_ready)
                    $display("HEADING_WEIGHTS %08h %08h %08h", dut.correction_program.regfile[34],
                             dut.correction_program.regfile[35], dut.correction_program.regfile[33]);
                if (heading_projection_ready)
                    $display("HEADING_PROJECTION %08h %08h %08h %08h", dut.correction_program.regfile[36],
                             dut.correction_program.regfile[34], dut.correction_program.regfile[35],
                             dut.correction_program.regfile[33]);
                if (heading_vector_ready) begin
                    $write("XYZ_VECTOR");
                    for (int a = 25; a <= 33; a++)
                        if (a != 28) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (rate_weights_ready)
                    $display("RATE_WEIGHTS %08h %08h %08h %08h", dut.correction_program.regfile[34],
                             dut.correction_program.regfile[35], dut.correction_program.regfile[33],
                             dut.correction_program.regfile[25]);
                if (rate_projection_ready)
                    $display("RATE_PROJECTION %08h %08h %08h %08h %08h", dut.correction_program.regfile[36],
                             dut.correction_program.regfile[34], dut.correction_program.regfile[35],
                             dut.correction_program.regfile[33], dut.correction_program.regfile[25]);
                if (rate_variance_ready)
                    rate_variance = dut.correction_program.regfile[28];
                if (rate_vector_ready) begin
                    $write("FULL_VECTOR %08h", rate_variance);
                    for (int a = 26; a <= 33; a++)
                        if (a != 28) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (TEST_FOURTH && xyz_update_valid) begin
                    if (!busy || done || failed || x_update_valid || xy_update_valid || ukf_output.ukf_valid)
                        $fatal(1, "xyz-stage pulse does not precede the terminal pulse");
                    xyz_count++;
                    xyz_tick = ticks;
                    $display("XYZ_STATE %08h %08h %08h %08h %08h", ukf_output.ukf_x, ukf_output.ukf_y,
                             ukf_output.ukf_theta, ukf_output.ukf_omega, ukf_output.ukf_wheelB);
                    $write("XYZ_FACTOR");
                    for (int i = 0; i < 15; i++) $write(" %08h", corrected_s[i]);
                    $display("");
                    $write("XYZ_RESIDUAL");
                    for (int a = 21; a <= 24; a++) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (TEST_THIRD && xy_update_valid) begin
                    if (!busy || done || failed || x_update_valid || xyz_update_valid || ukf_output.ukf_valid)
                        $fatal(1, "xy-stage pulse does not precede the terminal pulse");
                    xy_count++;
                    xy_tick = ticks;
                    $display("XY_STATE %08h %08h %08h %08h %08h", ukf_output.ukf_x, ukf_output.ukf_y,
                             ukf_output.ukf_theta, ukf_output.ukf_omega, ukf_output.ukf_wheelB);
                    $write("XY_FACTOR");
                    for (int i = 0; i < 15; i++) $write(" %08h", corrected_s[i]);
                    $display("");
                    $write("XY_RESIDUAL");
                    for (int a = 21; a <= 24; a++) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                if (TEST_SECOND && x_update_valid) begin
                    if (!busy || done || failed || xy_update_valid || xyz_update_valid || ukf_output.ukf_valid)
                        $fatal(1, "x-stage pulse does not precede the terminal pulse");
                    x_count++;
                    x_tick = ticks;
                    $display("X_STATE %08h %08h %08h %08h %08h", ukf_output.ukf_x, ukf_output.ukf_y,
                             ukf_output.ukf_theta, ukf_output.ukf_omega, ukf_output.ukf_wheelB);
                    $write("X_FACTOR");
                    for (int i = 0; i < 15; i++) $write(" %08h", corrected_s[i]);
                    $display("");
                    $write("X_RESIDUAL");
                    for (int a = 21; a <= 24; a++) $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
            end
        end else begin
            running = 0;
        end
    end

    logic [31:0] fixtures [0:33];
    string vector_path;
    integer fixture_file, read_count, test_id, should_fail, abort_transaction;

    initial begin
        if (PROM_MATH_ZERO != 93 || PROM_MATH_ONE != 94 ||
            PROM_SR_BASE <= LAST_REG || PROM_SR_BASE + 9 >= PROM_MATH_BASE ||
            $bits(dut.correction_program.uop.src_a) < 7 ||
            $bits(dut.correction_program.uop.src_b) < 7 ||
            $bits(dut.correction_program.pc) != 7)
            $fatal(1, "invalid ROM or PC address map");
        for (int addr = 0; addr < 128; addr++)
            if (prom_math_read(addr[6:0]) !== (addr == 94 ? 32'h3f800000 : 32'b0))
                $fatal(1, "unexpected ROM value or address alias at %0d", addr);
        if ($low(dut.correction_program.regfile) != 0 ||
            $high(dut.correction_program.regfile) != LAST_REG)
            $fatal(1, "unexpected physical register allocation");
        if (!$value$plusargs("vectors=%s", vector_path))
            $fatal(1, "missing +vectors");
        fixture_file = $fopen(vector_path, "r");
        if (!fixture_file) $fatal(1, "could not open fixtures");
        repeat (2) @(negedge clock);
        reset = 0;
        while ($fscanf(fixture_file, "%d %d %d", test_id, should_fail, abort_transaction) == 3) begin
            for (int i = 0; i < 34; i++) begin
                read_count = $fscanf(fixture_file, "%h", fixtures[i]);
                if (read_count != 1) $fatal(1, "incomplete fixture");
            end
            $display("CASE %0d", test_id);
            x_count = 0;
            xy_count = 0;
            xyz_count = 0;
            third_mac_cycles = 0;
            third_div_cycles = 0;
            total_mac_cycles = 0;
            total_div_cycles = 0;
            pred_packet = {fixtures[0], fixtures[1], fixtures[2], fixtures[3], fixtures[4], 1'b1};
            convert_result = {fixtures[5], fixtures[6], fixtures[7], fixtures[8]};
            for (int i = 0; i < 15; i++) pred_s[i] = fixtures[9+i];
            for (int i = 0; i < 10; i++) measurement_s[i] = fixtures[24+i];
            corr_packet.ukf_pulse = 1;
            corr_packet.ukf_vn_valid = 1;
            request_tick = ticks + 1;
            @(negedge clock);
            corr_packet = '0;
            pred_packet = '0;
            pred_s = '0;
            if (!convert_start || !busy) $fatal(1, "converter was not started");
            repeat (3) @(negedge clock);
            convert_done = 1;
            @(negedge clock);
            convert_done = 0;
            // A second measurement beat during correction must be ignored.
            repeat (16) @(negedge clock);
            corr_packet.ukf_pulse = 1;
            corr_packet.ukf_vn_valid = 1;
            @(negedge clock);
            corr_packet = '0;
            if (abort_transaction) begin
                if (abort_transaction == 2) begin
                    wait (x_update_valid);
                    repeat (50) @(negedge clock);
                    if (x_count != 1 || !busy)
                        $fatal(1, "second-stage reset did not follow the x update");
                end else if (abort_transaction == 3) begin
                    wait (xy_update_valid);
                    repeat (50) @(negedge clock);
                    if (xy_count != 1 || !busy)
                        $fatal(1, "third-stage reset did not follow the xy update");
                end else if (abort_transaction == 4) begin
                    wait (xyz_update_valid);
                    repeat (50) @(negedge clock);
                    if (xyz_count != 1 || !busy)
                        $fatal(1, "fourth-stage reset did not follow the xyz update");
                end
                reset = 1;
                repeat (2) @(negedge clock);
                if (done || busy || failed || x_update_valid || xy_update_valid || xyz_update_valid ||
                    ukf_output.ukf_valid || corrected_s !== '0)
                    $fatal(1, "reset did not cancel correction");
                $display("RESET");
                reset = 0;
            end else begin
                wait (done);
                @(negedge clock);
                if (failed !== (should_fail != 0) ||
                    x_update_valid !== (!TEST_SECOND && should_fail == 0) ||
                    xy_update_valid !== (TEST_SECOND && !TEST_THIRD && should_fail == 0) ||
                    xyz_update_valid !== (TEST_THIRD && !TEST_FOURTH && should_fail == 0) ||
                    ukf_output.ukf_valid !== (TEST_FOURTH && should_fail == 0))
                    $fatal(1, "wrong completion status at PC %0d", dut.correction_program.pc);
                if (TEST_SECOND && x_count != (should_fail != 1 ? 1 : 0))
                    $fatal(1, "wrong number of stage pulses");
                if (TEST_THIRD && xy_count != ((should_fail == 0 || should_fail >= 3) ? 1 : 0))
                    $fatal(1, "wrong number of xy-stage pulses");
                if (TEST_FOURTH && xyz_count != ((should_fail == 0 || should_fail == 4) ? 1 : 0))
                    $fatal(1, "wrong number of xyz-stage pulses");
                if (should_fail) begin
                    $display("FAILURE %0d %0d", dut.correction_program.row, dut.correction_program.pc);
                end else begin
                    $display("CYCLES %0d %0d", ticks - request_tick, ticks - microcode_tick - 1);
                    if (TEST_FOURTH) begin
                        $display("STAGE_CYCLES %0d %0d %0d %0d", x_tick - microcode_tick - 1,
                                 xy_tick - x_tick, xyz_tick - xy_tick, ticks - xyz_tick);
                    end else if (TEST_THIRD) begin
                        $display("STAGE_CYCLES %0d %0d %0d", x_tick - microcode_tick - 1,
                                 xy_tick - x_tick, ticks - xy_tick);
                        $display("ARITH_CYCLES %0d %0d %0d %0d", third_mac_cycles, third_div_cycles,
                                 total_mac_cycles, total_div_cycles);
                    end else if (TEST_SECOND)
                        $display("STAGE_CYCLES %0d %0d", x_tick - microcode_tick - 1, ticks - x_tick);
                    $write("STATE %08h %08h %08h %08h %08h", ukf_output.ukf_x, ukf_output.ukf_y,
                           ukf_output.ukf_theta, ukf_output.ukf_omega, ukf_output.ukf_wheelB);
                    $display("");
                    $write("FACTOR");
                    for (int i = 0; i < 15; i++) $write(" %08h", corrected_s[i]);
                    $display("");
                    $write("RESIDUAL");
                    for (int a = 21; a <= 24; a++)
                        $write(" %08h", dut.correction_program.regfile[a]);
                    $display("");
                end
                @(negedge clock);
                if (done || busy || failed || x_update_valid || xy_update_valid || xyz_update_valid || ukf_output.ukf_valid)
                    $fatal(1, "completion pulses did not clear");
            end
        end
        if (mac_stalls == 0 || div_stalls == 0) $fatal(1, "arithmetic stalls were not exercised");
        $display("PASS stalls %0d %0d", mac_stalls, div_stalls);
        $finish;
    end

    initial #8000000 $fatal(1, "timeout");
endmodule
