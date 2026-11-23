`timescale 1ns/1ps
module tb_sigma_watchdog;
    parameter integer TEST_TIMEOUT=3;
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0, start=0, grant=1;
    logic signed [7:0] state [0:3], S [0:3][0:3], gamma;
    wire signed [7:0] sigma [0:8][0:3];
    wire busy, done, overflow, request, error;
    wire launch, clear, last, lane_busy, valid, lane_done, ov, acc_ov;
    wire signed [7:0] a, b, result;
    logic drop_valid=0, injected_valid=0;
    integer launches=0;
    sigma_generator #(.WIDTH(8), .FRAC(0), .MAC_TIMEOUT_CYCLES(TEST_TIMEOUT)) dut (
        .clk(clk), .resetn(resetn), .start(start), .state(state), .S(S),
        .gamma(gamma), .sigma(sigma), .busy(busy), .done(done),
        .overflow(overflow), .mac_request(request), .mac_grant(grant),
        .mac_start(launch), .mac_clear(clear), .mac_last(last),
        .mac_a(a), .mac_b(b), .mac_busy(lane_busy),
        .mac_result_valid((valid & ~drop_valid) | injected_valid), .mac_overflow(ov),
        .mac_acc_overflow(acc_ov), .mac_result(result), .error(error)
    );
    MAC_accumulator #(.WIDTH(8), .FRAC(0)) shared_mac (
        .clk(clk), .resetn(resetn), .start(launch), .clear(clear), .last(last),
        .a(a), .b(b), .result(result), .busy(lane_busy), .done(lane_done),
        .result_valid(valid), .overflow(ov), .acc_overflow(acc_ov)
    );
    always @(posedge clk) if(resetn && |launch) begin
        if(launch !== clear || launch !== last || !request || !grant)
            $fatal(1,"Invalid launch controls/ownership");
        launches=launches+1;
    end
    task tick;
        @(posedge clk); #1;
    endtask
    task begin_transaction;
        @(negedge clk); start=1;
        tick();
        @(negedge clk); start=0;
    endtask
    task expect_success;
        integer cycles;
        begin
            cycles=0;
            while(!done && !error && cycles<60) begin tick(); cycles++; end
            if(!done || error || busy || request || overflow)
                $fatal(1,"Normal completion failed");
            for(integer i=0;i<9;i++)
                for(integer j=0;j<4;j++) begin
                    if(i==0 || j<((i<=4) ? i-1 : i-5)) begin
                        if(sigma[i][j] !== 8'sd10) $fatal(1,"Center mismatch");
                    end else if(sigma[i][j] !== ((i<=4) ? 8'sd12 : 8'sd8))
                        $fatal(1,"Product mismatch: simultaneous clear/start/last");
                end
            tick();
            if(done) $fatal(1,"done must pulse");
        end
    endtask
    initial begin
        gamma=2;
        for(integer i=0;i<4;i++) begin
            state[i]=10;
            for(integer j=0;j<4;j++) S[i][j]=1;
        end
        tick(); @(negedge clk); resetn=1;
        // Waiting for ownership must not consume the completion watchdog.
        grant=0; begin_transaction();
        repeat(8) tick();
        if(error || launches!=0) $fatal(1,"Watchdog ran before launch");
        @(negedge clk); grant=1;
        wait(dut.phase==dut.MAC_LAUNCH);
        @(negedge clk); injected_valid=1;
        tick();
        if(launch || launches!=0 || error) $fatal(1,"Stale valid was not drained");
        @(negedge clk); injected_valid=0;
        expect_success();
        if(launches!=10) $fatal(1,"Expected ten products");
        // Lose the final product after nine successful products.
        begin_transaction();
        wait(launches==19);
        wait(dut.phase==dut.MAC_LAUNCH && dut.entry_index==9);
        @(negedge clk); drop_valid=1;
        wait(dut.phase==dut.WAIT_MAC_RESULT);
        repeat(TEST_TIMEOUT-1) tick();
        if(error) $fatal(1,"Timeout fired early");
        tick();
        if(!error || !busy || !request || done || |launch)
            $fatal(1,"Timeout did not enter safe error state");
        // Late completions and starts must not publish or release ownership.
        @(negedge clk); drop_valid=0; start=1; injected_valid=1;
        tick();
        @(negedge clk); injected_valid=0;
        repeat(5) begin
            tick();
            if(!error || !request || done || |launch || sigma[1][0]!==8'sd12)
                $fatal(1,"Fault state did not retain outputs/ownership");
        end
        @(negedge clk); resetn=0; start=0;
        tick();
        if(error || busy || request) $fatal(1,"Reset failed");
        @(negedge clk); resetn=1;
        begin_transaction(); expect_success();
        $display("PASS: simultaneous controls, watchdog, ownership retention, reset recovery");
        $finish;
    end
    initial begin #10000; $fatal(1,"Test timed out"); end
endmodule
