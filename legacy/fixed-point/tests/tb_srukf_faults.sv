`timescale 1ns/1ps
module tb_srukf_faults;
    reg clk=0;
    always #10 clk=~clk;
    reg resetn=0, init=0, sample_valid=0;
    reg [63:0] init_ts=1234, sample_ts=3734;
    reg [127:0] init_state=0, sample=0;
    reg [511:0] init_root, process_root=0, measurement_root=0;
    reg signed [31:0] wheel_velocity=0;
    reg mac_available=1, cordic_available=1;
    wire sample_pop, sample_rejected, predict_start, correct_start;
    wire predict_done, correct_done, state_valid, fault, deadline_missed;
    wire overflow, saturation, negative_radicand, invalid_pivot;
    wire [63:0] state_ts;
    wire [127:0] state;
    wire [511:0] root;
    srukf_filter dut(.*);

    task automatic boot(input signed [31:0] x, pivot, input reg sample_on, allow_mac);
        begin
            @(negedge clk); resetn=0; init=0;
            repeat(2) @(negedge clk);
            init_state={96'd0,x};
            init_root=0;
            for (integer i=0;i<4;i=i+1) init_root[160*i+:32]=pivot;
            sample_valid=sample_on; mac_available=allow_mac;
            resetn=1; init=1;
            @(negedge clk); init=0;
        end
    endtask
    task automatic expect_fault(input reg deadline, range_error, pivot_error,
                                input reg [63:0] stamp);
        begin
            wait(fault); #2;
            if ({deadline_missed,overflow,saturation,invalid_pivot} !==
                {deadline,range_error,range_error,pivot_error} ||
                negative_radicand || state_ts!=stamp || state_valid)
                $fatal(1,"unexpected fault cycle or flags");
            repeat(2) @(negedge clk);
            if (!fault || state_valid || predict_done || correct_done) $fatal(1,"fault not sticky");
        end
    endtask
    initial begin
        boot(0,0,0,1);
        expect_fault(0,0,1,1234);
        boot(32'sh7fffffff,32'sd33554432,0,1);
        expect_fault(0,1,0,1234);
        if (state !== init_state || root !== init_root) $fatal(1,"prediction fault wrote partial state");

        // Zero noise makes the first corrected factor singular. Preserve
        // the entire committed prediction on rejection of the downdate.
        boot(0,32'sd33554432,1,1);
        wait(state_valid); #2;
        begin : preserve_prediction
            reg [511:0] last_root;
            reg [127:0] last_state;
            last_root=root; last_state=state;
            expect_fault(0,0,1,3734);
            if (state !== last_state || root !== last_root) $fatal(1,"correction fault wrote partial state");
        end

        boot(0,32'sd33554432,1,0);
        expect_fault(1,0,0,1234);
        if (state !== init_state || root !== init_root) $fatal(1,"timeout wrote partial state");
        $display("PASS invalid initialization, state saturation, singular downdate, deadline and atomic commit");
        $finish;
    end
    initial begin
        repeat(600000) @(posedge clk);
        $fatal(1,"timeout");
    end
endmodule
