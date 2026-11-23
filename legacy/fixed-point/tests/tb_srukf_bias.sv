`timescale 1ns/1ps
module tb_srukf_bias #(parameter integer STEPS=12);
    localparam integer SLOT=62500;
    localparam time TIMEOUT_NS=((64'(STEPS)*2+2)*SLOT)*20;
    localparam logic [63:0] EPOCH=64'd4200000000;
    reg clk=0;
    always #10 clk=~clk;
    reg resetn=0, init=0, sample_valid=0;
    reg [63:0] init_ts=EPOCH, sample_ts=0;
    reg [159:0] init_state, process_std;
    reg [127:0] sample;
    reg [479:0] init_root;
    reg [319:0] measurement_root;
    reg signed [31:0] wheel_offset, wheel_std;
    reg signed [31:0] wheel_velocity;
    reg mac_available=1, cordic_available=1;
    wire sample_pop, sample_rejected, predict_start, correct_start;
    wire predict_done, correct_done, state_valid, fault, deadline_missed;
    wire overflow, saturation, negative_radicand, invalid_pivot;
    wire [63:0] state_ts;
    wire [159:0] state;
    wire [479:0] root;
    srukf_bias_filter dut(.*);

    reg [31:0] fixture[0:36+46*STEPS];
    string path;
    integer cycle=0, epoch, started, max_pred=0, max_corr=0, corrections=0;
    reg skip_idle, stalls, reject_zero_bias;
    integer k=0, offset=0, step_entered=0;

    always @(posedge clk) begin
        cycle=cycle+1;
        if (init) epoch=cycle;
        if (k>0 && k<=STEPS && cycle-step_entered>3*SLOT)
            $fatal(1,"step %0d did not complete",k);
        if (resetn && fault && !reject_zero_bias)
            $fatal(1,"k=%0d phase=%0d overflow=%0b saturation=%0b negative=%0b pivot=%0b deadline=%0b",
                   k,dut.phase,overflow,saturation,negative_radicand,invalid_pivot,deadline_missed);
    end
    always @(negedge clk) begin
        if (stalls) begin
            mac_available <= cycle%127>=19;
            cordic_available <= cycle%101>=23;
        end
    end

    task automatic check_commit(input integer start_cycle, expected_base, input reg correction);
        integer elapsed;
        begin
            wait (state_valid === 1'b1);
            #2;
            elapsed=cycle-start_cycle;
            if (elapsed<=0 || elapsed>SLOT || state_ts!=EPOCH+2500*k)
                $fatal(1,"commit deadline/timestamp k=%0d elapsed=%0d ts=%0d",k,elapsed,state_ts);
            for (integer i=0;i<5;i=i+1)
                if (state[32*i+:32] !== fixture[expected_base+i])
                    $fatal(1,"k=%0d corr=%0d state[%0d] RTL %0d reference %0d",
                           k,correction,i,$signed(state[32*i+:32]),$signed(fixture[expected_base+i]));
            for (integer i=0;i<15;i=i+1)
                if (root[32*i+:32] !== fixture[expected_base+5+i])
                    $fatal(1,"k=%0d corr=%0d factor[%0d] RTL %0d reference %0d",
                           k,correction,i,$signed(root[32*i+:32]),$signed(fixture[expected_base+5+i]));
            if (correction && elapsed>max_corr) max_corr=elapsed;
            if (!correction && elapsed>max_pred) max_pred=elapsed;
            @(negedge clk);
        end
    endtask

    // Accelerate empty time between slots only. The measured phase itself
    // always runs on the complete RTL with real clock edges and FSM state.
    task automatic idle_to_boundary;
        begin
            if (skip_idle) begin
                if (dut.scheduler.operation!=0) $fatal(1,"attempt to skip active clock");
                dut.scheduler.counter=16'(SLOT-1);
            end
        end
    endtask

    initial begin
        if (!$value$plusargs("vectors=%s",path)) $fatal(1,"missing fixture");
        $readmemh(path,fixture);
        skip_idle=$test$plusargs("skip_idle");
        stalls=$test$plusargs("stalls");
        reject_zero_bias=$test$plusargs("reject_zero_bias");
        for (integer i=0;i<5;i=i+1) begin
            init_state[32*i+:32]=fixture[i];
            process_std[32*i+:32]=fixture[20+i];
        end
        for (integer i=0;i<15;i=i+1) init_root[32*i+:32]=fixture[5+i];
        for (integer i=0;i<10;i=i+1) measurement_root[32*i+:32]=fixture[25+i];
        wheel_offset=fixture[35]; wheel_std=fixture[36];
        if (reject_zero_bias) init_root[448+:32]=0;
        repeat (3) @(negedge clk);
        resetn=1; init=1;
        @(negedge clk);
        init=0;
        if (reject_zero_bias) begin
            repeat (4) @(negedge clk);
            if (!fault || !invalid_pivot || state_valid || predict_start || deadline_missed)
                $fatal(1,"zero bias covariance was accepted");
            $display("PASS rejected zero initial bias covariance");
            $finish;
        end
        for (k=1;k<=STEPS;k=k+1) begin
            step_entered=cycle;
            offset=37+(k-1)*46;
            wheel_velocity=fixture[offset];
            sample_valid=fixture[offset+1]!=0;
            for (integer i=0;i<4;i=i+1) sample[32*i+:32]=fixture[offset+2+i];
            sample_ts=EPOCH+2500*k;
            idle_to_boundary();
            wait (predict_start === 1'b1);
            started=cycle;
            if (!skip_idle && cycle!=epoch+(2*k-1)*SLOT) $fatal(1,"prediction schedule");
            check_commit(started,offset+6,0);
            idle_to_boundary();
            if (sample_valid) begin
                wait (correct_start === 1'b1);
                started=cycle;
                if (!skip_idle && cycle!=epoch+2*k*SLOT) $fatal(1,"correction schedule");
                if (!sample_pop) $fatal(1,"sample not popped at correction start");
                @(posedge clk); #2;
                // A real FIFO may change its output after consuming the sample.
                sample_valid=0; sample='1;
                check_commit(started,offset+26,1);
                corrections=corrections+1;
            end else begin
                wait (dut.scheduler.prediction_slot === 1'b1);
                #2;
                if (correct_start || sample_pop) $fatal(1,"correction launched during dropout");
                @(negedge clk);
            end
            if (k%2000==0 && k<STEPS) $display("Checked %0d/%0d predictions",k,STEPS);
        end
        $display("PASS predictions=%0d corrections=%0d max_predict=%0d max_correct=%0d",
                 STEPS,corrections,max_pred,max_corr);
        $finish;
    end
    initial begin
        #TIMEOUT_NS;
        $fatal(1,"timeout");
    end
endmodule
