`timescale 1ns/1ps
module tb_sigma_reference;
    parameter integer WIDTH=16, FRAC=4, CASES=128;
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0, start=0, contender=0;
    logic signed [WIDTH-1:0] state [0:3], S [0:3][0:3], gamma;
    wire signed [WIDTH-1:0] sigma [0:8][0:3];
    wire busy, done, overflow, request, error;
    wire launch, clear, last, lane_busy, valid, lane_done, ov, acc_ov;
    wire signed [WIDTH-1:0] a, b, result;
    logic [WIDTH-1:0] data [0:CASES*58-1];
    string vector_path;
    integer owner=0, other_grants=0, launches=0;
    wire grant=owner==1;
    wire other_grant=owner==2;
    // Test-only non-preemptive arbiter. Production arbitration is not present
    // in this repository. Release only after request falls and lanes drain.
    always @(posedge clk) begin
        if(!resetn) owner<=0;
        else case(owner)
            0: if(!(|lane_busy) && !(|valid)) begin
                if(request) owner<=1;
                else if(contender) owner<=2;
            end
            1: if(!request && !(|lane_busy) && !(|valid)) owner<=0;
            2: if(!contender && !(|lane_busy) && !(|valid)) owner<=0;
        endcase
        if(resetn && other_grant) other_grants<=other_grants+1;
    end
    wire other_start=other_grant && contender && !(|lane_busy);
    wire array_start=grant ? launch : (other_start ? 1'b1 : 1'b0);
    wire array_clear=grant ? clear : 1'b1;
    wire array_last=grant ? last : 1'b1;
    wire signed [WIDTH-1:0] array_a=grant ? a : WIDTH'(1);
    wire signed [WIDTH-1:0] array_b=grant ? b : WIDTH'(1);
    sigma_generator #(.WIDTH(WIDTH), .FRAC(FRAC)) dut (
        .clk(clk), .resetn(resetn), .start(start), .state(state), .S(S),
        .gamma(gamma), .sigma(sigma), .busy(busy), .done(done),
        .overflow(overflow), .mac_request(request), .mac_grant(grant),
        .mac_start(launch), .mac_clear(clear), .mac_last(last),
        .mac_a(a), .mac_b(b), .mac_busy(lane_busy), .mac_result_valid(valid),
        .mac_overflow(ov), .mac_acc_overflow(acc_ov), .mac_result(result), .error(error)
    );
    MAC_accumulator #(.WIDTH(WIDTH), .FRAC(FRAC)) shared_mac (
        .clk(clk), .resetn(resetn), .start(array_start), .clear(array_clear), .last(array_last),
        .a(array_a), .b(array_b), .result(result), .busy(lane_busy), .done(lane_done),
        .result_valid(valid), .overflow(ov), .acc_overflow(acc_ov)
    );
    logic delay1=0, delay2=0;
    logic ownership_held=0, overflow_seen=0;
    always @(posedge clk) begin
        if(!resetn) begin
            delay1<=0; delay2<=0; ownership_held<=0; overflow_seen<=0;
        end
        else begin
            if(valid !== delay2) $fatal(1,"Missing, extra or mistimed lane completion");
            delay1<=array_start; delay2<=delay1;
            if(request && grant && other_grant) $fatal(1,"Ownership stolen");
            if(ownership_held && request && !grant) $fatal(1,"Grant withdrawn before release");
            if(!request) ownership_held<=0;
            else if(grant) ownership_held<=1;
            if(start && !busy) overflow_seen<=0;
            else begin
                if(overflow_seen && !overflow) $fatal(1,"Overflow cleared within transaction");
                if(overflow) overflow_seen<=1;
            end
            if(|launch) begin
                if(!grant || launch!==clear || launch!==last)
                    $fatal(1,"Invalid simultaneous controls");
                if(lane_busy || valid || delay1 || delay2)
                    $fatal(1,"Overlapping products");
                launches<=launches+1;
            end
        end
    end
    task tick;
        @(posedge clk); #1;
    endtask
    task reset_all;
        @(negedge clk); resetn=0; start=0; contender=0;
        tick();
        if(busy || done || overflow || error || request || |lane_busy || |valid)
            $fatal(1,"Reset did not clear status");
        for(integer i=0;i<9;i++) for(integer j=0;j<4;j++)
            if(sigma[i][j] !== 0) $fatal(1,"Reset did not clear sigma");
        @(negedge clk); resetn=1;
    endtask
    task begin_transaction;
        @(negedge clk); start=1;
        tick();
        if(overflow) $fatal(1,"Overflow not cleared at acceptance");
        @(negedge clk); start=0;
    endtask
    integer base, before_launches;
    initial begin
        if(!$value$plusargs("vectors=%s",vector_path)) $fatal(1,"Missing vectors");
        $readmemh(vector_path,data);
        reset_all();
        for(integer n=0;n<CASES;n++) begin
            base=n*58;
            for(integer j=0;j<4;j++) state[j]=data[base+j];
            for(integer r=0;r<4;r++) for(integer c=0;c<4;c++) S[r][c]=data[base+4+r*4+c];
            gamma=data[base+20]; before_launches=launches;
            begin_transaction();
            wait(grant); @(negedge clk); contender=1;
            // A competing datapath continuously requests the shared array.
            while(!done) begin
                tick();
                if(error || (request && other_grant)) $fatal(1,"Contention failure");
            end
            if(launches-before_launches!=10) $fatal(1,"Expected ten products");
            if(overflow !== data[base+57][0]) $fatal(1,"Overflow mismatch case %0d",n);
            for(integer i=0;i<9;i++) for(integer j=0;j<4;j++)
                if(sigma[i][j] !== data[base+21+i*4+j])
                    $fatal(1,"Reference mismatch case=%0d point=%0d component=%0d",n,i,j);
            repeat(3) begin
                tick();
                if(done || overflow !== data[base+57][0]) $fatal(1,"Sticky overflow/done failure");
            end
            wait(other_grant); tick();
            @(negedge clk); contender=0;
            wait(owner==0);
        end
        if(other_grants==0) $fatal(1,"Contender was never served");
        // Abort in every active FSM phase, including in-flight MAC work and
        // immediately before publication. The joint reset must clear old data.
        for(integer p=1;p<=7;p++) begin
            begin_transaction();
            wait(dut.phase==p);
            reset_all();
            repeat(4) begin tick(); if(done || busy) $fatal(1,"Aborted transaction resumed"); end
        end
        begin_transaction();
        wait(done); tick();
        if(error) $fatal(1,"Recovery failed");
        $display("PASS: %0d software-reference cases WIDTH=%0d FRAC=%0d; timing, overflow, contention, active resets",CASES,WIDTH,FRAC);
        $finish;
    end
    initial begin #1000000; $fatal(1,"Regression timed out"); end
endmodule
