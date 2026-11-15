`timescale 1ns/1ps
module tb_sigma_reference;
    parameter integer WIDTH=16, FRAC=4, CASES=127;
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0, start=0, contender=0;
    logic signed [WIDTH-1:0] state [0:4], S [0:4][0:4], gamma;
    wire signed [WIDTH-1:0] sigma [0:10][0:4];
    wire busy, done, overflow, request, error;
    wire [10:0] launch, clear, last, lane_busy, valid, lane_done, ov, acc_ov;
    wire signed [WIDTH-1:0] a [0:10], b [0:10], result [0:10];
    logic [WIDTH-1:0] data [0:CASES*87-1];
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
    wire [10:0] array_start=grant ? launch : (other_start ? 11'h001 : 11'h000);
    wire [10:0] array_clear=grant ? clear : 11'h001;
    wire [10:0] array_last=grant ? last : 11'h001;
    wire signed [WIDTH-1:0] array_a [0:10], array_b [0:10];
    for(genvar l=0;l<11;l++) begin
        assign array_a[l]=grant ? a[l] : 1;
        assign array_b[l]=grant ? b[l] : 1;
    end
    sigma_generator #(.WIDTH(WIDTH), .FRAC(FRAC)) dut (
        .clk(clk), .resetn(resetn), .start(start), .state(state), .S(S),
        .gamma(gamma), .sigma(sigma), .busy(busy), .done(done),
        .overflow(overflow), .mac_request(request), .mac_grant(grant),
        .mac_start(launch), .mac_clear(clear), .mac_last(last),
        .mac_a(a), .mac_b(b), .mac_busy(lane_busy), .mac_result_valid(valid),
        .mac_overflow(ov), .mac_acc_overflow(acc_ov), .mac_result(result), .error(error)
    );
    MAC_array #(.WIDTH(WIDTH), .FRAC(FRAC)) array_dut (
        .clk(clk), .resetn(resetn), .start(array_start), .clear(array_clear), .last(array_last),
        .a(array_a), .b(array_b), .result(result), .busy(lane_busy), .done(lane_done),
        .result_valid(valid), .overflow(ov), .acc_overflow(acc_ov)
    );
    logic [10:0] delay1=0, delay2=0;
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
                if(launch !== (dut.batch ? 11'h00f : 11'h7ff))
                    $fatal(1,"Incorrect batch mask");
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
        for(integer i=0;i<11;i++) for(integer j=0;j<5;j++)
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
            base=n*87;
            for(integer j=0;j<5;j++) state[j]=data[base+j];
            for(integer r=0;r<5;r++) for(integer c=0;c<5;c++) S[r][c]=data[base+5+r*5+c];
            gamma=data[base+30]; before_launches=launches;
            begin_transaction();
            wait(grant); @(negedge clk); contender=1;
            // A competing datapath continuously requests the shared array.
            while(!done) begin
                tick();
                if(error || (request && other_grant)) $fatal(1,"Contention failure");
            end
            if(launches-before_launches!=2) $fatal(1,"Expected two batches");
            if(overflow !== data[base+86][0]) $fatal(1,"Overflow mismatch case %0d",n);
            for(integer i=0;i<11;i++) for(integer j=0;j<5;j++)
                if(sigma[i][j] !== data[base+31+i*5+j])
                    $fatal(1,"Reference mismatch case=%0d point=%0d component=%0d",n,i,j);
            repeat(3) begin
                tick();
                if(done || overflow !== data[base+86][0]) $fatal(1,"Sticky overflow/done failure");
            end
            wait(other_grant); tick();
            @(negedge clk); contender=0;
            wait(owner==0);
        end
        if(other_grants==0) $fatal(1,"Contender was never served");
        // Abort in every active FSM phase, including in-flight MAC work and
        // immediately before publication. The joint reset must clear old data.
        for(integer p=1;p<=6;p++) begin
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
