`timescale 1ns/1ps
module tb_cordic;
    parameter integer WIDTH=32, FRAC=16, ITERATIONS=20, CASES=1;
    parameter integer LATENCY=72;
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0, start=0;
    logic [1:0] mode=0;
    logic signed [WIDTH-1:0] theta=0;
    wire signed [WIDTH-1:0] sin_theta, cos_theta;
    wire busy, done, overflow, error;
    logic [WIDTH-1:0] vectors [0:CASES-1];
    logic [2*WIDTH+1:0] held;
    string vector_path;

    cordic #(.WIDTH(WIDTH), .FRAC(FRAC), .ITERATIONS(ITERATIONS)) dut (.*);

    task tick;
        @(posedge clk); #1;
    endtask

    task launch(input logic [1:0] request_mode, input logic [WIDTH-1:0] angle);
        @(negedge clk);
        mode=request_mode;
        theta=angle;
        start=1;
        tick();
        if (!busy || done) $fatal(1,"Request acceptance failed");
        @(negedge clk);
        start=0;
    endtask

    initial begin
        if (!$value$plusargs("vectors=%s",vector_path)) $fatal(1,"Missing vectors");
        $readmemh(vector_path,vectors);
        tick();
        @(negedge clk); resetn=1;
        for (integer i=0; i<CASES; i++) begin
            held={sin_theta,cos_theta,overflow,error};
            launch(0,vectors[i]);
            for (integer cycle=1; cycle<=LATENCY; cycle++) begin
                // Drive different inputs and intervening requests while busy.
                theta=~vectors[i];
                mode=2;
                start=cycle==2 || cycle==LATENCY;
                tick();
                if (cycle<LATENCY) begin
                    if (!busy || done || {sin_theta,cos_theta,overflow,error} !== held)
                        $fatal(1,"Early completion or publication");
                end else if (busy || !done || error) begin
                    $fatal(1,"Rotation completion failed");
                end
                @(negedge clk);
            end
            start=0;
            $display("R %0d %0d %0d",sin_theta,cos_theta,overflow);
            tick();
            if (done || busy) $fatal(1,"Done pulse or ignored start failed");
        end
        for (integer unsupported=1; unsupported<=3; unsupported++) begin
            held={sin_theta,cos_theta,overflow,error};
            launch(2'(unsupported),'0);
            tick();
            if (busy || !done || !error || overflow ||
                {sin_theta,cos_theta} !== held[2*WIDTH+1:2])
                $fatal(1,"Unsupported mode status or output changed");
            tick();
            if (done) $fatal(1,"Unsupported done must pulse");
        end
        // Abort during angle reduction and during coordinate iteration.
        for (integer abort_at=2; abort_at<LATENCY-1; abort_at+=LATENCY-4) begin
            launch(0,'1);
            repeat (abort_at) tick();
            if (!busy) $fatal(1,"Expected active request at reset");
            @(negedge clk); resetn=0;
            if (!busy) $fatal(1,"Reset took effect before its sampling edge");
            tick();
            if (busy || done || error || overflow || sin_theta !== '0 || cos_theta !== '0)
                $fatal(1,"Reset did not abort and clear outputs");
            @(negedge clk); resetn=1;
            repeat (LATENCY) begin
                tick();
                if (busy || done) $fatal(1,"Aborted transaction resumed");
            end
        end
        launch(0,'0);
        repeat (LATENCY) tick();
        if (!done || busy || error) $fatal(1,"Reset recovery failed");
        $finish;
    end
endmodule
