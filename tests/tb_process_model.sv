`timescale 1ns/1ps
module process_test_arbiter (
    input logic clk, resetn, request, peer_request, busy, completion,
    output wire grant, peer_grant
);
    typedef enum logic [1:0] {FREE, PROCESS, PEER} owner_t;
    owner_t owner;
    wire drained = !busy && !completion;
    assign peer_grant = owner == PEER || (owner == FREE && peer_request && drained);
    assign grant = owner == PROCESS || (owner == FREE && !peer_request && request && drained);

    always @(posedge clk) begin
        if (!resetn) owner <= FREE;
        else case (owner)
            FREE: if (peer_grant) owner <= PEER;
                  else if (grant) owner <= PROCESS;
            PROCESS: if (!request && drained) owner <= FREE;
            PEER: if (!peer_request && drained) owner <= FREE;
            default: owner <= FREE;
        endcase
    end
endmodule

module tb_process_model;
    parameter integer CASES = 1;
    localparam integer MAC_TIMEOUT = 5, CORDIC_TIMEOUT = 80;
    localparam integer EXPECTED_CYCLES = 760;
    localparam integer WORDS = 38;
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0, start=0;
    logic signed [31:0] sigma [0:8][0:3], v_wheel, dt;
    wire signed [31:0] propagated [0:8][0:3];
    wire busy, done, overflow, error;
    wire mac_request, mac_grant, mac_start, mac_clear, mac_last;
    wire mac_busy, mac_done, mac_result_valid, mac_overflow, mac_acc_overflow;
    wire signed [31:0] mac_a, mac_b, mac_result;
    wire signed [71:0] mac_full_result;
    wire cordic_request, cordic_grant, cordic_start, cordic_busy, cordic_done;
    wire cordic_overflow, cordic_error;
    wire [1:0] cordic_mode;
    wire signed [31:0] cordic_theta, cordic_sin, cordic_cos;
    wire unit_mac_valid, unit_cordic_done, unit_cordic_error;
    logic drop_mac=0, drop_cordic=0, inject_cordic_error=0;
    logic late_mac=0, late_cordic=0;
    logic mac_peer=0, cordic_peer=0;
    wire mac_peer_grant, cordic_peer_grant;
    wire peer_mac_start = mac_peer && mac_peer_grant && !mac_busy && !unit_mac_valid;
    wire peer_cordic_start = cordic_peer && cordic_peer_grant && !cordic_busy && !unit_cordic_done;

    assign mac_result_valid = (unit_mac_valid && !drop_mac) || late_mac;
    assign cordic_done = (unit_cordic_done && !drop_cordic) || late_cordic;
    assign cordic_error = unit_cordic_error || inject_cordic_error;

    process_model #(.MAC_TIMEOUT_CYCLES(MAC_TIMEOUT), .CORDIC_TIMEOUT_CYCLES(CORDIC_TIMEOUT)) dut (.*);
    process_test_arbiter mac_arbiter (
        clk, resetn, mac_request, mac_peer, mac_busy, unit_mac_valid, mac_grant, mac_peer_grant
    );
    process_test_arbiter cordic_arbiter (
        clk, resetn, cordic_request, cordic_peer, cordic_busy, unit_cordic_done,
        cordic_grant, cordic_peer_grant
    );
    MAC_accumulator shared_mac (
        .clk(clk), .resetn(resetn), .start(mac_grant ? mac_start : peer_mac_start),
        .clear(mac_grant ? mac_clear : 1'b1), .last(mac_grant ? mac_last : 1'b1),
        .a(mac_grant ? mac_a : 32'sd131072), .b(mac_grant ? mac_b : 32'sd196608),
        .result(mac_result), .busy(mac_busy), .done(mac_done), .result_valid(unit_mac_valid),
        .overflow(mac_overflow), .acc_overflow(mac_acc_overflow), .full_result(mac_full_result)
    );
    cordic shared_cordic (
        .clk(clk), .resetn(resetn), .start(cordic_grant ? cordic_start : peer_cordic_start),
        .mode(cordic_grant ? cordic_mode : 2'd0), .theta(cordic_grant ? cordic_theta : 32'sd0),
        .busy(cordic_busy), .done(unit_cordic_done), .overflow(cordic_overflow),
        .error(unit_cordic_error), .sin_theta(cordic_sin), .cos_theta(cordic_cos)
    );

    integer mac_launches=0, cordic_launches=0;
    logic hold_mac=0, hold_cordic=0;
    always @(posedge clk) begin
        if (!resetn) begin
            hold_mac <= 0;
            hold_cordic <= 0;
        end else begin
            if (mac_request && cordic_request) $fatal(1,"Resources requested together");
            if ((mac_grant && mac_peer_grant) || (cordic_grant && cordic_peer_grant))
                $fatal(1,"Overlapping grants");
            if ((hold_mac && mac_request && !mac_grant) ||
                (hold_cordic && cordic_request && !cordic_grant))
                $fatal(1,"Ownership withdrawn before release");
            hold_mac <= mac_request && mac_grant;
            hold_cordic <= cordic_request && cordic_grant;
            if (mac_start) begin
                if (!mac_grant || !mac_request || mac_busy || unit_mac_valid || !mac_clear || !mac_last)
                    $fatal(1,"Invalid MAC launch");
                mac_launches <= mac_launches+1;
            end
            if (cordic_start) begin
                if (!cordic_grant || !cordic_request || cordic_busy || unit_cordic_done || cordic_mode!=0)
                    $fatal(1,"Invalid CORDIC launch");
                cordic_launches <= cordic_launches+1;
            end
            if (mac_peer_grant && unit_mac_valid && mac_result !== 32'sd393216)
                $fatal(1,"MAC contender result");
        end
    end

    logic [31:0] vectors [0:CASES*WORDS-1];
    logic signed [31:0] held [0:8][0:3];
    string path;
    task tick;
        @(posedge clk); #1;
    endtask
    task snapshot;
        for (integer i=0; i<9; i++)
            for (integer j=0; j<4; j++) held[i][j]=propagated[i][j];
    endtask
    task check_held;
        for (integer i=0; i<9; i++)
            for (integer j=0; j<4; j++)
                if (held[i][j] !== propagated[i][j]) $fatal(1,"Partial publication [%0d][%0d]",i,j);
    endtask
    task set_inputs(input integer index);
        v_wheel=vectors[index*WORDS]; dt=vectors[index*WORDS+1];
        for (integer i=0; i<9; i++)
            for (integer j=0; j<4; j++) sigma[i][j]=vectors[index*WORDS+2+4*i+j];
    endtask
    task launch;
        @(negedge clk); start=1;
        tick();
        if (!busy || done) $fatal(1,"Start acceptance");
        @(negedge clk); start=0;
    endtask
    task reset_all;
        @(negedge clk); resetn=0; start=0; mac_peer=0; cordic_peer=0;
        tick();
        if (busy || error || overflow || done || mac_request || cordic_request || mac_busy || cordic_busy)
            $fatal(1,"Reset status");
        for (integer i=0; i<9; i++)
            for (integer j=0; j<4; j++)
                if (propagated[i][j] !== 0) $fatal(1,"Reset results");
        @(negedge clk); resetn=1; drop_mac=0; drop_cordic=0; inject_cordic_error=0;
    endtask
    task complete(input integer target_latency, input bit report_values);
        integer cycles, mac_before, cordic_before;
        bit seen_overflow;
        begin
            cycles=0; seen_overflow=0;
            mac_before=mac_launches; cordic_before=cordic_launches;
            while (!done && !error && cycles<1200) begin
                // Inputs and start are disturbed only after acceptance.
                start=(cycles==4);
                tick();
                cycles++;
                if (!done) begin
                    check_held();
                    if (!busy) $fatal(1,"Busy dropped early");
                end
                if (seen_overflow && !overflow) $fatal(1,"Overflow was not sticky");
                seen_overflow |= overflow;
                @(negedge clk);
            end
            start=0;
            if (!done || error || busy || mac_request || cordic_request ||
                (target_latency != 0 && cycles != target_latency) ||
                (target_latency == 0 && cycles <= EXPECTED_CYCLES) ||
                mac_launches-mac_before != 28 || cordic_launches-cordic_before != 9)
                $fatal(1,"Completion cycles=%0d mac=%0d cordic=%0d error=%b",
                       cycles,mac_launches-mac_before,cordic_launches-cordic_before,error);
            if (report_values) begin
                $write("R %0d",overflow);
                for (integer i=0; i<9; i++)
                    for (integer j=0; j<4; j++) $write(" %0d",propagated[i][j]);
                $display("");
            end
            tick();
            if (done || busy) $fatal(1,"Done pulse");
        end
    endtask
    task fault_check(input integer limit, input bit expect_mac, expect_cordic);
        repeat (limit-1) begin
            tick();
            if (error || done) $fatal(1,"Early timeout");
            check_held();
        end
        tick();
        if (!error || done || !busy || mac_request !== expect_mac || cordic_request !== expect_cordic)
            $fatal(1,"Fault did not retain resource ownership");
        @(negedge clk); start=1; drop_mac=0; drop_cordic=0;
        late_mac=expect_mac; late_cordic=expect_cordic;
        tick();
        @(negedge clk); late_mac=0; late_cordic=0;
        repeat (4) begin
            tick();
            check_held();
            if (!error || done || !busy || mac_start || cordic_start ||
                mac_request !== expect_mac || cordic_request !== expect_cordic)
                $fatal(1,"Fault recovery without reset");
        end
        reset_all();
    endtask
    initial begin
        if (!$value$plusargs("vectors=%s",path)) $fatal(1,"Missing vectors");
        $readmemh(path,vectors);
        reset_all();
        for (integer n=0; n<CASES; n++) begin
            @(negedge clk); set_inputs(n); snapshot();
            launch();
            dt=-1; v_wheel=0;
            for (integer i=0; i<9; i++)
                for (integer j=0; j<4; j++) sigma[i][j]='x;
            complete(EXPECTED_CYCLES,1);
        end

        // Existing clients hold both resources before process starts.
        @(negedge clk); set_inputs(0); snapshot(); mac_peer=1; cordic_peer=1;
        wait (cordic_busy && mac_busy);
        launch();
        fork
            begin
                repeat (100) tick();
                if (cordic_launches != CASES*9 || mac_launches != CASES*28)
                    $fatal(1,"Operation issued while competing client held ownership");
                @(negedge clk); cordic_peer=0;
                wait (mac_request);
                repeat (9) tick();
                if (mac_launches != CASES*28) $fatal(1,"MAC launched without ownership");
                @(negedge clk); mac_peer=0;
            end
            complete(0,0);
        join

        // Drop a completion after real operations launch and verify exact deadlines.
        @(negedge clk); set_inputs(0); snapshot(); drop_mac=1;
        launch();
        wait(mac_busy); #1;
        fault_check(MAC_TIMEOUT,1,0);
        @(negedge clk); set_inputs(0); snapshot(); drop_cordic=1;
        launch();
        wait(cordic_busy); #1;
        fault_check(CORDIC_TIMEOUT,0,1);

        @(negedge clk); set_inputs(0); snapshot(); inject_cordic_error=1;
        launch();
        wait(unit_cordic_done); tick();
        check_held();
        if (!error || done || !cordic_request || mac_request) $fatal(1,"CORDIC error ignored");
        reset_all();

        @(negedge clk); set_inputs(0); snapshot(); dt=-1;
        launch();
        check_held();
        if (!error || mac_request || cordic_request) $fatal(1,"Negative timestep accepted");
        reset_all();

        // Reset each real unit with work in flight, then exercise recovery.
        @(negedge clk); set_inputs(0); launch();
        wait(cordic_busy); reset_all();
        @(negedge clk); set_inputs(0); launch();
        wait(mac_busy); reset_all();
        @(negedge clk); set_inputs(0); snapshot(); launch();
        complete(EXPECTED_CYCLES,0);
        $display("PASS process timing, capture, arbitration, faults and reset");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Testbench timed out"); end
endmodule
