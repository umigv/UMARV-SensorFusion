`timescale 1ns/1ps
module tb_fp32_mac;
    logic clk = 0;
    always #10 clk = ~clk;
    integer edge_count = 0;
    integer accepted_edge, previous_done_edge;
    bit trace_latency;
    always @(posedge clk) edge_count = edge_count + 1;
    logic rst_n = 1, start = 0, clear = 0, last = 0;
    logic [31:0] a = 0, b = 0, result;
    logic busy, done, result_valid;
    logic [4:0] exception_flags;
    fp32_mac dut (.*);

    string path;
    integer file_handle, fields, count, latency, max_latency;
    logic request_clear, request_last;
    logic [31:0] operand_a, operand_b, expected;
    logic [4:0] expected_flags;
    logic [31:0] accepted_addend, expected_accumulator;

    task automatic reset_dut;
        @(negedge clk);
        #1 rst_n = 0;
        #1;
        if ({busy, done, result_valid, result, exception_flags} !== '0)
            $fatal(1, "Reset failed");
        @(negedge clk);
        rst_n = 1;
    endtask

    initial begin
        reset_dut();
        trace_latency = $test$plusargs("latency");
        previous_done_edge = -1;
        if (!$value$plusargs("vectors=%s", path)) $fatal(1, "Missing vector path");
        file_handle = $fopen(path, "r");
        if (!file_handle) $fatal(1, "Cannot open vectors");
        count = 0;
        max_latency = 0;
        fields = $fscanf(file_handle, "%d %d %h %h %h %h %h",
                         request_clear, request_last, operand_a, operand_b, expected, expected_accumulator, expected_flags);
        while (fields == 7) begin
            if (busy !== 1'b0) $fatal(1, "Unit was not idle before acceptance");
            accepted_edge = edge_count + 1;
            if (previous_done_edge >= 0 && accepted_edge != previous_done_edge + 1)
                $fatal(1, "Back-to-back acceptance gap");
            start = 1;
            clear = request_clear;
            last = request_last;
            accepted_addend = request_clear ? 32'b0 : dut.accumulator;
            a = operand_a;
            b = operand_b;
            @(negedge clk);
            if ({busy, done, result_valid} !== 3'b100)
                $fatal(1, "Operation %0d was not accepted cleanly", count);
            // The correctness run also injects a busy-cycle request.
            start = !trace_latency;
            clear = !trace_latency;
            last = !request_last;
            a = 32'h7f800001;
            b = 0;
            latency = 0;
            while (!done && latency < 256) begin
                if ({busy, result_valid} !== 2'b10)
                    $fatal(1, "Invalid pending handshake at operation %0d", count);
                @(negedge clk);
                start = 0;
                clear = 0;
                last = 0;
                latency = edge_count - accepted_edge;
            end
            if (done !== 1'b1)
                $fatal(1, "MAC timeout: op %0d a=%h b=%h c=%h",
                       count, operand_a, operand_b, accepted_addend);
            previous_done_edge = edge_count;
            if (trace_latency)
                $display("LAT MAC %0d %h %h %h %h %h %0d", count, operand_a,
                         operand_b, accepted_addend, dut.accumulator, exception_flags, latency);
            if (latency > max_latency) max_latency = latency;
            if ({busy, done, result_valid} !== {2'b01, request_last}
                || result !== expected || exception_flags !== expected_flags)
                $fatal(1, "Op %0d a=%h b=%h got %h flags=%b expected %h flags=%b",
                       count, operand_a, operand_b, result, exception_flags, expected, expected_flags);
            if (dut.accumulator !== expected_accumulator)
                $fatal(1, "Op %0d accumulator got %h expected %h",
                       count, dut.accumulator, expected_accumulator);
            count++;
            fields = $fscanf(file_handle, "%d %d %h %h %h %h %h",
                             request_clear, request_last, operand_a, operand_b, expected, expected_accumulator, expected_flags);
        end
        if (!$feof(file_handle) || count == 0) $fatal(1, "Malformed vector file");
        $fclose(file_handle);

        @(negedge clk);
        if ({busy, done, result_valid} !== '0) $fatal(1, "Completion pulses did not expire");
        // Clear alone with last held high. The published result remains latched.
        clear = 1;
        last = 1;
        @(negedge clk);
        clear = 0;
        if ({busy, done, result_valid} !== '0 || result !== 32'h44800000)
            $fatal(1, "Unexpected publication on clear");
        start = 1;
        a = 0;
        b = 0;
        @(negedge clk);
        start = 0;
        @(negedge clk);
        if (!done || !result_valid || result !== 0 || exception_flags !== 0)
            $fatal(1, "Stand-alone clear failed");

        start = 1;
        a = 32'h3f800000;
        b = 32'h3f800000;
        @(negedge clk);
        start = 0;
        if (!busy) $fatal(1, "Reset test did not start an operation");
        reset_dut();
        repeat (256) begin
            @(negedge clk);
            if ({busy, done, result_valid} !== '0)
                $fatal(1, "Reset did not cancel pending operation");
        end
        start = 1;
        a = 32'h40000000;
        @(negedge clk);
        start = 0;
        wait (done);
        @(negedge clk);
        if (!result_valid || result !== 32'h40000000 || exception_flags !== 0)
            $fatal(1, "Operation after reset failed");

        $display("PASS: %0d reference MAC operations, max sampled latency %0d cycles", count, max_latency);
        $finish;
    end

    initial begin
        #60000000;
        $fatal(1, "Simulation timeout");
    end
endmodule
