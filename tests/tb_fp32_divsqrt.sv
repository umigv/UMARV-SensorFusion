`timescale 1ns/1ps
module tb_fp32_divsqrt;
    logic clk = 0;
    always #10 clk = ~clk;
    integer edge_count = 0;
    integer accepted_edge, previous_done_edge;
    bit trace_latency;
    always @(posedge clk) edge_count = edge_count + 1;
    logic rst_n = 1, start = 0, mode = 0;
    logic [31:0] a = 0, b = 0, result;
    logic busy, done, invalid, divide_by_zero, overflow, underflow, inexact;
    wire [4:0] flags = {invalid, divide_by_zero, overflow, underflow, inexact};
    fp32_divsqrt dut (.*);

    string path;
    integer file_handle, fields, latency, operation;
    integer count [0:1], total [0:1], minimum [0:1], maximum [0:1];
    integer finite_count [0:1], finite_total [0:1], finite_min [0:1], finite_max [0:1];
    logic request_mode;
    logic [31:0] operand_a, operand_b, expected, previous_result;
    logic [4:0] expected_flags, previous_flags;

    task automatic reset_dut;
        #1 rst_n = 0;
        #1;
        if ({busy, done, result, flags} !== '0) $fatal(1, "Asynchronous reset failed");
        @(negedge clk);
        rst_n = 1;
    endtask

    initial begin
        @(negedge clk);
        reset_dut();
        for (integer i = 0; i < 2; i++) begin
            count[i] = 0;
            total[i] = 0;
            minimum[i] = 256;
            maximum[i] = 0;
            finite_count[i] = 0;
            finite_total[i] = 0;
            finite_min[i] = 256;
            finite_max[i] = 0;
        end
        trace_latency = $test$plusargs("latency");
        previous_done_edge = -1;
        if (!$value$plusargs("vectors=%s", path)) $fatal(1, "Missing vector file");
        file_handle = $fopen(path, "r");
        if (!file_handle) $fatal(1, "Cannot open vectors");
        previous_result = 0;
        previous_flags = 0;
        operation = 0;
        fields = $fscanf(file_handle, "%d %h %h %h %h",
                         request_mode, operand_a, operand_b, expected, expected_flags);
        while (fields == 5) begin
            if (busy !== 1'b0) $fatal(1, "Unit was not idle before acceptance");
            accepted_edge = edge_count + 1;
            if (previous_done_edge >= 0 && accepted_edge != previous_done_edge + 1)
                $fatal(1, "Back-to-back acceptance gap");
            start = 1;
            mode = request_mode;
            a = operand_a;
            b = operand_b;
            @(negedge clk);
            if ({busy, done} !== 2'b10) $fatal(1, "Operation was not accepted");
            start = !trace_latency;
            // The correctness run also holds an unrelated request while busy.
            mode = !request_mode;
            a = 32'h7f800001;
            b = 0;
            latency = 0;
            while (!done && latency < 256) begin
                if (busy !== 1'b1 || result !== previous_result || flags !== previous_flags)
                    $fatal(1, "Pending handshake or output hold failure on operation %0d", operation);
                @(negedge clk);
                latency = edge_count - accepted_edge;
            end
            if (done !== 1'b1)
                $fatal(1, "Div/sqrt timeout: op %0d mode=%b a=%h b=%h",
                       operation, request_mode, operand_a, operand_b);
            if ({busy, done} !== 2'b01 || result !== expected)
                $fatal(1, "Op %0d mode=%b a=%h b=%h got %h expected %h",
                       operation, request_mode, operand_a, operand_b, result, expected);
            if (flags !== expected_flags)
                $fatal(1, "Op %0d mode=%b a=%h b=%h flags=%b expected=%b",
                       operation, request_mode, operand_a, operand_b, flags, expected_flags);
            previous_done_edge = edge_count;
            if (trace_latency)
                $display("LAT %s %0d %h %h 00000000 %h %h %0d",
                         request_mode ? "SQRT" : "DIV ", operation,
                         operand_a, operand_b, result, flags, latency);
            previous_result = result;
            previous_flags = flags;
            count[request_mode]++;
            total[request_mode] += latency;
            if (latency < minimum[request_mode]) minimum[request_mode] = latency;
            if (latency > maximum[request_mode]) maximum[request_mode] = latency;
            if (latency > 1) begin
                finite_count[request_mode]++;
                finite_total[request_mode] += latency;
                if (latency < finite_min[request_mode]) finite_min[request_mode] = latency;
                if (latency > finite_max[request_mode]) finite_max[request_mode] = latency;
            end
            operation++;
            fields = $fscanf(file_handle, "%d %h %h %h %h",
                             request_mode, operand_a, operand_b, expected, expected_flags);
        end
        if (!$feof(file_handle) || finite_count[0] < 10000 || finite_count[1] < 10000)
            $fatal(1, "Missing or malformed reference vectors");
        $fclose(file_handle);
        start = 0;
        @(negedge clk);
        if ({busy, done} !== 0 || result !== previous_result || flags !== previous_flags)
            $fatal(1, "Idle output hold or single-cycle done failed");

        for (integer i = 0; i < 2; i++) begin
            start = 1;
            mode = i[0];
            a = 32'h40400000;
            b = 32'h40000000;
            @(negedge clk);
            start = 0;
            repeat (10) @(negedge clk);
            if (!busy || done) $fatal(1, "Reset test did not reach the arithmetic loop");
            reset_dut();
            repeat (110) begin
                @(negedge clk);
                if ({busy, done, result, flags} !== '0) $fatal(1, "Pending operation survived reset");
            end
        end
        start = 1;
        mode = 1;
        a = 32'h40800000;
        b = 32'h7f800001;
        @(negedge clk);
        start = 0;
        wait (done);
        @(negedge clk);
        if (busy || result !== 32'h40000000 || flags !== 0) $fatal(1, "Recovery failed");

        for (integer i = 0; i < 2; i++) begin
            $display("PASS mode %0d: %0d results and flags, latency min/max/mean %0d/%0d/%0.3f cycles",
                     i, count[i], minimum[i], maximum[i], $itor(total[i]) / count[i]);
            $display("     recurrence only: %0d operations, min/max/mean %0d/%0d/%0.3f cycles",
                     finite_count[i], finite_min[i], finite_max[i], $itor(finite_total[i]) / finite_count[i]);
        end
        $finish;
    end

    initial begin
        #60000000;
        $fatal(1, "Simulation timeout");
    end
endmodule
