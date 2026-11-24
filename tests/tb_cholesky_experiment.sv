`timescale 1ns/1ps
module tb_cholesky_experiment #(
    parameter integer RF_WORDS = 29
);
    localparam MAC_CLEAR = 0, MAC_CONTINUE = 1, DIV = 2, SQRT = 3, MOVE = 4,
               LOAD = 5, RUN = 6, ROW_END = 7;
    logic clk = 0, rst_n = 1;
    always #10 clk = ~clk;
    integer edge_number = 0;
    always @(posedge clk) edge_number = edge_number + 1;

    logic mac_start = 0, mac_clear, mac_last, mac_busy, mac_done, mac_valid;
    logic [31:0] a, b, mac_result;
    logic [4:0] mac_flags;
    fp32_mac mac (.clk, .rst_n, .start(mac_start), .clear(mac_clear), .last(mac_last),
                  .a, .b, .busy(mac_busy), .done(mac_done), .result_valid(mac_valid),
                  .result(mac_result), .exception_flags(mac_flags));

    logic div_start = 0, root_mode, div_busy, div_done;
    logic [31:0] div_result;
    logic [4:0] div_flags;
    fp32_divsqrt divsqrt (.clk, .rst_n, .start(div_start), .mode(root_mode), .a, .b,
                          .busy(div_busy), .done(div_done), .result(div_result),
                          .invalid(div_flags[4]), .divide_by_zero(div_flags[3]),
                          .overflow(div_flags[2]), .underflow(div_flags[1]), .inexact(div_flags[0]));

    logic [31:0] rf [0:RF_WORDS-1];
    logic [31:0] expected, actual;
    logic [4:0] expected_flags, actual_flags;
    string path;
    bit report_latency;
    integer file_handle, fields, op, src_a, src_b, dst, reads, requests;
    integer run_id = -1, run_edge, row_edge, row_number, acceptance, command_number;

    function automatic logic [31:0] operand(input integer address);
        operand = address < 0 ? rf[-address] ^ 32'h80000000 : rf[address];
    endfunction

    initial begin
        #1 rst_n = 0;
        @(negedge clk);
        rst_n = 1;
        rf[0] = 0;
        rf[1] = 32'h3f800000;
        report_latency = $test$plusargs("latency");
        if (!$value$plusargs("vectors=%s", path)) $fatal(1, "Missing vector file");
        file_handle = $fopen(path, "r");
        if (!file_handle) $fatal(1, "Cannot open vectors");
        fields = $fscanf(file_handle, "%d %d %d %d %h %h %d",
                         op, src_a, src_b, dst, expected, expected_flags, reads);
        while (fields == 7) begin
            case (op)
                RUN: begin
                    if (run_id >= 0) $display("RUN %0d %0d %0d", run_id, edge_number-run_edge, requests);
                    run_id = $signed(expected);
                    run_edge = edge_number;
                    row_edge = edge_number;
                    row_number = 0;
                    requests = 0;
                    command_number = 0;
                end
                ROW_END: begin
                    row_number++;
                    $display("ROW %0d %0d %0d", run_id, row_number, edge_number-row_edge);
                    row_edge = edge_number;
                end
                LOAD: rf[dst] = expected;
                default: begin
                    command_number++;
                    // Serialize operand reads, then use one edge for issue.
                    repeat (reads) @(negedge clk);
                    if (mac_busy || div_busy) $fatal(1, "Arithmetic units were not idle");
                    a = operand(src_a);
                    b = operand(src_b);
                    mac_start = op <= MAC_CONTINUE;
                    div_start = op == DIV || op == SQRT;
                    mac_clear = op == MAC_CLEAR;
                    mac_last = dst != 0;
                    root_mode = op == SQRT;
                    @(negedge clk);
                    mac_start = 0;
                    mac_clear = 0;
                    div_start = 0;
                    acceptance = edge_number;
                    actual_flags = 0;
                    if (op == MOVE) begin
                        actual = a;
                    end else begin
                        requests++;
                        if (op <= MAC_CONTINUE) begin
                            if (!mac_busy) $fatal(1, "MAC did not accept request");
                            while (!mac_done && edge_number-acceptance < 256) @(negedge clk);
                            if (!mac_done || mac_valid !== mac_last) $fatal(1, "MAC timeout or valid mismatch");
                            actual = mac_last ? mac_result : mac.accumulator;
                            actual_flags = mac_flags;
                        end else begin
                            if (!div_busy) $fatal(1, "Divider did not accept request");
                            while (!div_done && edge_number-acceptance < 256) @(negedge clk);
                            if (!div_done) $fatal(1, "Division/square-root timeout");
                            actual = div_result;
                            actual_flags = div_flags;
                        end
                        if (report_latency)
                            $display("LAT %0d %0d %0d %0d", run_id, op,
                                     edge_number-acceptance, actual_flags);
                    end
                    if (actual !== expected || actual_flags !== expected_flags)
                        $fatal(1, "Run %0d cmd %0d op %0d a=%h b=%h got %h/%h expected %h/%h",
                               run_id, command_number, op, a, b, actual, actual_flags, expected, expected_flags);
                    // One retire edge, followed by a serialized register write.
                    @(negedge clk);
                    if (dst != 0) begin
                        @(negedge clk);
                        rf[dst] = actual;
                    end
                end
            endcase
            fields = $fscanf(file_handle, "%d %d %d %d %h %h %d",
                             op, src_a, src_b, dst, expected, expected_flags, reads);
        end
        if (!$feof(file_handle) || run_id != -1) $fatal(1, "Malformed experiment stream");
        $fclose(file_handle);
        $finish;
    end

    initial begin
        #100000000;
        $fatal(1, "Simulation timeout");
    end
endmodule
