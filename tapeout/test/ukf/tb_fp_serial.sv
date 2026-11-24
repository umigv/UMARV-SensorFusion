`timescale 1ns/1ps
// Checks fp_serial against vectors made with numpy float32 (vectors.hex: op a b expected).
module tb_fp_serial;
    logic clk = 0, rst_n = 0, load_a = 0, load_b = 0, start = 0, busy, done;
    logic [2:0] op = 0;
    logic [31:0] din = 0, result;
    fp_serial dut (.*);
    always #10 clk = ~clk;

    logic [99:0] vec [0:59999];
    integer n, i, bad, maxclk, clks, total;
    initial begin
        $readmemh("vectors.hex", vec);
        n = 0; while (n < 60000 && vec[n] !== 100'bx) n = n + 1;
        bad = 0; maxclk = 0; total = 0;
        repeat (3) @(posedge clk); rst_n = 1; @(posedge clk);
        for (i = 0; i < n; i = i + 1) begin
            @(negedge clk); op = vec[i][98:96]; din = vec[i][95:64]; load_a = 1;
            @(negedge clk); load_a = 0; din = vec[i][63:32]; load_b = 1;
            @(negedge clk); load_b = 0; start = 1;
            @(negedge clk); start = 0; clks = 0;
            while (!done) begin @(negedge clk); clks = clks + 1; if (clks > 400) begin $display("HANG at %0d", i); $finish; end end
            if (clks > maxclk) maxclk = clks;
            total = total + clks;
            if (result !== vec[i][31:0]) begin
                bad = bad + 1;
                if (bad <= 20) $display("MISMATCH %0d: op %0d a %h b %h got %h want %h", i, vec[i][98:96], vec[i][95:64], vec[i][63:32], result, vec[i][31:0]);
            end
        end
        $display("vectors %0d, mismatches %0d, longest %0d clocks, average %0d clocks", n, bad, maxclk, total / n);
        $finish;
    end
endmodule
