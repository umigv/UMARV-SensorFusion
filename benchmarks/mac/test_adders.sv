`timescale 1ns/1ps
module test_adders #(parameter integer VECTORS = 100000);
    logic [51:0] a, b, expected;
    logic cin;
    wire [51:0] result [0:4];
    integer seed = 32'h4c413532;

    for (genvar t = 0; t < 5; t++) begin : topology
        experiment_adder #(.TOPOLOGY(t)) dut (a, b, cin, result[t]);
    end

    task automatic check;
        expected = a + b + cin;
        #1;
        for (integer t = 0; t < 5; t++)
            if (result[t] !== expected)
                $fatal(1, "topology=%0d a=%h b=%h cin=%b result=%h expected=%h",
                       t, a, b, cin, result[t], expected);
    endtask

    initial begin
        for (integer i = 0; i < 52; i++) begin
            a = (52'b1 << i) - 1;
            b = 0;
            cin = 1;
            check();
        end
        repeat (VECTORS) begin
            a = {$random(seed), $random(seed)};
            b = {$random(seed), $random(seed)};
            cin = 1'($random(seed));
            check();
        end
        $display("PASS %0d exact adder vectors per topology (all five topologies)", VECTORS+52);
        $finish;
    end
endmodule
