module adder_miter(input wire [51:0] a, b, input wire cin, output wire ok);
    wire [51:0] expected = a + b + cin;
    wire [4:0] equivalent_sum;
    for (genvar i = 0; i < 5; i++) begin : topology
        wire [51:0] result;
        experiment_adder #(.TOPOLOGY(i)) dut (a, b, cin, result);
        assign equivalent_sum[i] = result == expected;
    end
    assign ok = &equivalent_sum;
endmodule
