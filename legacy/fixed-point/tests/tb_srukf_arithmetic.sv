`timescale 1ns/1ps
module tb_srukf_arithmetic;
    reg clk=0;
    always #5 clk=~clk;
    reg resetn=0, root_start=0, div_start=0, mac_start=0;
    reg clear=0, subtract=0;
    reg [2:0] align=0;
    reg signed [31:0] a=0, b=0, denominator=0;
    reg signed [71:0] radicand=0, numerator=0;
    wire signed [31:0] root_value, div_value;
    wire signed [71:0] mac_value;
    wire root_done, negative, root_ov, div_done, zero, div_ov, mac_done, mac_ov;
    srukf_full_root sq(.clk,.resetn,.start(root_start),.radicand,.result(root_value),
                       .done(root_done),.negative,.overflow(root_ov));
    srukf_full_div dv(.clk,.resetn,.start(div_start),.numerator,.denominator,.result(div_value),
                      .done(div_done),.zero,.overflow(div_ov));
    srukf_mac mac(.clk,.resetn,.start(mac_start),.clear,.subtract,.a,.b,.align,
                  .result(mac_value),.done(mac_done),.overflow(mac_ov));

    task automatic check_root(input signed [71:0] value, input signed [31:0] expected,
                              input reg neg, ov);
        begin
            @(negedge clk); radicand=value; root_start=1;
            @(negedge clk); root_start=0;
            wait(root_done); #1;
            if ({root_value,negative,root_ov} !== {expected,neg,ov})
                $fatal(1,"root %0d -> %0d neg=%b ov=%b",value,root_value,negative,root_ov);
        end
    endtask
    task automatic check_div(input signed [71:0] value, input signed [31:0] den, expected,
                             input reg z, ov);
        begin
            @(negedge clk); numerator=value; denominator=den; div_start=1;
            @(negedge clk); div_start=0;
            wait(div_done); #1;
            if ({div_value,zero,div_ov} !== {expected,z,ov})
                $fatal(1,"divide %0d by %0d -> %0d zero=%b ov=%b",value,den,div_value,zero,div_ov);
        end
    endtask
    task automatic check_mac(input signed [31:0] x,y, input [2:0] shift,
                             input reg cl, sub, input signed [71:0] value, input reg ov);
        begin
            @(negedge clk); a=x; b=y; align=shift; clear=cl; subtract=sub; mac_start=1;
            @(negedge clk); mac_start=0;
            wait(mac_done); #1;
            if (mac_value !== value || mac_ov !== ov)
                $fatal(1,"MAC -> %0d expected %0d ov=%b",mac_value,value,mac_ov);
        end
    endtask

    initial begin
        repeat(2) @(negedge clk); resetn=1;
        check_root(72'd1,32'd1,0,0);
        check_root(72'd77285650,32'd8791,0,0);  // old prototype lost this QR norm
        check_root(72'd2003,32'd44,0,0);       // 1002^2 - 1001^2
        check_root(0,0,0,0);
        check_root(-1,0,1,0);
        check_root(72'd4611686018427387904,32'sh7fffffff,0,1);
        check_div(21,3,7,0,0);
        check_div(-21,4,-5,0,0);
        check_div(72'sd60 <<< 50,32'sd8 <<< 25,251658240,0,0);
        check_div(-72'sd2147483648,1,32'sh80000000,0,0);
        check_div(72'sd2147483648,1,32'sh7fffffff,0,1);
        check_div(1,0,32'sh7fffffff,1,0);
        check_mac(121,1,0,1,0,121,0);
        check_mac(10,10,0,0,1,21,0);
        check_mac(32'sh80000000,-1,4,1,0,72'sd34359738368,0);
        check_mac(32'sh80000000,-1,4,0,1,0,0);
        check_mac(32'sh80000000,32'sh80000000,7,1,0,72'sd1 <<< 69,0);
        check_mac(32'sh80000000,32'sh80000000,7,0,0,72'sd2 <<< 69,0);
        check_mac(32'sh80000000,32'sh80000000,7,0,0,72'sd3 <<< 69,0);
        check_mac(32'sh80000000,32'sh80000000,7,0,0,72'sh800000000000000000,1);
        $display("PASS full-root, cancellation, wide solves, rounding signs and arithmetic fault flags");
        $finish;
    end
    initial begin
        repeat(1500) @(posedge clk);
        $fatal(1,"timeout");
    end
endmodule
