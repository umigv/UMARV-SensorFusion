`timescale 1ns/1ps
module tb;
    reg clk = 0;
`ifdef GL_TEST
    // Leave time for the template's unit-delay cell models to settle.
    always #1000 clk = !clk;
`else
    always #10 clk = !clk;
`endif
    reg rst_n = 0, ena = 0;
    reg [7:0] ui_in = 0, uio_in = 0;
    wire [7:0] uo_out, uio_out, uio_oe;
    reg done = 0;
    integer cycle = 0;
    always @(posedge clk) cycle <= cycle + 1;

    tt_um_umarv_srukf dut (
`ifdef GL_TEST
        .VPWR(1'b1), .VGND(1'b0),
`endif
        .clk(clk), .rst_n(rst_n), .ena(ena), .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe)
    );

    localparam [511:0] VN_FRAME = 512'hbebe2d1d9979de293eae2d1d9979de29424800003f4ccccd3e0f5c293c9ba5e33db851ecbc23d70a3c8b43963e3851ec3c449ba6bc23d70a3ccccccd3e4ccccd;
    // State and upper-triangular factor from the independent SR-UKF reference.
    localparam [639:0] FIRST = 640'h3e61709cbc5cc5433de263963beb137d3c79efe33d93a195bb5108ecbb4e40463bbb1ef63e1390d23c64bce23bab0a27bb6bdd353ca445153e2947d03e8a1ec9bf4ccc443f32b83fbe4b3d143dd03a78;
    localparam [639:0] SECOND = 640'h3e619908bc2b728a3db1ff143bb623773c459e4c3d6757a8bb1a0f2fbb7d4df53be593df3de66fff3c2276ee3bb0925fbb8af2ef3c8242cc3e0293883e8a13f1bf4ccb8e3f3269e7be48fe413dd5ebf8;

    task command(input [2:0] op, input [7:0] data);
        begin
            ui_in = data;
            uio_in = {4'b0, 1'b1, op};
            @(negedge clk);
            uio_in = 0;
        end
    endtask

    task update(input [31:0] wheel, input [639:0] expected);
        integer i, release_cycle;
        reg [639:0] sampled;
        reg [31:0] got, want, difference;
        begin
            for (i = 3; i >= 0; i = i - 1)
                command(1, wheel[i*8 +: 8]);
            release_cycle = cycle;
            command(2, 0);
            for (i = 63; i >= 0; i = i - 1)
                command(1, VN_FRAME[i*8 +: 8]);
            while (cycle < release_cycle + 10000) @(negedge clk);
            if (uo_out[0] !== 1'b1) $fatal(1, "prediction must be running at VN release");
            command(3, 0);
            while (uo_out[1:0] !== 2'b10 && cycle < release_cycle + 125000) begin
                if (uo_out[5:2] !== 0) $fatal(1, "UKF status error %h", uo_out);
                @(negedge clk);
            end
            if (uo_out !== 8'h02) $fatal(1, "missing clean result by deadline");
            uio_in = 8'h10;
            for (i = 0; i < 80; i = i + 1) begin
                ui_in = i;
                @(negedge clk);
                sampled[i*8 +: 8] = uo_out;
            end
            uio_in = 0;
            for (i = 0; i < 20; i = i + 1) begin
                got = sampled[i*32 +: 32];
                want = expected[i*32 +: 32];
                difference = got > want ? got - want : want - got;
                if ((^got === 1'bx) || got[31] !== want[31] || difference > 4096)
                    $fatal(1, "result word %0d got %h expected %h", i, got, want);
            end
            $display("PASS wheel %0d, corrected state and factor by %0d cycles", wheel, cycle-release_cycle-80);
            // Leave four cycles to stage the next wheel word.
            while (cycle < release_cycle + 124996) @(negedge clk);
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        ena = 1;
        rst_n = 1;
        if (uio_oe !== 0 || uio_out !== 0) $fatal(1, "unexpected bidirectional driver");
        command(3, 0);
        #2;
        if (uo_out[5] !== 1) $fatal(1, "missing short-frame error");
        @(negedge clk);
        command(4, 0);
        update(24, FIRST);
        update(26, SECOND);
        done = 1;
`ifndef COCOTB_TEST
        $finish;
`endif
    end
endmodule
