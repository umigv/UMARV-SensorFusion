// SPDX-License-Identifier: Apache-2.0
`timescale 1ns/1ps
module tb;
    reg clk = 0;
`ifdef GL_TEST
    always #1000 clk = !clk;
`else
    always #10 clk = !clk;
`endif
    reg rst_n = 0;
    reg [7:0] ui_in = 8'h01, uio_in = 0;
    wire [7:0] uo_out, uio_out, uio_oe;
    reg done = 0;
    tt_um_umigv_sensorfusion user_project (
`ifdef GL_TEST
        .VPWR(1'b1), .VGND(1'b0),
`endif
        .clk(clk), .rst_n(rst_n), .ena(1'b1), .ui_in(ui_in),
        .uo_out(uo_out), .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe)
    );

    integer tick = 0;
    always @(posedge clk) tick <= tick + 1;
    reg [7:0] packet [0:55];

    function [15:0] amt(input [13:0] data);
        amt = {~^{data[13], data[11], data[9], data[7], data[5], data[3], data[1]},
               ~^{data[12], data[10], data[8], data[6], data[4], data[2], data[0]}, data};
    endfunction

    task encoder(input [13:0] position);
        reg [31:0] bits;
        integer i;
        begin
            bits = {amt(position), amt(14'd0)};
            @(negedge uo_out[1]);
            for (i = 31; i >= 0; i = i - 1) begin
                @(negedge clk);
                ui_in[1] = bits[i];
                @(posedge uo_out[2]);
                @(negedge uo_out[2]);
            end
        end
    endtask

    task uart_packet;
        integer i, bit_index;
        begin
            @(posedge uo_out[0]);
            repeat (10000) @(negedge clk);
            for (i = 0; i < 56; i = i + 1) begin
                ui_in[0] = 0;
                repeat (54) @(negedge clk);
                for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                    ui_in[0] = packet[i][bit_index];
                    repeat (54) @(negedge clk);
                end
                ui_in[0] = 1;
                repeat (54) @(negedge clk);
            end
        end
    endtask

    task check_frame(input [159:0] expected);
        reg [167:0] frame;
        reg [31:0] got, want, difference;
        integer i, release_tick;
        begin
            @(posedge uo_out[4]);
            release_tick = tick;
            @(posedge uo_out[6]);
            if (tick - release_tick >= 125000) $fatal(1, "correction deadline missed");
            for (i = 167; i >= 0; i = i - 1) begin
                repeat (4) @(negedge clk);
                if (uo_out[6] !== 1'b1) $fatal(1, "short result frame");
                frame[i] = uo_out[5];
                repeat (4) @(negedge clk);
            end
            if (frame[167:160] !== 8'h01 || uo_out[7] !== 1'b0)
                $fatal(1, "bad result status %h", frame[167:160]);
            for (i = 0; i < 5; i = i + 1) begin
                got = frame[32*i +: 32];
                want = expected[32*i +: 32];
                difference = got > want ? got - want : want - got;
                if ((^got === 1'bx) || got[31] !== want[31] || difference > 256)
                    $fatal(1, "state word %0d got %h expected %h", i, got, want);
            end
            $display("PASS corrected state %h", frame[159:0]);
        end
    endtask

    localparam [63:0] LAT = 64'hbebe2d1d9979de29, LON = 64'h3eae2d1d9979de29;
    localparam [31:0] YAW = 32'h42480000, RATE = 32'h3f4ccccd;
    localparam [319:0] NOISE = 320'h3e0f5c293c9ba5e33db851ecbc23d70a3c8b43963e3851ec3c449ba6bc23d70a3ccccccd3e99999a;
    integer i, j;
    reg [15:0] crc;
    initial begin
        for (i = 0; i < 56; i = i + 1) packet[i] = 0;
        packet[0] = 8'hfa;
        packet[1] = 8'h01;
        packet[2] = 8'h68;
        packet[3] = 8'h10;
        for (i = 0; i < 4; i = i + 1) begin
            packet[4+i] = YAW[8*i +: 8];
            packet[24+i] = RATE[8*i +: 8];
        end
        for (i = 0; i < 8; i = i + 1) begin
            packet[28+i] = LAT[8*i +: 8];
            packet[36+i] = LON[8*i +: 8];
        end
        packet[52] = 6;
        crc = 0;
        for (i = 1; i < 54; i = i + 1) begin
            crc = crc ^ {packet[i], 8'b0};
            for (j = 0; j < 8; j = j + 1)
                crc = (crc << 1) ^ (crc[15] ? 16'h1021 : 16'h0000);
        end
        {packet[54], packet[55]} = crc;

        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (4) @(negedge clk);
        ui_in[4] = 1;
        for (i = 39; i >= 0; i = i - 1) begin
            uio_in = NOISE[8*i +: 8];
            @(negedge clk);
        end
        ui_in[4] = 0;
        ui_in[3:2] = 2'b11;
        fork
            begin encoder(0); encoder(24); end
            begin uart_packet; uart_packet; end
            begin
                check_frame(160'h3dccd66fbe4cd19b3f32b59abf4cccb73e8a3f16);
                check_frame(160'h3dd43160be4a2f7b3f3267bcbf4ccba53e8a3f8d);
            end
        join
        done = 1;
`ifndef COCOTB_TEST
        $finish;
`endif
    end
    initial begin
        repeat (450000) @(negedge clk);
        if (!done) $fatal(1, "missing sensor-fusion results");
    end
endmodule
