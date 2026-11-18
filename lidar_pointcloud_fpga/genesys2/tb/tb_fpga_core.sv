// Self-checking testbench for fpga_core, driven at the RGMII pins.
//
//  * sends VLP-16 data packets (and frames that must be ignored) into the MAC
//  * captures every frame the design transmits
//  * checks FCS, Ethernet/IPv4/UDP headers, IP checksum, the PC2X header and
//    every XYZ float against a floating point reference of the transform
`timescale 1ns / 1ps

module tb_fpga_core;

    // ---------------------------------------------------------------- clocks
    logic clk = 0, clk90 = 0, rst = 1;
    always #4 clk = ~clk;
    always @(clk) clk90 <= #2 clk;

    logic       phy_rx_clk = 0;
    logic [3:0] phy_rxd = 0;
    logic       phy_rx_ctl = 0;
    wire        phy_tx_clk;
    wire  [3:0] phy_txd;
    wire        phy_tx_ctl;
    wire        phy_reset_n;
    wire  [7:0] led;

    initial begin #1.3; forever #4 phy_rx_clk = ~phy_rx_clk; end

    localparam logic [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01;
    localparam logic [31:0] LOCAL_IP  = {8'd192, 8'd168, 8'd1, 8'd50};
    localparam logic [15:0] DEST_PORT = 16'd2370;

    fpga_core #(
        .SIM(1'b1),
        .VENDOR("XILINX"),
        .FAMILY("kintex7"),
        .PHY_RST_BITS(4),
        .LOCAL_MAC(LOCAL_MAC),
        .LOCAL_IP(LOCAL_IP),
        .DEST_PORT(DEST_PORT)
    ) dut (.*);

    // ------------------------------------------------------------- utilities
    int errors = 0;
    `define CHECK(cond, msg) if (!(cond)) begin errors++; $display("ERROR @%0t: %s", $time, msg); end

    function automatic logic [31:0] crc32(input byte unsigned d[$]);
        logic [31:0] c = 32'hFFFFFFFF;
        foreach (d[i]) begin
            c ^= {24'h0, d[i]};
            for (int k = 0; k < 8; k++) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1);
        end
        return ~c;
    endfunction

    // ------------------------------------------------------- RGMII transmitter
    // Data changes 2 ns before each clock edge (clock centred in the data eye,
    // as the PHY's internal RX delay provides on the board).
    task automatic rgmii_byte(input byte unsigned b, input logic dv);
        @(negedge phy_rx_clk); #2; phy_rxd = b[3:0]; phy_rx_ctl = dv;
        @(posedge phy_rx_clk); #2; phy_rxd = b[7:4]; phy_rx_ctl = dv;
    endtask

    task automatic send_frame(input byte unsigned f[$], input bit corrupt_fcs = 0);
        logic [31:0] fcs = crc32(f);
        if (corrupt_fcs) fcs ^= 32'h00010000;
        repeat (7) rgmii_byte(8'h55, 1);
        rgmii_byte(8'hD5, 1);
        foreach (f[i]) rgmii_byte(f[i], 1);
        for (int i = 0; i < 4; i++) rgmii_byte(fcs[8*i +: 8], 1);
        repeat (12) rgmii_byte(8'h00, 0);          // inter-frame gap
    endtask

    // ------------------------------------------------- reference point model
    real ELEV_DEG [16] = '{-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
                           -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0};
    localparam real PI = 3.14159265358979;

    typedef struct { real x, y, z; int az; } exp_pt_t;
    exp_pt_t exp_q[$];

    function automatic void expect_point(input int rng, input int az, input int laser);
        exp_pt_t p;
        // same azimuth binning as trig_lut (1 degree bins, reciprocal multiply)
        int  deg = (az * 655) >> 16;
        real a   = deg * PI / 180.0;
        real e   = ELEV_DEG[laser] * PI / 180.0;
        real d   = rng * 0.002;
        // REP-103 frame: x forward, y left, z up
        p.x  =  d * $cos(e) * $cos(a);
        p.y  = -d * $cos(e) * $sin(a);
        p.z  =  d * $sin(e);
        p.az = az;
        exp_q.push_back(p);
    endfunction

    // --------------------------------------------------------- frame builders
    function automatic void put16(ref byte unsigned f[$], input logic [15:0] v);
        f.push_back(v[15:8]); f.push_back(v[7:0]);
    endfunction

    function automatic void udp_headers(ref byte unsigned f[$], input int port, input int payload_len);
        logic [47:0] dmac = '1, smac = 48'h60_76_88_00_00_01;
        for (int i = 5; i >= 0; i--) f.push_back(dmac[8*i +: 8]);
        for (int i = 5; i >= 0; i--) f.push_back(smac[8*i +: 8]);
        put16(f, 16'h0800);
        put16(f, 16'h4500); put16(f, 16'(20 + 8 + payload_len));
        put16(f, 16'h0000); put16(f, 16'h4000);
        put16(f, 16'hFF11); put16(f, 16'h0000);          // checksum not checked by DUT
        put16(f, 16'hC0A8); put16(f, 16'h01C9);          // 192.168.1.201
        put16(f, 16'hFFFF); put16(f, 16'hFFFF);
        put16(f, 16'd2368); put16(f, 16'(port));
        put16(f, 16'(8 + payload_len)); put16(f, 16'h0000);
    endfunction

    // mode 0: random distances with some zeros     mode 1: far returns (> 65.5 m)
    // bad_block: index of a block sent with a corrupt flag (-1 = none)
    function automatic void vlp16_frame(ref byte unsigned f[$], input int az0, input int port,
                                        input int mode, input int bad_block, input bit expect_pts);
        f = {};
        udp_headers(f, port, 1206);
        for (int b = 0; b < 12; b++) begin
            int az = (az0 + b * 40) % 36000;
            f.push_back(8'hFF);
            f.push_back(b == bad_block ? 8'hEF : 8'hEE);
            f.push_back(az[7:0]); f.push_back(az[15:8]);
            for (int ch = 0; ch < 32; ch++) begin
                int rng;
                case (mode)
                    1:       rng = 32768 + $urandom_range(0, 17000);
                    default: rng = ($urandom_range(0, 9) == 0) ? 0 : $urandom_range(1, 32767);
                endcase
                if (ch == 5 && b == 3) rng = 32767;
                if (ch == 6 && b == 3) rng = 1;
                f.push_back(rng[7:0]); f.push_back(rng[15:8]);
                f.push_back(8'($urandom));
                if (expect_pts && b != bad_block && rng != 0) expect_point(rng, az, ch % 16);
            end
        end
        repeat (6) f.push_back(8'($urandom));          // timestamp + factory
    endfunction

    // ---------------------------------------------------------- RGMII receiver
    byte unsigned rxf[$];
    logic [3:0]   lo;
    logic         ctl_lo;
    int           frames_rx = 0, points_rx = 0;
    int           exp_seq = 0;
    real          max_err = 0;

    function automatic logic [15:0] be16(input int o); return {rxf[o], rxf[o+1]}; endfunction
    function automatic logic [15:0] le16(input int o); return {rxf[o+1], rxf[o]}; endfunction
    function automatic logic [31:0] le32(input int o);
        return {rxf[o+3], rxf[o+2], rxf[o+1], rxf[o]};
    endfunction

    function automatic real absr(input real v); return v < 0 ? -v : v; endfunction

    task automatic check_frame();
        byte unsigned body[$];
        logic [31:0]  fcs, sum;
        int           n, plen;
        exp_pt_t      e;
        real          x, y, z;

        // strip preamble + SFD
        `CHECK(rxf.size() > 8 + 64 - 1, "runt frame")
        for (int i = 0; i < 7; i++) `CHECK(rxf[i] == 8'h55, "bad preamble")
        `CHECK(rxf[7] == 8'hD5, "bad SFD")
        rxf = rxf[8:$];
        body = rxf[0:rxf.size()-5];
        fcs  = {rxf[rxf.size()-1], rxf[rxf.size()-2], rxf[rxf.size()-3], rxf[rxf.size()-4]};
        `CHECK(fcs == crc32(body), "bad FCS on transmitted frame")
        `CHECK(body.size() <= 1514, "frame larger than MTU")

        // Ethernet
        for (int i = 0; i < 6; i++) `CHECK(rxf[i] == 8'hFF, "dest MAC not broadcast")
        for (int i = 0; i < 6; i++) `CHECK(rxf[6+i] == LOCAL_MAC[8*(5-i) +: 8], "bad source MAC")
        `CHECK(be16(12) == 16'h0800, "bad ethertype")
        // IPv4
        `CHECK(rxf[14] == 8'h45, "bad IP version/IHL")
        `CHECK(rxf[23] == 8'h11, "IP protocol not UDP")
        sum = 0;
        for (int i = 0; i < 10; i++) sum += be16(14 + 2*i);
        sum = (sum & 16'hFFFF) + (sum >> 16);
        sum = (sum & 16'hFFFF) + (sum >> 16);
        `CHECK(sum == 16'hFFFF, "bad IP header checksum")
        `CHECK({be16(26), be16(28)} == LOCAL_IP, "bad source IP")
        `CHECK({be16(30), be16(32)} == 32'hFFFFFFFF, "bad dest IP")
        `CHECK(be16(16) == body.size() - 14, "IP total length does not match frame")
        // UDP
        `CHECK(be16(36) == DEST_PORT, "bad UDP dest port")
        `CHECK(be16(38) == be16(16) - 20, "bad UDP length")
        plen = be16(38) - 8;
        // PC2X header
        `CHECK(rxf[42] == "P" && rxf[43] == "C" && rxf[44] == "2" && rxf[45] == "X", "bad magic")
        `CHECK(rxf[46] == 1,  "bad version")
        `CHECK(rxf[47] == 12, "bad point_step")
        n = le16(48);
        `CHECK(n >= 1 && n <= 120, "bad num_points")
        `CHECK(plen == 16 + 12*n, "payload length does not match num_points")
        `CHECK(le32(50) == exp_seq, "bad sequence number")
        exp_seq++;
        `CHECK(le16(56) == 0, "bad flags")
        `CHECK(exp_q.size() >= n, "more points than expected")
        if (exp_q.size() >= 1) `CHECK(le16(54) == exp_q[0].az, "header azimuth is not first point's")

        for (int i = 0; i < n && exp_q.size() > 0; i++) begin
            e = exp_q.pop_front();
            x = $bitstoshortreal(le32(58 + 12*i));
            y = $bitstoshortreal(le32(58 + 12*i + 4));
            z = $bitstoshortreal(le32(58 + 12*i + 8));
            if (absr(x - e.x) > max_err) max_err = absr(x - e.x);
            if (absr(y - e.y) > max_err) max_err = absr(y - e.y);
            if (absr(z - e.z) > max_err) max_err = absr(z - e.z);
            if (absr(x - e.x) > 0.012 || absr(y - e.y) > 0.012 || absr(z - e.z) > 0.012) begin
                errors++;
                if (errors < 20)
                    $display("ERROR: point %0d got (%f %f %f) expected (%f %f %f)",
                             points_rx, x, y, z, e.x, e.y, e.z);
            end
            points_rx++;
        end
        frames_rx++;
    endtask

    always @(posedge phy_tx_clk) begin lo <= phy_txd; ctl_lo <= phy_tx_ctl; end
    always @(negedge phy_tx_clk) begin
        if (ctl_lo) begin
            `CHECK(phy_tx_ctl, "TX_ER asserted")
            rxf.push_back({phy_txd, lo});
        end else if (rxf.size() != 0) begin
            check_frame();
            rxf = {};
        end
    end

    // ------------------------------------------------------------------- test
    byte unsigned f[$];
    int total_expected;

    initial begin
        if ($test$plusargs("dump")) begin $dumpfile("tb_fpga_core.vcd"); $dumpvars(0, tb_fpga_core); end
        repeat (20) @(posedge clk);
        rst = 0;
        wait (phy_reset_n);
        repeat (400) @(posedge clk);

        // 1: plain packet
        vlp16_frame(f, 0, 2368, 0, -1, 1);       send_frame(f);
        // 2..4: back to back (exercises FIFO back-pressure), wrap of azimuth, far returns
        vlp16_frame(f, 35800, 2368, 0, -1, 1);   send_frame(f);
        vlp16_frame(f, 12345, 2369, 1, -1, 1);   send_frame(f);
        vlp16_frame(f, 27000, 2368, 0, 4, 1);    send_frame(f);
        // frames that must produce nothing:
        //   bad FCS (dropped by MAC)
        vlp16_frame(f, 100, 2368, 0, -1, 0);     send_frame(f, 1);
        //   UDP with the wrong length (e.g. VLP-16 position packet, 512 bytes)
        f = {}; udp_headers(f, 8308, 512); repeat (512) f.push_back(8'hFF); send_frame(f);
        //   ARP-sized non-IP frame
        f = {}; repeat (12) f.push_back(8'hFF); put16(f, 16'h0806); repeat (46) f.push_back(8'hEE);
        send_frame(f);
        // 8: and it still works afterwards
        vlp16_frame(f, 9000, 2368, 0, -1, 1);    send_frame(f);

        total_expected = points_rx + exp_q.size();
        // wait for everything to drain
        fork
            begin wait (exp_q.size() == 0); repeat (4000) @(posedge clk); end
            begin repeat (400000) @(posedge clk); end
        join_any
        disable fork;

        `CHECK(exp_q.size() == 0, $sformatf("%0d expected points never arrived", exp_q.size()))
        `CHECK(led[7:5] == 3'b001, $sformatf("error LEDs = %b (expect only bad-FCS set)", led[7:5]))
        $display("frames transmitted by DUT: %0d, points checked: %0d, max |error| = %.4f m",
                 frames_rx, points_rx, max_err);
        if (errors == 0 && points_rx > 1500) $display("TEST PASSED");
        else                                 $display("TEST FAILED (%0d errors)", errors);
        $finish;
    end

endmodule
