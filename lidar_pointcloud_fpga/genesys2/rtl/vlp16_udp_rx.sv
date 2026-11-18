// VLP-16 receive front end.
//
// Takes whole Ethernet frames from the MAC (preamble and FCS already removed,
// bad-FCS frames already dropped by the MAC frame FIFO), checks the
// Ethernet / IPv4 / UDP headers, and walks the 1206-byte Velodyne payload:
//
//   12 x { 0xFFEE, azimuth[15:0] (LE, 0.01 deg),
//          32 x { distance[15:0] (LE, 2 mm), reflectivity[7:0] } }
//   timestamp[31:0], factory[15:0]          (ignored)
//
// One point is emitted per (distance, reflectivity) triple.  The 32 returns of
// a block are two firings of the 16 lasers, so laser_id = channel index mod 16.
//
// Not handled: VLAN tags, IP options, IP fragments, dual-return mode.
module vlp16_udp_rx #(
    // UDP destination port the lidar sends to.  0 = accept any port (the
    // UDP length must still be exactly a VLP-16 data packet).
    parameter logic [15:0] LIDAR_PORT = 16'd0,
    // Drop returns with distance 0 (no return)
    parameter logic        DROP_ZERO  = 1'b1
) (
    input  logic        clk,
    input  logic        rst,

    // Ethernet frame from MAC
    input  logic [7:0]  s_tdata,
    input  logic        s_tvalid,
    output logic        s_tready,
    input  logic        s_tlast,

    // Downstream back-pressure (stops consuming bytes)
    input  logic        pause,

    // Parsed points
    output logic        pt_valid,
    output logic [15:0] pt_distance,     // raw, 2 mm / LSB, unsigned
    output logic [15:0] pt_azimuth,      // raw, 0.01 deg / LSB, 0..35999
    output logic [3:0]  pt_laser_id,
    output logic [7:0]  pt_reflectivity,

    // Status (single-cycle pulses)
    output logic        pkt_accepted,    // a complete lidar data packet was parsed
    output logic        pkt_ignored      // a frame that was not a lidar data packet
);

    localparam int HDR_BYTES   = 14 + 20 + 8;      // Ethernet + IPv4 + UDP
    localparam int BLOCK_BYTES = 100;
    localparam int DATA_END    = HDR_BYTES + 12*BLOCK_BYTES;
    // UDP length field of a VLP-16 data packet: 8 + 1206
    localparam logic [15:0] UDP_LEN = 16'd1214;

    logic [10:0] cnt;        // byte index in frame, saturating
    logic        hdr_ok;
    logic [6:0]  boff;       // byte index in block, 0..99
    logic [1:0]  phase;
    logic [4:0]  ch;
    logic        flag_ok;
    logic        blk_ok;
    logic [7:0]  az_lo;
    logic [7:0]  dist_lo;
    logic [7:0]  dist_hi;

    assign s_tready = !pause;
    wire xfer = s_tvalid && s_tready;

    // hdr_ok as seen by the byte currently on the bus
    wire hdr_ok_cur = (cnt == 0) ? 1'b1 : hdr_ok;

    logic byte_ok;
    always_comb begin
        byte_ok = 1'b1;
        case (cnt)
            11'd12: byte_ok = (s_tdata == 8'h08);              // EtherType IPv4
            11'd13: byte_ok = (s_tdata == 8'h00);
            11'd14: byte_ok = (s_tdata == 8'h45);              // IPv4, no options
            11'd20: byte_ok = ((s_tdata & 8'h3F) == 8'h00);    // not a fragment
            11'd21: byte_ok = (s_tdata == 8'h00);
            11'd23: byte_ok = (s_tdata == 8'h11);              // UDP
            11'd36: byte_ok = (LIDAR_PORT == 0) || (s_tdata == LIDAR_PORT[15:8]);
            11'd37: byte_ok = (LIDAR_PORT == 0) || (s_tdata == LIDAR_PORT[7:0]);
            11'd38: byte_ok = (s_tdata == UDP_LEN[15:8]);
            11'd39: byte_ok = (s_tdata == UDP_LEN[7:0]);
            default: byte_ok = 1'b1;
        endcase
    end

    always_ff @(posedge clk) begin
        pt_valid     <= 1'b0;
        pkt_accepted <= 1'b0;
        pkt_ignored  <= 1'b0;

        if (xfer) begin
            if (cnt < HDR_BYTES) begin
                hdr_ok <= hdr_ok_cur && byte_ok;
                boff   <= '0;
            end else if (cnt < DATA_END && hdr_ok) begin
                case (boff)
                    7'd0: flag_ok <= (s_tdata == 8'hFF);
                    7'd1: blk_ok  <= flag_ok && (s_tdata == 8'hEE);
                    7'd2: az_lo   <= s_tdata;
                    7'd3: begin
                        pt_azimuth <= {s_tdata, az_lo};
                        phase      <= 2'd0;
                        ch         <= '0;
                    end
                    default: begin
                        case (phase)
                            2'd0: begin dist_lo <= s_tdata; phase <= 2'd1; end
                            2'd1: begin dist_hi <= s_tdata; phase <= 2'd2; end
                            default: begin
                                pt_distance     <= {dist_hi, dist_lo};
                                pt_reflectivity <= s_tdata;
                                pt_laser_id     <= ch[3:0];
                                pt_valid        <= blk_ok &&
                                    (!DROP_ZERO || ({dist_hi, dist_lo} != 16'd0));
                                ch              <= ch + 5'd1;
                                phase           <= 2'd0;
                            end
                        endcase
                    end
                endcase
                boff <= (boff == 7'(BLOCK_BYTES-1)) ? '0 : boff + 7'd1;
            end

            if (s_tlast) begin
                cnt <= '0;
                if (hdr_ok_cur && cnt >= 11'(DATA_END-1))
                    pkt_accepted <= 1'b1;
                else
                    pkt_ignored <= 1'b1;
            end else if (cnt != '1) begin
                cnt <= cnt + 11'd1;
            end
        end

        if (rst) begin
            cnt          <= '0;
            hdr_ok       <= 1'b0;
            boff         <= '0;
            phase        <= '0;
            ch           <= '0;
            flag_ok      <= 1'b0;
            blk_ok       <= 1'b0;
            pt_valid     <= 1'b0;
            pkt_accepted <= 1'b0;
            pkt_ignored  <= 1'b0;
        end
    end

endmodule
