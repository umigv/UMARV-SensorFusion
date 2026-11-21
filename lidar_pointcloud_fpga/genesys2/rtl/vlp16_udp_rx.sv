// VLP-16 receive front end.
//
// Takes whole Ethernet frames from the MAC (preamble and FCS already removed,
// bad-FCS frames already dropped by the MAC frame FIFO) and turns them into
// points.  The work is split in two:
//
//   * v16PacketParser (William Kletzka, lidar_fpga/v16PacketParser.v, used
//     unmodified) walks the 1206-byte Velodyne payload and produces
//     azimuth / distance / reflectivity for every return:
//
//       12 x { 0xFFEE, azimuth[15:0] (LE, 0.01 deg),
//              32 x { distance[15:0] (LE, 2 mm), reflectivity[7:0] } }
//       timestamp[31:0], factory[15:0]          (ignored)
//
//   * this wrapper adds what that parser leaves to its surroundings: the
//     Ethernet / IPv4 / UDP header check (so only real lidar data packets
//     produce points), back-pressure, the laser id, and packet status.
//
// The 32 returns of a block are two firings of the 16 lasers, so
// laser_id = channel index mod 16.
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
    localparam int DATA_END    = HDR_BYTES + 1200; // end of the 12 data blocks
    // UDP length field of a VLP-16 data packet: 8 + 1206
    localparam logic [15:0] UDP_LEN = 16'd1214;

    logic [10:0] cnt;        // byte index in frame, saturating
    logic        hdr_ok;
    logic [3:0]  laser;

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

    // ---------------------------------------------------------------- parser
    // The parser is always ready and has no stall input; it only advances on
    // tvalid, so back-pressure is applied by withholding tvalid while paused.
    logic [15:0] p_azimuth;
    logic [15:0] p_distance;
    logic [7:0]  p_reflectivity;
    logic        p_valid;

    v16PacketParser #(
        .HEADER_BYTES(HDR_BYTES)
    ) parser_inst (
        .clk                (clk),
        .resetn             (!rst),
        .s_AXIstream_tdata  (s_tdata),
        .s_AXIstream_tvalid (xfer),
        .s_AXIstream_tlast  (s_tlast),
        .s_AXIstream_tready (),
        .out_azimuth        (p_azimuth),
        .out_distance       (p_distance),
        .out_reflectivity   (p_reflectivity),
        .out_data_valid     (p_valid)
    );

    // The first point of a frame appears several bytes after the last header
    // byte, so hdr_ok is final by the time it is used here.
    assign pt_azimuth      = p_azimuth;
    assign pt_distance     = p_distance;
    assign pt_reflectivity = p_reflectivity;
    assign pt_laser_id     = laser;
    assign pt_valid        = p_valid && hdr_ok &&
                             (!DROP_ZERO || (p_distance != 16'd0));

    always_ff @(posedge clk) begin
        pkt_accepted <= 1'b0;
        pkt_ignored  <= 1'b0;

        // The parser emits either all 32 returns of a block or none (bad
        // flag), so counting its strobes modulo 16 gives the laser id.
        if (p_valid)
            laser <= laser + 4'd1;

        if (xfer) begin
            if (cnt == 0)
                laser <= '0;

            if (cnt < HDR_BYTES)
                hdr_ok <= hdr_ok_cur && byte_ok;

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
            laser        <= '0;
            pkt_accepted <= 1'b0;
            pkt_ignored  <= 1'b0;
        end
    end

endmodule
