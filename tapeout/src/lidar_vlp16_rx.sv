// VLP-16 receive front end for the lidar block.
//
// Takes whole Ethernet frames as a byte stream (preamble and frame check already removed)
// and turns them into points. This is vlp16_udp_rx.sv from the FPGA design, rewritten in the
// plain SystemVerilog subset the Tiny Tapeout tools accept. The logic is unchanged; the UDP
// port and the drop-zero option are now inputs (settings) where they used to be parameters.
//
//   * v16PacketParser (William Kletzka, used unmodified) walks the 1206-byte Velodyne
//     payload and produces azimuth / distance / reflectivity for every return:
//
//       12 x { 0xFFEE, azimuth[15:0] (LE, 0.01 deg),
//              32 x { distance[15:0] (LE, 2 mm), reflectivity[7:0] } }
//       timestamp[31:0], factory[15:0]          (ignored)
//
//   * this wrapper adds the Ethernet / IPv4 / UDP header check (so only real lidar data
//     packets produce points), the laser id, and packet status.
//
// The 32 returns of a block are two firings of the 16 lasers, so
// laser_id = channel index mod 16.
//
// Not handled: VLAN tags, IP options, IP fragments, dual-return mode.
module lidar_vlp16_rx (
    input  logic        clk,
    input  logic        rst,             // synchronous, active high

    // Settings
    input  logic [15:0] lidar_port,      // UDP destination port the lidar sends to. 0 = accept any port
    input  logic        drop_zero,       // 1: returns with distance 0 (no return) produce no point

    // Ethernet frame bytes
    input  logic [7:0]  s_tdata,
    input  logic        s_tvalid,
    output logic        s_tready,
    input  logic        s_tlast,

    // Stops consuming bytes while high. The chip cannot pause the incoming link, so this is
    // tied low there; it is kept so the block still matches the FPGA version.
    input  logic        pause,

    // Parsed points
    output logic        pt_valid,
    output logic [15:0] pt_distance,     // raw, 2 mm / LSB, unsigned
    output logic [15:0] pt_azimuth,      // raw, 0.01 deg / LSB, 0..35999 (of the block's first firing)
    output logic [3:0]  pt_laser_id,
    output logic [7:0]  pt_reflectivity,

    // Status (single-cycle pulses)
    output logic        pkt_accepted,    // a complete lidar data packet was parsed
    output logic        pkt_ignored      // a frame that was not a lidar data packet
);

    localparam HDR_BYTES = 42;                  // Ethernet 14 + IPv4 20 + UDP 8
    localparam DATA_END  = HDR_BYTES + 1200;    // end of the 12 data blocks
    localparam [15:0] UDP_LEN = 16'd1214;       // UDP length field of a VLP-16 data packet: 8 + 1206

    logic [10:0] cnt;        // byte index in frame, saturating
    logic        hdr_ok;
    logic [3:0]  laser;

    assign s_tready = !pause;
    wire xfer = s_tvalid && s_tready;

    // hdr_ok as seen by the byte currently on the bus
    wire hdr_ok_cur = (cnt == 11'd0) ? 1'b1 : hdr_ok;
    wire any_port   = (lidar_port == 16'd0);

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
            11'd36: byte_ok = any_port || (s_tdata == lidar_port[15:8]);
            11'd37: byte_ok = any_port || (s_tdata == lidar_port[7:0]);
            11'd38: byte_ok = (s_tdata == UDP_LEN[15:8]);
            11'd39: byte_ok = (s_tdata == UDP_LEN[7:0]);
            default: byte_ok = 1'b1;
        endcase
    end

    // ---------------------------------------------------------------- parser
    // The parser is always ready and has no stall input; it only advances on
    // tvalid, so a pause is applied by withholding tvalid.
    logic [15:0] p_azimuth;
    logic [15:0] p_distance;
    logic [7:0]  p_reflectivity;
    logic        p_valid;
    logic        p_tready_unused;

    v16PacketParser #(
        .HEADER_BYTES(HDR_BYTES)
    ) parser_inst (
        .clk                (clk),
        .resetn             (!rst),
        .s_AXIstream_tdata  (s_tdata),
        .s_AXIstream_tvalid (xfer),
        .s_AXIstream_tlast  (s_tlast),
        .s_AXIstream_tready (p_tready_unused),
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
                             (!drop_zero || (p_distance != 16'd0));

    always_ff @(posedge clk) begin
        if (rst) begin
            cnt          <= 11'd0;
            hdr_ok       <= 1'b0;
            laser        <= 4'd0;
            pkt_accepted <= 1'b0;
            pkt_ignored  <= 1'b0;
        end else begin
            pkt_accepted <= 1'b0;
            pkt_ignored  <= 1'b0;

            // The parser emits either all 32 returns of a block or none (bad
            // flag), so counting its strobes modulo 16 gives the laser id.
            if (p_valid)
                laser <= laser + 4'd1;

            if (xfer) begin
                if (cnt == 11'd0)
                    laser <= 4'd0;

                if (cnt < HDR_BYTES)
                    hdr_ok <= hdr_ok_cur && byte_ok;

                if (s_tlast) begin
                    cnt <= 11'd0;
                    if (hdr_ok_cur && (cnt >= DATA_END - 1))
                        pkt_accepted <= 1'b1;
                    else
                        pkt_ignored <= 1'b1;
                end else if (cnt != 11'h7FF) begin
                    cnt <= cnt + 11'd1;
                end
            end
        end
    end

    wire _unused = &{p_tready_unused, 1'b0};

endmodule
