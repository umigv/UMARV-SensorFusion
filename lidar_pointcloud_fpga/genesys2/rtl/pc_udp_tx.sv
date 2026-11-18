// Point cloud UDP transmitter.
//
// Pulls points out of the point FIFO and emits complete Ethernet frames
// (Ethernet + IPv4 + UDP headers generated here, FCS added by the MAC).
//
// UDP payload, all little endian:
//
//   off  size  field
//     0     4  magic        "PC2X"
//     4     1  version      1
//     5     1  point_step   12
//     6     2  num_points   1..MAX_POINTS
//     8     4  seq          datagram counter
//    12     2  azimuth      raw lidar azimuth (0.01 deg) of the first point
//    14     2  flags        0
//    16   12*N points       N x { float32 x, float32 y, float32 z }  [metres]
//
// The point area is byte-for-byte the `data` array of a
// sensor_msgs/PointCloud2 with fields x,y,z = FLOAT32 at offsets 0,4,8,
// point_step = 12, is_bigendian = false.
//
// A datagram is sent when MAX_POINTS are waiting, or when points are waiting
// and none has arrived for IDLE_FLUSH clocks (end of a lidar packet).
module pc_udp_tx #(
    parameter int          MAX_POINTS = 120,      // 16 + 12*120 = 1456 B payload
    parameter int          IDLE_FLUSH = 255,
    parameter int          LEVEL_W    = 10,
    parameter logic [47:0] SRC_MAC    = 48'h02_00_00_00_00_01,
    parameter logic [47:0] DST_MAC    = 48'hFF_FF_FF_FF_FF_FF,
    parameter logic [31:0] SRC_IP     = {8'd192, 8'd168, 8'd1, 8'd50},
    parameter logic [31:0] DST_IP     = {8'd255, 8'd255, 8'd255, 8'd255},
    parameter logic [15:0] SRC_PORT   = 16'd2370,
    parameter logic [15:0] DST_PORT   = 16'd2370
) (
    input  logic               clk,
    input  logic               rst,

    // Point FIFO (registered read).  Word = {azimuth[15:0], z, y, x}
    input  logic [LEVEL_W-1:0] fifo_level,
    input  logic               fifo_wr,      // a point was written this cycle
    output logic               fifo_rd,
    input  logic [111:0]       fifo_data,

    // Ethernet frame to MAC
    output logic [7:0]         m_tdata,
    output logic               m_tvalid,
    input  logic               m_tready,
    output logic               m_tlast,

    output logic               pkt_sent      // pulse
);

    localparam int HDR_BYTES = 14 + 20 + 8 + 16;   // 58

    typedef enum logic [2:0] {S_IDLE, S_PREP, S_HDR, S_LOAD, S_AZ, S_PT} state_t;
    state_t state;

    logic [7:0]  idle_cnt;
    logic [7:0]  n_pts;        // points in this datagram
    logic [7:0]  remaining;
    logic [31:0] seq;
    logic [15:0] azimuth;
    logic [15:0] ip_csum;
    logic [5:0]  hidx;
    logic [3:0]  bidx;
    logic [95:0] shreg;

    // ---- lengths and IP header checksum ----------------------------------
    wire [15:0] pts_bytes = {4'b0, n_pts, 3'b0} + {5'b0, n_pts, 2'b0};   // 12*n
    wire [15:0] udp_len   = 16'd8 + 16'd16 + pts_bytes;
    wire [15:0] ip_len    = 16'd20 + udp_len;

    localparam logic [15:0] IP_FLAGS  = 16'h4000;         // DF
    localparam logic [15:0] IP_TTL_PR = {8'd64, 8'h11};
    localparam logic [19:0] CSUM_CONST =
        20'h04500 + 20'(IP_FLAGS) + 20'(IP_TTL_PR) +
        20'(SRC_IP[31:16]) + 20'(SRC_IP[15:0]) +
        20'(DST_IP[31:16]) + 20'(DST_IP[15:0]);

    logic [19:0] csum_sum;
    logic [16:0] csum_fold1;
    logic [15:0] csum_fold2;
    always_comb begin
        csum_sum   = CSUM_CONST + 20'(ip_len) + 20'(seq[15:0]);
        csum_fold1 = 17'(csum_sum[15:0]) + 17'(csum_sum[19:16]);
        csum_fold2 = csum_fold1[15:0] + 16'(csum_fold1[16]);
    end

    // ---- header image -----------------------------------------------------
    logic [HDR_BYTES*8-1:0] hdr;
    always_comb begin
        hdr = {
            // Ethernet
            DST_MAC, SRC_MAC, 16'h0800,
            // IPv4
            16'h4500, ip_len, seq[15:0], IP_FLAGS, IP_TTL_PR, ip_csum, SRC_IP, DST_IP,
            // UDP (checksum 0 = not used, legal for IPv4)
            SRC_PORT, DST_PORT, udp_len, 16'h0000,
            // Application header (little endian)
            "P", "C", "2", "X",
            8'd1, 8'd12, n_pts, 8'd0,
            seq[7:0], seq[15:8], seq[23:16], seq[31:24],
            azimuth[7:0], azimuth[15:8], 16'h0000
        };
    end

    wire [7:0] hdr_byte = hdr[(HDR_BYTES-1-int'(hidx))*8 +: 8];

    // ---- outputs ------------------------------------------------------------
    assign m_tvalid = (state == S_HDR) || (state == S_PT);
    assign m_tdata  = (state == S_HDR) ? hdr_byte : shreg[7:0];
    assign m_tlast  = (state == S_PT) && (bidx == 4'd11) && (remaining == 8'd1);

    wire xfer = m_tvalid && m_tready;

    // First point of a datagram is read early (in S_IDLE) so its azimuth can
    // go in the header; later points are read as the previous one finishes.
    wire start     = (state == S_IDLE) &&
                     ((fifo_level >= LEVEL_W'(MAX_POINTS)) ||
                      ((fifo_level != '0) && (idle_cnt == 8'(IDLE_FLUSH))));
    wire next_pt   = (state == S_PT) && xfer && (bidx == 4'd11) && (remaining != 8'd1);
    assign fifo_rd = start || next_pt;

    always_ff @(posedge clk) begin
        pkt_sent <= 1'b0;

        // idle timer: time since the last point arrived
        if (fifo_wr)                         idle_cnt <= '0;
        else if (idle_cnt != 8'(IDLE_FLUSH)) idle_cnt <= idle_cnt + 8'd1;

        case (state)
            S_IDLE: begin
                if (start) begin
                    n_pts <= (fifo_level >= LEVEL_W'(MAX_POINTS)) ? 8'(MAX_POINTS)
                                                                  : 8'(fifo_level);
                    state <= S_AZ;
                end
            end
            // FIFO output now holds the first point
            S_AZ: begin
                shreg     <= fifo_data[95:0];
                azimuth   <= fifo_data[111:96];
                remaining <= n_pts;
                state     <= S_PREP;
            end
            S_PREP: begin
                ip_csum <= ~csum_fold2;
                hidx    <= '0;
                bidx    <= '0;
                state   <= S_HDR;
            end
            S_HDR: begin
                if (xfer) begin
                    hidx <= hidx + 6'd1;
                    if (hidx == 6'(HDR_BYTES-1)) state <= S_PT;
                end
            end
            S_LOAD: begin
                shreg <= fifo_data[95:0];
                bidx  <= '0;
                state <= S_PT;
            end
            S_PT: begin
                if (xfer) begin
                    shreg <= {8'h00, shreg[95:8]};
                    bidx  <= bidx + 4'd1;
                    if (bidx == 4'd11) begin
                        remaining <= remaining - 8'd1;
                        if (remaining == 8'd1) begin
                            seq      <= seq + 32'd1;
                            pkt_sent <= 1'b1;
                            state    <= S_IDLE;
                        end else begin
                            state <= S_LOAD;
                        end
                    end
                end
            end
            default: state <= S_IDLE;
        endcase

        if (rst) begin
            state    <= S_IDLE;
            idle_cnt <= '0;
            seq      <= '0;
            pkt_sent <= 1'b0;
        end
    end

endmodule
