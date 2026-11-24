// Lidar stage 8 (and 9): build the outgoing frames and write them to the transmit FIFO.
//
// Normal mode. As soon as a block of points is ready, one frame is started and written without
// a pause: Ethernet, IPv4 and UDP headers, a 48-byte payload header, then 6 blocks. Each block is
// a 4-byte prefix {azimuth, flags, 0} followed by 32 points of 6 bytes (x, y, z, 16 bits each,
// low byte first). The layout is in docs/lidar_output_format.md.
//
//   The line cannot be paused once a frame has begun. If a block is not ready when its turn
//   comes, the builder waits up to 40 clocks (the transmit FIFO covers that), then sends a
//   padding block in its place: flag bit 1 set, every point marked "no return".
//
// Bypass mode (stage 9). Received frames are written to the transmit FIFO as they are, without
// their 4 checksum bytes; the transmitter adds a fresh checksum. A frame that arrived damaged
// is passed on marked bad, so it leaves with a wrong checksum too.
//
// Words written to the FIFO are {bad, last, byte}.
module lidar_frame_tx (
    input  logic        clk,
    input  logic        rst,                // synchronous, active high

    // settings
    input  logic        bypass,
    input  logic [47:0] src_mac, dst_mac,
    input  logic [31:0] src_ip, dst_ip,
    input  logic [15:0] src_port, dst_port,
    input  logic [1:0]  lidar_model,
    input  logic        scale_4mm,

    // for the payload header
    input  logic        state_seen,         // the filter has produced a state
    input  logic [31:0] state_ts, state_x, state_y, state_theta, state_omega, state_v,
    input  logic [31:0] arr_sweep_time,     // chip time at which the newest sweep started arriving
    input  logic        flag_rx_bad,        // one clock: a received frame failed its checksum
    input  logic        flag_overflow,      // one clock: the buffer discarded a block

    // from the transform
    input  logic        blk_valid,
    input  logic [15:0] blk_azimuth,
    input  logic        blk_sweep,
    output logic        blk_take,
    output logic        pt_req,
    input  logic        pt_done,
    input  logic signed [15:0] x, y, z,

    // received frame bytes, for bypass
    input  logic [7:0]  rx_tdata,
    input  logic        rx_tvalid,
    input  logic        rx_tlast,
    input  logic        rx_bad,             // with rx_tlast

    // write side of the transmit FIFO
    output logic        wr_en,
    output logic [9:0]  wr_data,
    input  logic        wr_full,

    output logic        frame_started,      // one clock: a point frame was begun
    output logic        tx_dropped          // one clock: a bypass byte was lost because the FIFO was full
);

    localparam [15:0] IP_LEN  = 16'd1252;   // 20 + 8 + 1224
    localparam [15:0] UDP_LEN = 16'd1232;   // 8 + 1224
    localparam [6:0]  HDR_LAST = 7'd89;     // 14 + 20 + 8 + 48 header bytes, numbered 0..89

    localparam [3:0] B_IDLE = 4'd0, B_CSUM = 4'd1, B_HDR = 4'd2, B_BLK = 4'd3, B_PRE = 4'd4,
                     B_PTREQ = 4'd5, B_PTWAIT = 4'd6, B_PT = 4'd7;
    logic [3:0] st;

    // ---- values held for the whole frame ---------------------------------------------------------
    logic [31:0] h_state_ts, h_x, h_y, h_theta, h_omega, h_v, h_sweep_time;
    logic [15:0] h_sweep_id, h_first_az;
    logic [7:0]  h_flags;
    logic [31:0] seq;
    logic [15:0] rd_sweep_id;       // sweep of the block most recently taken
    logic [31:0] rd_sweep_time;
    logic        sticky_rx_bad, sticky_overflow;

    // ---- IPv4 header checksum: fixed words plus the two addresses --------------------------------
    logic [19:0] csum;
    logic [2:0]  cstep;
    wire  [16:0] fold1 = {1'b0, csum[15:0]} + {13'd0, csum[19:16]};
    wire  [15:0] fold2 = fold1[15:0] + {15'd0, fold1[16]};
    logic [15:0] ip_csum;

    // ---- position in the frame ---------------------------------------------------------------------
    logic [6:0]  hidx;
    logic [2:0]  blk_i;             // 0..5
    logic [5:0]  pt_i;              // 0..31
    logic [2:0]  byte_i;
    logic [5:0]  wait_cnt;          // clocks spent waiting for a block
    logic        pad_blk;           // the current block is padding
    logic [15:0] p_az;
    logic        p_sweep;
    logic signed [15:0] px, py, pz;

    // ---- header bytes ----------------------------------------------------------------------------
    logic [7:0] hbyte;
    always_comb begin
        case (hidx)
            7'd0:  hbyte = dst_mac[47:40];  7'd1:  hbyte = dst_mac[39:32];  7'd2:  hbyte = dst_mac[31:24];
            7'd3:  hbyte = dst_mac[23:16];  7'd4:  hbyte = dst_mac[15:8];   7'd5:  hbyte = dst_mac[7:0];
            7'd6:  hbyte = src_mac[47:40];  7'd7:  hbyte = src_mac[39:32];  7'd8:  hbyte = src_mac[31:24];
            7'd9:  hbyte = src_mac[23:16];  7'd10: hbyte = src_mac[15:8];   7'd11: hbyte = src_mac[7:0];
            7'd12: hbyte = 8'h08;           7'd13: hbyte = 8'h00;                                   // IPv4
            7'd14: hbyte = 8'h45;           7'd15: hbyte = 8'h00;
            7'd16: hbyte = IP_LEN[15:8];    7'd17: hbyte = IP_LEN[7:0];
            7'd18: hbyte = 8'h00;           7'd19: hbyte = 8'h00;                                   // identification
            7'd20: hbyte = 8'h40;           7'd21: hbyte = 8'h00;                                   // don't fragment
            7'd22: hbyte = 8'h40;           7'd23: hbyte = 8'h11;                                   // TTL 64, UDP
            7'd24: hbyte = ip_csum[15:8];   7'd25: hbyte = ip_csum[7:0];
            7'd26: hbyte = src_ip[31:24];   7'd27: hbyte = src_ip[23:16];   7'd28: hbyte = src_ip[15:8];   7'd29: hbyte = src_ip[7:0];
            7'd30: hbyte = dst_ip[31:24];   7'd31: hbyte = dst_ip[23:16];   7'd32: hbyte = dst_ip[15:8];   7'd33: hbyte = dst_ip[7:0];
            7'd34: hbyte = src_port[15:8];  7'd35: hbyte = src_port[7:0];
            7'd36: hbyte = dst_port[15:8];  7'd37: hbyte = dst_port[7:0];
            7'd38: hbyte = UDP_LEN[15:8];   7'd39: hbyte = UDP_LEN[7:0];
            7'd40: hbyte = 8'h00;           7'd41: hbyte = 8'h00;                                   // UDP checksum not used
            // payload header, low byte first
            7'd42: hbyte = 8'h50;           7'd43: hbyte = 8'h43;           7'd44: hbyte = 8'h32;          7'd45: hbyte = 8'h58;   // "PC2X"
            7'd46: hbyte = 8'd3;                                                                    // version
            7'd47: hbyte = h_flags;
            7'd48: hbyte = 8'd192;          7'd49: hbyte = 8'd0;                                    // points
            7'd50: hbyte = seq[7:0];        7'd51: hbyte = seq[15:8];       7'd52: hbyte = seq[23:16];     7'd53: hbyte = seq[31:24];
            7'd54: hbyte = h_sweep_id[7:0]; 7'd55: hbyte = h_sweep_id[15:8];
            7'd56: hbyte = h_first_az[7:0]; 7'd57: hbyte = h_first_az[15:8];
            7'd58: hbyte = {6'd0, lidar_model};
            7'd59: hbyte = 8'd6;                                                                    // bytes per point
            7'd60: hbyte = 8'd6;                                                                    // blocks
            7'd61: hbyte = 8'd32;                                                                   // points per block
            7'd62: hbyte = h_sweep_time[7:0]; 7'd63: hbyte = h_sweep_time[15:8]; 7'd64: hbyte = h_sweep_time[23:16]; 7'd65: hbyte = h_sweep_time[31:24];
            7'd66: hbyte = h_state_ts[7:0];   7'd67: hbyte = h_state_ts[15:8];   7'd68: hbyte = h_state_ts[23:16];   7'd69: hbyte = h_state_ts[31:24];
            7'd70: hbyte = h_x[7:0];          7'd71: hbyte = h_x[15:8];          7'd72: hbyte = h_x[23:16];          7'd73: hbyte = h_x[31:24];
            7'd74: hbyte = h_y[7:0];          7'd75: hbyte = h_y[15:8];          7'd76: hbyte = h_y[23:16];          7'd77: hbyte = h_y[31:24];
            7'd78: hbyte = h_theta[7:0];      7'd79: hbyte = h_theta[15:8];      7'd80: hbyte = h_theta[23:16];      7'd81: hbyte = h_theta[31:24];
            7'd82: hbyte = h_omega[7:0];      7'd83: hbyte = h_omega[15:8];      7'd84: hbyte = h_omega[23:16];      7'd85: hbyte = h_omega[31:24];
            7'd86: hbyte = h_v[7:0];          7'd87: hbyte = h_v[15:8];          7'd88: hbyte = h_v[23:16];          7'd89: hbyte = h_v[31:24];
            default: hbyte = 8'h00;
        endcase
    end

    logic [7:0] pre_byte, pt_byte;
    always_comb begin
        case (byte_i[1:0])
            2'd0:    pre_byte = p_az[7:0];
            2'd1:    pre_byte = p_az[15:8];
            2'd2:    pre_byte = {6'd0, pad_blk, p_sweep};
            default: pre_byte = 8'h00;
        endcase
        case (byte_i)
            3'd0:    pt_byte = px[7:0];
            3'd1:    pt_byte = px[15:8];
            3'd2:    pt_byte = py[7:0];
            3'd3:    pt_byte = py[15:8];
            3'd4:    pt_byte = pz[7:0];
            default: pt_byte = pz[15:8];
        endcase
    end

    wire last_byte = (st == B_PT) && (byte_i == 3'd5) && (pt_i == 6'd31) && (blk_i == 3'd5);

    // ---- bypass: hold back 4 bytes so the received checksum can be left off --------------------------
    logic [31:0] dl;                // the last four bytes received, oldest in the top byte
    logic [2:0]  dl_n;              // how many of them are waiting (0..4)
    wire         by_write = bypass && rx_tvalid && (dl_n == 3'd4);

    // ---- what goes to the FIFO this clock -------------------------------------------------------------
    always_comb begin
        wr_en   = 1'b0;
        wr_data = 10'd0;
        if (bypass) begin
            // the byte leaving the delay line is the last of the frame when the frame ends now
            wr_en   = by_write && !wr_full;
            wr_data = {rx_tlast && rx_bad, rx_tlast, dl[31:24]};
        end else begin
            case (st)
                B_HDR:   begin wr_en = !wr_full; wr_data = {2'b00, hbyte};    end
                B_PRE:   begin wr_en = !wr_full; wr_data = {2'b00, pre_byte}; end
                B_PT:    begin wr_en = !wr_full; wr_data = {1'b0, last_byte, pt_byte}; end
                default: begin wr_en = 1'b0;     wr_data = 10'd0; end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            st <= B_IDLE;
            h_state_ts <= 32'd0; h_x <= 32'd0; h_y <= 32'd0; h_theta <= 32'd0; h_omega <= 32'd0; h_v <= 32'd0;
            h_sweep_time <= 32'd0; h_sweep_id <= 16'd0; h_first_az <= 16'd0; h_flags <= 8'd0;
            seq <= 32'd0; rd_sweep_id <= 16'd0; rd_sweep_time <= 32'd0;
            sticky_rx_bad <= 1'b0; sticky_overflow <= 1'b0;
            csum <= 20'd0; cstep <= 3'd0; ip_csum <= 16'd0;
            hidx <= 7'd0; blk_i <= 3'd0; pt_i <= 6'd0; byte_i <= 3'd0; pad_blk <= 1'b0; wait_cnt <= 6'd0;
            p_az <= 16'd0; p_sweep <= 1'b0; px <= 16'sd0; py <= 16'sd0; pz <= 16'sd0;
            blk_take <= 1'b0; pt_req <= 1'b0; frame_started <= 1'b0; tx_dropped <= 1'b0;
            dl <= 32'd0; dl_n <= 3'd0;
        end else begin
            blk_take      <= 1'b0;
            pt_req        <= 1'b0;
            frame_started <= 1'b0;
            tx_dropped    <= 1'b0;

            if (flag_rx_bad)   sticky_rx_bad   <= 1'b1;
            if (flag_overflow) sticky_overflow <= 1'b1;

            // ---- bypass delay line
            if (bypass) begin
                if (rx_tvalid) begin
                    dl <= {dl[23:0], rx_tdata};
                    if (rx_tlast)            dl_n <= 3'd0;              // the four bytes left in it are the checksum
                    else if (dl_n != 3'd4)   dl_n <= dl_n + 3'd1;
                    if (by_write && wr_full) tx_dropped <= 1'b1;
                end
                st <= B_IDLE;
            end else begin
                dl_n <= 3'd0;
                case (st)
                    B_IDLE: if (blk_valid) begin
                        // freeze everything the header needs
                        h_state_ts <= state_ts; h_x <= state_x; h_y <= state_y;
                        h_theta <= state_theta; h_omega <= state_omega; h_v <= state_v;
                        h_first_az   <= blk_azimuth;
                        h_sweep_id   <= blk_sweep ? (rd_sweep_id + 16'd1) : rd_sweep_id;
                        h_sweep_time <= blk_sweep ? arr_sweep_time : rd_sweep_time;
                        h_flags <= {4'd0, sticky_overflow, sticky_rx_bad, scale_4mm, state_seen};
                        sticky_rx_bad   <= flag_rx_bad;
                        sticky_overflow <= flag_overflow;
                        csum <= 20'h0C9F5;                              // 4500 + 04E4 + 4000 + 4011
                        cstep <= 3'd0;
                        frame_started <= 1'b1;
                        st <= B_CSUM;
                    end
                    B_CSUM: begin
                        cstep <= cstep + 3'd1;
                        case (cstep)
                            3'd0: csum <= csum + {4'd0, src_ip[31:16]};
                            3'd1: csum <= csum + {4'd0, src_ip[15:0]};
                            3'd2: csum <= csum + {4'd0, dst_ip[31:16]};
                            3'd3: csum <= csum + {4'd0, dst_ip[15:0]};
                            default: begin ip_csum <= ~fold2; hidx <= 7'd0; st <= B_HDR; end
                        endcase
                    end
                    B_HDR: if (!wr_full) begin
                        if (hidx == HDR_LAST) begin blk_i <= 3'd0; st <= B_BLK; end
                        hidx <= hidx + 7'd1;
                    end
                    // start of a block: take it if it is ready, otherwise send padding
                    B_BLK: if (blk_valid || (wait_cnt == 6'd40)) begin
                        wait_cnt <= 6'd0;
                        if (blk_valid) begin
                            blk_take <= 1'b1;
                            pad_blk  <= 1'b0;
                            p_az     <= blk_azimuth;
                            p_sweep  <= blk_sweep;
                            if (blk_sweep) begin rd_sweep_id <= rd_sweep_id + 16'd1; rd_sweep_time <= arr_sweep_time; end
                        end else begin
                            pad_blk <= 1'b1; p_az <= 16'd0; p_sweep <= 1'b0;
                        end
                        byte_i <= 3'd0; pt_i <= 6'd0;
                        st <= B_PRE;
                    end else begin
                        wait_cnt <= wait_cnt + 6'd1;
                    end
                    B_PRE: if (!wr_full) begin
                        byte_i <= byte_i + 3'd1;
                        if (byte_i == 3'd3) st <= B_PTREQ;
                    end
                    B_PTREQ: begin
                        byte_i <= 3'd0;
                        if (pad_blk) begin
                            px <= -16'sd32768; py <= -16'sd32768; pz <= -16'sd32768;
                            st <= B_PT;
                        end else begin
                            pt_req <= 1'b1;
                            st <= B_PTWAIT;
                        end
                    end
                    B_PTWAIT: if (pt_done) begin px <= x; py <= y; pz <= z; st <= B_PT; end
                    B_PT: if (!wr_full) begin
                        byte_i <= byte_i + 3'd1;
                        if (byte_i == 3'd5) begin
                            if (pt_i == 6'd31) begin
                                if (blk_i == 3'd5) begin seq <= seq + 32'd1; st <= B_IDLE; end
                                else begin blk_i <= blk_i + 3'd1; st <= B_BLK; end
                            end else begin
                                pt_i <= pt_i + 6'd1;
                                st <= B_PTREQ;
                            end
                        end
                    end
                    default: st <= B_IDLE;
                endcase
            end
        end
    end

endmodule
