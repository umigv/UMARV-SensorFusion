// Lidar stage 7: turn buffered distances into motion-corrected x, y, z.
//
// Two jobs:
//
// 1. Sequencing. Blocks are taken from the buffer in order. Each block is one firing of 32
//    lasers or two firings of 16. For every firing, lidar_motion is asked for the corrected
//    angle's sine and cosine and the shift. That takes about 60 clocks, so it is always asked
//    for one firing ahead of the one whose points are going out.
//
// 2. Arithmetic, per point, on one multiplier (about 10 clocks):
//        ground = distance * cos(elevation)
//        x = ground * cos(a') + shift_x          forward
//        y = -ground * sin(a') + shift_y         left
//        z = distance * sin(elevation)           up
//    a' is the corrected azimuth from lidar_motion. Results are in output counts (2 or 4 mm),
//    limited to +/-32767. A return with distance 0 ("no return") gives x = y = z = -32768.
//
// The frame builder drives it: out_blk_valid says a block is ready (its first firing has been
// worked out); out_blk_take claims it; then 32 times pt_req -> pt_done with one point each.
module lidar_xyz (
    input  logic               clk,
    input  logic               rst,                 // synchronous, active high

    // settings
    input  logic               lasers_32,           // 1: 32 lasers, one firing per block. 0: 16 lasers, two firings
    input  logic               dist_4mm,            // unit of the lidar's distances: 0 = 2 mm, 1 = 4 mm
    input  logic               scale_4mm,           // unit of the output: 0 = 2 mm per count, 1 = 4 mm

    // buffer, read side
    input  logic               blk0_valid,
    input  logic        [15:0] blk0_azimuth,
    input  logic               blk0_sweep,
    input  logic               blk1_valid,
    input  logic        [15:0] blk1_azimuth,
    input  logic               blk1_sweep,
    output logic               blk_pop,
    input  logic        [15:0] rd_data,
    output logic               rd_pop,

    // lidar_motion
    output logic               firing_start,
    output logic               firing_second,
    output logic               sweep_start,
    output logic        [15:0] azimuth,
    input  logic               fire_done,
    input  logic signed [15:0] m_sin0, m_cos0, m_sin1, m_cos1, m_sin2, m_cos2, m_sin3, m_cos3,
    input  logic signed [15:0] m_shift_x, m_shift_y,

    // lidar_laser_table
    output logic        [4:0]  tbl_laser,
    input  logic signed [15:0] tbl_sin_el, tbl_cos_el,
    input  logic signed [9:0]  tbl_az_offset,

    // frame builder
    output logic               out_blk_valid,
    output logic        [15:0] out_blk_azimuth,
    output logic               out_blk_sweep,
    input  logic               out_blk_take,
    input  logic               pt_req,              // one clock: work out the next point of the block
    output logic               pt_done,             // one clock: x, y, z are valid and stay until the next pt_done
    output logic signed [15:0] x, y, z
);

    // ---- which firing lidar_motion has been asked for ---------------------------------------------
    localparam [1:0] N_NONE = 2'd0, N_BUSY = 2'd1, N_READY = 2'd2;
    localparam [1:0] D_HEAD_FIRST = 2'd0, D_HEAD_SECOND = 2'd1, D_NEXT_FIRST = 2'd2;
    logic [1:0] n_state, n_desc;

    logic       in_block;           // a block has been claimed by the frame builder
    logic       cur_second;         // the firing going out is the second of its block
    logic [5:0] pt_idx;             // points of the block already handed over

    // values for the firing whose points are going out
    logic signed [15:0] c_sin0, c_cos0, c_sin1, c_cos1, c_sin2, c_cos2, c_sin3, c_cos3;
    logic signed [15:0] c_shift_x, c_shift_y;

    assign out_blk_valid   = !in_block && (n_state == N_READY) && (n_desc == D_HEAD_FIRST);
    assign out_blk_azimuth = blk0_azimuth;
    assign out_blk_sweep   = blk0_sweep;

    // ---- per-point arithmetic ---------------------------------------------------------------------
    localparam [3:0] X_IDLE = 4'd0, X_SWAP = 4'd1, X_READ = 4'd2, X_G1 = 4'd3, X_G2 = 4'd4, X_X1 = 4'd5,
                     X_X2 = 4'd6, X_Y1 = 4'd7, X_Y2 = 4'd8, X_Z1 = 4'd9, X_Z2 = 4'd10, X_OUT = 4'd11;
    logic [3:0] xs;

    logic signed [16:0] ma, mb;
    logic signed [33:0] mp;
    always_ff @(posedge clk) mp <= ma * mb;
    // product with 15 fractional bits removed, rounded
    wire signed [33:0] mp_r   = mp + 34'sd16384;
    wire signed [18:0] mp_q   = mp_r[33:15];

    logic [15:0]        r_dist;
    logic signed [16:0] ground;
    logic signed [18:0] xg, yg, zg;

    assign tbl_laser = lasers_32 ? pt_idx[4:0] : {1'b0, pt_idx[3:0]};

    // sine and cosine for this laser's horizontal offset class
    logic signed [15:0] sel_sin, sel_cos;
    always_comb begin
        if (tbl_az_offset > 10'sd280)       begin sel_sin = c_sin0; sel_cos = c_cos0; end     // +4.2 degrees
        else if (tbl_az_offset > 10'sd0)    begin sel_sin = c_sin1; sel_cos = c_cos1; end     // +1.4
        else if (tbl_az_offset < -10'sd280) begin sel_sin = c_sin3; sel_cos = c_cos3; end     // -4.2
        else if (tbl_az_offset < 10'sd0)    begin sel_sin = c_sin2; sel_cos = c_cos2; end     // -1.4
        else                                begin sel_sin = c_sin0; sel_cos = c_cos0; end     // no offset
    end

    // lidar units to output units, then add the shift and limit to 16 bits
    function automatic signed [15:0] finish;
        input signed [18:0] v;
        input signed [15:0] shift;
        input               up;         // lidar 4 mm, output 2 mm: double
        input               down;       // lidar 2 mm, output 4 mm: halve
        reg signed [20:0] s;
        begin
            s = up ? {v[18], v, 1'b0} : down ? {{3{v[18]}}, v[18:1]} : {{2{v[18]}}, v};
            s = s + {{5{shift[15]}}, shift};
            finish = (s > 21'sd32767) ? 16'sd32767 : (s < -21'sd32767) ? -16'sd32767 : s[15:0];
        end
    endfunction
    wire unit_up   = dist_4mm && !scale_4mm;
    wire unit_down = !dist_4mm && scale_4mm;

    wire last_point   = (pt_idx == 6'd31);
    wire need_second  = !lasers_32 && (pt_idx == 6'd16) && !cur_second;

    always_ff @(posedge clk) begin
        if (rst) begin
            n_state <= N_NONE; n_desc <= D_HEAD_FIRST;
            in_block <= 1'b0; cur_second <= 1'b0; pt_idx <= 6'd0;
            firing_start <= 1'b0; firing_second <= 1'b0; sweep_start <= 1'b0; azimuth <= 16'd0;
            c_sin0 <= 16'sd0; c_sin1 <= 16'sd0; c_sin2 <= 16'sd0; c_sin3 <= 16'sd0;
            c_cos0 <= 16'sd32767; c_cos1 <= 16'sd32767; c_cos2 <= 16'sd32767; c_cos3 <= 16'sd32767;
            c_shift_x <= 16'sd0; c_shift_y <= 16'sd0;
            blk_pop <= 1'b0; rd_pop <= 1'b0; pt_done <= 1'b0;
            x <= 16'sd0; y <= 16'sd0; z <= 16'sd0;
            xs <= X_IDLE; ma <= 17'sd0; mb <= 17'sd0;
            r_dist <= 16'd0; ground <= 17'sd0; xg <= 19'sd0; yg <= 19'sd0; zg <= 19'sd0;
        end else begin
            firing_start <= 1'b0;
            blk_pop      <= 1'b0;
            rd_pop       <= 1'b0;
            pt_done      <= 1'b0;

            // ---- lidar_motion finished the firing it was asked for
            if (fire_done && (n_state == N_BUSY)) n_state <= N_READY;

            // ---- ask lidar_motion for the next firing, one ahead of the one going out
            if ((n_state == N_NONE) && !firing_start) begin
                if (!in_block) begin
                    if (blk0_valid && !blk_pop) begin
                        azimuth <= blk0_azimuth; firing_second <= 1'b0; sweep_start <= blk0_sweep;
                        firing_start <= 1'b1; n_state <= N_BUSY; n_desc <= D_HEAD_FIRST;
                    end
                end else if (!lasers_32 && !cur_second) begin
                    azimuth <= blk0_azimuth; firing_second <= 1'b1; sweep_start <= 1'b0;
                    firing_start <= 1'b1; n_state <= N_BUSY; n_desc <= D_HEAD_SECOND;
                end else if (blk1_valid) begin
                    azimuth <= blk1_azimuth; firing_second <= 1'b0; sweep_start <= blk1_sweep;
                    firing_start <= 1'b1; n_state <= N_BUSY; n_desc <= D_NEXT_FIRST;
                end
            end

            // ---- the frame builder claims the block: its first firing's values become current
            if (out_blk_take && out_blk_valid) begin
                c_sin0 <= m_sin0; c_cos0 <= m_cos0; c_sin1 <= m_sin1; c_cos1 <= m_cos1;
                c_sin2 <= m_sin2; c_cos2 <= m_cos2; c_sin3 <= m_sin3; c_cos3 <= m_cos3;
                c_shift_x <= m_shift_x; c_shift_y <= m_shift_y;
                in_block <= 1'b1; cur_second <= 1'b0; pt_idx <= 6'd0;
                n_state <= N_NONE;
            end

            // ---- one point
            case (xs)
                X_IDLE: if (pt_req && in_block) xs <= need_second ? X_SWAP : X_READ;
                X_SWAP: begin
                    // 17th point of a 16-laser block: switch to the second firing's values
                    if ((n_state == N_READY) && (n_desc == D_HEAD_SECOND)) begin
                        c_sin0 <= m_sin0; c_cos0 <= m_cos0; c_sin1 <= m_sin1; c_cos1 <= m_cos1;
                        c_sin2 <= m_sin2; c_cos2 <= m_cos2; c_sin3 <= m_sin3; c_cos3 <= m_cos3;
                        c_shift_x <= m_shift_x; c_shift_y <= m_shift_y;
                        cur_second <= 1'b1;
                        n_state <= N_NONE;
                        xs <= X_READ;
                    end
                end
                X_READ: begin
                    r_dist   <= rd_data;
                    rd_pop <= 1'b1;
                    ma <= $signed({1'b0, rd_data}); mb <= {tbl_cos_el[15], tbl_cos_el};
                    xs <= X_G1;
                end
                X_G1: xs <= X_G2;
                X_G2: begin ground <= mp_q[16:0];
                            ma <= mp_q[16:0]; mb <= {sel_cos[15], sel_cos}; xs <= X_X1; end
                X_X1: xs <= X_X2;
                X_X2: begin xg <= mp_q;
                            ma <= ground; mb <= {sel_sin[15], sel_sin}; xs <= X_Y1; end
                X_Y1: xs <= X_Y2;
                X_Y2: begin yg <= -mp_q;
                            ma <= $signed({1'b0, r_dist}); mb <= {tbl_sin_el[15], tbl_sin_el}; xs <= X_Z1; end
                X_Z1: xs <= X_Z2;
                X_Z2: begin zg <= mp_q; xs <= X_OUT; end
                X_OUT: begin
                    if (r_dist == 16'd0) begin
                        x <= -16'sd32768; y <= -16'sd32768; z <= -16'sd32768;       // no return
                    end else begin
                        x <= finish(xg, c_shift_x, unit_up, unit_down);
                        y <= finish(yg, c_shift_y, unit_up, unit_down);
                        z <= finish(zg, 16'sd0,    unit_up, unit_down);
                    end
                    pt_done <= 1'b1;
                    if (last_point) begin
                        // the block is finished: the buffer moves on, and so does our view of it
                        blk_pop  <= 1'b1;
                        in_block <= 1'b0;
                        pt_idx   <= 6'd0;
                        n_desc   <= D_HEAD_FIRST;       // a firing asked for ahead now belongs to the head block
                    end else begin
                        pt_idx <= pt_idx + 6'd1;
                    end
                    xs <= X_IDLE;
                end
                default: xs <= X_IDLE;
            endcase
        end
    end

endmodule
