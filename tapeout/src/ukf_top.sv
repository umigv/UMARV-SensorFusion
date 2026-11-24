// Five-state square-root unscented Kalman filter, as a small stored-program machine.
//
// States: x east and y north (metres from the first position fix), theta (radians, anticlockwise
// from east), omega (rad/s), and r, the dead wheel's radius. The filter's equations and tuning
// are in ukf_prog.py, which writes the step table and the constants (ukf_tables.sv). This file is
// only the machine that runs them:
//
//   ukf_steps     the table of steps (about 540). A step is  d = a op b,  or a jump/call/return.
//   ukf_consts    the constants the steps use
//   ukf_store     64 words of 32 bits, held in latches. Words 0-5 are wired straight out as the
//                 state the lidar block reads.
//   fp_serial     one single-precision unit: add, subtract, multiply, divide, square root, and
//                 integer difference to float. One bit per clock.
//
// One beat takes about 3,200 operations. The filter starts on the measurement pulse and raises
// state_valid when the new state is in the output words (see the testbench for the measured
// time). A pulse that arrives while it is still working is ignored and counted.
//
// The first beat with a good VN-300 message sets the origin and the starting state. Until then,
// and while enable is low, the filter produces nothing. Dropping enable makes it start again.
//
// What the steps assume about the sensors (ukf_prog.py has the arithmetic):
//   - yaw is degrees clockwise from north; turn rate is rad/s, positive clockwise
//   - latitude is between 32 and 64 degrees north and longitude between 64 and 128 degrees
//     west, so that a fixed slice of each double's bits can be subtracted from the origin's.
//     One count of the latitude slice is 2^-27 degree (0.8 mm); of the longitude slice, 2^-26.
//   - the wheel encoder has 16,384 counts per turn and moves fewer than 2^25 counts a beat
//   - the beat is 2.5 ms (400 Hz); dt is a constant in the table
// Tuning values are constants in the table. The tuning writes from the SPI block (cfg_*) are
// not used yet.
//
// The output words change one after another over about 30 clocks at the end of a beat, just
// before state_valid. A reader that must have all six from the same beat should take them on
// state_valid.
//
// Written in the plain SystemVerilog subset the Tiny Tapeout tools accept.
module ukf_top (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,

    input  logic        meas_pulse,         // one clock: the measurements below belong to a new beat
    input  logic [31:0] beat_ts,
    input  logic        vn_valid,
    input  logic [31:0] vn_yaw_f32,
    input  logic [31:0] vn_rate_f32,
    input  logic [63:0] vn_lat_f64,
    input  logic [63:0] vn_lon_f64,
    input  logic        dw_valid,
    input  logic signed [27:0] dw_delta,

    input  logic        cfg_we,
    input  logic [6:0]  cfg_addr,
    input  logic [7:0]  cfg_wdata,

    output logic        state_valid,        // one clock: the six words below are a new state
    output logic [31:0] state_ts,
    output logic [31:0] state_x, state_y, state_theta, state_omega, state_v
);
    // ---- the current step ----------------------------------------------------------------------
    logic [9:0]  pc;
    wire  [25:0] word;
    ukf_steps u_steps (.pc(pc), .word(word));

    wire [2:0] fn     = word[25:23];
    wire       setf   = word[22];
    wire [1:0] wcond  = word[21:20];
    wire [6:0] a_addr = word[19:13];
    wire [6:0] b_addr = word[12:6];
    wire [5:0] d_addr = word[5:0];
    wire [2:0] cop    = word[22:20];
    wire [2:0] ccond  = word[19:17];
    wire [9:0] target = word[9:0];

    localparam [2:0] FN_RAW = 3'd6, FN_CTL = 3'd7;
    localparam [2:0] C_JMP = 3'd0, C_CALL = 3'd1, C_RET = 3'd2, C_HALT = 3'd3, C_DONE = 3'd4, C_SETI = 3'd5;

    // ---- storage, constants, sensor words -----------------------------------------------------
    logic        m_we;
    logic [5:0]  m_waddr;
    logic [31:0] m_wdata;
    logic [6:0]  rd_addr;
    wire  [31:0] m_rdata, k_value;
    wire  [191:0] m_first;
    ukf_store u_store (.clk(clk), .we(m_we), .waddr(m_waddr), .wdata(m_wdata),
                       .raddr(rd_addr[5:0]), .rdata(m_rdata), .first6(m_first));
    ukf_consts u_consts (.index(rd_addr[5:0]), .value(k_value));

    logic [31:0] ext;
    always_comb begin
        case (rd_addr[2:0])
            3'd0:    ext = vn_yaw_f32;
            3'd1:    ext = vn_rate_f32;
            3'd2:    ext = {6'd0, vn_lat_f64[45:20]};
            3'd3:    ext = {6'd0, vn_lon_f64[45:20]};
            3'd4:    ext = {{4{dw_delta[27]}}, dw_delta};
            3'd5:    ext = beat_ts;
            // latitude as single precision: same sign, exponent re-biased, top of the fraction
            3'd6:    ext = {vn_lat_f64[63], ~vn_lat_f64[59], vn_lat_f64[58:52], vn_lat_f64[51:29]};
            default: ext = 32'd0;
        endcase
    end
    wire [31:0] din = !rd_addr[6]             ? m_rdata :
                      (rd_addr[6:3] == 4'd15) ? ext     : k_value;

    // ---- arithmetic ---------------------------------------------------------------------------
    logic        f_load_a, f_load_b, f_start;
    wire         f_busy, f_done;
    wire  [31:0] f_result;
    fp_serial u_fp (.clk(clk), .rst_n(rst_n), .load_a(f_load_a), .load_b(f_load_b), .start(f_start), .op(fn),
                    .din(din), .busy(f_busy), .done(f_done), .result(f_result));

    // ---- sequencer ----------------------------------------------------------------------------
    localparam [2:0] Q_CLEAR = 3'd0, Q_IDLE = 3'd1, Q_A = 3'd2, Q_B = 3'd3, Q_GO = 3'd4, Q_WAIT = 3'd5,
                     Q_SETTLE1 = 3'd6, Q_SETTLE2 = 3'd7;
    logic [2:0]  q;
    logic [4:0]  clr;                   // after reset the six output words are cleared, one every four clocks
    logic        flag, started, vn_ok, dw_ok;
    logic [9:0]  ret0, ret1;            // return addresses: calls go two deep at most
    logic [1:0]  depth;
    logic [7:0]  overruns;              // measurement pulses that arrived while a beat was still being worked on

    wire is_ctl = (fn == FN_CTL);
    wire running = (q != Q_CLEAR) && (q != Q_IDLE);
    assign f_load_a = (q == Q_A) && !is_ctl && (fn != FN_RAW);
    assign f_load_b = (q == Q_B);
    assign f_start  = (q == Q_GO);
    assign rd_addr  = (q == Q_B) ? b_addr : a_addr;

    wire write_ok = (wcond == 2'd0) || (wcond == 2'd2 && flag) || (wcond == 2'd3 && !flag);

    logic take;
    always_comb begin
        case (ccond)
            3'd1:    take = flag;
            3'd2:    take = !flag;
            3'd3:    take = vn_ok;
            3'd4:    take = !vn_ok;
            3'd5:    take = dw_ok;
            3'd6:    take = !dw_ok;
            3'd7:    take = !started;
            default: take = 1'b1;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q <= Q_CLEAR; clr <= 5'd0; pc <= 10'd0; flag <= 1'b0; started <= 1'b0; vn_ok <= 1'b0; dw_ok <= 1'b0;
            ret0 <= 10'd0; ret1 <= 10'd0; depth <= 2'd0; overruns <= 8'd0;
            m_we <= 1'b0; m_waddr <= 6'd0; m_wdata <= 32'd0; state_valid <= 1'b0;
        end else begin
            m_we <= 1'b0; state_valid <= 1'b0;
            if (!enable) started <= 1'b0;
            if (meas_pulse && enable && running && overruns != 8'hFF) overruns <= overruns + 8'd1;
            case (q)
                Q_CLEAR: begin
                    clr <= clr + 5'd1;
                    if (clr[1:0] == 2'd0) begin m_we <= 1'b1; m_waddr <= {3'd0, clr[4:2]}; m_wdata <= 32'd0; end
                    if (clr == 5'd23) q <= Q_IDLE;
                end
                Q_IDLE: begin
                    if (meas_pulse && enable) begin
                        vn_ok <= vn_valid; dw_ok <= dw_valid; pc <= 10'd0; depth <= 2'd0; q <= Q_A;
                    end
                end
                Q_A: begin
                    if (is_ctl) begin
                        pc <= pc + 10'd1;
                        case (cop)
                            C_JMP:  if (take) pc <= target;
                            C_CALL: begin
                                if (depth == 2'd0) ret0 <= pc + 10'd1; else ret1 <= pc + 10'd1;
                                depth <= depth + 2'd1; pc <= target;
                            end
                            C_RET:  begin
                                pc <= (depth == 2'd2) ? ret1 : ret0;
                                depth <= depth - 2'd1;
                            end
                            C_SETI: started <= 1'b1;
                            C_DONE: begin state_valid <= 1'b1; q <= Q_IDLE; end
                            default: q <= Q_IDLE;       // C_HALT
                        endcase
                    end else if (fn == FN_RAW) begin
                        if (write_ok) begin m_we <= 1'b1; m_waddr <= d_addr; m_wdata <= din; end
                        if (setf) flag <= din[31];
                        q <= Q_SETTLE1;
                    end else begin
                        q <= Q_B;
                    end
                end
                Q_B:  q <= Q_GO;
                Q_GO: q <= Q_WAIT;
                Q_WAIT: if (f_done) begin
                    if (write_ok) begin m_we <= 1'b1; m_waddr <= d_addr; m_wdata <= f_result; end
                    if (setf) flag <= f_result[31];
                    q <= Q_SETTLE1;
                end
                Q_SETTLE1: q <= Q_SETTLE2;
                default: begin          // Q_SETTLE2
                    pc <= pc + 10'd1;
                    q <= Q_A;
                end
            endcase
        end
    end

    assign state_ts    = m_first[31:0];
    assign state_x     = m_first[63:32];
    assign state_y     = m_first[95:64];
    assign state_theta = m_first[127:96];
    assign state_omega = m_first[159:128];
    assign state_v     = m_first[191:160];

    wire _unused = &{f_busy, cfg_we, cfg_addr, cfg_wdata, vn_lat_f64[62:60], vn_lat_f64[19:0], vn_lon_f64[63:46],
                     vn_lon_f64[19:0], overruns, 1'b0};
endmodule

// 64 x 32 latch memory. Same write scheme as lidar_latch_mem: the data is registered and held,
// one falling-edge flip-flop per word is that word's latch enable, and writes must be at least
// 3 clocks apart (the sequencer above leaves 3 or more). A word must not be read until 2 clocks
// after its write.
module ukf_store (
    input  logic        clk,
    input  logic        we,
    input  logic [5:0]  waddr,
    input  logic [31:0] wdata,
    input  logic [5:0]  raddr,
    output logic [31:0] rdata,
    output logic [191:0] first6
);
    logic        we_q;
    logic [5:0]  wa_q;
    logic [31:0] wd_q;
    always_ff @(posedge clk) begin
        we_q <= we;
        if (we) begin
            wa_q <= waddr;
            wd_q <= wdata;
        end
    end

    logic [63:0] gate;
    integer n;
    always_ff @(negedge clk) begin
        for (n = 0; n < 64; n = n + 1)
            gate[n] <= we_q && (wa_q == n[5:0]);
    end

    wire [31:0] q [0:63];
    assign rdata = q[raddr];

    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin : word
`ifdef SYNTHESIS
            genvar b;
            for (b = 0; b < 32; b = b + 1) begin : bit_
                sky130_fd_sc_hd__dlxtp_1 u_latch (.D(wd_q[b]), .GATE(gate[i]), .Q(q[i][b]));
            end
`else
            reg [31:0] d;
            always_latch if (gate[i]) d = wd_q;
            assign q[i] = d;
`endif
        end
        for (i = 0; i < 6; i = i + 1) begin : outw
            assign first6[32*i +: 32] = q[i];
        end
    endgenerate
endmodule
