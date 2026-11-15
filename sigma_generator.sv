`timescale 1ns/1ps
// Five-state SR-UKF sigma points for P=S^T*S (rows of upper-triangular S).
// Inputs are captured on start while idle; lower-triangular entries are ignored.
// All data uses signed WIDTH/FRAC fixed point. gamma is supplied by the caller.
// Shared-array protocol: request exclusive ownership with mac_request. Grant
// only after previous clients have drained; retain ownership until request falls.
// This client clears each used lane and finalizes a single product. The array
// must have 11 lanes with matching WIDTH/FRAC. No multipliers instantiated here.
// Products use the array's rounding/saturation; final +/- operations saturate.
// overflow is sticky for this transaction (including any array fault). A flagged
// transaction may have clipped offsets and must be handled by the caller.
// sigma publishes atomically on completion and holds until the next completion.
// done pulses one cycle. Synchronous active-low reset aborts and clears outputs.
// Reset the shared array too when aborting, or drain it before granting again.
module sigma_generator #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16
)(
    input logic clk, resetn, start,
    input logic signed [WIDTH-1:0] state [0:4],
    input logic signed [WIDTH-1:0] S [0:4][0:4],
    input logic signed [WIDTH-1:0] gamma,
    output logic signed [WIDTH-1:0] sigma [0:10][0:4],
    output logic busy, done, overflow,
    output wire mac_request,
    input logic mac_grant,
    output logic [10:0] mac_start, mac_clear, mac_last,
    output logic signed [WIDTH-1:0] mac_a [0:10],
    output logic signed [WIDTH-1:0] mac_b [0:10],
    input logic [10:0] mac_busy, mac_result_valid, mac_overflow, mac_acc_overflow,
    input logic signed [WIDTH-1:0] mac_result [0:10]
);
    typedef enum logic [3:0] {IDLE, GENERATE_CENTER, WAIT_GRANT,
        CALCULATE_OFFSETS, WAIT_MAC_DONE, GENERATE_POS_NEG,
        STORE_SIGMA_POINTS} phase_t;
    phase_t phase;
    logic signed [WIDTH-1:0] state_reg [0:4];
    logic signed [WIDTH-1:0] s_reg [0:4][0:4];
    logic signed [WIDTH-1:0] gamma_reg;
    logic signed [WIDTH-1:0] offsets [0:14];
    logic signed [WIDTH-1:0] work_sigma [0:10][0:4];
    logic batch;
    logic [10:0] pending;
    wire [10:0] active_mask = batch ? 11'h00f : 11'h7ff;
    wire [10:0] received = pending & mac_result_valid;
    localparam logic signed [WIDTH-1:0] MAX_POS = {1'b0, {(WIDTH-1){1'b1}}};
    localparam logic signed [WIDTH-1:0] MIN_NEG = {1'b1, {(WIDTH-1){1'b0}}};
    localparam logic signed [WIDTH:0] MAX_EXT = {1'b0, MAX_POS};
    localparam logic signed [WIDTH:0] MIN_EXT = {1'b1, MIN_NEG};

    // Row-major packed upper triangle: (0,0)..(0,4),(1,1)..(4,4).
    function automatic integer row_of(input integer index);
        integer r, c, k;
        begin
            row_of=0; k=0;
            for(r=0;r<5;r=r+1)
                for(c=r;c<5;c=c+1) begin
                    if(k==index) row_of=r;
                    k=k+1;
                end
        end
    endfunction
    function automatic integer col_of(input integer index);
        integer r, c, k;
        begin
            col_of=0; k=0;
            for(r=0;r<5;r=r+1)
                for(c=r;c<5;c=c+1) begin
                    if(k==index) col_of=c;
                    k=k+1;
                end
        end
    endfunction
    function automatic signed [WIDTH-1:0] saturate(input logic signed [WIDTH:0] value);
        if(value > MAX_EXT) saturate=MAX_POS;
        else if(value < MIN_EXT) saturate=MIN_NEG;
        else saturate=value[WIDTH-1:0];
    endfunction

    assign mac_request = busy;
    always_comb begin
        mac_start='0; mac_clear='0; mac_last='0;
        for(integer l=0;l<11;l=l+1) begin
            mac_a[l]='0; mac_b[l]='0;
            if(!batch || l<4) begin
                mac_a[l]=gamma_reg;
                mac_b[l]=s_reg[row_of((batch ? 11 : 0)+l)][col_of((batch ? 11 : 0)+l)];
            end
        end
        if(phase==CALCULATE_OFFSETS && mac_grant && !(|(mac_busy & active_mask))) begin
            mac_start=active_mask; mac_clear=active_mask; mac_last=active_mask;
        end
    end

    // Constant-index adders avoid dynamic indexing across sigma-point rows.
    wire signed [WIDTH:0] positive [0:14];
    wire signed [WIDTH:0] negative [0:14];
    genvar entry;
    generate for(entry=0;entry<15;entry=entry+1) begin : gen_signs
        localparam integer R=row_of(entry), C=col_of(entry);
        assign positive[entry]=$signed({state_reg[C][WIDTH-1],state_reg[C]}) +
                               $signed({offsets[entry][WIDTH-1],offsets[entry]});
        assign negative[entry]=$signed({state_reg[C][WIDTH-1],state_reg[C]}) -
                               $signed({offsets[entry][WIDTH-1],offsets[entry]});
    end endgenerate

    initial begin
        if(WIDTH<2 || FRAC<0 || FRAC>=WIDTH)
            $fatal(1,"Require WIDTH >= 2 and 0 <= FRAC < WIDTH");
    end
    always_ff @(posedge clk) begin
        if(!resetn) begin
            phase<=IDLE; busy<=0; done<=0; overflow<=0;
            batch<=0; pending<='0; gamma_reg<='0;
            for(integer j=0;j<5;j=j+1) begin
                state_reg[j]<='0;
                for(integer k=0;k<5;k=k+1) s_reg[j][k]<='0;
            end
            for(integer i=0;i<15;i=i+1) offsets[i]<='0;
            for(integer i=0;i<11;i=i+1)
                for(integer j=0;j<5;j=j+1) begin
                    sigma[i][j]<='0; work_sigma[i][j]<='0;
                end
        end else begin
            done<=0;
            case(phase)
                IDLE: if(start) begin
                    busy<=1; overflow<=0; batch<=0;
                    gamma_reg<=gamma;
                    for(integer j=0;j<5;j=j+1) begin
                        state_reg[j]<=state[j];
                        for(integer k=0;k<5;k=k+1) s_reg[j][k]<=S[j][k];
                    end
                    phase<=GENERATE_CENTER;
                end
                GENERATE_CENTER: begin
                    for(integer i=0;i<11;i=i+1)
                        for(integer j=0;j<5;j=j+1) work_sigma[i][j]<=state_reg[j];
                    phase<=WAIT_GRANT;
                end
                WAIT_GRANT: if(mac_grant && !(|mac_busy)) phase<=CALCULATE_OFFSETS;
                CALCULATE_OFFSETS: if(mac_grant && !( |(mac_busy & active_mask))) begin
                    pending<=active_mask; phase<=WAIT_MAC_DONE;
                end
                WAIT_MAC_DONE: begin
                    pending<=pending & ~received;
                    for(integer l=0;l<11;l=l+1)
                        if(received[l] && (!batch || l<4))
                            offsets[(batch ? 11 : 0)+l]<=mac_result[l];
                    if(|(received & (mac_overflow | mac_acc_overflow))) overflow<=1;
                    if((pending & ~received)==0) begin
                        if(!batch) begin batch<=1; phase<=CALCULATE_OFFSETS; end
                        else phase<=GENERATE_POS_NEG;
                    end
                end
                GENERATE_POS_NEG: begin
                    for(integer e=0;e<15;e=e+1) begin
                        work_sigma[row_of(e)+1][col_of(e)]<=saturate(positive[e]);
                        work_sigma[row_of(e)+6][col_of(e)]<=saturate(negative[e]);
                        if(positive[e]>MAX_EXT || positive[e]<MIN_EXT ||
                           negative[e]>MAX_EXT || negative[e]<MIN_EXT) overflow<=1;
                    end
                    phase<=STORE_SIGMA_POINTS;
                end
                STORE_SIGMA_POINTS: begin
                    for(integer i=0;i<11;i=i+1)
                        for(integer j=0;j<5;j=j+1) sigma[i][j]<=work_sigma[i][j];
                    busy<=0; done<=1; phase<=IDLE;
                end
                default: begin phase<=IDLE; busy<=0; end
            endcase
        end
    end
endmodule
