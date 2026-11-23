`timescale 1ns/1ps
// Four-state SR-UKF sigma points for P=S^T*S, using rows of upper-triangular S.
// The arbiter grants a drained MAC and holds grant until mac_request falls.
// A result timeout retains the request until the generator and MAC are reset.
module sigma_generator #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16,
    parameter integer MAC_TIMEOUT_CYCLES = 16
)(
    input logic clk, resetn, start,
    input logic signed [WIDTH-1:0] state [0:3],
    input logic signed [WIDTH-1:0] S [0:3][0:3],
    input logic signed [WIDTH-1:0] gamma,
    output logic signed [WIDTH-1:0] sigma [0:8][0:3],
    output logic busy, done, overflow,
    output wire mac_request,
    input logic mac_grant,
    output logic mac_start, mac_clear, mac_last,
    output logic signed [WIDTH-1:0] mac_a,
    output logic signed [WIDTH-1:0] mac_b,
    input logic mac_busy, mac_result_valid, mac_overflow, mac_acc_overflow,
    input logic signed [WIDTH-1:0] mac_result,
    output logic error
);
    typedef enum logic [3:0] {IDLE, GENERATE_CENTER, WAIT_MAC_GRANT,
        MAC_LAUNCH, WAIT_MAC_RESULT, STORE_OFFSET, GENERATE_POS_NEG,
        PUBLISH_SIGMA, MAC_ERROR} phase_t;
    phase_t phase;
    logic signed [WIDTH-1:0] state_reg [0:3];
    logic signed [WIDTH-1:0] s_reg [0:9];
    logic signed [WIDTH-1:0] gamma_reg;
    logic signed [WIDTH-1:0] offsets [0:9];
    logic signed [WIDTH-1:0] work_sigma [0:8][0:3];
    logic [3:0] entry_index;
    logic saw_mac_busy;
    logic signed [WIDTH-1:0] completed_result;
    localparam integer WATCHDOG_WIDTH = (MAC_TIMEOUT_CYCLES > 1) ?
                                        $clog2(MAC_TIMEOUT_CYCLES) : 1;
    localparam logic [WATCHDOG_WIDTH-1:0] WATCHDOG_LIMIT =
        WATCHDOG_WIDTH'(MAC_TIMEOUT_CYCLES-1);
    logic [WATCHDOG_WIDTH-1:0] mac_wait_cycles;
    localparam logic signed [WIDTH-1:0] MAX_POS = {1'b0, {(WIDTH-1){1'b1}}};
    localparam logic signed [WIDTH-1:0] MIN_NEG = {1'b1, {(WIDTH-1){1'b0}}};
    localparam logic signed [WIDTH:0] MAX_EXT = {1'b0, MAX_POS};
    localparam logic signed [WIDTH:0] MIN_EXT = {1'b1, MIN_NEG};

    // Row-major packed upper triangle: (0,0)..(0,3),(1,1)..(3,3).
    function automatic integer row_of(input integer index);
        integer r, c, k;
        begin
            row_of=0; k=0;
            for(r=0;r<4;r=r+1)
                for(c=r;c<4;c=c+1) begin
                    if(k==index) row_of=r;
                    k=k+1;
                end
        end
    endfunction
    function automatic integer col_of(input integer index);
        integer r, c, k;
        begin
            col_of=0; k=0;
            for(r=0;r<4;r=r+1)
                for(c=r;c<4;c=c+1) begin
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
    // clear and last are qualifiers sampled with start by MAC_accumulator.
    // Drain any previous valid before issuing a new one-term sum.
    assign mac_start = resetn && phase == MAC_LAUNCH && mac_grant &&
                       !mac_busy && !mac_result_valid;
    assign mac_clear = mac_start;
    assign mac_last = mac_start;
    assign mac_a = gamma_reg;
    assign mac_b = s_reg[entry_index];

    // Constant-index adders avoid dynamic indexing across sigma-point rows.
    wire signed [WIDTH:0] positive [0:9];
    wire signed [WIDTH:0] negative [0:9];
    genvar entry;
    generate for(entry=0;entry<10;entry=entry+1) begin : gen_signs
        localparam integer C=col_of(entry);
        assign positive[entry]=$signed({state_reg[C][WIDTH-1],state_reg[C]}) +
                               $signed({offsets[entry][WIDTH-1],offsets[entry]});
        assign negative[entry]=$signed({state_reg[C][WIDTH-1],state_reg[C]}) -
                               $signed({offsets[entry][WIDTH-1],offsets[entry]});
    end endgenerate

    initial begin
        if(WIDTH<2 || FRAC<0 || FRAC>=WIDTH)
            $fatal(1,"Require WIDTH >= 2 and 0 <= FRAC < WIDTH");
        if(MAC_TIMEOUT_CYCLES<1)
            $fatal(1,"Require MAC_TIMEOUT_CYCLES >= 1");
    end
    always_ff @(posedge clk) begin
        if(!resetn) begin
            phase<=IDLE; busy<=0; done<=0; overflow<=0;
            error<=0; mac_wait_cycles<='0;
            entry_index<=0; saw_mac_busy<=0;
            completed_result<='0; gamma_reg<='0;
            for(integer j=0;j<4;j=j+1) state_reg[j]<='0;
            for(integer e=0;e<10;e=e+1) begin
                s_reg[e]<='0;
                offsets[e]<='0;
            end
            for(integer i=0;i<9;i=i+1)
                for(integer j=0;j<4;j=j+1) begin
                    sigma[i][j]<='0; work_sigma[i][j]<='0;
                end
        end else begin
            done<=0;
            case(phase)
                IDLE: if(start) begin
                    busy<=1; overflow<=0; entry_index<=0;
                    gamma_reg<=gamma;
                    for(integer j=0;j<4;j=j+1) state_reg[j]<=state[j];
                    for(integer e=0;e<10;e=e+1)
                        s_reg[e]<=S[row_of(e)][col_of(e)];
                    phase<=GENERATE_CENTER;
                end
                GENERATE_CENTER: begin
                    for(integer i=0;i<9;i=i+1)
                        for(integer j=0;j<4;j=j+1) work_sigma[i][j]<=state_reg[j];
                    phase<=WAIT_MAC_GRANT;
                end
                WAIT_MAC_GRANT: if(mac_grant && !mac_busy && !mac_result_valid)
                    phase<=MAC_LAUNCH;
                MAC_LAUNCH: if(mac_start) begin
                    saw_mac_busy<=0;
                    mac_wait_cycles<=0;
                    phase<=WAIT_MAC_RESULT;
                end
                WAIT_MAC_RESULT: begin
                    if(mac_busy) saw_mac_busy<=1;
                    if(mac_grant && saw_mac_busy && !mac_busy && mac_result_valid) begin
                        completed_result<=mac_result;
                        if(mac_overflow || mac_acc_overflow) overflow<=1;
                        phase<=STORE_OFFSET;
                    end else if(mac_wait_cycles == WATCHDOG_LIMIT) begin
                        error<=1;
                        phase<=MAC_ERROR;
                    end else begin
                        mac_wait_cycles<=mac_wait_cycles+1'b1;
                    end
                end
                STORE_OFFSET: begin
                    offsets[entry_index]<=completed_result;
                    if(entry_index==9) phase<=GENERATE_POS_NEG;
                    else begin
                        entry_index<=entry_index+1'b1;
                        phase<=MAC_LAUNCH;
                    end
                end
                GENERATE_POS_NEG: begin
                    for(integer e=0;e<10;e=e+1) begin
                        work_sigma[row_of(e)+1][col_of(e)]<=saturate(positive[e]);
                        work_sigma[row_of(e)+5][col_of(e)]<=saturate(negative[e]);
                        if(positive[e]>MAX_EXT || positive[e]<MIN_EXT ||
                           negative[e]>MAX_EXT || negative[e]<MIN_EXT) overflow<=1;
                    end
                    phase<=PUBLISH_SIGMA;
                end
                PUBLISH_SIGMA: begin
                    for(integer i=0;i<9;i=i+1)
                        for(integer j=0;j<4;j=j+1) sigma[i][j]<=work_sigma[i][j];
                    busy<=0; done<=1; phase<=IDLE;
                end
                // Keep ownership on a fault: a late result must not leak
                // into another client's transaction. Reset generator and array.
                MAC_ERROR: begin
                    error<=1;
                end
                default: begin phase<=IDLE; busy<=0; end
            endcase
        end
    end
endmodule
