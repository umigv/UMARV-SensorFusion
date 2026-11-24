module UKF_pred_engine (
    input clock, reset,
    input UKF_OUT_PACKET corr_packet,
    input meas_regfile_t timestamp_pack,
    input  logic [6:0]  velocity_scale_addr,
    input  logic [31:0] mem_rdata,
    output logic [6:0]  mem_addr,
    output logic        mem_we,
    output logic [31:0] mem_wdata,
    output logic        mac_start,
    output logic        mac_clear,
    output logic        mac_last,
    output logic [31:0] mac_a,
    output logic [31:0] mac_b,
    input  logic        mac_busy,
    input  logic        mac_result_valid,
    input  logic [31:0] mac_result,
    output busy, done, 
    output PRED_PACKET pred_pack
);

typedef enum logic [2:0]{
    IDLE, LOAD_INPUTS, CALC_WHEEL_V, RUN_PRED_MICROCODE, PRED_DONE
} pred_states;
pred_states s, ns;
logic [31:0] dw_delta_f32;
logic wheel_velocity_done;

int_to_fp32 wheel_delta_converter (
    .int_in(timestamp_pack.ukf_dw_delta),
    .fp_out(dw_delta_f32)
);

deadwheel_velocity wheel_velocity (
    .clk(clock),
    .rst_n(!reset),
    .start(s == LOAD_INPUTS),
    .dw_delta_f32(dw_delta_f32),
    .velocity_scale_addr(velocity_scale_addr),
    .mem_rdata(mem_rdata),
    .mem_addr(mem_addr),
    .mem_we(mem_we),
    .mem_wdata(mem_wdata),
    .mac_start(mac_start),
    .mac_clear(mac_clear),
    .mac_last(mac_last),
    .mac_a(mac_a),
    .mac_b(mac_b),
    .mac_busy(mac_busy),
    .mac_result_valid(mac_result_valid),
    .mac_result(mac_result),
    .busy(),
    .done(wheel_velocity_done)
);

assign busy = (s != IDLE);
assign done = (s == PRED_DONE);
// The prediction datapath has yet to drive a result.
assign pred_pack = '0;

always_ff @(posedge clock or posedge reset)begin
    if(reset)
        s<=IDLE;
    else 
        s<=ns;
end 

always_comb begin
    ns = s;
    case(s)
        IDLE: begin
            if(timestamp_pack.ukf_dw_valid)begin
                ns = LOAD_INPUTS;
            end
        end
            LOAD_INPUTS: begin
                ns = CALC_WHEEL_V;
            end 
            CALC_WHEEL_V: begin
                if (wheel_velocity_done)
                    ns = RUN_PRED_MICROCODE;
            end 
            RUN_PRED_MICROCODE: begin
                ns = PRED_DONE;
            end
            PRED_DONE:begin
                ns = IDLE;
            end
            default: ns=IDLE;
    endcase
end



endmodule
