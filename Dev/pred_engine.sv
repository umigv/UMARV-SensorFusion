module UKF_pred_engine #()(
    input clock, reset,
    input UKF_OUT_PACKET corr_packet,
    input meas_regfile_t timestamp_pack,
    output busy, done, 
    output PRED_PACKET pred_pack
);

typedef enum logic [2:0]{
    IDLE, LOAD_INPUTS, CALC_WHEEL_V, RUN_PRED_MICROCODE, PRED_DONE
} pred_states;
pred_states s, ns;

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
