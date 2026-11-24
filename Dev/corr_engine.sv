module UKF_corr_engine #()(
    input clock, reset,
    input meas_regfile_t corr_packet,
    input PRED_PACKET pred_packet,

    output UKF_OUT_PACKET ukf_output
);

typedef enum logic[4:0] {  
    IDLE, ERROR_CALC
} corr_states;

corr_states s, ns;

always_comb begin : state_transition
    ns = s;
    case(s)
        IDLE: begin
            if(corr_packet.ukf_vn_valid)begin
                ns = ERROR_CALC;
            end
            else begin
                ns = IDLE;
            end
        end
        ERROR_CALC: ns = IDLE;

        default: ns = IDLE;
    endcase
end


always_ff @(posedge clock) begin
    if (reset)
        s <= IDLE;
    else
        s <= ns;
end

// The correction datapath has yet to drive a result.
assign ukf_output = '0;

endmodule
