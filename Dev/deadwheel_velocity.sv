module deadwheel_velocity(
    input clock, reset, start,
    input logic [31:0] dw_delta_f32, velocity_scale_f32,
    output logic mac_start, mac_clear, mac_last,
    output logic [31:0] mac_a,mac_b,
    input logic mac_done, mac_result_valid,
    input logic [31:0] mac_result,
    output logic busy, done,
    output logic [31:0] wheel_velocity
);
typedef enum logic [1:0] {
    IDLE,
    ISSUE_MAC
    WAIT_MAC
    FINISHED
}state_t;

state_t state, next_state;
logic [31:0] delta_reg
logic [31:0] scale_reg;
assign mac_a = delta_reg;
assign mac_b = scale_reg
assign mac_start = (state==ISSUE_MAC);
assign mac_clear = (state==ISSUE_MAC);
assign mac_last = (state==ISSUE_MAC);
assign busy = (state!=)