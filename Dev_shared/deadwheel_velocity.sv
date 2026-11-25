module deadwheel_velocity (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    // Prepared FP32 count delta, held valid through the MAC start edge.
    input  logic [31:0] dw_delta_f32,
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

    output logic        busy,
    output logic        done
);

    localparam logic [6:0] REG_V_WHEEL = 7'd5;

    typedef enum logic [1:0] {IDLE, START_MAC, WAIT_MAC} state_t;
    state_t state;

    assign busy = state != IDLE;
    // The rising edge that samples done also commits the memory write.
    assign done = mem_we;
    assign mac_a = dw_delta_f32;
    assign mac_b = mem_rdata;
    assign mac_clear = mac_start;
    assign mac_last = mac_start;
    assign mem_wdata = mac_result;

    always_comb begin
        mem_addr = '0;
        mem_we = 1'b0;
        mac_start = 1'b0;

        case (state)
            START_MAC: begin
                mem_addr = velocity_scale_addr;
                mac_start = rst_n && !mac_busy;
            end
            WAIT_MAC: begin
                mem_addr = REG_V_WHEEL;
                mem_we = rst_n && mac_result_valid;
            end
            default: begin end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE: if (start) state <= START_MAC;
                START_MAC: if (mac_start) state <= WAIT_MAC;
                WAIT_MAC: if (mem_we) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end

endmodule
