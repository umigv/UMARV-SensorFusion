// One command per rising edge. A read returns two words for sampling at
// the next rising edge. A write stores one full word and returns no data.
module ukf_regfile #(
    parameter bit USE_SKY130 = 1'b0
) (
    input logic clock,
    input logic reset,
    input logic read_enable,
    input logic [7:0] read_addr_a, read_addr_b,
    input logic write_enable,
    input logic [7:0] write_addr,
    input logic [31:0] write_data,
    output logic read_valid,
    output logic [31:0] read_data_a, read_data_b
`ifdef USE_POWER_PINS
    , inout wire vccd1, vssd1
`endif
);
    logic read_cycle, write_cycle;

    // A write takes priority if both enables are asserted.
    assign write_cycle = !reset && write_enable;
    assign read_cycle = !reset && read_enable && !write_enable;

    always_ff @(posedge clock or posedge reset) begin
        if (reset)
            read_valid <= 1'b0;
        else
            read_valid <= read_cycle;
    end

    if (USE_SKY130) begin : gen_sky130
        sky130_sram_1kbyte_1rw1r_32x256_8 storage (
`ifdef USE_POWER_PINS
            .vccd1(vccd1),
            .vssd1(vssd1),
`endif
            .clk0(clock),
            .csb0(!(read_cycle || write_cycle)),
            .web0(!write_cycle),
            .wmask0(4'b1111),
            .addr0(write_cycle ? write_addr : read_addr_a),
            .din0(write_data),
            .dout0(read_data_a),
            .clk1(clock),
            .csb1(!read_cycle),
            .addr1(read_addr_b),
            .dout1(read_data_b)
        );
    end else begin : gen_rtl
        logic [31:0] storage [0:255];

        always_ff @(posedge clock) begin
            if (write_cycle)
                storage[write_addr] <= write_data;
            if (read_cycle) begin
                read_data_a <= storage[read_addr_a];
                read_data_b <= storage[read_addr_b];
            end
        end
    end
endmodule
