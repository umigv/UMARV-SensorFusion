"""Simulate prediction wrap, sine, and cosine through the real FP32 MAC."""

from math import cos, pi, sin
from pathlib import Path
from random import Random
from subprocess import run
from tempfile import TemporaryDirectory

from run_corr_x import f32, word, wrap_heading


def main():
    angles = [0.0, -0.0, pi, -pi, pi / 2, -pi / 2, 128 * pi, -128 * pi]
    for boundary in (-3 * pi / 4, -pi / 4, pi / 4, 3 * pi / 4, -pi, pi):
        angles.extend((boundary - 2e-7, boundary, boundary + 2e-7))
    angles.extend(i * pi / 32 for i in range(-256, 257))
    random = Random(400)
    angles.extend(random.uniform(-128 * pi, 128 * pi) for _ in range(150))
    angles.extend(sign * (n + .49) * 2 * pi for n in (15, 31, 63) for sign in (-1, 1))
    vectors = []
    for angle in map(f32, angles):
        wrapped = wrap_heading(angle, rounded=True)
        for operation, cosine, expected in (
            ("UOP_WRAP", 0, wrapped),
            ("UOP_TRIG", 0, sin(wrapped)),
            ("UOP_TRIG", 1, cos(wrapped)),
        ):
            vectors.append(f"check({operation}, {cosine}, 32'h{word(angle)}, 32'h{word(expected)}, 0);")
    for angle in (float("nan"), float("inf"), -float("inf"), 129 * pi):
        vectors.append(f"check(UOP_WRAP, 0, 32'h{word(angle)}, 0, 1);")

    bench = """
module pred_angle_math_tb;
    logic clk = 0, rst_n = 0, pulse = 0, prediction_done = 0;
    always #5 clk = !clk;
    logic prediction_start, ukf_busy, mac_grant, div_grant;
    ukf_mac_request_t mac_request = '0;
    ukf_mac_response_t mac_response;
    ukf_divsqrt_request_t div_request = '0;
    ukf_divsqrt_response_t div_response;
    logic [8:0] pc;
    pred_ukf_uop uop;
    logic [31:0] memory [0:63];
    int wrap_steps, max_wrap_steps;
    int cycles, wrap_zero, wrap_pi, sine_zero, cosine_zero;

    UKF #(.EXTERNAL_PREDICTION(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .ukf_beat(1'b0), .ukf_pulse(pulse),
        .ukf_beat_ts(32'b0), .ukf_vn_valid(1'b0), .ukf_vn_yaw_f32(32'b0),
        .ukf_vn_rate_f32(32'b0), .ukf_vn_lat_f64(64'b0), .ukf_vn_lon_f64(64'b0),
        .ukf_dw_valid(1'b1), .ukf_dw_delta(28'sd0), .measurement_s(320'b0),
        .prediction_start(prediction_start), .prediction_done(prediction_done),
        .prediction_result(161'b0), .prediction_result_s(480'b0), .ukf_busy(ukf_busy),
        .prediction_mac_request(mac_request), .prediction_mac_response(mac_response),
        .prediction_mac_grant(mac_grant), .prediction_div_request(div_request),
        .prediction_div_response(div_response), .prediction_div_grant(div_grant)
    );
    pred_microcode_prog rom (.pc(pc), .central_downdate(1'b1), .wheel_valid(1'b1), .uop(uop));

    function automatic logic [31:0] operand(input logic [6:0] addr);
        if (addr == PROM_WRAP_STEP) return angle_wrap_step(memory[PRED_ANGLE_ARG]);
        return addr < 64 ? memory[addr[5:0]] : prom_math_read(addr);
    endfunction

    task automatic tick;
        @(negedge clk);
        cycles++;
        if (cycles > 20000) $fatal(1, "microcode timeout at PC %0d", pc);
    endtask

    task automatic execute(input logic [8:0] entry, output bit rejected);
        logic [8:0] next_pc, return_pc;
        logic [31:0] a, b;
        @(negedge clk);
        cycles = 0;
        wrap_steps = 0;
        rejected = 0;
        return_pc = 9'd511;
        pc = entry;
        while (1) begin
            #1;
            a = operand(uop.src_a);
            b = operand(uop.src_b);
            next_pc = pc + 9'd1;
            if (uop.dp == UOP_MAC) begin
                if (uop.src_a == PROM_WRAP_STEP) wrap_steps++;
                if (!mac_grant || mac_response.busy || !uop.instr_done)
                    $fatal(1, "unexpected MAC issue at PC %0d", pc);
                mac_request = {1'b1, uop.clr_acc, uop.wb_en, a, b[31] ^ uop.neg_b, b[30:0]};
                tick();
                mac_request = '0;
                if (!mac_response.busy)
                    $fatal(1, "shared MAC did not accept request at PC %0d", pc);
                if (uop.wb_en) begin
                    while (!mac_response.valid) tick();
                    memory[uop.dest] = mac_response.result;
                end else begin
                    while (mac_response.busy) tick();
                    if (mac_response.valid) $fatal(1, "unexpected MAC writeback");
                end
            end else if (uop.dp != UOP_NOP) begin
                $fatal(1, "unexpected datapath opcode at PC %0d", pc);
            end
            case (uop.seq)
                SEQ_ANGLE_CHECK: if (!angle_supported(a)) begin
                    rejected = 1;
                    disable execute;
                end
                SEQ_FP_LT: if (fp32_lt(a, b)) next_pc = uop.targ_pc;
                SEQ_JUMP: next_pc = uop.targ_pc;
                SEQ_CALL: begin
                    return_pc = next_pc;
                    next_pc = uop.targ_pc;
                end
                SEQ_RET: next_pc = return_pc;
                SEQ_HALT: disable execute;
                default: begin end
            endcase
            tick();
            if (next_pc == 511) disable execute;
            pc = next_pc;
        end
    endtask

    task automatic check(input dp_uop op, input logic use_cos,
                         input logic [31:0] input_word, expected, input logic reject);
        real actual, target, error;
        logic [31:0] result;
        bit rejected;
        int wrap_cycles;
        memory[PRED_ANGLE_ARG] = input_word;
        execute(PRED_ANGLE_WRAP, rejected);
        wrap_cycles = cycles;
        if (wrap_steps > 7) $fatal(1, "wrap exceeded bound");
        if (wrap_steps > max_wrap_steps) max_wrap_steps = wrap_steps;
        if (rejected !== reject) $fatal(1, "invalid rejection for %h", input_word);
        if (!reject && op == UOP_TRIG) begin
            execute(use_cos ? PRED_ANGLE_COS : PRED_ANGLE_SIN, rejected);
            if (rejected) $fatal(1, "unexpected trig rejection");
        end
        result = op == UOP_WRAP ? memory[PRED_ANGLE_ARG] : memory[PRED_ANGLE_RESULT];
        if (!reject && op == UOP_WRAP && result !== expected)
            $fatal(1, "wrap(%h) = %h expected %h", input_word, result, expected);
        if (!reject && op == UOP_TRIG) begin
            actual = fp_to_real(result);
            target = fp_to_real(expected);
            error = actual - target;
            if (error < -4e-6 || error > 4e-6 || result[30:23] == 8'hff)
                $fatal(1, "trig(%h), cosine=%d: got %g expected %g", input_word, use_cos, actual, target);
        end
        if (input_word == 32'h40490fdb && op == UOP_WRAP) wrap_pi = wrap_cycles;
        if (input_word == 0) begin
            if (op == UOP_WRAP) wrap_zero = wrap_cycles;
            else if (use_cos) cosine_zero = wrap_cycles + cycles;
            else sine_zero = wrap_cycles + cycles;
        end
    endtask

    function automatic real fp_to_real(input logic [31:0] bits);
        if (bits[30:0] == 0)
            return 0.0;
        return $bitstoreal({bits[31], 11'(int'(bits[30:23]) + 896), bits[22:0], 29'b0});
    endfunction

    initial begin
        bit rejected;
        repeat (2) @(negedge clk);
        rst_n = 1;
        pulse = 1;
        @(negedge clk);
        pulse = 0;
        wait (prediction_start);
        @(negedge clk);
        if (!mac_grant || !div_grant) $fatal(1, "backend arithmetic grant missing");
        div_request = {1'b1, 1'b0, 32'h40c00000, 32'h40000000};
        @(negedge clk);
        div_request = '0;
        cycles = 0;
        while (!div_response.done) tick();
        if (div_response.result !== 32'h40400000 || div_response.invalid)
            $fatal(1, "shared DIV failed");
        div_request = {1'b1, 1'b1, 32'h40800000, 32'b0};
        @(negedge clk);
        div_request = '0;
        while (!div_response.done) tick();
        if (div_response.result !== 32'h40000000 || div_response.invalid)
            $fatal(1, "shared SQRT failed");
        VECTORS

        $display("prediction angle vectors passed; max wrap reductions %0d", max_wrap_steps);
        $display("zero-angle wrap/sine/cosine cycles: %0d/%0d/%0d; wrap(pi): %0d",
                 wrap_zero, sine_zero, cosine_zero, wrap_pi);
        prediction_done = 1;
        @(negedge clk);
        if (mac_grant || div_grant || ukf_busy) $fatal(1, "backend retained arithmetic grant");
        $finish;
    end
endmodule
"""
    root = Path(__file__).resolve().parents[1]
    with TemporaryDirectory() as tmp:
        tb = Path(tmp) / "pred_angle_math_tb.sv"
        elf = Path(tmp) / "pred_angle_math.vvp"
        tb.write_text(bench.replace("VECTORS", "\n        ".join(vectors)))
        run(["iverilog", "-g2012", "-s", "pred_angle_math_tb", "-o", elf,
             *[f"Dev/{name}.sv" for name in (
                 "sys_defs", "FP_MAC", "FP_SQRTDIV", "corr_microcode", "corr_engine",
                 "int_to_fp32", "deadwheel_velocity", "lat_to_xy", "pred_engine",
                 "pred_microcode", "pred_sequencer", "vn_to_correction", "UKF",
             )], tb],
            cwd=root, check=True)
        hierarchy = elf.read_text().splitlines()
        assert not any(' "pred_angle_math" ' in line for line in hierarchy)
        for module in ("fp32_mac", "fp32_divsqrt"):
            count = sum(f'"{module}"' in line and ".scope module," in line
                        for line in hierarchy)
            assert count == 1, (module, count)
        run(["vvp", elf], cwd=root, check=True, timeout=30)


if __name__ == "__main__":
    main()
