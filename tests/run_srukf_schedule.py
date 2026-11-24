"""Exercise actual UKF input acceptance over four 400 Hz periods."""

from argparse import ArgumentParser
from pathlib import Path
from subprocess import run
from tempfile import TemporaryDirectory

from run_srukf_end_to_end import INITIAL_FACTOR, PROCESS_FACTOR, SOURCES, packed
from run_corr_x import word
from run_ukf_integration import bits

DRIVER = r"""
    integer phase, layout, index;
    integer overflows = 0, refills = 0, corrected_count = 0;
    integer mac_wait, pred_mac_busy, pred_div_busy, corr_mac_busy, corr_div_busy;
    integer first_mac, correction_first;
    PRED_PACKET matching_prediction;
    pred_s_factor_t matching_factor;
    always @(posedge clk) if (rst_n) begin
        if (ukf_pulse)
            $display("RELEASE %0d %0d %0b %0b %0b", ($time-35)/10, ukf_beat_ts,
                     ukf_dw_valid, ukf_vn_valid, dut.time_pack.ukf_pulse);
        if (dut.vn_arrival)
            $display("VN %0d %0d %0b %0b", ($time-35)/10, ukf_beat_ts,
                     dut.vn_dispatch && !dut.vn_pending || dut.vn_store, dut.vn_store);
        if (ukf_vn_overflow) overflows++;
        if (dut.vn_pending && dut.vn_dispatch && dut.vn_store) refills++;
        if (dut.prediction_start) begin
            mac_wait = 0;
            pred_mac_busy = 0;
            pred_div_busy = 0;
            first_mac = -1;
            $display("PSTART %0d %0d", ($time-35)/10, dut.prediction_ts);
        end
        if (dut.backend_active) begin
            if (!dut.prediction_mac_grant) mac_wait++;
            if (dut.prediction_mac_response.busy) pred_mac_busy++;
            if (dut.prediction_div_response.busy) pred_div_busy++;
            if (dut.backend_mac_request.start && first_mac < 0) first_mac = ($time-35)/10;
        end
        if (ukf_pred_done) begin
            matching_prediction = dut.pred_packet;
            matching_factor = dut.pred_s;
            $display("PDONE %0d %0d %0d %0d %0d %0d", ($time-35)/10, dut.prediction_ts,
                     first_mac, mac_wait, pred_mac_busy, pred_div_busy);
        end
        if (dut.correction.state == dut.correction.LOAD_INPUTS) begin
            if (layout == 3) begin
                case (dut.correction.convert_input.ukf_beat_ts)
                    0: if (dut.noise_snapshot[0] !== 32'h3e4ccccd)
                        $fatal(1, "current noise changed while waiting");
                    1: if (dut.noise_snapshot[0] !== 32'h3e800000)
                        $fatal(1, "queued noise was not kept with sample 1");
                    2: if (dut.noise_snapshot[0] !== 32'h3e99999a)
                        $fatal(1, "refill noise was not kept with sample 2");
                    default: $fatal(1, "unexpected queue sample");
                endcase
            end
            if (dut.correction.prediction !== matching_prediction ||
                dut.correction.prediction_s !== matching_factor)
                $fatal(1, "correction consumed the wrong prediction");
            corr_mac_busy = 0;
            corr_div_busy = 0;
            correction_first = -1;
            $display("CSTART %0d %0d", ($time-35)/10, dut.correction.convert_input.ukf_beat_ts);
        end
        if (dut.correction.state == dut.correction.RUN_CORR_MICROCODE) begin
            if (dut.correction.correction_program.mac_start && correction_first < 0)
                correction_first = ($time-35)/10;
            if (dut.correction.correction_program.mac_busy) corr_mac_busy++;
            if (dut.correction.correction_program.div_busy) corr_div_busy++;
        end
        if (ukf_valid) begin
            if (layout == 3 && dut.correction.convert_input.ukf_beat_ts != corrected_count)
                $fatal(1, "VN queue order or duplicate consumption");
            corrected_count++;
            $display("CDONE %0d %0d %0d %0d %0d", ($time-35)/10,
                     dut.correction.convert_input.ukf_beat_ts, correction_first,
                     corr_mac_busy, corr_div_busy);
        end
        if (ukf_failed) $fatal(1, "schedule fixture failed");
    end

    task automatic pulse(input bit wheel, vn, input integer timestamp);
        ukf_dw_valid = wheel;
        ukf_vn_valid = vn;
        ukf_beat_ts = timestamp;
        ukf_pulse = 1;
        @(negedge clk);
        ukf_pulse = 0;
    endtask

    initial begin
        if (!$value$plusargs("phase=%d", phase) ||
            !$value$plusargs("layout=%d", layout)) $fatal(1, "missing settings");
        ukf_vn_lat_f64 = LAT;
        ukf_vn_lon_f64 = LON;
        ukf_vn_yaw_f32 = YAW;
        ukf_vn_rate_f32 = RATE;
        measurement_s = NOISE;
        repeat (3) @(negedge clk);
        rst_n = 1;
        if (layout == 3) begin
            ukf_dw_delta = 24;
            pulse(1, 1, 0);
            repeat (phase-1) @(negedge clk);
            measurement_s[0] = 32'h3e800000;
            pulse(0, 1, 1);
            measurement_s[0] = 32'h3f000000;
            pulse(0, 1, 99);
            while (!dut.vn_dispatch) @(negedge clk);
            measurement_s[0] = 32'h3e99999a;
            pulse(0, 1, 2);
            while (ukf_busy || dut.vn_pending) @(negedge clk);
            if (overflows != 1 || refills != 1 || corrected_count != 3)
                $fatal(1, "queue counts overflow=%0d refill=%0d corrections=%0d",
                       overflows, refills, corrected_count);
            $display("QUEUE PASS one overflow, one simultaneous consume/refill, three results");
            $finish;
        end
        for (index = 0; index < 4; index++) begin
            ukf_dw_delta = 24 + 2*index;
            if (layout == 0) begin
                pulse(1, 0, index);
                repeat (phase-1) @(negedge clk);
                pulse(0, 1, 1000+index);
                repeat (125000-phase-1) @(negedge clk);
            end else if (layout == 1) begin
                pulse(0, 1, index);
                repeat (phase-1) @(negedge clk);
                pulse(1, 0, index);
                repeat (125000-phase-1) @(negedge clk);
            end else begin
                repeat (phase) @(negedge clk);
                pulse(1, 1, index);
                repeat (125000-phase-1) @(negedge clk);
            end
        end
        repeat (phase + 1) @(negedge clk);
        $finish;
    end
endmodule
"""


def bench(root):
    from run_srukf_end_to_end import NOISE
    source = (root / "tests/srukf_end_to_end_tb.sv").read_text()
    source = source[:source.index("    task automatic reset_filter")]
    source = source.replace("srukf_end_to_end_tb", "srukf_schedule_tb")
    driver = DRIVER
    for key, value in {
        "LAT": f"64'h{bits(-.2 / 111194.9, 64):016x}",
        "LON": f"64'h{bits(.1 / 111194.9, 64):016x}",
        "YAW": "32'h42480000", "RATE": "32'h3f4ccccd",
        "NOISE": f"320'h{packed(NOISE)}",
    }.items():
        driver = driver.replace(f"= {key};", f"= {value};")
    return source + driver


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--heading", type=float, default=.7)
    parser.add_argument("--phase", type=int, default=10000)
    parser.add_argument("--check-deadlines", action="store_true",
                        help="assert acceptance and completion for wheel-then-VN")
    args = parser.parse_args()
    if not 0 < args.phase < 125000:
        parser.error("phase must be in (0, 125000)")
    root = Path(__file__).resolve().parents[1]
    with TemporaryDirectory() as directory:
        tb = Path(directory) / "schedule.sv"
        elf = Path(directory) / "schedule.vvp"
        tb.write_text(bench(root))
        initial = (int("".join(map(word, [.1, -.2, args.heading, -.8, .27])), 16) << 1) | 1
        run(["iverilog", "-g2012", "-s", "srukf_schedule_tb", "-o", elf,
             f"-Psrukf_schedule_tb.START_STATE=161'h{initial:x}",
             f"-Psrukf_schedule_tb.START_FACTOR=480'h{packed(INITIAL_FACTOR)}",
             f"-Psrukf_schedule_tb.PROCESS_FACTOR=480'h{packed(PROCESS_FACTOR)}",
             *[f"Dev/{name}.sv" for name in SOURCES], tb], cwd=root, check=True)
        for mode, label in enumerate(("wheel then VN", "VN then wheel", "paired at phase", "queue overflow and refill")):
            print(f"MODE {label}", flush=True)
            sim = run(["vvp", elf, f"+phase={args.phase}", f"+layout={mode}"],
                      cwd=root, timeout=120, check=True, capture_output=True, text=True)
            if mode == 0 and args.check_deadlines:
                rows = [line.split() for line in sim.stdout.splitlines() if line.strip()]
                vn = {int(row[2]): (int(row[1]), int(row[3])) for row in rows if row[0] == "VN"}
                pred = {int(row[2]): int(row[1]) for row in rows if row[0] == "PDONE"}
                corrected = {int(row[2]): int(row[1]) for row in rows if row[0] == "CDONE"}
                assert len(vn) == len(pred) == len(corrected) == 4, sim.stdout
                for index in range(4):
                    arrival, accepted = vn[1000 + index]
                    assert accepted and pred[index] <= (index + 1) * 125000, sim.stdout
                    assert corrected[1000 + index] <= arrival + 125000, sim.stdout
            print("\n".join(line for line in sim.stdout.splitlines()
                            if line.startswith(("RELEASE ", "VN ", "PSTART ", "PDONE ", "CSTART ", "CDONE ", "QUEUE "))))


if __name__ == "__main__":
    main()
