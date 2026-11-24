"""Prepare isolated full, placeholder, and shared-decode SR-UKF RTL copies."""

from argparse import ArgumentParser
from pathlib import Path
import re
from shutil import copytree


def angle_rows(source, defs):
    constants = {name: int(value) for name, value in
                 re.findall(r"(\w+)\s*=\s*9'd(\d+)", source + defs)}
    rows = {}
    for match in re.finditer(
        r"^\s*(9'd\d+|[A-Z][A-Z_0-9]+): begin ([^\n]+?) end\s*$", source, re.M
    ):
        label, body = match.groups()
        pc = int(label[3:]) if label.startswith("9'd") else constants.get(label, -1)
        if 416 <= pc <= 478:
            rows[pc] = match, dict(re.findall(r"uop\.(\w+) = ([^;]+);", body))
    assert set(rows) == set(range(416, 479))
    return rows


def share_decode(source, rows):
    # PC and sequence remain unchanged. A single row generates the common
    # fields for the two addresses, with a selector only on differing fields.
    pairs = [(418, 423), (419, 424), (420, 425)]
    pairs += [(pc, pc + 18) for pc in range(426, 444)]
    pairs += [(pc, pc + 9) for pc in range(462, 469)] + [(470, 478)]
    edits = []
    for a, b in pairs:
        first, left = rows[a]
        second, right = rows[b]
        fields = dict.fromkeys([*left, *right])
        assignments = []
        for field in fields:
            l, r = left.get(field, "1'b0"), right.get(field, "1'b0")
            value = l if l == r else f"pc == 9'd{a} ? {l} : {r}"
            assignments.append(f"uop.{field} = {value};")
        edits.append((first.start(), first.end(),
                      f"\n        9'd{a}, 9'd{b}: begin {' '.join(assignments)} end\n"))
        edits.append((second.start(), second.end(), "\n"))
    for start, end, replacement in sorted(edits, reverse=True):
        source = source[:start] + replacement + source[end:]
    return source


def placeholders(source, rows):
    first = rows[416][0].start()
    last = rows[478][0].end()
    return source[:first] + """
        // Test-only area control. Identity wrap, sin=0, cos=1.
        PRED_ANGLE_WRAP: begin uop.seq = SEQ_RET; end
        PRED_ANGLE_SIN, PRED_ANGLE_COS: begin
            uop.dp = UOP_MAC;
            uop.clr_acc = 1'b1;
            uop.src_a = pc == PRED_ANGLE_SIN ? PROM_MATH_ZERO : PROM_MATH_ONE;
            uop.src_b = PROM_MATH_ONE;
            uop.dest = PRED_ANGLE_RESULT;
            uop.wb_en = 1'b1;
            uop.instr_done = 1'b1;
            uop.seq = SEQ_RET;
        end
""" + source[last:]


def strip_unused_angle_hardware(rtl):
    # The ROM is hierarchical. Remove unreachable sequencer operations
    # explicitly so cross-module constant propagation is unnecessary.
    rtl = rtl.replace("    localparam int ANGLE_INDEX = int'(PRED_ANGLE_ARG) - int'(SPAD_FIRST);\n", "")
    rtl = rtl.replace(", sq_word, wrap_word", ", sq_word")
    rtl = rtl.replace(", spad[ANGLE_INDEX]", "")
    rtl = rtl.replace("                  (angle_wrap_step(wrap_word) & {32{addr == PROM_WRAP_STEP}}) |\n", "")
    rtl = rtl.replace("default: return prom_math_read(addr);",
                      "default: return addr == PROM_MATH_ONE ? 32'h3f800000 : 32'b0;")
    rtl = rtl.replace("(uop.seq == SEQ_RET && depth == 0) ||\n"
                      "                         (uop.seq == SEQ_ANGLE_CHECK && !angle_supported(a));",
                      "(uop.seq == SEQ_RET && depth == 0);")
    return rtl.replace("                    SEQ_FP_LT: if (fp32_lt(a, b)) pc <= uop.targ_pc;\n", "")


def comparison_bench(defs, source, shared):
    full = source.replace("module pred_microcode_prog", "module full_angle_rom", 1)
    candidate = shared.replace("module pred_microcode_prog", "module shared_angle_rom", 1)
    bench = """
module compare_angle_rom;
    logic [8:0] pc;
    logic central_downdate, wheel_valid;
    pred_ukf_uop expected, actual;
    full_angle_rom reference_rom (pc, central_downdate, wheel_valid, expected);
    shared_angle_rom trial_rom (pc, central_downdate, wheel_valid, actual);
    initial begin
        for (int flags = 0; flags < 4; flags++) begin
            {central_downdate, wheel_valid} = flags[1:0];
            for (int address = 0; address < 512; address++) begin
                pc = address[8:0];
                #1;
                if (expected !== actual)
                    $fatal(1, "instruction mismatch: PC %0d, flags %0d", pc, flags);
            end
        end
        $display("PASS: all 2048 instruction/control combinations match");
        $finish;
    end
endmodule
"""
    return "\n".join((defs, full, candidate, bench))


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="empty directory for the isolated variants")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = (root / "Dev/pred_microcode.sv").read_text()
    defs = (root / "Dev/sys_defs.sv").read_text()
    rows = angle_rows(source, defs)
    shared = share_decode(source, rows)
    for name in ("full", "placeholder", "shared"):
        target = args.output / name
        for folder in ("Dev", "tests"):
            copytree(root / folder, target / folder)
        rom = target / "Dev/pred_microcode.sv"
        if name == "placeholder":
            rom.write_text(placeholders(source, rows))
            seq = target / "Dev/pred_sequencer.sv"
            seq.write_text(strip_unused_angle_hardware(seq.read_text()))
        elif name == "shared":
            rom.write_text(shared)
    (args.output / "compare_angle_rom.sv").write_text(comparison_bench(defs, source, shared))


if __name__ == "__main__":
    main()
