"""Prepare isolated correction-control area experiments."""

from argparse import ArgumentParser
from pathlib import Path
import re
from shutil import copytree


def delete_rows(source, addresses):
    for address in addresses:
        source, count = re.subn(rf"(?m)^ *7'd{address}:.*\n", "", source)
        assert count == 1, address
    return source


def stage_loop(source):
    source = delete_rows(source, (37, 38, 39))
    for name in ("PC_XY_DONE", "PC_XYZ_DONE", "PC_FULL_DONE"):
        source = re.sub(rf"({name}\s*= )7'd\d+", r"\1PC_X_DONE", source)
    source = source.replace("PC_X_DONE, PC_XY_DONE, PC_XYZ_DONE, PC_FULL_DONE:", "PC_X_DONE:")
    source = source.replace(
        "uop.src_a = REG_D; uop.dest = REG_TEMP; end",
        "uop.src_a = update_stage == UPDATE_XYZW ? REG_TEMP : REG_D; uop.dest = REG_TEMP; end", 1
    ).replace(
        "uop.src_a = REG_MEAS_X; uop.src_b = REG_Q; uop.dest = REG_G;",
        "uop.src_a = update_stage == UPDATE_X ? REG_MEAS_X : REG_EPS; "
        "uop.src_b = REG_Q; uop.dest = REG_G;", 1
    )
    a = source.index("                            case (update_stage)")
    b = source.index("                        end else", a)
    source = source[:a] + "                            pc <= PC_X_DONE;\n" + source[b:]
    source = source.replace("                    7'd36: if (update_stage != UPDATE_XY)\n"
                            "                        pc <= PC_HEADING_VARIANCE;",
                            """                    7'd8: if (update_stage != UPDATE_X)
                        pc <= PC_XY_VECTOR;
                    7'd36: pc <= update_stage == UPDATE_XY ? 7'd6 : PC_HEADING_VARIANCE;""")
    source = source.replace("PC_RATE_VARIANCE : 7'd37", "PC_RATE_VARIANCE : 7'd6")
    source = source.replace("PC_RATE_VARIANCE: pc <= 7'd37;", "PC_RATE_VARIANCE: pc <= 7'd6;")
    a = source.index("                            x_update_valid <= pc == PC_X_DONE;")
    b = source.index("                        end else if (mac_start)", a)
    return source[:a] + """                            x_update_valid <= update_stage == UPDATE_X;
                            xy_update_valid <= update_stage == UPDATE_XY;
                            xyz_update_valid <= update_stage == UPDATE_XYZ;
                            case (update_stage)
                                UPDATE_X: pc <= PC_WHITEN;
                                UPDATE_XY: pc <= PC_HEADING_WHITEN;
                                default: pc <= PC_RATE_WHITEN;
                            endcase
                            if ((update_stage == UPDATE_X && ENABLE_SECOND_UPDATE) ||
                                (update_stage == UPDATE_XY && ENABLE_THIRD_UPDATE) ||
                                (update_stage == UPDATE_XYZ && ENABLE_FOURTH_UPDATE)) begin
                                case (update_stage)
                                    UPDATE_X: update_stage <= UPDATE_XY;
                                    UPDATE_XY: update_stage <= UPDATE_XYZ;
                                    default: update_stage <= UPDATE_XYZW;
                                endcase
                                row <= '0;
                                col <= '0;
                            end else begin
                                full_update_valid <= update_stage == UPDATE_XYZW;
                                done <= 1'b1;
                                state <= IDLE;
                            end
""" + source[b:]


def whitening(source):
    source = delete_rows(source, (50, 51, 67, 68, 72, 73, 74))
    source = re.sub(r"(?m)^ *PC_RATE_WHITEN:.*\n", "", source)
    source = source.replace("PC_RATE_WHITEN = 7'd62", "PC_RATE_WHITEN = PC_HEADING_WHITEN")
    source = source.replace(
        "uop.src_b = SRC_R33; uop.dest = REG_T3; end",
        "uop.src_b = update_stage == UPDATE_XYZW ? SRC_R44 : SRC_R33; "
        "uop.dest = update_stage == UPDATE_XYZW ? REG_T4 : REG_T3; end", 1
    )
    source = source.replace("uop.src_b = REG_MEAS_Y; uop.dest = REG_EPS; end",
                            "uop.src_b = REG_MEAS_Y; uop.dest = REG_EPS; "
                            "uop.wb_en = update_stage == UPDATE_XY; end", 1)
    source = source.replace("uop.src_b = REG_MEAS_THETA; uop.dest = REG_EPS; end",
                            "uop.src_b = REG_MEAS_THETA; uop.dest = REG_EPS; "
                            "uop.wb_en = update_stage == UPDATE_XYZ; end", 1)
    return source.replace("                    7'd58, 7'd85: pc <= 7'd34;", """                    PC_HEADING_WHITEN: if (update_stage == UPDATE_XYZW) pc <= 7'd63;
                    7'd66: pc <= 7'd46;
                    7'd47: if (update_stage == UPDATE_XYZW) pc <= 7'd69;
                    7'd49, 7'd71: pc <= 7'd29;
                    7'd30: if (update_stage != UPDATE_XY) pc <= 7'd52;
                    7'd52: if (update_stage == UPDATE_XYZW) pc <= 7'd75;
                    7'd58, 7'd85: pc <= 7'd34;""")


def writeback(source):
    return source.replace(
        "                if (arithmetic_complete && uop.wb_en && uop.dest <= RF_LAST)\n"
        "                    regfile[uop.dest] <= arithmetic_result;",
        """                if (arithmetic_complete && uop.wb_en)
                    for (int i = 0; i <= RF_LAST; i++)
                        if (uop.dest == 7'(i))
                            regfile[i] <= arithmetic_result;"""
    )


def bank_writeback(source):
    a = source.index("            if (SHARED_STORAGE && state == IDLE && prediction_load)")
    b = source.index("            if (!SHARED_STORAGE", a)
    source = source[:a] + """            if (SHARED_STORAGE) begin
                for (int i = 0; i <= RF_LAST; i++) begin
                    if (i != 5 && (load_main || (retire_main && uop.dest == 7'(i)))) begin
                        if (i < 5)
                            regfile[i] <= ({32{load_main}} & prediction[160-32*i -: 32]) |
                                          main_write_data;
                        else
                            regfile[i] <= ({32{load_main}} & prediction_words[32*(i-6) +: 32]) |
                                          main_write_data;
                    end
                end
            end
""" + source[b:]
    source = source.replace("if (arithmetic_complete && uop.wb_en && uop.dest <= RF_LAST)",
                            "if (!SHARED_STORAGE && arithmetic_complete && uop.wb_en && uop.dest <= RF_LAST)")
    a = source.index("    always_ff @(posedge clock")
    return source[:a] + """    wire load_main = state == IDLE && prediction_load;
    wire retire_main = arithmetic_complete && !arithmetic_failed && uop.wb_en;
    wire [31:0] main_write_data = arithmetic_result & {32{!load_main}};

""" + source[a:]


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--baseline-rtl", type=Path, required=True,
                        help="path to corr_microcode.sv before the bank writeback optimization")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    baseline = args.baseline_rtl.read_text()
    for name, change in (("baseline", lambda text: text),
                         ("stage_loop", stage_loop), ("whitening", whitening), ("writeback", writeback),
                         ("bank_writeback", bank_writeback),
                         ("loop_whitening", lambda text: whitening(stage_loop(text))),
                         ("combined", lambda text: bank_writeback(whitening(stage_loop(text))))):
        target = args.output / name
        for folder in ("Dev", "tests"):
            copytree(root / folder, target / folder)
        rtl = target / "Dev/corr_microcode.sv"
        rtl.write_text(change(baseline))
        # Check the same events even where the experimental ROM moves PCs.
        if name in ("stage_loop", "loop_whitening", "combined"):
            bench = target / "tests/corr_x_tb.sv"
            bench.write_text(bench.read_text().replace(
                "dut.correction_program.pc != 88",
                "dut.correction_program.pc != dut.correction_program.PC_FULL_DONE"))
        if name in ("whitening", "loop_whitening", "combined"):
            test = target / "tests/run_corr_full.py"
            test.write_text(test.read_text().replace("else [0, 62]", "else [0, 44]"))


if __name__ == "__main__":
    main()
