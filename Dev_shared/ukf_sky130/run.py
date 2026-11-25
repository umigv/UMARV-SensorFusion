"""Run from any directory with Python 3. Yosys and sta must share EDA_BIN."""
import os
from decimal import Decimal
from pathlib import Path
import re
from subprocess import run
from tempfile import TemporaryDirectory

script_dir = Path(__file__).resolve().parent
rtl = script_dir.parent
report = Path(os.environ.get("UKF_REPORT_DIR", script_dir)).resolve()
report.mkdir(parents=True, exist_ok=True)
period = Decimal(os.environ.get("UKF_CLOCK_PERIOD_NS", "20"))
if not period.is_finite() or period <= 0:
    raise ValueError("UKF_CLOCK_PERIOD_NS must be a positive finite number")
abc_delay = Decimal(os.environ.get("UKF_ABC_DELAY_NS", str(period)))
if not abc_delay.is_finite() or abc_delay <= 0:
    raise ValueError("UKF_ABC_DELAY_NS must be a positive finite number")
bin_dir = Path(os.environ.get("EDA_BIN", "/tmp/eda/env/bin"))
pdk = Path(os.environ.get("PDK_ROOT", "~/.volare")).expanduser() / "sky130A"
libs = pdk / "libs.ref/sky130_fd_sc_hd/lib"
openlane = pdk / "libs.tech/openlane/sky130_fd_sc_hd"
names = ("FP_MAC FP_SQRTDIV corr_microcode corr_engine int_to_fp32 "
         "deadwheel_velocity pred_engine pred_microcode pred_sequencer UKF").split()

defs = (rtl / "sys_defs.sv").read_text()
# Yosys 0.38 treats a word index on these typedefs as a single-bit select.
# Flatten the packed dimensions and preserve each 32-bit word's bit positions.
defs = defs.replace("[14:0][31:0]", "[479:0]").replace("[9:0][31:0]", "[319:0]")
# Qualify compilation-unit declarations for the installed Yosys SV frontend.
symbols = set(re.findall(r"^\s*}\s*(\w+)\s*;", defs, re.M))
symbols.update(re.findall(r"^localparam.*?\b(\w+)\s*=", defs, re.M))
symbols.update(re.findall(r"^function.*?\b(\w+)\s*\(", defs, re.M))
symbols.update(re.findall(r"\b((?:SEQ|UOP)_\w+)\b", defs, re.M))
symbols.update(re.findall(r"^typedef logic[^;]+\s(\w+)\s*;", defs, re.M))
units = ["package ukf_defs;\n" + defs + "\nendpackage\n"]
for name in names:
    source = (rtl / (name + ".sv")).read_text()
    if name == "int_to_fp32":
        source = source.replace("integer i;\n", "").replace("for(i = 0;", "for(int i = 0;")
    if name == "pred_sequencer":
        for factor in ("prior_s", "prediction_s", "correction_noise", "staged_correction_noise"):
            source = source.replace(f"{factor}[i]", f"{factor}[32*i +: 32]")
        source = re.sub(r"(: )return ([^;]+);", r"\1read_word = \2;", source)
        source = re.sub(r"^        (if .*?\)) return ([^;]+);",
                        lambda m: ("        " if m[2] == "prior_word" else "        else ")
                        + m[1] + " operand = " + m[2] + ";", source, flags=re.M)
        source = source.replace("        return read_word(addr);", "        else operand = read_word(addr);")
    local_symbols = set(re.findall(r"^\s*localparam.*?\b(\w+)\s*=", source, re.M))
    references = "|".join(sorted(symbols - local_symbols))
    reference = re.compile(r"//[^\n]*|/\*[\s\S]*?\*/|\b(?:" + references + r")\b")
    units.append(reference.sub(lambda m: m[0] if m[0].startswith("/") else "ukf_defs::" + m[0], source))

def liberty(corner):
    return libs / f"sky130_fd_sc_hd__{corner}.lib"

def capture(tool, args, filename):
    with (report / filename).open("w") as log:
        run([bin_dir / tool, *args], cwd=work, stdout=log, stderr=log, check=True)

with TemporaryDirectory(prefix="ukf-sky130-") as temp:
    work = Path(temp)
    (work / "sources.sv").write_text("\n".join(units))
    (work / "abc.constr").write_text("set_driving_cell sky130_fd_sc_hd__buf_4\nset_load 5.0\n")
    excluded = {line.split("#")[0].strip()
                for line in (openlane / "drc_exclude.cells").read_text().splitlines()}
    exclusions = " ".join("-dont_use " + cell for cell in sorted(excluded - {""}))
    corners = ("tt_025C_1v80", "ss_100C_1v60", "ff_n40C_1v95")
    map_corner = os.environ.get("UKF_MAPPING_CORNER", corners[0])
    if map_corner not in corners:
        raise ValueError(f"UKF_MAPPING_CORNER must be one of {corners}")
    mapping_lib = liberty(map_corner)
    scale = os.environ.get("UKF_WHEEL_SCALE_HEX", "3e000000")
    if not re.fullmatch(r"[a-fA-F0-9]{8}", scale):
        raise ValueError("UKF_WHEEL_SCALE_HEX must contain eight hex digits")
    script = "\n".join([
        "read_verilog -sv -noautowire sources.sv",
        f"chparam -set WHEEL_VELOCITY_SCALE 32'h{scale} UKF",
        "hierarchy -check -top UKF",
        "synth -noabc -flatten -top UKF",
        "delete t:$scopeinfo",
        "check -assert",
        f'dfflibmap -liberty "{mapping_lib}" {exclusions}',
        f'abc -liberty "{mapping_lib}" {exclusions} -constr abc.constr -D {abc_delay * 1000:f}',
        f'stat -liberty "{mapping_lib}"',
        "write_verilog -noattr -noexpr -simple-lhs mapped.v",
        f'read_liberty -lib -ignore_miss_func "{mapping_lib}"',
        "hierarchy -check -top UKF",
        "check -mapped -assert",
    ])
    (work / "synth.ys").write_text(script + "\n")
    capture("yosys", ["-s", "synth.ys"], "yosys.log")
    for corner in corners:
        (work / "sta.tcl").write_text(f"""
read_liberty {{{liberty(corner)}}}
read_verilog mapped.v
link_design UKF
create_clock -name core -period {period:f} [get_ports clk]
set data_inputs [get_ports {{ukf_pulse ukf_beat_ts* ukf_vn_valid ukf_vn_meas* ukf_dw_valid ukf_dw_delta* measurement_s* rst_n}}]
set_input_delay -clock core 0 $data_inputs
set_output_delay -clock core 0 [all_outputs]
set_false_path -from [get_ports rst_n]
check_setup -verbose
puts "Setup paths"
report_checks -path_delay max -group_count 1 -fields {{slew cap fanout net}} -digits 3
report_worst_slack -max -digits 3
report_tns -digits 3
puts "Hold paths"
report_checks -path_delay min -group_count 1 -digits 3
report_worst_slack -min -digits 3
exit
""")
        capture("sta", ["-no_init", "-exit", "sta.tcl"], corner + ".log")
