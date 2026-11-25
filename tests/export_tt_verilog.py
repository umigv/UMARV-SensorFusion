"""Lower the active SystemVerilog core and Tiny Tapeout wrapper to Verilog."""
from pathlib import Path
from subprocess import run
from tempfile import TemporaryDirectory

from synth_srukf import yosys_source


def main():
    root = Path(__file__).resolve().parents[1]
    top = "tt_um_umarv_srukf"
    wrapper = root / "Dev" / f"{top}.v"
    with TemporaryDirectory() as directory:
        unit = Path(directory) / "design.sv"
        output = Path(directory) / "design.v"
        unit.write_text(yosys_source(root) + "\n" + wrapper.read_text())
        commands = (
            f'read_verilog -sv "{unit.name}"',
            f"hierarchy -check -top {top}",
            "proc",
            "opt_clean",
            "check -assert",
            f'write_verilog -noattr "{output.name}"',
        )
        result = run(["yosys", "-Q", "-T", "-p", "; ".join(commands)],
                     cwd=directory, capture_output=True, text=True)
        if result.returncode:
            raise SystemExit(result.stdout + result.stderr)
        # Remove the temporary source filename from generated identifiers.
        emitted = output.read_text().replace(str(unit), "design.sv")
        (root / "src" / f"{top}.v").write_text(emitted)


if __name__ == "__main__":
    main()
