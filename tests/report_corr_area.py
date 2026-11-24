"""Compare hierarchical cell area and attribute register-file cells."""

from collections import defaultdict
from json import loads
from pathlib import Path
from re import MULTILINE, search, split
from sys import argv


def library(path):
    chunks = split(r'^    cell \("([^"]+)"\) \{', path.read_text(), flags=MULTILINE)
    cells = {}
    for name, body in zip(chunks[1::2], chunks[2::2], strict=True):
        area = float(search(r"\barea\s*:\s*([\d.]+);", body)[1])
        kind = "DFF" if search(r"^        ff \(", body, flags=MULTILINE) else (
            "latch" if search(r"^        latch \(", body, flags=MULTILINE) else "logic")
        pins = split(r'^        pin \("([^"]+)"\) \{', body, flags=MULTILINE)
        outputs = {pin for pin, spec in zip(pins[1::2], pins[2::2], strict=True)
                   if search(r'direction\s*:\s*"output"', spec)}
        cells[name] = (area, kind, outputs)
    return cells


def statistics(cells, lib):
    selected = [lib[cell["type"]] for cell in cells if cell["type"] in lib]
    return (len(selected), sum(k == "DFF" for _, k, _ in selected),
            sum(k == "latch" for _, k, _ in selected), sum(a for a, _, _ in selected))


def attribute_register_file(core, modules, lib):
    cells = core["cells"]
    word_bits = {int(name[8:-1]): set(net["bits"]) for name, net in core["netnames"].items()
                 if name.startswith("regfile[")}
    register_bits = set.union(*word_bits.values()) - {"0", "1"}
    scratch_bits = set.union(*(word_bits[i] for i in range(25, 37)))
    storage = {n for n, c in cells.items()
               if c["type"] in lib and lib[c["type"]][1] == "DFF" and
               set(c["connections"].get("Q", [])) & register_bits}
    scratch = {n for n in storage if set(cells[n]["connections"]["Q"]) & scratch_bits}

    drivers, users = {}, defaultdict(set)
    inputs, outputs = defaultdict(set), defaultdict(set)
    logic = set()
    for name, cell in cells.items():
        cell_type = cell["type"]
        if cell_type in lib:
            _, kind, out_pins = lib[cell_type]
            if kind == "logic":
                logic.add(name)
        else:
            out_pins = {p for p, port in modules[cell_type]["ports"].items()
                        if port["direction"] == "output"}
        for pin, bits in cell["connections"].items():
            for bit in (b for b in bits if isinstance(b, int)):
                if pin in out_pins:
                    drivers[bit] = name
                    outputs[name].add(bit)
                else:
                    users[bit].add(name)
                    inputs[name].add(bit)
    for port in core["ports"].values():
        if port["direction"] == "output":
            for bit in port["bits"]:
                users[bit].add(None)

    # Assign the exclusive D-input logic to the file. Shared writeback/control
    # logic stays with the controller.
    write = set()
    pending = [drivers.get(bit) for name in storage for bit in cells[name]["connections"]["D"]]
    while pending:
        name = pending.pop()
        if name in logic and name not in write and all(
                users[b] <= storage | write for b in outputs[name]):
            write.add(name)
            pending.extend(drivers.get(b) for b in inputs[name])

    # Assign combinational cells on a path from a regfile Q to a read operand.
    ancestors = set()
    pending = [drivers.get(bit) for port in ("operand_a", "operand_b")
               for bit in core["netnames"][port]["bits"]]
    while pending:
        name = pending.pop()
        if name in logic and name not in ancestors:
            ancestors.add(name)
            pending.extend(drivers.get(b) for b in inputs[name])
    read = set()
    pending = [n for bit in register_bits for n in users[bit]]
    while pending:
        name = pending.pop()
        if name in ancestors and name not in read:
            read.add(name)
            pending.extend(user for b in outputs[name] for user in users[b])

    file_cells = storage | write | read
    return {
        "Register file (storage and attributable muxes)": file_cells,
        "  Register file storage only": storage,
        "  SPAD storage only": scratch,
        "Controller and shared logic": cells.keys() - file_cells,
    }


def breakdown(path, lib):
    modules = loads(path.read_text())["modules"]
    core_name = modules["UKF_corr_engine"]["cells"]["correction_program"]["type"]
    core = modules[core_name]
    local = {label: statistics(modules[name]["cells"].values(), lib) for label, name in (
        ("Engine wrapper", "UKF_corr_engine"), ("FP32 MAC", "fp32_mac"),
        ("FP32 divide/sqrt", "fp32_divsqrt"), ("Microcode local subtotal", core_name))}
    hierarchy = tuple(map(sum, zip(*local.values(), strict=True)))
    for label, names in attribute_register_file(core, modules, lib).items():
        local[label] = statistics((core["cells"][n] for n in sorted(names)), lib)
    return {**local, "Full correction engine": hierarchy}


def main():
    if len(argv) != 4:
        raise SystemExit("Usage: python3 tests/report_corr_area.py <sky130.lib> <baseline.json> <extended.json>")
    lib = library(Path(argv[1]))
    baseline, extended = (breakdown(Path(path), lib) for path in argv[2:])
    for label, before in baseline.items():
        after = extended[label]
        print(f"{label:46s} {before[0]:5d} -> {after[0]:5d} cells  "
              f"{before[1]:4d} -> {after[1]:4d} DFF  {before[2]} -> {after[2]} latch  "
              f"{before[3]:10,.2f} -> {after[3]:10,.2f} um^2")


if __name__ == "__main__":
    main()
