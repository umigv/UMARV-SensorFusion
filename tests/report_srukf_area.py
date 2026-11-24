"""Attribute mapped SR-UKF standard cells without double-counting hierarchy."""

from argparse import ArgumentParser
from collections import defaultdict
from json import loads
from pathlib import Path

from report_corr_area import library, statistics

BANKS = {
    "UKF": {"retained S": ("latest_s",)},
    "UKF_corr_engine": {"correction staging S": ("prediction_s",)},
    "corr_microcode": {"correction working S": tuple(f"regfile[{i}]" for i in range(6, 21))},
    "UKF_pred_sequencer": {
        "prediction S": tuple(f"registers[{i}]" for i in range(6, 21)),
        "prediction working S_Q": tuple(f"working_sq[{i}]" for i in range(15)),
    },
}


def attributed_storage(module, mapped, modules, lib, registers, read_ports, *, split=False, read_bits=()):
    nets = module["netnames"]
    register_bits = {b for name in registers for b in nets[name]["bits"] if isinstance(b, int)}
    storage = {name for name, cell in mapped.items() if lib[cell["type"]][1] == "DFF"
               and register_bits.intersection(cell["connections"].get("Q", []))}
    driver, users = {}, defaultdict(set)
    inputs, outputs = defaultdict(set), defaultdict(set)
    for name, cell in module["cells"].items():
        cell_type = cell["type"]
        out_pins = lib[cell_type][2] if cell_type in lib else {
            pin for pin, port in modules[cell_type]["ports"].items()
            if port["direction"] == "output"}
        for pin, bits in cell["connections"].items():
            for bit in (bit for bit in bits if isinstance(bit, int)):
                if pin in out_pins:
                    outputs[name].add(bit)
                    driver[bit] = name
                else:
                    inputs[name].add(bit)
                    users[bit].add(name)
    for port in module["ports"].values():
        if port["direction"] == "output":
            for bit in port["bits"]:
                users[bit].add(None)

    logic = {name for name, cell in mapped.items() if lib[cell["type"]][1] == "logic"}
    write = set()
    pending = [driver.get(bit) for name in storage for bit in mapped[name]["connections"]["D"]]
    while pending:
        name = pending.pop()
        if name in logic and name not in write and all(
                users[bit] <= storage | write for bit in outputs[name]):
            write.add(name)
            pending.extend(driver.get(bit) for bit in inputs[name])

    ancestors, read = set(), set()
    pending = [driver.get(bit) for bit in read_bits]
    pending.extend(driver.get(bit) for port in read_ports for bit in nets[port]["bits"])
    while pending:
        name = pending.pop()
        if name in logic and name not in ancestors:
            ancestors.add(name)
            pending.extend(driver.get(bit) for bit in inputs[name])
    pending = [name for bit in register_bits for name in users[bit]]
    while pending:
        name = pending.pop()
        if name in ancestors and name not in read:
            read.add(name)
            pending.extend(user for bit in outputs[name] for user in users[bit])
    if split:
        return storage, write, read - write
    return storage | write | read


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("liberty", type=Path)
    parser.add_argument("netlist", type=Path)
    parser.add_argument("--top", default="UKF")
    args = parser.parse_args()
    lib = library(args.liberty)
    modules = loads(args.netlist.read_text())["modules"]
    groups = defaultdict(list)
    instances = defaultdict(int)
    sequential_area = 0.0

    def visit(module_name, hierarchy):
        nonlocal sequential_area
        module = modules[module_name]
        base = (module_name.split("\\")[1] if module_name.startswith("$paramod")
                else module_name)
        instances[base] += 1
        cells = {name: cell for name, cell in module["cells"].items()
                 if cell["type"] in lib}
        cells_total = statistics(cells.values(), lib)
        print(f"{hierarchy}: {cells_total[0]} cells, {cells_total[1]} DFF, {cells_total[3]:.2f} um²")
        sequential_area += sum(lib[cell["type"]][0] for cell in cells.values()
                               if lib[cell["type"]][1] in ("DFF", "latch"))
        banks = BANKS.get(base, {})
        for label, words in banks.items():
            bits = {bit for word in words if word in module["netnames"]
                    for bit in module["netnames"][word]["bits"]}
            flops = [cell for cell in cells.values() if lib[cell["type"]][1] == "DFF"
                     and bits.intersection(cell["connections"].get("Q", []))]
            print(f"  {label}: {len(flops)} DFF, {statistics(flops, lib)[3]:.2f} um²")

        registers = tuple(word for bank in banks.values() for word in bank
                          if word in module["netnames"])
        read_ports = ()
        if base == "corr_microcode":
            registers = tuple(name for name in module["netnames"] if name.startswith("regfile["))
            read_ports = ("operand_a", "operand_b")
        elif base == "UKF_pred_sequencer":
            registers = tuple(name for name in module["netnames"]
                              if name.startswith(("registers[", "working_sq[", "angle_words[", "spad[")))
            read_ports = ("a", "b")
        register_cells = attributed_storage(module, cells, modules, lib, registers, read_ports)
        if register_cells:
            attributed = statistics((cells[name] for name in register_cells), lib)
            print(f"  attributed register-file area: {attributed[3]:.2f} um²")
        group = {
            "fp32_mac": "Shared MAC", "fp32_divsqrt": "Shared DIVSQRT",
            "UKF_pred_sequencer": "Prediction control/microcode",
            "pred_microcode_prog": "Prediction control/microcode",
            "corr_microcode": "Correction control/microcode",
            "UKF_corr_engine": "Correction control/microcode",
            "vn_to_correction": "Conversion and interfaces",
            "lat_to_xy": "Conversion and interfaces",
            "deadwheel_velocity": "Conversion and interfaces",
            "int_to_fp32": "Conversion and interfaces",
            "UKF_pred_engine": "Conversion and interfaces",
        }.get(base, "Other logic")
        groups[group].extend(cell for name, cell in cells.items() if name not in register_cells)
        groups["Register files and muxes"].extend(cells[name] for name in register_cells)
        for name, cell in module["cells"].items():
            if cell["type"] not in lib:
                visit(cell["type"], f"{hierarchy}/{name}")

    visit(args.top, args.top)
    total = statistics((cell for group in groups.values() for cell in group), lib)
    for name, cells in groups.items():
        count, dff, latch, area = statistics(cells, lib)
        print(f"{name}: {area:.2f} um² ({100 * area / total[3]:.2f}%), {count} cells, {dff} DFF")
    print(f"TOTAL: {total[3]:.2f} um², {total[0]} cells, {total[1]} DFF, {total[2]} latches")
    print(f"Sequential area: {sequential_area:.2f} um²; combinational: {total[3]-sequential_area:.2f} um²")
    assert instances["fp32_mac"] == instances["fp32_divsqrt"] == 1
    assert instances["pred_angle_math"] == 0


if __name__ == "__main__":
    main()
