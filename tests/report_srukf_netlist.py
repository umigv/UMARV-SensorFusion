"""Audit SKY130 ties, constant sinks, buffer trees, decode and mux cones."""

from argparse import ArgumentParser
from collections import Counter
from json import loads
from pathlib import Path
import re

from report_corr_area import library
from report_srukf_area import attributed_storage


def base_name(name):
    return name.split("\\")[1] if name.startswith("$paramod") else name


def outputs(cell, modules, lib):
    kind = cell["type"]
    if kind in lib:
        return lib[kind][2]
    return {pin for pin, p in modules[kind]["ports"].items() if p["direction"] == "output"}


def cone(mapped, lib, roots):
    drivers = {bit: cell for cell in mapped.values() if lib[cell["type"]][1] == "logic"
               for pin in lib[cell["type"]][2] for bit in cell["connections"].get(pin, [])
               if isinstance(bit, int)}
    pending = list(roots)
    seen = set()
    while pending:
        bit = pending.pop()
        if bit in seen or bit not in drivers:
            continue
        seen.add(bit)
        cell = drivers[bit]
        pending.extend(bit for pin, bits in cell["connections"].items()
                       if pin not in lib[cell["type"]][2] for bit in bits)
    # Deduplicate multi-output cells.
    result = {id(drivers[bit]): drivers[bit] for bit in seen}
    return len(result), sum(lib[cell["type"]][0] for cell in result.values())


def decode_support(module, mapped, lib):
    """Count gates whose nonconstant inputs come only from PC/loop fields."""
    names = ("pc", "row", "col", "update_stage")
    available = {"0", "1"} | {b for name in names for b in module["netnames"][name]["bits"]}
    pending = [c for c in mapped.values() if lib[c["type"]][1] == "logic"]
    count = 0
    area = 0.0
    while pending:
        rest = []
        for cell in pending:
            output = lib[cell["type"]][2]
            sources = {b for pin, bits in cell["connections"].items()
                       if pin not in output for b in bits}
            if sources and sources <= available and sources - {"0", "1"}:
                count += 1
                area += lib[cell["type"]][0]
                available.update(b for pin, bits in cell["connections"].items()
                                 if pin in output for b in bits)
            else:
                rest.append(cell)
        if len(rest) == len(pending):
            break
        pending = rest
    return count, area


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("liberty", type=Path)
    parser.add_argument("netlist", type=Path)
    args = parser.parse_args()
    lib = library(args.liberty)
    modules = loads(args.netlist.read_text())["modules"]
    stats, ties, buffers, const = Counter(), Counter(), Counter(), Counter()
    highest = []

    def visit(name, path):
        m = modules[name]
        base = base_name(name)
        mapped = {n: c for n, c in m["cells"].items() if c["type"] in lib}
        local_const, fanout = Counter(), Counter()
        for n, cell in m["cells"].items():
            kind = cell["type"]
            pins = outputs(cell, modules, lib)
            if kind in lib:
                area, classification, _ = lib[kind]
                stats["cells"] += 1
                stats["area"] += area
                stats[classification] += 1
                if "__conb_" in kind:
                    ties["HI"] += "HI" in cell["connections"]
                    ties["LO"] += "LO" in cell["connections"]
                    ties["area"] += area
                if "__lpflow_isobuf" in kind:
                    buffers["isolation_count"] += 1
                    buffers["isolation_area"] += area
                if re.search(r"__(?:lpflow_)?(?:clk)?buf(?:kapwr)?_", kind):
                    buffers["count"] += 1
                    buffers["area"] += area
                    if n.startswith("fanout_buf_"):
                        buffers["inserted"] += 1
                        buffers["inserted_area"] += area
                    buffers["literal_driven"] += bool(
                        set(cell["connections"].get("A", [])) & {"0", "1"})
            for pin, bits in cell["connections"].items():
                if pin not in pins:
                    local_const.update((bit, "leaf" if kind in lib else "boundary")
                                       for bit in bits if bit in ("0", "1"))
                    fanout.update(bit for bit in bits if isinstance(bit, int))
            if kind not in lib:
                visit(kind, f"{path}/{n}")
        const.update(local_const)
        const.update((bit, "output") for port in m["ports"].values()
                     if port["direction"] == "output" for bit in port["bits"] if bit in ("0", "1"))
        aliases = {}
        for net_name, data in m["netnames"].items():
            for i, bit in enumerate(data["bits"]):
                if not isinstance(bit, int):
                    continue
                label = net_name if len(data["bits"]) == 1 else f"{net_name}[{i}]"
                if bit not in aliases or (aliases[bit].startswith("$") and not label.startswith("$")):
                    aliases[bit] = label
        highest.extend((count, path, aliases.get(bit, str(bit)))
                       for bit, count in fanout.most_common(10))
        if local_const:
            print(path, "constant sink appearances", dict(local_const))
        if base == "pred_microcode_prog":
            print(path, "standalone decode", sum(lib[c["type"]][0] for c in mapped.values()))
        if base in ("UKF_pred_sequencer", "corr_microcode"):
            if base == "UKF_pred_sequencer":
                regs = [n for n in m["netnames"] if n.startswith(("spad[", "working_sq["))]
                # Data bits only of the exported packed requests.
                read_bits = [bit for port in ("mac_request", "div_request")
                             for bit in m["ports"][port]["bits"][:64]]
            else:
                regs = [n for n in m["netnames"] if n.startswith("regfile[")]
                read_bits = [bit for child in ("correction_divsqrt", "residual_mac")
                             for pin in ("a", "b")
                             for bit in m["cells"][child]["connections"][pin]]
                print(path, "PC/loop-field-only decode support (cells, um²)", decode_support(m, mapped, lib))
            storage, write, read = attributed_storage(m, mapped, modules, lib, regs, [],
                                                      split=True, read_bits=read_bits)
            print(path, "attributed storage / write / read area",
                  [round(sum(lib[mapped[n]["type"]][0] for n in group), 2)
                   for group in (storage, write, read)])
            print(path, "operand cones, including address decode (cells, um²)",
                  cone(mapped, lib, read_bits))

    visit("UKF", "UKF")
    print("Totals", dict(stats))
    print("Ties", dict(ties), "buffers", dict(buffers), "constant sink appearances", dict(const))
    print("Largest local fanouts (one hierarchy level)")
    for row in sorted(highest, reverse=True)[:18]:
        print(*row)


if __name__ == "__main__":
    main()
