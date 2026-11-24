"""Materialize SKY130 tie cells with a bounded number of sinks per output."""

from argparse import ArgumentParser
from collections import defaultdict
from json import dumps, loads
from pathlib import Path

from report_corr_area import library


def insert(module, modules, cells, limit):
    sinks = defaultdict(list)
    for cell in module["cells"].values():
        kind = cell["type"]
        outputs = cells[kind][2] if kind in cells else {
            name for name, port in modules[kind]["ports"].items() if port["direction"] == "output"}
        for name, bits in cell["connections"].items():
            if name not in outputs:
                for index, bit in enumerate(bits):
                    if bit in ("0", "1"):
                        sinks[bit].append((bits, index))
    for name, port in module["ports"].items():
        if port["direction"] == "output":
            for index, bit in enumerate(port["bits"]):
                if bit in ("0", "1"):
                    sinks[bit].append(((port["bits"], module["netnames"][name]["bits"]), index))
    nets = module["netnames"]
    highest = max((bit for net in nets.values() for bit in net["bits"] if isinstance(bit, int)),
                  default=0)
    count = defaultdict(int)
    for value, loads in sinks.items():
        for offset in range(0, len(loads), limit):
            highest += 1
            name = f"local_tie_{highest}"
            nets[name + "_net"] = {"hide_name": 0, "bits": [highest], "attributes": {}}
            pin = "HI" if value == "1" else "LO"
            module["cells"][name] = {
                "hide_name": 0, "type": "sky130_fd_sc_hd__conb_1",
                "parameters": {}, "attributes": {},
                "port_directions": {pin: "output"}, "connections": {pin: [highest]},
            }
            for target, index in loads[offset:offset + limit]:
                if isinstance(target, tuple):
                    for bits in target:
                        bits[index] = highest
                else:
                    assert target[index] == value
                    target[index] = highest
            count[value] += 1
    return count


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("liberty", type=Path)
    parser.add_argument("mapped", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-sinks", type=int, required=True)
    args = parser.parse_args()
    if args.max_sinks < 1:
        parser.error("--max-sinks must be positive")
    cells = library(args.liberty)
    data = loads(args.mapped.read_text())
    totals = defaultdict(int)
    for module in data["modules"].values():
        for value, count in insert(module, data["modules"], cells, args.max_sinks).items():
            totals[value] += count
    args.output.write_text(dumps(data))
    print(f"LO {totals['0']}; HI {totals['1']}; max sinks {args.max_sinks}")


if __name__ == "__main__":
    main()
