"""Insert transparent SKY130 buffer trees on high-fanout data/control nets."""

from argparse import ArgumentParser
from collections import defaultdict
from json import dumps, loads
from pathlib import Path

from report_corr_area import library


def buffer_module(module, modules, lib, limit, buffer):
    nets = module["netnames"]
    cells = module["cells"]
    loads = defaultdict(list)
    highest = max(bit for net in nets.values() for bit in net["bits"] if isinstance(bit, int))
    protected = {bit for name, port in module["ports"].items()
                 if name in ("clk", "clock", "rst_n", "reset", "rst")
                 for bit in port["bits"]}
    for name, cell in cells.items():
        kind = cell["type"]
        output_pins = lib[kind][2] if kind in lib else {
            pin for pin, port in modules[kind]["ports"].items() if port["direction"] == "output"}
        for pin, bits in cell["connections"].items():
            if pin not in output_pins:
                for index, bit in enumerate(bits):
                    if isinstance(bit, int) and bit not in protected:
                        loads[bit].append((name, pin, index))

    inserted = 0
    inserted_outputs = {}
    for root, sinks in sorted(loads.items()):
        while len(sinks) > limit:
            branches = []
            for offset in range(0, len(sinks), limit):
                group = sinks[offset:offset + limit]
                highest += 1
                name = f"fanout_buf_{highest}"
                nets[name + "_net"] = {"hide_name": 0, "bits": [highest], "attributes": {}}
                cells[name] = {
                    "hide_name": 0, "type": buffer, "parameters": {}, "attributes": {},
                    "port_directions": {"A": "input", "X": "output"},
                    "connections": {"A": [root], "X": [highest]},
                }
                inserted_outputs[highest] = name
                for sink, pin, index in group:
                    cells[sink]["connections"][pin][index] = highest
                branches.append((name, "A", 0))
                inserted += 1
            sinks = branches
    # Confirm that every prior sink still connects to the same root after
    # walking back through only the newly inserted non-inverting buffers.
    for root, sinks in loads.items():
        for sink, pin, index in sinks:
            bit = cells[sink]["connections"][pin][index]
            while bit in inserted_outputs:
                bit = cells[inserted_outputs[bit]]["connections"]["A"][0]
            assert bit == root, (sink, pin, index, root, bit)
    return inserted


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("liberty", type=Path)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fanout", type=int, default=16)
    args = parser.parse_args()
    if args.fanout < 2:
        parser.error("fanout must be at least two")
    lib = library(args.liberty)
    data = loads(args.input.read_text())
    modules = data["modules"]
    cell = "sky130_fd_sc_hd__buf_4"
    assert lib[cell][2] == {"X"}
    count = sum(buffer_module(module, modules, lib, args.fanout, cell)
                for module in modules.values())
    args.output.write_text(dumps(data))
    print(f"Inserted {count} transparent buffers ({count * lib[cell][0]:.2f} um²)")


if __name__ == "__main__":
    main()
