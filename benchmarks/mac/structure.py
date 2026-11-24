"""Record Icarus synthesis-elaboration structure without treating it as cell area."""
from collections import Counter
import json
import re
import subprocess
import tempfile
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from synthesize import RTL, ROOT, settings
from benchmark import RESULTS


def main():
    result = {}
    with tempfile.TemporaryDirectory() as temp:
        for name, top, topology, fast in settings():
            sources = [ROOT / f"{top}.sv"] if topology is None else RTL
            params = [] if topology is None else [f"-P{top}.TOPOLOGY={topology}",
                                                  f"-P{top}.FAST_SHIFT={fast}"]
            dump = Path(temp) / "netlist"
            subprocess.run(["iverilog", "-g2012", "-Wall", "-S", "-tnull", "-s", top,
                            "-N", dump, *params, *sources], cwd=ROOT, check=True)
            data = dump.read_text()
            symbols = {f"{scope}.{signal}": int(width) for signal, scope, width in
                       re.findall(r"^    (?:wire|reg): (\w+) .*? scope=(\S+) .*?vector_width=(\d+)", data, re.M)}
            # Pin 7 is the Q output of an LPM_FF. Restrict matches to FF records.
            groups = [re.search(r"^    7 pin7 O .*? [0-9a-fx]+ (\S+)$", block, re.M)[1]
                      for block in re.split(r"^LPM_FF:", data, flags=re.M)[1:]]
            nodes = Counter(re.findall(r"^(logic|reduction logic|Adder \(NetAddSub\)|Multiplexer \(NetMux\)|"
                                       r"LPM_MULT\b|LPM_FF\b|LPM_LATCH\b)", data, re.M))
            result[name] = {"register_groups": len(groups),
                            "unpruned_register_bits": sum(symbols[group] for group in groups),
                            "nodes": dict(nodes)}
            print(name, result[name])
    (RESULTS / "iverilog_structure.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
