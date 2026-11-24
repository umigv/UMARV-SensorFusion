"""Apply saved MAC latencies to the documented serialized SR-UKF schedules."""
import gzip
import json
import sys
from statistics import mean

sys.dont_write_bytecode = True
from benchmark import RESULTS, fma, magnitude, normalized, operations

# Prediction uses the corrected, dense rank-one kernel from the Cholesky report.
# Correction follows the archived H=[I4 0] four-measurement schedule.
SCHEDULES = {
    "prediction": {"clear": 605, "continue": 550, "div": 220, "root": 55, "overhead": 6340},
    "correction": {"clear": 179, "continue": 274, "div": 140, "root": 30, "overhead": 3505},
}


def main():
    workloads = list(operations(timing=True))
    classes = []
    c = 0
    for clear, _, a, b in workloads:
        c = 0 if clear else c
        classes.append(1 if normalized(a) and normalized(b) and magnitude(c) == 0 else
                       2 if normalized(a) and normalized(b) and normalized(c) else 0)
        c, _ = fma(a, b, c)
    summary = json.loads((RESULTS / "summary.json").read_text())
    estimates = {}
    for name, row in summary.items():
        with gzip.open(RESULTS / f"{name}.cycles.gz", "rt") as raw:
            measured = [int(line) for line in raw]
        matched = measured[:len(classes)-1024] + measured[-1024:]
        assert len(matched) == len(classes)
        first = mean(x for x, group in zip(matched, classes) if group == 1)
        continued = mean(x for x, group in zip(matched, classes) if group == 2)
        first_bound = 128 if name == "baseline" else row["bound"] - (3 if row["fast_shift"] else 49)
        record = estimates[name] = {"clear_mean": first, "continue_mean": continued,
                                    "clear_bound": first_bound}
        for phase, count in SCHEDULES.items():
            model = (count["clear"]*first + count["continue"]*continued +
                     count["div"]*33.771 + count["root"]*31 + count["overhead"])
            bound = (count["clear"]*first_bound + count["continue"]*row["bound"] +
                     count["div"]*81 + count["root"]*54 + count["overhead"])
            record[phase] = {"proxy_cycles": model, "bounded_subtotal": bound,
                             "proxy_room": 62500-model, "bounded_room": 62500-bound}
        print(name, f"{first:.3f}", f"{continued:.3f}",
              *(f"{record[p]['proxy_cycles']:.0f}/{record[p]['bounded_subtotal']}"
                for p in SCHEDULES))
    (RESULTS / "schedule.json").write_text(json.dumps(estimates, indent=2) + "\n")


if __name__ == "__main__":
    main()
