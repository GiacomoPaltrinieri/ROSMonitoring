#!/usr/bin/env python3
"""Run the baseline/monitored action benchmark and append results to CSV."""

import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
WORKSPACE = PACKAGE_DIR.parent.parent
RESULTS = WORKSPACE.parent / "results"
RUNNER = SCRIPT_DIR / "run_benchmark.sh"
HEADER = ["n", "goals", "order", "repetition", "time_no_mon", "time_mon"]


def positive(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-min", type=positive, default=1)
    parser.add_argument("--n-max", type=positive, default=20)
    parser.add_argument("--n-step", type=positive, default=1)
    parser.add_argument("--goals-min", type=positive, default=1)
    parser.add_argument("--goals-max", type=positive, default=20)
    parser.add_argument("--goals-step", type=positive, default=1)
    parser.add_argument("--order-min", type=positive, default=2)
    parser.add_argument("--order-max", type=positive, default=20)
    parser.add_argument("--order-step", type=positive, default=1)
    parser.add_argument("--repetitions", type=positive, default=5)
    parser.add_argument(
        "--output", type=Path, default=RESULTS / "benchmark_results.csv"
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def values(minimum, maximum, step):
    if minimum > maximum:
        raise ValueError("minimum cannot be greater than maximum")
    return range(minimum, maximum + 1, step)


def write_config(n):
    config = PACKAGE_DIR / "config" / f"rosmonitoring_n{n}.yaml"
    lines = [
        f"path: {WORKSPACE / 'src'}/",
        "nodes:",
        "  - node:",
        "      name: bench_client",
        "      package: action_benchmark",
        f"      path: {PACKAGE_DIR / 'launch' / 'run.launch'}",
        "  - node:",
        "      name: bench_server",
        "      package: action_benchmark",
        f"      path: {PACKAGE_DIR / 'launch' / 'run.launch'}",
        "monitors:",
        "  - monitor:",
        "      id: action_benchmark_monitor",
        "      log: /dev/null",
        "      silent: True",
        "      oracle:",
        "        port: 8080",
        "        url: 127.0.0.1",
        "        action: nothing",
        "      actions:",
    ]
    for index in range(n):
        lines += [
            f"        - name: /bench_action_{index}",
            "          type: example_interfaces.action.Fibonacci",
            "          action: filter",
            "          clients: [bench_client]",
            "          servers: [bench_server]",
        ]
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text("\n".join(lines) + "\n", encoding="utf-8")


def previous_results(path, overwrite):
    if overwrite or not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerow(HEADER)
        return set()

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != HEADER:
            raise ValueError(f"invalid CSV header in {path}")
        return {
            (int(row["n"]), int(row["goals"]), int(row["order"]), int(row["repetition"]))
            for row in reader
        }


def measure(n, goals, order, prepare):
    environment = os.environ.copy()
    if not prepare:
        environment["ACTION_BENCHMARK_SKIP_PREPARE"] = "1"
    result = subprocess.run(
        [str(RUNNER), str(n), str(goals), str(order)],
        text=True,
        capture_output=True,
        env=environment,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "benchmark failed")
    try:
        baseline, monitored = result.stdout.strip().splitlines()[-1].split(",")
        return float(baseline), float(monitored)
    except (IndexError, ValueError) as error:
        raise RuntimeError(f"invalid benchmark output: {result.stdout!r}") from error


def main():
    args = arguments()
    n_values = values(args.n_min, args.n_max, args.n_step)
    goal_values = values(args.goals_min, args.goals_max, args.goals_step)
    order_values = values(args.order_min, args.order_max, args.order_step)
    output = args.output.expanduser().resolve()
    completed = previous_results(output, args.overwrite)
    total = len(n_values) * len(goal_values) * len(order_values) * args.repetitions
    current = len(completed)

    with output.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        for n in n_values:
            write_config(n)
            prepare = True
            for goals in goal_values:
                for order in order_values:
                    for repetition in range(1, args.repetitions + 1):
                        key = (n, goals, order, repetition)
                        if key in completed:
                            continue
                        baseline, monitored = measure(n, goals, order, prepare)
                        prepare = False
                        writer.writerow([*key, baseline, monitored])
                        stream.flush()
                        current += 1
                        print(
                            f"[{current}/{total}] n={n} goals={goals} "
                            f"order={order} rep={repetition}: "
                            f"{baseline:.6f}s -> {monitored:.6f}s",
                            flush=True,
                        )
    print(f"Results: {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
