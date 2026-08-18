# ROS 2 action benchmark

This benchmark measures the duration of the same sequential ROS 2 action
workload:

- without ROSMonitoring (`time_no_mon`);
- with ROSMonitoring and an always-true oracle (`time_mon`).

The workload varies the number of actions (`n`), goals per action (`goals`),
Fibonacci order (`order`) and independent repetitions. Only the result CSV is
stored. Monitor events are sent to `/dev/null`.

The ROSMonitoring generator is executed from a temporary copy, so running the
benchmark does not modify the generator source tree.

## Run

From this directory, a short validation is:

```bash
./ros2_ws/src/action_benchmark/scripts/run_benchmark_grid.py \
  --n-max 2 --goals-max 2 --order-max 3 \
  --repetitions 2 --overwrite \
  --output results/validation.csv
```

The complete default grid is `n=1..20`, `goals=1..20`, `order=2..20`:

```bash
./ros2_ws/src/action_benchmark/scripts/run_benchmark_grid.py \
  --repetitions 5 \
  --output results/benchmark_results.csv
```

Existing rows are skipped, so the same command resumes an interrupted run.
Use `--overwrite` to start the selected CSV again from zero.

The CSV contains:

```text
n,goals,order,repetition,time_no_mon,time_mon
```

Each row comes from fresh baseline and monitored launches.

## Structure

```text
action_benchmark/
├── results/
└── ros2_ws/
    └── src/
        ├── action_benchmark/
        │   ├── action_benchmark/
        │   ├── config/
        │   ├── launch/
        │   └── scripts/
        ├── monitor/
        └── rosmonitoring_interfaces/
```

## Files

- `client.py`: sends the action goals and prints `BENCH_RESULT`;
- `server.py`: computes Fibonacci and returns the action result;
- `run.launch`: starts one client and one server;
- `run_benchmark.sh`: runs one baseline/monitored pair;
- `run_benchmark_grid.py`: loops over the parameters and writes the CSV.

## Results

The benchmark results are saved in `results/`. The dataset used in the paper
is `benchmark_results_independent_rep5_clean.csv`: it contains five independent
repetitions of all 7,600 combinations of `n=1..20`, `goals=1..20`, and
`order=2..20`.
