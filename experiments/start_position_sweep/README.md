# Start-position sweep

Runs a placement node (`object_placement` or the `random_placement` baseline) from every start on an x/y/z grid
(abs, base frame) and reports how often it reaches every viewpoint.

## Run

```bash
cd ~/robot-control-and-planning   # after colcon build

# Your method, default grid, 6 parallel workers, seed 42
experiments/start_position_sweep/sweep.sh gradient_v1

# Random-sampling baseline with a 90 s budget per start
METHOD=random_placement TIME_BUDGET=90 experiments/start_position_sweep/sweep.sh random_90s

# Custom grid / params
X="0.3 0.5" Y="0.0" Z="0.1 0.2" BASEYAML=path/to.yaml experiments/start_position_sweep/sweep.sh small_test 2
```

Each run goes to `runs/NAME/`: `starts.txt`, a snapshot of the params (`params.yaml`), `run_info.txt`
(method, seed, workers, git commit), `results.csv` (one row per start) and `summary.txt`.
A run that was stopped can be resumed by running the same command again: finished starts are skipped.

## Files

- `sweep.sh` — builds the grid, snapshots the params, runs, summarizes.
- `run_list.sh` — runs a list of starts with N parallel workers (own ROS domain and log folder each).
- `summarize.py` — success rate, misses, time per start, viewpoints reached at the start, restarts needed.
