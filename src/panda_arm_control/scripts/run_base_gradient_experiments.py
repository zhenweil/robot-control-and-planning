#!/usr/bin/env python3
"""Seed-sweep the base_gradient attribution experiment and aggregate the results.

For each seed it launches `base_gradient.launch.py experiment_mode:=true random_seed:=<s>`,
which descends once for b*, then cold/warm re-solves the tour at b*, at offset 0, and at
`--num-random-dirs` random offsets of the same mixed-metric magnitude. Each launch writes
base_gradient_experiment_seed<s>.json into --output-dir; this script then pools them and
prints the attribution report:

  exp 1 + 2  cold re-solve at endpoints, across seeds -- is the cost drop the object move?
  exp 3      cold placebo -- does the optimized direction beat random ones of size |b*|?
  exp 4      warm placebo -- how much of any drop is warm-starting, and does the gap survive it?

The tour it optimizes over is whatever base_gradient.launch.py points tour_input_dir at
(default /tmp/viewpoint_planner_output/selected_robot_poses.json) -- populate that first.
"""

import argparse
import json
import os
import statistics
import subprocess
import sys
from math import degrees


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seeds", type=int, nargs="+", help="explicit list of seeds")
    p.add_argument("--num-seeds", type=int, default=5, help="if --seeds not given: sweep this many")
    p.add_argument("--start-seed", type=int, default=1, help="first seed when using --num-seeds")
    p.add_argument("--num-random-dirs", type=int, default=10, help="random offsets per seed (exp 3/4)")
    p.add_argument("--output-dir", default="/tmp/base_gradient_experiment_output")
    p.add_argument("--launch-timeout", type=float, default=1800.0, help="seconds per seed before giving up")
    p.add_argument("--skip-launch", action="store_true", help="only aggregate JSONs already in --output-dir")
    p.add_argument("--plot", action="store_true", help="also write a strip-plot PNG (needs matplotlib)")
    p.add_argument("--extra", nargs="*", default=[], metavar="name:=value",
                   help="extra args passed through to ros2 launch, e.g. --extra tour_input_dir:=/path")
    return p.parse_args()


def run_one_seed(seed, args):
    cmd = [
        "ros2", "launch", "panda_arm_control", "base_gradient.launch.py",
        "experiment_mode:=true",
        "use_rviz:=false",
        f"random_seed:={seed}",
        f"experiment_num_random_dirs:={args.num_random_dirs}",
        f"output_dir:={args.output_dir}",
    ] + list(args.extra)
    print(f"\n=== seed {seed}: {' '.join(cmd)}", flush=True)
    try:
        subprocess.run(cmd, timeout=args.launch_timeout, check=False)
    except subprocess.TimeoutExpired:
        print(f"!!! seed {seed}: launch timed out after {args.launch_timeout}s", flush=True)


def load_results(output_dir, seeds):
    out = []
    for s in seeds:
        path = os.path.join(output_dir, f"base_gradient_experiment_seed{s}.json")
        if not os.path.isfile(path):
            print(f"!!! missing {path} -- seed {s} skipped in the report", flush=True)
            continue
        with open(path) as f:
            out.append(json.load(f))
    return out


def mean_sd(xs):
    xs = list(xs)
    if not xs:
        return float("nan"), float("nan")
    if len(xs) == 1:
        return xs[0], 0.0
    return statistics.fmean(xs), statistics.pstdev(xs)


def fmt_offset(off):
    x, y, z, roll, pitch = off
    return f"({x:+.3f},{y:+.3f},{z:+.3f} m / {degrees(roll):+.1f},{degrees(pitch):+.1f} deg)"


def report(results):
    if not results:
        print("no results to aggregate.")
        return

    rot_scale = results[0].get("rot_metric_scale", 0.3)
    ndirs = len(results[0]["exp3_cold_placebo"])
    print("\n" + "=" * 78)
    print("BASE-GRADIENT ATTRIBUTION EXPERIMENT")
    print(f"seeds: {[r['seed'] for r in results]}   random dirs/seed: {ndirs}   "
          f"rot_metric_scale: {rot_scale}")
    print("=" * 78)

    # ---- descent summary -------------------------------------------------------------------
    print("\nDESCENT (per seed)")
    print(f" {'seed':>4}  {'ok':>3}  {'|b*|_metric':>11}  {'b*':>44}  {'reported D':>10}")
    for r in results:
        d = r["descent"]
        floored = "  [probe floored]" if r.get("probe_d_floored") else ""
        print(f" {r['seed']:>4}  {('yes' if d['ok'] else 'NO'):>3}  {d['d_metric']:>11.4f}  "
              f"{fmt_offset(d['offset']):>44}  {d['reported_weighted_cost']:>10.3f}{floored}")

    # ---- exp 1 + 2 -----------------------------------------------------------------------
    print("\nEXP 1 + 2 -- cold re-solve at endpoints (is the drop the object move?)")
    print(f" {'seed':>4}  {'C0_cold':>9}  {'Copt_cold':>9}  {'dPhi':>8}  {'dPhi%':>7}  "
          f"{'reach C0':>9}  {'reach Copt':>10}")
    d_abs, d_pct, n_better = [], [], 0
    for r in results:
        c0 = r["exp1_cold_endpoints"]["c0_cold"]
        co = r["exp1_cold_endpoints"]["copt_cold"]
        dphi = co["honest_cost"] - c0["honest_cost"]
        pct = 100.0 * dphi / c0["honest_cost"] if c0["honest_cost"] else float("nan")
        d_abs.append(dphi)
        d_pct.append(pct)
        n_better += dphi < 0
        print(f" {r['seed']:>4}  {c0['honest_cost']:>9.3f}  {co['honest_cost']:>9.3f}  {dphi:>+8.3f}  "
              f"{pct:>+6.1f}%  {c0['num_reachable']:>4}/{r['num_total']:<4}  "
              f"{co['num_reachable']:>5}/{r['num_total']:<4}")
    m, sd = mean_sd(d_abs)
    mp, _ = mean_sd(d_pct)
    print(f" {'mean':>4}  {'':>9}  {'':>9}  {m:>+8.3f}  {mp:>+6.1f}%")
    print(f" -> Copt < C0 in {n_better}/{len(results)} seeds; mean dPhi {m:+.3f} ({mp:+.1f}%), sd {sd:.3f}")

    # ---- exp 3 -------------------------------------------------------------------------
    print("\nEXP 3 -- cold placebo (does the optimized direction beat random ones of size |b*|?)")
    print(f" {'seed':>4}  {'Copt_cold':>9}  {'rand mean':>9}  {'rand min':>9}  {'rand max':>9}  "
          f"{'#rand<opt':>9}")
    pooled_better, pooled_total = 0, 0
    gap_mean, gap_best = [], []
    for r in results:
        co = r["exp1_cold_endpoints"]["copt_cold"]["honest_cost"]
        rand = [e["honest_cost"] for e in r["exp3_cold_placebo"]]
        rm, _ = mean_sd(rand)
        better = sum(x < co for x in rand)
        pooled_better += better
        pooled_total += len(rand)
        gap_mean.append(co - rm)
        gap_best.append(co - min(rand))
        print(f" {r['seed']:>4}  {co:>9.3f}  {rm:>9.3f}  {min(rand):>9.3f}  {max(rand):>9.3f}  "
              f"{better:>4}/{len(rand):<4}")
    gm, _ = mean_sd(gap_mean)
    gb, _ = mean_sd(gap_best)
    print(f" -> optimized beats {pooled_total - pooled_better}/{pooled_total} random directions")
    print(f"    mean(Copt - rand_mean) = {gm:+.3f}   mean(Copt - rand_best) = {gb:+.3f}")

    # ---- exp 4 -------------------------------------------------------------------------
    print("\nEXP 4 -- warm placebo (how much is warm-starting, and does the gap survive it?)")
    opt_disc, rand_disc, pooled_better_w, pooled_total_w = [], [], 0, 0
    gap_warp = []
    for r in results:
        co_cold = r["exp1_cold_endpoints"]["copt_cold"]["honest_cost"]
        co_warm = r["exp4_warm_placebo"]["copt_warm"]["honest_cost"]
        opt_disc.append(co_warm - co_cold)
        rc = [e["honest_cost"] for e in r["exp3_cold_placebo"]]
        rw = [e["honest_cost"] for e in r["exp4_warm_placebo"]["random"]]
        rand_disc.extend(w - c for c, w in zip(rc, rw))
        better = sum(x < co_warm for x in rw)
        pooled_better_w += better
        pooled_total_w += len(rw)
        rwm, _ = mean_sd(rw)
        gap_warp.append(co_warm - rwm)
    od, ods = mean_sd(opt_disc)
    rd, rds = mean_sd(rand_disc)
    gw, _ = mean_sd(gap_warp)
    print(f" warm-start discount at b*:      mean(Copt_warm - Copt_cold)   = {od:+.3f}  (sd {ods:.3f})")
    print(f" warm-start discount at random:  mean(Crand_warm - Crand_cold) = {rd:+.3f}  (sd {rds:.3f}, paired)")
    print(f" -> optimized beats {pooled_total_w - pooled_better_w}/{pooled_total_w} random dirs even warm;"
          f"  mean(Copt_warm - rand_warm_mean) = {gw:+.3f}")

    # ---- one-line verdict ----------------------------------------------------------------
    print("\nSUMMARY")
    print(f"  move effect (exp1):        mean cold dPhi {m:+.3f}  ({n_better}/{len(results)} seeds improve)")
    print(f"  direction quality (exp3):  optimized < random in "
          f"{pooled_total - pooled_better}/{pooled_total}; edge {gm:+.3f} vs mean / {gb:+.3f} vs best")
    print(f"  warm-start confound (exp4): ~{od:+.3f} at b* vs ~{rd:+.3f} at random -- "
          f"{'roughly uniform, gap survives' if abs(od - rd) < abs(gw) else 'direction-dependent, inspect'}")
    print("=" * 78)


def make_plot(results, output_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping --plot", flush=True)
        return

    series = {
        "C0\ncold": [r["exp1_cold_endpoints"]["c0_cold"]["honest_cost"] for r in results],
        "Copt\ncold": [r["exp1_cold_endpoints"]["copt_cold"]["honest_cost"] for r in results],
        "rand\ncold": [e["honest_cost"] for r in results for e in r["exp3_cold_placebo"]],
        "Copt\nwarm": [r["exp4_warm_placebo"]["copt_warm"]["honest_cost"] for r in results],
        "rand\nwarm": [e["honest_cost"] for r in results for e in r["exp4_warm_placebo"]["random"]],
    }
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for i, (label, ys) in enumerate(series.items()):
        xs = [i + 0.06 * ((j % 7) - 3) for j in range(len(ys))]
        ax.scatter(xs, ys, s=18, alpha=0.7)
        if ys:
            ax.hlines(statistics.fmean(ys), i - 0.25, i + 0.25, color="k", lw=2)
    ax.set_xticks(range(len(series)))
    ax.set_xticklabels(list(series.keys()))
    ax.set_ylabel("honest weighted tour cost")
    ax.set_title("base-gradient attribution: endpoints vs random-offset placebo")
    fig.tight_layout()
    path = os.path.join(output_dir, "base_gradient_experiment_summary.png")
    fig.savefig(path, dpi=130)
    print(f"wrote {path}", flush=True)


def main():
    args = parse_args()
    seeds = args.seeds if args.seeds else list(range(args.start_seed, args.start_seed + args.num_seeds))
    os.makedirs(args.output_dir, exist_ok=True)

    if not args.skip_launch:
        for s in seeds:
            run_one_seed(s, args)

    results = load_results(args.output_dir, seeds)
    report(results)

    summary_path = os.path.join(args.output_dir, "base_gradient_experiment_summary.json")
    with open(summary_path, "w") as f:
        json.dump({"seeds": seeds, "results": results}, f, indent=2)
    print(f"\nwrote {summary_path}")

    if args.plot:
        make_plot(results, args.output_dir)

    return 0 if results else 1


if __name__ == "__main__":
    sys.exit(main())
