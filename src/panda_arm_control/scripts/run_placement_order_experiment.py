#!/usr/bin/env python3
"""Drive + report the base_gradient placement / order separability experiment.

For each seed this launches `base_gradient.launch.py placement_experiment:=true`, optionally
sharding the grid_n^3 (x,y,z) grid across `--parallel` processes (each on its own
ROS_DOMAIN_ID). Every shard writes placement_experiment_seed<seed>_g<start>.json into
--output-dir; this script merges the shards and prints the three-experiment report:

  EXP 1  route frozen (route 0 = input tour order): does the object (x,y,z) position change the tour cost?
  EXP 2  at that route's best position, does re-routing (full GTSP) lower the cost further?
  EXP 3  does the best position move when the route changes -- placement/routing separable?

The tour is whatever base_gradient.launch.py points tour_input_dir at (default
/tmp/viewpoint_planner_output/selected_robot_poses.json) -- populate that first.
"""

import argparse
import glob
import json
import math
import os
import statistics
import subprocess
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seeds", type=int, nargs="+", help="explicit list of seeds")
    p.add_argument("--num-seeds", type=int, default=1, help="if --seeds not given: sweep this many")
    p.add_argument("--start-seed", type=int, default=1, help="first seed when using --num-seeds")
    p.add_argument("--grid-n", type=int, default=5, help="grid points per axis (grid_n^3 total)")
    p.add_argument("--range", type=float, default=0.15,
                   help="half-extent (m) of the (x,y,z) offset box, symmetric about nominal; "
                        "sets x/y/z min/max unless overridden in --extra")
    p.add_argument("--no-z", action="store_true",
                   help="x-y sweep only (z pinned to 0): grid_n^2 points instead of grid_n^3")
    p.add_argument("--parallel", type=int, default=1,
                   help="split each seed's grid into this many shards, run concurrently")
    p.add_argument("--output-dir", default="/tmp/placement_experiment_output")
    p.add_argument("--ros-domain-base", type=int, default=50,
                   help="ROS_DOMAIN_ID for the first shard (then +1 per shard)")
    p.add_argument("--launch-timeout", type=float, default=7200.0, help="seconds per shard before giving up")
    p.add_argument("--skip-launch", action="store_true", help="only merge + report JSONs already in --output-dir")
    p.add_argument("--plot", action="store_true", help="also write cost-surface heatmap PNGs (needs matplotlib)")
    p.add_argument("--extra", nargs="*", default=[], metavar="name:=value",
                   help="extra args passed through to ros2 launch, e.g. --extra tour_input_dir:=/path")
    return p.parse_args()


def shard_cmd(seed, grid_n, start, count, args, routes_file=""):
    extra_names = {e.split(":=")[0] for e in args.extra}
    cmd = [
        "ros2", "launch", "panda_arm_control", "base_gradient.launch.py",
        "placement_experiment:=true",
        "use_rviz:=false",
        f"random_seed:={seed}",
        f"placement_grid_n:={grid_n}",
        f"placement_grid_start:={start}",
        f"placement_grid_count:={count}",
        f"output_dir:={args.output_dir}",
    ]
    r = abs(args.range)
    zr = 0.0 if args.no_z else r
    for name, val in (("x_min", -r), ("x_max", r), ("y_min", -r), ("y_max", r), ("z_min", -zr), ("z_max", zr)):
        if name not in extra_names:
            cmd.append(f"{name}:={val}")
    if routes_file:
        cmd.append(f"placement_reference_orders_file:={routes_file}")
    return cmd + list(args.extra)


def routes_path(output_dir, seed):
    return os.path.join(output_dir, f"placement_reference_orders_seed{seed}.json")


def grid_shards(total, parts):
    """Contiguous [start, count) ranges covering range(total), balanced."""
    parts = max(1, parts)
    base, rem = divmod(total, parts)
    out, start = [], 0
    for i in range(parts):
        count = base + (1 if i < rem else 0)
        if count == 0:
            continue
        out.append((start, count))
        start += count
    return out


def _wait(running, launch_timeout):
    while running:
        for p, (label, log, deadline) in list(running.items()):
            if p.poll() is None and time.time() > deadline:
                p.kill()
                print(f"!!! {label}: killed after {launch_timeout:.0f}s", flush=True)
            if p.poll() is not None:
                running.pop(p)
                log.close()
                print(f"=== {label} finished (exit {p.returncode})", flush=True)
        if running:
            time.sleep(5)


def run_seed(seed, args):
    total = args.grid_n ** (2 if args.no_z else 3)
    rfile = routes_path(args.output_dir, seed)

    # Prep run: solve the reference routes once and write them, so every grid shard scores
    # against an identical set (route generation at reachability-breaking offsets is
    # IK-timeout / CPU-load sensitive and does not reproduce across processes).
    logpath = os.path.join(args.output_dir, f"seed{seed}_routes.log")
    log = open(logpath, "w")
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain_base))
    p = subprocess.Popen(shard_cmd(seed, args.grid_n, 0, 0, args),
                         env=env, stdout=log, stderr=subprocess.STDOUT)
    print(f"=== seed {seed} reference-route prep started (log {logpath})", flush=True)
    _wait({p: (f"seed {seed} routes", log, time.time() + args.launch_timeout)}, args.launch_timeout)
    if not os.path.isfile(rfile):
        print(f"!!! seed {seed}: prep run did not write {rfile} -- see {logpath}", flush=True)
        return

    shards = grid_shards(total, args.parallel)
    running = {}  # popen -> (label, log, deadline)
    for slot, (start, count) in enumerate(shards):
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain_base + slot))
        logpath = os.path.join(args.output_dir, f"seed{seed}_g{start}.log")
        log = open(logpath, "w")
        p = subprocess.Popen(shard_cmd(seed, args.grid_n, start, count, args, routes_file=rfile),
                             env=env, stdout=log, stderr=subprocess.STDOUT)
        running[p] = (f"seed {seed} g[{start}:{start + count}]", log, time.time() + args.launch_timeout)
        print(f"=== {running[p][0]} started (domain {env['ROS_DOMAIN_ID']}, log {logpath})", flush=True)
    _wait(running, args.launch_timeout)


def merge_seed(output_dir, seed):
    paths = sorted(glob.glob(os.path.join(output_dir, f"placement_experiment_seed{seed}_g*.json")),
                   key=lambda s: int(s.rsplit("_g", 1)[1].split(".")[0]))
    if not paths:
        print(f"!!! no shard JSONs for seed {seed}", flush=True)
        return None
    shards = [json.load(open(p)) for p in paths]
    m = dict(shards[0])
    m["grid"] = []
    for sh in shards:
        if sh["order_labels"] != m["order_labels"] or sh["reference_orders"] != m["reference_orders"]:
            print(f"!!! seed {seed}: shard reference routes disagree -- rebuild/rerun all shards", flush=True)
            return None
        m["grid"].extend(sh["grid"])
    want = m["grid_shape"][0] * m["grid_shape"][1] * m["grid_shape"][2]
    if len(m["grid"]) != want:
        print(f"!!! seed {seed}: merged {len(m['grid'])}/{want} grid points -- report is partial", flush=True)
    return m


# --------------------------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------------------------

def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def steps(m):
    lo, hi, shp = m["grid_min"], m["grid_max"], m["grid_shape"]
    return [(hi[i] - lo[i]) / (shp[i] - 1) if shp[i] > 1 else 0.0 for i in range(3)]


def nominal_point(grid):
    return min(grid, key=lambda gp: max(abs(c) for c in gp["offset"]))


def best_for_route(grid, k, n):
    """Grid point with the lowest weighted cost for route k, preferring all-n-reachable points.
    Returns (point, had_full_reach_option)."""
    pool = [gp for gp in grid if gp["order_num_reachable"][k] == n]
    return min(pool or grid, key=lambda gp: gp["order_weighted_cost"][k]), bool(pool)


def report(m, seed):
    n = m["num_total"]
    labels = m["order_labels"]
    grid = m["grid"]
    step = steps(m)
    step_norm = math.sqrt(sum(s * s for s in step))

    print("\n" + "=" * 82)
    print(f"PLACEMENT / ORDER SEPARABILITY  --  seed {seed}")
    print(f"grid {m['grid_shape'][0]}^3 = {len(grid)} pts   "
          f"x,y in [{m['grid_min'][0]:+.2f},{m['grid_max'][0]:+.2f}]  z in "
          f"[{m['grid_min'][2]:+.2f},{m['grid_max'][2]:+.2f}]   step ~"
          f"({step[0]:.3f},{step[1]:.3f},{step[2]:.3f}) m   penalty {m['unreachable_penalty']:.0f}/pose")
    n_full = sum(gp["order_num_reachable"][0] == n for gp in grid)
    print(f"routes: {', '.join(labels)}")
    print(f"grid points that keep all {n} poses (route 0): {n_full}/{len(grid)}")

    # ---- EXP 1 --------------------------------------------------------------------------------
    nom = nominal_point(grid)
    nom_hc = nom["order_honest_cost"][0]
    nom_nr = nom["order_num_reachable"][0]
    best, _ = best_for_route(grid, 0, n)
    d = dist(best["offset"], nom["offset"])
    full_hcs = [gp["order_honest_cost"][0] for gp in grid if gp["order_num_reachable"][0] == n]
    print("\nEXP 1 -- route frozen (route 0 = input tour order): does object position change cost?")
    print(f"  nominal (0,0,0):   honest cost {nom_hc:8.3f}   reach {nom_nr}/{n}")
    if full_hcs:
        print(f"  best full-reach:   honest cost {min(full_hcs):8.3f}   at "
              f"({best['offset'][0]:+.3f},{best['offset'][1]:+.3f},{best['offset'][2]:+.3f})   "
              f"{d * 100:.1f} cm from nominal")
        print(f"  full-reach spread: min {min(full_hcs):.2f}  median {statistics.median(full_hcs):.2f}  "
              f"max {max(full_hcs):.2f}  (n={len(full_hcs)})")
        impr = 100.0 * (nom_hc - min(full_hcs)) / nom_hc if nom_hc else 0.0
        flat = d <= 1.01 * step_norm and impr < 2.0
        print(f"  -> best position {impr:+.1f}% vs nominal, {d * 100:.1f} cm away  "
              f"{'<- FLAT: position barely matters' if flat else '<- position has real structure'}")
    else:
        print("  no grid point keeps all poses with the frozen route -- position hurts reachability badly")
        flat = False

    # ---- EXP 2 --------------------------------------------------------------------------------
    print("\nEXP 2 -- re-routing (full GTSP) vs the frozen route")
    for tag, gp in (("at nominal (0,0,0)", nom), ("at route-0 best position", best)):
        fr_hc, fr_nr = gp["order_honest_cost"][0], gp["order_num_reachable"][0]
        fu_hc, fu_nr = gp["full_honest_cost"], gp["full_num_reachable"]
        dd = 100.0 * (fr_hc - fu_hc) / fr_hc if fr_hc else 0.0
        print(f"  {tag:26s}: frozen {fr_hc:8.3f} ({fr_nr}/{n})   re-routed {fu_hc:8.3f} ({fu_nr}/{n})   "
              f"re-route saves {dd:+.1f}%")

    # ---- EXP 3 --------------------------------------------------------------------------------
    print("\nEXP 3 -- does the best position move with the route?")
    print(f"  {'route':>14}  {'best (x,y,z) m':>26}  {'honest':>8}  {'reach':>6}")
    argmins = []
    for k, lab in enumerate(labels):
        pk, has_full_k = best_for_route(grid, k, n)
        argmins.append(pk["offset"])
        flag = "" if has_full_k else "  (no full-reach pt)"
        print(f"  {lab:>14}  ({pk['offset'][0]:+.3f},{pk['offset'][1]:+.3f},{pk['offset'][2]:+.3f})"
              f"  {pk['order_honest_cost'][k]:8.3f}  {pk['order_num_reachable'][k]:>3}/{n}{flag}")
    pfull = min(grid, key=lambda gp: gp["full_weighted_cost"])
    print(f"  {'free-routing':>14}  ({pfull['offset'][0]:+.3f},{pfull['offset'][1]:+.3f},"
          f"{pfull['offset'][2]:+.3f})  {pfull['full_honest_cost']:8.3f}  {pfull['full_num_reachable']:>3}/{n}")

    spread = max((dist(a, b) for a in argmins for b in argmins), default=0.0)
    per_axis = [max(o[i] for o in argmins) - min(o[i] for o in argmins) for i in range(3)]
    print(f"  argmin spread: max pairwise {spread * 100:.1f} cm   per-axis range "
          f"({per_axis[0] * 100:.1f}, {per_axis[1] * 100:.1f}, {per_axis[2] * 100:.1f}) cm   "
          f"(grid step ~{step_norm * 100:.1f} cm)")
    if spread <= 1.5 * step_norm:
        print("  -> best position is STABLE across routes  <- placement & routing separable")
    else:
        print("  -> best position SHIFTS with the route     <- placement & routing coupled")
    if flat:
        print("  NOTE: EXP 1 found the cost surface ~flat, so these argmins are noise-dominated "
              "-- EXP 3 is inconclusive here.")
    print("=" * 82)


def make_plots(m, seed, output_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping --plot", flush=True)
        return
    nx, ny, nz = m["grid_shape"]
    grid = m["grid"]
    if len(grid) != nx * ny * nz:
        print("grid incomplete -- skipping --plot", flush=True)
        return
    kz = min(range(nz), key=lambda iz: abs(m["grid_min"][2] + (m["grid_max"][2] - m["grid_min"][2]) *
                                            (iz / (nz - 1) if nz > 1 else 0.5)))

    def slice_grid(getter):
        return [[getter(grid[ix * ny * nz + iy * nz + kz]) for ix in range(nx)] for iy in range(ny)]

    panels = [("route 0 (input order), frozen", lambda gp: gp["order_weighted_cost"][0]),
              ("free-routing (full GTSP)", lambda gp: gp["full_weighted_cost"])]
    fig, axes = plt.subplots(1, len(panels), figsize=(5.5 * len(panels), 4.6))
    for ax, (title, getter) in zip(axes, panels):
        im = ax.imshow(slice_grid(getter), origin="lower", aspect="auto",
                       extent=[m["grid_min"][0], m["grid_max"][0], m["grid_min"][1], m["grid_max"][1]])
        ax.set_title(title)
        ax.set_xlabel("x offset (m)")
        ax.set_ylabel("y offset (m)")
        fig.colorbar(im, ax=ax, label="weighted tour cost")
    fig.suptitle(f"placement cost surface, z-slice ~0  (seed {seed})")
    fig.tight_layout()
    path = os.path.join(output_dir, f"placement_experiment_seed{seed}_surface.png")
    fig.savefig(path, dpi=130)
    print(f"wrote {path}", flush=True)


def main():
    args = parse_args()
    seeds = args.seeds if args.seeds else list(range(args.start_seed, args.start_seed + args.num_seeds))
    os.makedirs(args.output_dir, exist_ok=True)

    merged_all = {}
    for seed in seeds:
        if not args.skip_launch:
            run_seed(seed, args)
        m = merge_seed(args.output_dir, seed)
        if m is None:
            continue
        merged_all[seed] = m
        report(m, seed)
        if args.plot:
            make_plots(m, seed, args.output_dir)

    if merged_all:
        summary_path = os.path.join(args.output_dir, "placement_experiment_summary.json")
        with open(summary_path, "w") as f:
            json.dump(merged_all, f)
        print(f"\nwrote {summary_path}")
    return 0 if merged_all else 1


if __name__ == "__main__":
    sys.exit(main())
