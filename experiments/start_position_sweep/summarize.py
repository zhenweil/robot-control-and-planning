#!/usr/bin/env python3
"""Summarizes a start-position sweep's results.csv: success rate, misses, timing, viewpoints reached at the start,
and (for object_placement) how many successes needed a restart."""
import csv
import re
import statistics
import sys


def first_match(path, pattern):
    try:
        with open(path) as f:
            for line in f:
                m = re.search(pattern, line)
                if m:
                    return m
    except OSError:
        pass
    return None


def reached_all(reached):
    parts = reached.split("/")
    return len(parts) == 2 and parts[0] == parts[1]


def main(path):
    rows = [r for r in csv.DictReader(open(path)) if r["x"] != "ALLDONE"]
    if not rows:
        print("no results")
        return
    ok = [r for r in rows if reached_all(r["reached"])]
    print(f"starts: {len(rows)}, reached all: {len(ok)} ({100.0 * len(ok) / len(rows):.1f}%), "
          f"errors: {sum(r['reached'] == 'error' for r in rows)}")
    secs = sorted(int(r["seconds"]) for r in rows)
    print(f"seconds per start: median {statistics.median(secs):.0f}, max {secs[-1]}")

    # object_placement logs "iter 0: ... reachable N/M" at each descent's start and "restart K/..." per restart.
    at_start = []
    after_restart = 0
    for r in rows:
        m = first_match(r["log"], r"iter 0: .*reachable (\d+)/(\d+)")
        if m:
            at_start.append((int(m.group(1)), int(m.group(2))))
        if reached_all(r["reached"]) and first_match(r["log"], r"restart 2/\d+ iter 0"):
            after_restart += 1
    if at_start:
        print(f"already reached all at the start: {sum(a == b for a, b in at_start)}; "
              f"median reached at the start: {statistics.median(a for a, _ in at_start):.0f}")
        print(f"successes on the first descent: {len(ok) - after_restart}, after a restart: {after_restart}")

    misses = [r for r in rows if not reached_all(r["reached"])]
    if misses:
        print("misses:")
        for r in misses:
            print(f"  start ({r['x']}, {r['y']}, {r['z']}): {r['reached']} -> end "
                  f"({r['final_x']}, {r['final_y']}, {r['final_z']})")


if __name__ == "__main__":
    main(sys.argv[1])
