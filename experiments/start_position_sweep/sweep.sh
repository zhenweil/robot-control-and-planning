#!/bin/bash
# Start-position sweep: runs the placement node from every start on an x/y/z grid and summarizes the results.
#
# Usage: sweep.sh NAME [NW=6] [SEED=42]
# Env:   METHOD, BASEYAML, TIME_BUDGET, TIMEOUT, ROBOT (see run_list.sh)
#        X="0.2 0.4 0.6"  Y="0.0 0.2 0.4 0.6"  Z="0.0 0.2 0.4 0.6"   (abs m, inside the search box)
# Output: experiments/start_position_sweep/runs/NAME/{starts.txt,params.yaml,run_info.txt,results.csv,summary.txt}
set -u
NAME=$1; NW=${2:-6}; SEED=${3:-42}
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../.." && pwd)
X=${X:-"0.2 0.4 0.6"}
Y=${Y:-"0.0 0.2 0.4 0.6"}
Z=${Z:-"0.0 0.2 0.4 0.6"}
DIR=$HERE/runs/$NAME
mkdir -p "$DIR"

# Snapshot the params, so editing the YAML mid-sweep doesn't change this run.
cp "${BASEYAML:-$WS/src/object_placement_planning/config/object_placement_run.yaml}" "$DIR/params.yaml"
export BASEYAML=$DIR/params.yaml
for x in $X; do for y in $Y; do for z in $Z; do echo "$x $y $z"; done; done; done > "$DIR/starts.txt"
echo "method=${METHOD:-object_placement} robot=${ROBOT:-panda} viewpoints=${TOUR_INPUT_DIR:-default} seed=$SEED workers=$NW starts=$(wc -l < "$DIR/starts.txt") git=$(git -C "$WS" rev-parse --short HEAD)" \
	> "$DIR/run_info.txt"

"$HERE/run_list.sh" "$DIR/starts.txt" "$DIR/results.csv" "$NW" "$SEED"
python3 "$HERE/summarize.py" "$DIR/results.csv" | tee "$DIR/summary.txt"
