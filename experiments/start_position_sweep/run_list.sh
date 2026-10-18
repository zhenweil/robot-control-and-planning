#!/bin/bash
# Runs one placement node from each start in LIST ("x y z" per line, abs m), NW at a time, each worker in its own
# ROS domain and log folder. Appends one CSV row per start to OUT; a start already in OUT is skipped (resumable).
#
# Usage: run_list.sh LIST OUT [NW=6] [SEED=42]
# Env:   METHOD=object_placement|random_placement|sgd_placement   (default object_placement)
#        BASEYAML=<params yaml>                  (default: the package's config/object_placement_run.yaml)
#        TIME_BUDGET=<s>                         (random_placement and sgd_placement, default 90)
#        TIMEOUT=<s>                             (per start, default 900)
#        ROBOT=panda|kinova|xarm7|ur5e|ur3e      (default panda)
#        TOUR_INPUT_DIR=<dir>                    (viewpoints; default: the launch file's 38 checked ones)
set -u
LIST=$1; OUT=$2; NW=${3:-6}; SEED=${4:-42}
METHOD=${METHOD:-object_placement}
WS=$(cd "$(dirname "$0")/../.." && pwd)  # workspace root
BASE=${BASEYAML:-$WS/src/object_placement_planning/config/object_placement_run.yaml}
TIME_BUDGET=${TIME_BUDGET:-90}
TIMEOUT=${TIMEOUT:-900}
ROBOT=${ROBOT:-panda}
TOUR_INPUT_DIR=${TOUR_INPUT_DIR:-}
WORK=$(dirname "$OUT")/work
mkdir -p "$WORK"
[ -f "$OUT" ] || echo "x,y,z,reached,final_x,final_y,final_z,seconds,log" > "$OUT"

worker() {
  local W=$1
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  export ROS_DOMAIN_ID=$((80 + W))
  export ROS_LOG_DIR=$WORK/logs/w$W
  mkdir -p "$ROS_LOG_DIR"
  local n=0
  while read -r x y z; do
    [ -z "$x" ] && continue
    n=$((n + 1)); [ $(( (n - 1) % NW )) -ne "$W" ] && continue
    grep -q "^$x,$y,$z," "$OUT" && continue
    local yaml=$WORK/start_w$W.yaml
    sed -e "s/initial_x: .*/initial_x: $x/" -e "s/initial_y: .*/initial_y: $y/" \
        -e "s/initial_z: .*/initial_z: $z/" "$BASE" > "$yaml"
    local args="params_file:=$yaml random_seed:=$SEED robot:=$ROBOT"
    [ -n "$TOUR_INPUT_DIR" ] && args="$args tour_input_dir:=$TOUR_INPUT_DIR"
    [ "$METHOD" != object_placement ] && args="$args time_budget_sec:=$TIME_BUDGET"
    local before; before=$(ls -t "$ROS_LOG_DIR"/${METHOD}_*.log 2>/dev/null | head -1)
    local t0; t0=$(date +%s)
    setsid ros2 launch object_placement_planning "$METHOD.launch.py" $args > /dev/null 2>&1 &
    local pid=$! log="" cur
    for _ in $(seq 1 "$TIMEOUT"); do
      sleep 1
      cur=$(ls -t "$ROS_LOG_DIR"/${METHOD}_*.log 2>/dev/null | head -1)
      [ "$cur" != "$before" ] && log=$cur
      [ -n "$log" ] && grep -q "Done\. Object" "$log" && break
      kill -0 "$pid" 2>/dev/null || break
    done
    kill -INT -"$pid" 2>/dev/null; sleep 4; kill -KILL -"$pid" 2>/dev/null; sleep 2
    local secs=$(( $(date +%s) - t0 ))
    local done_line reached pos
    done_line=$( [ -n "$log" ] && grep -oE "Done\. Object abs \([^)]*\) m -- reaches (all [0-9]+|only [0-9]+/[0-9]+)" "$log" | tail -1)
    if [ -n "$done_line" ]; then
      reached=$(echo "$done_line" | grep -oE "(all [0-9]+|only [0-9]+/[0-9]+)" | sed -E 's/all ([0-9]+)/\1\/\1/; s/only //')
      pos=$(echo "$done_line" | grep -oE "\([^)]*\)" | tr -d '() ')
    else
      reached="error"; pos=",,"
    fi
    echo "$x,$y,$z,$reached,$pos,$secs,$log" >> "$OUT"
  done < "$LIST"
}

for w in $(seq 0 $((NW - 1))); do worker "$w" & done
wait
