#!/usr/bin/env bash
set -euo pipefail

# Guard against running via "sh script.sh" (dash), which lacks Bash features used here.
if [ -z "${BASH_VERSION:-}" ]; then
  echo "ERROR: Please run this script with bash, not sh."
  echo "Use: bash run_dstar_ros2_linux.sh --action demo --mode package"
  exit 1
fi

# One-file Linux launcher for ROS2 D* workflow.
# This script can:
# 1) run planner node,
# 2) publish a goal,
# 3) read plan output,
# 4) run end-to-end demo.

MODE="direct"
ACTION="node"
WORKSPACE="$HOME/vlab_ws"
PKG_NAME="dstar_planner"
SCRIPT_PATH="${BASH_SOURCE[0]:-$0}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
NODE_SRC="$SCRIPT_DIR/ros2_dstar_node.py"
VERIFY_LOG="/tmp/dstar_e2e_verify.log"

MAP_TOPIC="/map"
ODOM_TOPIC="/odom"
SCAN_TOPIC="/scan"
GOAL_TOPIC="/goal_pose"
PLAN_TOPIC="/plan"
OCC_THRESHOLD="50"
MAX_ITER="50000"

GOAL_X="7.0"
GOAL_Y="4.0"
GOAL_Z="0.0"
GOAL_FRAME="map"

WAIT_SECONDS="2"
BRINGUP_CMD=""
EXTRA_BRINGUP_CMD=""
HEALTH_RETRIES="10"
HEALTH_INTERVAL="2"
HZ_SECONDS="5"

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --mode direct|package       Run mode (default: direct)
  --action node|goal|plan|demo|health|oneclick|verify
                              node: run planner node only
                              goal: publish one goal only
                              plan: echo one /plan message only
                              demo: run node + publish goal + read one /plan
                              health: check if map/odom/scan are publishing
                              oneclick: health + UI + planner + goal + plan
                              verify: full end-to-end verification with report
  --workspace PATH            ROS workspace for package mode (default: ~/vlab_ws)
  --map-topic TOPIC           Map topic (default: /map)
  --odom-topic TOPIC          Odom topic (default: /odom)
  --scan-topic TOPIC          Scan topic (default: /scan)
  --goal-topic TOPIC          Goal topic (default: /goal_pose)
  --plan-topic TOPIC          Output plan topic (default: /plan)
  --occupied-threshold INT    Occupancy threshold (default: 50)
  --max-iterations INT        D* max iterations (default: 50000)
  --goal-x FLOAT              Goal x in map frame (default: 7.0)
  --goal-y FLOAT              Goal y in map frame (default: 4.0)
  --goal-z FLOAT              Goal z (default: 0.0)
  --goal-frame STR            Goal frame_id (default: map)
  --wait-seconds INT          Wait before sending goal in demo (default: 2)
  --bringup-cmd "CMD"        Optional sim/SLAM launch command for oneclick
  --extra-bringup-cmd "CMD"  Optional second launch command (e.g., SLAM)
  --health-retries INT        Health check retries in oneclick (default: 10)
  --health-interval INT       Seconds between health retries (default: 2)
  --verify-log PATH           Verification report path (default: /tmp/dstar_e2e_verify.log)
  --hz-seconds INT            Duration for topic rate sampling (default: 5)
  -h, --help                  Show this help

Examples:
  $0 --action node
  $0 --action goal --goal-x 6.0 --goal-y 3.5
  $0 --action plan
  $0 --action health
  $0 --action demo --mode package --workspace ~/vlab_ws
  $0 --action oneclick --mode package --goal-x 2.0 --goal-y 1.0
  $0 --action oneclick --mode package --bringup-cmd "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
  $0 --action verify --mode package --bringup-cmd "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" --extra-bringup-cmd "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
  $0 --action verify --mode package --goal-x 2.0 --goal-y 1.0
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --action) ACTION="$2"; shift 2 ;;
    --workspace) WORKSPACE="$2"; shift 2 ;;
    --map-topic) MAP_TOPIC="$2"; shift 2 ;;
    --odom-topic) ODOM_TOPIC="$2"; shift 2 ;;
    --scan-topic) SCAN_TOPIC="$2"; shift 2 ;;
    --goal-topic) GOAL_TOPIC="$2"; shift 2 ;;
    --plan-topic) PLAN_TOPIC="$2"; shift 2 ;;
    --occupied-threshold) OCC_THRESHOLD="$2"; shift 2 ;;
    --max-iterations) MAX_ITER="$2"; shift 2 ;;
    --goal-x) GOAL_X="$2"; shift 2 ;;
    --goal-y) GOAL_Y="$2"; shift 2 ;;
    --goal-z) GOAL_Z="$2"; shift 2 ;;
    --goal-frame) GOAL_FRAME="$2"; shift 2 ;;
    --wait-seconds) WAIT_SECONDS="$2"; shift 2 ;;
    --bringup-cmd) BRINGUP_CMD="$2"; shift 2 ;;
    --extra-bringup-cmd) EXTRA_BRINGUP_CMD="$2"; shift 2 ;;
    --health-retries) HEALTH_RETRIES="$2"; shift 2 ;;
    --health-interval) HEALTH_INTERVAL="$2"; shift 2 ;;
    --verify-log) VERIFY_LOG="$2"; shift 2 ;;
    --hz-seconds) HZ_SECONDS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "$NODE_SRC" ]]; then
  echo "ERROR: ros2_dstar_node.py not found at: $NODE_SRC"
  exit 1
fi

source_safe() {
  # ROS setup scripts can reference unset variables internally.
  # Temporarily disable nounset to avoid false failures.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

# Source ROS 2 setup.
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_safe "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source_safe /opt/ros/humble/setup.bash
elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  source_safe /opt/ros/jazzy/setup.bash
else
  echo "ERROR: Could not find ROS 2 setup.bash in /opt/ros/<distro>/"
  echo "Please install/source ROS 2 first."
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found after sourcing ROS environment."
  exit 1
fi

run_node_direct() {
  chmod +x "$NODE_SRC"
  python3 "$NODE_SRC" --ros-args \
    -p map_topic:="$MAP_TOPIC" \
    -p odom_topic:="$ODOM_TOPIC" \
    -p goal_topic:="$GOAL_TOPIC" \
    -p plan_topic:="$PLAN_TOPIC" \
    -p occupied_threshold:="$OCC_THRESHOLD" \
    -p max_iterations:="$MAX_ITER"
}

prepare_package_and_env() {
  if ! command -v colcon >/dev/null 2>&1; then
    echo "ERROR: colcon not found. Install with: sudo apt install python3-colcon-common-extensions"
    exit 1
  fi

  mkdir -p "$WORKSPACE/src"
  cd "$WORKSPACE/src"

  if [[ ! -d "$PKG_NAME" ]]; then
    ros2 pkg create --build-type ament_python "$PKG_NAME" --dependencies rclpy nav_msgs geometry_msgs
  fi

  PKG_DIR="$WORKSPACE/src/$PKG_NAME"
  PKG_PY_DIR="$PKG_DIR/$PKG_NAME"
  mkdir -p "$PKG_PY_DIR"
  cp "$NODE_SRC" "$PKG_PY_DIR/ros2_dstar_node.py"
  chmod +x "$PKG_PY_DIR/ros2_dstar_node.py"

  if [[ ! -f "$PKG_PY_DIR/__init__.py" ]]; then
    touch "$PKG_PY_DIR/__init__.py"
  fi

  python3 - "$PKG_DIR/setup.py" <<'PY'
import sys
from pathlib import Path

setup_path = Path(sys.argv[1])
text = setup_path.read_text(encoding="utf-8")
entry = "            'ros2_dstar_node = dstar_planner.ros2_dstar_node:main',\n"

if "ros2_dstar_node = dstar_planner.ros2_dstar_node:main" in text:
    raise SystemExit(0)

needle = "'console_scripts': [\n"
if needle in text:
    text = text.replace(needle, needle + entry)
else:
    marker = "setup(\n"
    if marker not in text:
        raise SystemExit("ERROR: Could not parse setup.py to insert console_scripts entry")
    insert = (
        "    entry_points={\n"
        "        'console_scripts': [\n"
        "            'ros2_dstar_node = dstar_planner.ros2_dstar_node:main',\n"
        "        ],\n"
        "    },\n"
    )
    idx = text.rfind(")")
    if idx == -1:
        raise SystemExit("ERROR: Could not find closing ')' in setup.py")
    text = text[:idx] + insert + text[idx:]

setup_path.write_text(text, encoding="utf-8")
PY

  cd "$WORKSPACE"
  colcon build --packages-select "$PKG_NAME"
  source_safe "$WORKSPACE/install/setup.bash"
}

run_node_package() {
  ros2 run "$PKG_NAME" ros2_dstar_node --ros-args \
    -p map_topic:="$MAP_TOPIC" \
    -p odom_topic:="$ODOM_TOPIC" \
    -p goal_topic:="$GOAL_TOPIC" \
    -p plan_topic:="$PLAN_TOPIC" \
    -p occupied_threshold:="$OCC_THRESHOLD" \
    -p max_iterations:="$MAX_ITER"
}

publish_goal_once() {
  ros2 topic pub -1 "$GOAL_TOPIC" geometry_msgs/PoseStamped "{header: {frame_id: '$GOAL_FRAME'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {w: 1.0}}}"
}

echo_plan_once() {
  ros2 topic echo --once "$PLAN_TOPIC"
}

check_topic_health() {
  local topic="$1"
  local label="$2"
  local qos_mode="${3:-default}"
  local info_out
  local msg_out

  echo "[INFO] Checking $label topic: $topic"

  if ! info_out="$(ros2 topic info "$topic" 2>&1)"; then
    echo "[FAIL] $label: topic does not exist"
    return 1
  fi

  echo "$info_out"
  if ! echo "$info_out" | grep -Eq "Publisher count:\s*[1-9]"; then
    echo "[FAIL] $label: no active publishers"
    return 1
  fi

  if [[ "$qos_mode" == "sensor_data" ]]; then
    if msg_out="$(timeout 6s ros2 topic echo --once "$topic" --qos-profile sensor_data 2>&1)"; then
      echo "[PASS] $label: received at least one message"
      return 0
    fi
  else
    if msg_out="$(timeout 6s ros2 topic echo --once "$topic" 2>&1)"; then
      echo "[PASS] $label: received at least one message"
      return 0
    fi
  fi

  echo "[FAIL] $label: publisher exists but no message received within timeout"
  echo "$msg_out"
  return 1
}

run_health_checks() {
  local failed=0

  echo "[INFO] Running topic publishing health checks"
  echo "[INFO] Tip: make sure Gazebo is playing (not paused)"

  if ! check_topic_health "$MAP_TOPIC" "map"; then
    failed=1
  fi

  if ! check_topic_health "$ODOM_TOPIC" "odom"; then
    failed=1
  fi

  if ! check_topic_health "$SCAN_TOPIC" "scan" "sensor_data"; then
    echo "[WARN] Scan check failed. If your stack does not publish scan, this can be ignored for planner-only runs."
  fi

  if [[ "$failed" -eq 0 ]]; then
    echo "[PASS] Core planner inputs look healthy: map and odom are publishing."
  else
    echo "[FAIL] Core planner inputs are not ready. Fix map/odom publishers before running planner demo."
    return 1
  fi
}

launch_ui_dashboards() {
  echo "[INFO] Launching recommended UI dashboards (if installed)"

  if command -v rviz2 >/dev/null 2>&1; then
    rviz2 >/dev/null 2>&1 &
    echo "[INFO] Started rviz2"
  else
    echo "[WARN] rviz2 not found; skipping RViz"
  fi

  if command -v rqt_graph >/dev/null 2>&1; then
    rqt_graph >/dev/null 2>&1 &
    echo "[INFO] Started rqt_graph"
  else
    echo "[WARN] rqt_graph not found; skipping graph UI"
  fi

  if command -v rqt_topic >/dev/null 2>&1; then
    rqt_topic >/dev/null 2>&1 &
    echo "[INFO] Started rqt_topic"
  else
    echo "[WARN] rqt_topic not found; skipping topic UI"
  fi
}

run_optional_bringup() {
  if [[ -z "$BRINGUP_CMD" && -z "$EXTRA_BRINGUP_CMD" ]]; then
    return 0
  fi

  if [[ ("$BRINGUP_CMD" == *"turtlebot3"* || "$EXTRA_BRINGUP_CMD" == *"turtlebot3"*) && -z "${TURTLEBOT3_MODEL:-}" ]]; then
    export TURTLEBOT3_MODEL=burger
    echo "[WARN] TURTLEBOT3_MODEL not set. Defaulting to burger."
  fi

  if [[ -n "$BRINGUP_CMD" ]]; then
    echo "[INFO] Starting bringup command in background"
    echo "[INFO] Command: $BRINGUP_CMD"
    bash -lc "$BRINGUP_CMD" >/tmp/dstar_bringup.log 2>&1 &
    BRINGUP_PID=$!
    echo "[INFO] Bringup PID: $BRINGUP_PID"
  fi

  if [[ -n "$EXTRA_BRINGUP_CMD" ]]; then
    echo "[INFO] Starting extra bringup command in background"
    echo "[INFO] Command: $EXTRA_BRINGUP_CMD"
    bash -lc "$EXTRA_BRINGUP_CMD" >/tmp/dstar_extra_bringup.log 2>&1 &
    EXTRA_BRINGUP_PID=$!
    echo "[INFO] Extra bringup PID: $EXTRA_BRINGUP_PID"
  fi
}

bringup_process_alive_or_report() {
  local pid="$1"
  local label="$2"
  local log_path="$3"

  if [[ -z "$pid" ]]; then
    return 0
  fi

  if kill -0 "$pid" >/dev/null 2>&1; then
    return 0
  fi

  echo "[ERROR] $label process exited early"
  echo "[INFO] Last log lines from $log_path"
  if [[ -f "$log_path" ]]; then
    tail -n 40 "$log_path"
  else
    echo "[WARN] Log file missing: $log_path"
  fi
  return 1
}

wait_for_health_ready() {
  local attempt=1
  while [[ "$attempt" -le "$HEALTH_RETRIES" ]]; do
    if ! bringup_process_alive_or_report "${BRINGUP_PID:-}" "Bringup" "/tmp/dstar_bringup.log"; then
      return 1
    fi
    if ! bringup_process_alive_or_report "${EXTRA_BRINGUP_PID:-}" "Extra bringup" "/tmp/dstar_extra_bringup.log"; then
      return 1
    fi

    echo "[INFO] Health attempt $attempt/$HEALTH_RETRIES"
    if run_health_checks; then
      return 0
    fi
    if [[ "$attempt" -lt "$HEALTH_RETRIES" ]]; then
      sleep "$HEALTH_INTERVAL"
    fi
    attempt=$((attempt + 1))
  done
  return 1
}

sample_topic_rate() {
  local topic="$1"
  local label="$2"
  local required="${3:-required}"
  local hz_out

  echo "[INFO] Sampling rate for $label topic: $topic" | tee -a "$VERIFY_LOG"
  hz_out="$(timeout "${HZ_SECONDS}s" ros2 topic hz "$topic" 2>&1 || true)"
  echo "$hz_out" | tee -a "$VERIFY_LOG"

  if echo "$hz_out" | grep -qi "average rate"; then
    echo "[PASS] $label rate detected" | tee -a "$VERIFY_LOG"
    return 0
  fi

  if [[ "$required" == "required" ]]; then
    echo "[FAIL] $label rate not detected" | tee -a "$VERIFY_LOG"
    return 1
  fi

  echo "[WARN] $label rate not detected (optional topic)" | tee -a "$VERIFY_LOG"
  return 0
}

run_verify_flow() {
  local verify_failed=0
  local plan_out

  : >"$VERIFY_LOG"
  echo "[INFO] Verification report: $VERIFY_LOG" | tee -a "$VERIFY_LOG"
  echo "[INFO] Starting full end-to-end verification" | tee -a "$VERIFY_LOG"

  launch_ui_dashboards
  run_optional_bringup

  cleanup() {
    if [[ -n "${NODE_PID:-}" ]] && kill -0 "$NODE_PID" >/dev/null 2>&1; then
      kill "$NODE_PID" >/dev/null 2>&1 || true
    fi
    if [[ -n "${BRINGUP_PID:-}" ]] && kill -0 "$BRINGUP_PID" >/dev/null 2>&1; then
      kill "$BRINGUP_PID" >/dev/null 2>&1 || true
    fi
    if [[ -n "${EXTRA_BRINGUP_PID:-}" ]] && kill -0 "$EXTRA_BRINGUP_PID" >/dev/null 2>&1; then
      kill "$EXTRA_BRINGUP_PID" >/dev/null 2>&1 || true
    fi
  }
  trap cleanup EXIT INT TERM

  if ! wait_for_health_ready; then
    echo "[FAIL] Health checks did not pass within retry window" | tee -a "$VERIFY_LOG"
    if [[ -n "${BRINGUP_PID:-}" ]]; then
      echo "[INFO] Bringup logs: /tmp/dstar_bringup.log" | tee -a "$VERIFY_LOG"
    fi
    if [[ -n "${EXTRA_BRINGUP_PID:-}" ]]; then
      echo "[INFO] Extra bringup logs: /tmp/dstar_extra_bringup.log" | tee -a "$VERIFY_LOG"
    fi
    return 1
  fi

  echo "[INFO] Topic list snapshot" | tee -a "$VERIFY_LOG"
  ros2 topic list 2>&1 | sort | tee -a "$VERIFY_LOG"

  echo "[INFO] Topic info snapshot" | tee -a "$VERIFY_LOG"
  for t in "$MAP_TOPIC" "$ODOM_TOPIC" "$SCAN_TOPIC" "$GOAL_TOPIC" "$PLAN_TOPIC"; do
    echo "[INFO] ros2 topic info $t" | tee -a "$VERIFY_LOG"
    ros2 topic info "$t" 2>&1 | tee -a "$VERIFY_LOG" || true
  done

  if ! sample_topic_rate "$MAP_TOPIC" "map" "required"; then
    verify_failed=1
  fi
  if ! sample_topic_rate "$ODOM_TOPIC" "odom" "required"; then
    verify_failed=1
  fi
  sample_topic_rate "$SCAN_TOPIC" "scan" "optional" || true

  run_node_cmd &
  NODE_PID=$!
  sleep "$WAIT_SECONDS"

  echo "[INFO] Publishing goal" | tee -a "$VERIFY_LOG"
  if ! publish_goal_once 2>&1 | tee -a "$VERIFY_LOG"; then
    verify_failed=1
  fi

  echo "[INFO] Reading plan output once" | tee -a "$VERIFY_LOG"
  plan_out="$(timeout 10s ros2 topic echo --once "$PLAN_TOPIC" 2>&1 || true)"
  echo "$plan_out" | tee -a "$VERIFY_LOG"

  if echo "$plan_out" | grep -Eq "poses:|frame_id|nav_msgs/msg/Path"; then
    echo "[PASS] Plan output detected" | tee -a "$VERIFY_LOG"
  else
    echo "[FAIL] Plan output not detected" | tee -a "$VERIFY_LOG"
    verify_failed=1
  fi

  if [[ "$verify_failed" -eq 0 ]]; then
    echo "[PASS] End-to-end verification PASSED" | tee -a "$VERIFY_LOG"
    return 0
  fi

  echo "[FAIL] End-to-end verification FAILED" | tee -a "$VERIFY_LOG"
  return 1
}

if [[ "$MODE" != "direct" && "$MODE" != "package" ]]; then
  echo "ERROR: Invalid mode '$MODE'. Use direct or package."
  exit 1
fi

if [[ "$ACTION" != "node" && "$ACTION" != "goal" && "$ACTION" != "plan" && "$ACTION" != "demo" && "$ACTION" != "health" && "$ACTION" != "oneclick" && "$ACTION" != "verify" ]]; then
  echo "ERROR: Invalid action '$ACTION'. Use node, goal, plan, demo, health, oneclick, or verify."
  exit 1
fi

if [[ "$MODE" == "package" ]]; then
  echo "[INFO] Preparing package mode workspace"
  prepare_package_and_env
fi

run_node_cmd() {
  if [[ "$MODE" == "direct" ]]; then
    run_node_direct
  else
    run_node_package
  fi
}

case "$ACTION" in
  node)
    echo "[INFO] Action=node: running planner"
    run_node_cmd
    ;;
  goal)
    echo "[INFO] Action=goal: publishing one goal on $GOAL_TOPIC"
    publish_goal_once
    ;;
  plan)
    echo "[INFO] Action=plan: reading one message from $PLAN_TOPIC"
    echo_plan_once
    ;;
  health)
    echo "[INFO] Action=health: checking map/odom/scan publishing"
    run_health_checks
    ;;
  demo)
    echo "[INFO] Action=demo: run planner + publish goal + read one plan"
    run_node_cmd &
    NODE_PID=$!

    cleanup() {
      if kill -0 "$NODE_PID" >/dev/null 2>&1; then
        kill "$NODE_PID" >/dev/null 2>&1 || true
      fi
    }
    trap cleanup EXIT INT TERM

    sleep "$WAIT_SECONDS"
    publish_goal_once

    # Wait briefly for planner output and print one plan message.
    sleep 1
    if ! echo_plan_once; then
      echo "[WARN] Could not read plan message. Ensure /map and /odom are being published."
      echo "[INFO] Run: bash run_dstar_ros2_linux.sh --action health --mode $MODE"
    fi

    echo "[INFO] Demo finished. Press Ctrl+C if node is still running."
    wait "$NODE_PID"
    ;;
  oneclick)
    echo "[INFO] Action=oneclick: running full automated flow"

    launch_ui_dashboards

    run_optional_bringup

    cleanup() {
      if [[ -n "${NODE_PID:-}" ]] && kill -0 "$NODE_PID" >/dev/null 2>&1; then
        kill "$NODE_PID" >/dev/null 2>&1 || true
      fi
      if [[ -n "${BRINGUP_PID:-}" ]] && kill -0 "$BRINGUP_PID" >/dev/null 2>&1; then
        kill "$BRINGUP_PID" >/dev/null 2>&1 || true
      fi
      if [[ -n "${EXTRA_BRINGUP_PID:-}" ]] && kill -0 "$EXTRA_BRINGUP_PID" >/dev/null 2>&1; then
        kill "$EXTRA_BRINGUP_PID" >/dev/null 2>&1 || true
      fi
    }
    trap cleanup EXIT INT TERM

    if ! wait_for_health_ready; then
      echo "[ERROR] Health checks failed after retries."
      echo "[INFO] If sim is not auto-started, rerun with --bringup-cmd."
      echo "[INFO] Example: --bringup-cmd \"ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py\""
      if [[ -n "${BRINGUP_PID:-}" ]]; then
        echo "[INFO] Bringup logs: /tmp/dstar_bringup.log"
      fi
      if [[ -n "${EXTRA_BRINGUP_PID:-}" ]]; then
        echo "[INFO] Extra bringup logs: /tmp/dstar_extra_bringup.log"
      fi
      exit 1
    fi

    run_node_cmd &
    NODE_PID=$!

    sleep "$WAIT_SECONDS"
    publish_goal_once

    sleep 1
    if ! echo_plan_once; then
      echo "[WARN] Could not read plan message. Check RViz/rqt_topic and rerun health."
      echo "[INFO] Run: bash run_dstar_ros2_linux.sh --action health --mode $MODE"
    fi

    echo "[INFO] One-click flow complete. Planner stays running for further goals. Press Ctrl+C to stop."
    wait "$NODE_PID"
    ;;
  verify)
    echo "[INFO] Action=verify: full end-to-end verification"
    run_verify_flow
    ;;
esac
