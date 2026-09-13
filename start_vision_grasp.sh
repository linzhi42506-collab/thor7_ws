#!/usr/bin/env bash
# Starts all components. Automatic grasp still requires an explicit service call.
set -e
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_mode=plan
task_auto_grasp=false
task_pick_place=false
task_loop=false
while [[ "${1:-}" == --* ]]; do
  if [[ "$1" == "--execute" ]]; then
    task_mode=execute
  elif [[ "$1" == "--auto-grasp" ]]; then
    task_mode=execute
    task_auto_grasp=true
  elif [[ "$1" == "--pick-place" ]]; then
    task_mode=execute
    task_auto_grasp=true
    task_pick_place=true
  elif [[ "$1" == "--auto-loop" ]]; then
    task_mode=execute
    task_auto_grasp=true
    task_pick_place=true
    task_loop=true
  elif [[ "$1" == "--preview-pick-place" ]]; then
    task_pick_place=true
  elif [[ "$1" == "--help" ]]; then
  echo 'Usage: bash start_vision_grasp.sh [--execute|--auto-grasp|--pick-place|--auto-loop|--preview-pick-place] [name:=value ...]'
  echo 'Default: real read-only feedback and planning. --execute: enable real controllers.'
  echo '--auto-grasp: enable one-trigger open, approach, descend, and close service.'
  echo '--pick-place: enable one cycle grasp, lift, place, open, retreat, Home; call run_pick_place.'
  echo '--auto-loop: enable repeated grasp/place/Home cycles; call run_pick_place once.'
  echo '--preview-pick-place: read-only hardware, enable preview_place_home planning service.'
  echo 'No mode moves until a prepare/execute or run_grasp service is called.'
  echo 'In execute mode the gripper may move during initialization.'
  exit 0
  else
    echo "Unknown option: $1" >&2
    exit 2
  fi
  shift
done
source /opt/ros/humble/setup.bash
source "$task_root/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# Prevent duplicate starts through this wrapper. Manually started nodes must
# still be stopped normally before switching to this all-in-one launch.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/thor7-vision-grasp-${UID}.lock"
flock -n 9 || { echo 'The THOR7 all-in-one launcher is already running.' >&2; exit 1; }
task_confirmation=DENIED
if [[ "$task_mode" == execute ]]; then task_confirmation=ARM_THOR7; fi
task_auto_confirmation=DENIED
if [[ "$task_auto_grasp" == true ]]; then task_auto_confirmation=ARM_THOR7_AUTO_GRASP; fi
task_retain_placed=true
if [[ "$task_loop" == true ]]; then task_retain_placed=false; fi
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch thor7_moveit_demo \
  thor7_vision_grasp_all.launch.py "workspace_root:=$task_root" \
  "mode:=$task_mode" "operator_confirmation:=$task_confirmation" \
  "auto_grasp:=$task_auto_grasp" \
  "pick_place:=$task_pick_place" \
  "loop:=$task_loop" \
  "retain_placed_objects:=$task_retain_placed" \
  "auto_grasp_confirmation:=$task_auto_confirmation" "$@"
