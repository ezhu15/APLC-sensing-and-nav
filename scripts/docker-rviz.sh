#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
renderer=auto
quest_network=false
live=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --renderer) renderer="${2:?Provide auto, software, mesa, or nvidia}"; shift 2 ;;
    --quest-network) quest_network=true; shift ;;
    --live) live=true; quest_network=true; shift ;;
    --help|-h)
      echo "Usage: $0 [--renderer auto|software|mesa|nvidia] [--quest-network] [--live] [ROS launch arguments]"
      echo "--quest-network publishes receiver TCP ports 5005/5006; a receiver must also be running."
      echo "--live starts the ROS Quest bridge and publishes both receiver ports."
      exit 0 ;;
    *) break ;;
  esac
done
if [[ "$quest_network" == true ]]; then
  if [[ -n "${APLC_GAZEBO_COMPOSE_OVERRIDE:-}" ]]; then
    echo 'Unset APLC_GAZEBO_COMPOSE_OVERRIDE before using --quest-network.' >&2
    exit 1
  fi
  export APLC_GAZEBO_COMPOSE_OVERRIDE="$repo_dir/docker/compose.quest-network.yml"
  export APLC_DOCKER_SERVICE_PORTS=true
fi
if [[ "$live" == true ]]; then set -- "$@" live:=true; fi
exec bash "$repo_dir/scripts/docker-gazebo.sh" --renderer "$renderer" \
  bash -c 'set -e; source /ws/install/setup.bash; exec ros2 launch aplc_scene_ros view.launch.py "$@"' bash "$@"
