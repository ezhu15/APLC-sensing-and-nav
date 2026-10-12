#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

renderer=auto
case "${1:-}" in
  --renderer)
    renderer="${2:-}"
    case "$renderer" in auto|nvidia|mesa|software) ;; *)
      echo "Usage: $0 [--renderer auto|nvidia|mesa|software] [command ...]" >&2; exit 1 ;;
    esac
    shift 2
    ;;
  --gpu) renderer=mesa; shift ;; # Compatibility with the original launcher.
  --help|-h)
    echo "Usage: $0 [--renderer auto|nvidia|mesa|software] [command ...]"
    echo "Default: auto (NVIDIA with Docker runtime, then Intel/AMD Mesa, then software)."
    exit 0
    ;;
esac

for tool in docker xauth; do
  command -v "$tool" > /dev/null || { echo "Required host tool missing: $tool" >&2; exit 1; }
done
if [[ -z "${DISPLAY:-}" ]]; then
  echo "Run from a desktop terminal with DISPLAY set (X11 or XWayland)." >&2
  exit 1
fi

compose_args=(--env-file "$repo_dir/.env" -f docker/compose.yml -f docker/compose.gazebo.yml)
# shellcheck source=scripts/docker-gpu-detect.sh
source "$repo_dir/scripts/docker-gpu-detect.sh"
if [[ "$renderer" != software ]]; then
  detect_gazebo_gpu
  if [[ "$renderer" == auto ]]; then renderer="$gazebo_renderer"; fi
fi
case "$renderer" in
  nvidia)
    if [[ "$gazebo_nvidia_ready" != true ]]; then
      echo "NVIDIA requires a working nvidia-smi and Docker's nvidia runtime." >&2
      echo "See https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html" >&2
      exit 1
    fi
    compose_args+=(-f docker/compose.gazebo.nvidia.yml)
    ;;
  mesa)
    if [[ -z "$gazebo_mesa_node" || ! -e "/dev/dri/$gazebo_mesa_node" ]]; then
      echo "No Intel/AMD render device available; try --renderer software." >&2
      exit 1
    fi
    export GAZEBO_RENDER_NODE="$gazebo_mesa_node"
    export GAZEBO_RENDER_DEVICE="/dev/dri/$gazebo_mesa_node"
    export GAZEBO_RENDER_GID="$(stat -c %g "$GAZEBO_RENDER_DEVICE")"
    compose_args+=(-f docker/compose.gazebo.gpu.yml)
    ;;
esac
printf 'Gazebo renderer: %s\n' "$renderer"

# Give this container a private copy of the current display cookie. Wildcard
# address family allows authentication from a different container hostname.
umask 077
GAZEBO_XAUTHORITY="$(mktemp /tmp/aplc-gazebo-xauth.XXXXXX)"
export GAZEBO_XAUTHORITY
trap 'rm -f -- "$GAZEBO_XAUTHORITY"' EXIT
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$GAZEBO_XAUTHORITY" nmerge -
if [[ ! -s "$GAZEBO_XAUTHORITY" ]]; then
  echo "No X11 cookie found. Check DISPLAY and XAUTHORITY in your desktop terminal." >&2
  exit 1
fi

# Extra arguments replace the default Gazebo command, e.g. 'bash'.
docker compose "${compose_args[@]}" run --rm gazebo "$@"
