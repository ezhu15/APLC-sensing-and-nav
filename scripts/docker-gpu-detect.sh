#!/usr/bin/env bash
# Sourced by docker-gazebo.sh. Detection targets the local Linux Docker host.

detect_gazebo_gpu() {
  local drm_root="${1:-/sys/class/drm}" node vendor runtimes
  gazebo_mesa_node=""
  gazebo_nvidia_ready=false

  for node in "$drm_root"/renderD*; do
    [[ -r "$node/device/vendor" ]] || continue
    read -r vendor < "$node/device/vendor"
    case "$vendor" in
      0x8086|0x1002) gazebo_mesa_node="${node##*/}"; break ;;
    esac
  done

  if command -v nvidia-smi > /dev/null && nvidia-smi -L > /dev/null 2>&1; then
    runtimes="$(docker info --format '{{json .Runtimes}}')" || return 1
    if [[ "$runtimes" == *'"nvidia"'* ]]; then
      gazebo_nvidia_ready=true
    else
      echo "NVIDIA detected, but Docker's nvidia runtime is not configured." >&2
      echo "Install/configure NVIDIA Container Toolkit, or use Mesa/software rendering." >&2
    fi
  fi

  if [[ "$gazebo_nvidia_ready" == true ]]; then
    gazebo_renderer=nvidia
  elif [[ -n "$gazebo_mesa_node" ]]; then
    gazebo_renderer=mesa
  else
    gazebo_renderer=software
  fi
}
