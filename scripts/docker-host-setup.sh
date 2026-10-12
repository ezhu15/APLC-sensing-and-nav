#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" == 0 ]]; then
  echo "Run this script as your regular host user, without sudo." >&2
  exit 1
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
env_file="$repo_dir/.env"

# Replace only the settings managed by this script.
umask 077
temp_file="$(mktemp "$repo_dir/.env.tmp.XXXXXX")"
trap 'rm -f -- "$temp_file"' EXIT

if [[ -f "$env_file" ]]; then
  awk '!/^[[:space:]]*(export[[:space:]]+)?LOCAL_(UID|GID)[[:space:]]*=/' \
    "$env_file" > "$temp_file"
fi

printf 'LOCAL_UID=%s\nLOCAL_GID=%s\n' "$(id -u)" "$(id -g)" >> "$temp_file"
mv -- "$temp_file" "$env_file"

printf 'Configured %s for host UID:GID %s:%s\n' "$env_file" "$(id -u)" "$(id -g)"
printf 'From the repository root, run:\n  bash scripts/docker-dev.sh build\n  bash scripts/docker-dev.sh run --rm dev\n'
