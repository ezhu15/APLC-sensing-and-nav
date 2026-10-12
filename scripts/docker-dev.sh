#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker compose --env-file "$repo_dir/.env" \
  -f "$repo_dir/docker/compose.yml" "$@"
