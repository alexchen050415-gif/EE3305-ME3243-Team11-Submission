#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
for variant in Original Improved; do
  echo "Building $variant ..."
  (cd "$ROOT/$variant" && colcon build --symlink-install)
done
printf 'Both isolated workspaces compiled.\n'
