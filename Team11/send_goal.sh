#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$#" -ne 3 ]; then
  echo "Usage: ./send_goal.sh original|improved|both X Y"
  exit 2
fi
mode="${1,,}"
source /opt/ros/jazzy/setup.bash
case "$mode" in
  original)
    export ROS_DOMAIN_ID=41
    source "$ROOT/Original/install/setup.bash"
    python3 "$ROOT/tools/send_goal.py" "$2" "$3";;
  improved)
    export ROS_DOMAIN_ID=42
    source "$ROOT/Improved/install/setup.bash"
    python3 "$ROOT/tools/send_goal.py" "$2" "$3";;
  both)
    export ROS_DOMAIN_ID=41
    source "$ROOT/Original/install/setup.bash"
    python3 "$ROOT/tools/send_goal.py" "$2" "$3"
    export ROS_DOMAIN_ID=42
    source "$ROOT/Improved/install/setup.bash"
    python3 "$ROOT/tools/send_goal.py" "$2" "$3";;
  *)echo "choose original / improved / both";exit 2;;
esac
