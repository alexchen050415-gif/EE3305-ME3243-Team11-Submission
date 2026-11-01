#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/Original"
export ROS_DOMAIN_ID=41
export GZ_PARTITION=ee3305_original
#保留原始ROS2项目和启动方式；只额外运行一个只读记录器
source /opt/ros/jazzy/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[Original] First run: building its own workspace."
  (cd "$WS" && colcon build --symlink-install)
fi
source "$WS/install/setup.bash"
echo "[Original] ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=$GZ_PARTITION"
python3 -u "$ROOT/tools/compare_record.py" --variant Original --output "$ROOT/results" &
REC_PID=$!
cleanup() {
  kill -TERM "$REC_PID" 2>/dev/null || true
  wait "$REC_PID" 2>/dev/null || true
}
trap cleanup EXIT
cd "$WS"
env -u SVGA_VGPU10 -u SVGA_GL43 \
  ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=True libgl:=False "$@"
