#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-lg}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CONFIG="${CONFIG:-src/lightning-lm-deep-robotics/config/default_deep_robotics.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-src/lightning-lm-deep-robotics/config/showbodypc.rviz}"
START_RVIZ="${START_RVIZ:-1}"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux attach-session -t "${SESSION}"
    exit 0
fi

cd "${WS_ROOT}"

if [[ -n "${ROS_SETUP:-}" ]]; then
    ros_setup="${ROS_SETUP}"
elif [[ -f /opt/robot/scripts/setup_ros2.sh ]]; then
    ros_setup="/opt/robot/scripts/setup_ros2.sh"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    ros_setup="/opt/ros/foxy/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ros_setup="/opt/ros/humble/setup.bash"
else
    echo "Cannot find a ROS 2 setup file. Set ROS_SETUP=/path/to/setup.bash and retry." >&2
    exit 1
fi

setup_cmd="source \"${ros_setup}\""
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    setup_cmd="${setup_cmd} && source \"${WS_ROOT}/install/setup.bash\""
fi

slam_cmd="cd \"${WS_ROOT}\" && ${setup_cmd} && ros2 run lightning run_slam_online --config \"${CONFIG}\""
rviz_cmd="cd \"${WS_ROOT}\" && ${setup_cmd} && rviz2 -d \"${RVIZ_CONFIG}\""
navstate_cmd="cd \"${WS_ROOT}\" && ${setup_cmd} && sleep 5 && ros2 topic echo /lightning/nav_state"
log_cmd="cd \"${WS_ROOT}\" && ${setup_cmd} && python3 src/lightning-lm-deep-robotics/scripts/ros2_log_navstate.py"
shell_cmd="cd \"${WS_ROOT}\" && ${setup_cmd} && exec bash"

tmux new-session -d -s "${SESSION}" -n "lightning"
tmux send-keys -t "${SESSION}:0" "${slam_cmd}" C-m

tmux new-window -t "${SESSION}:" -n "vis"
if [[ "${START_RVIZ}" == "1" ]]; then
    tmux send-keys -t "${SESSION}:vis" "${rviz_cmd}" C-m
else
    tmux send-keys -t "${SESSION}:vis" "${shell_cmd}" C-m
fi

tmux new-window -t "${SESSION}:" -n "navstate"
tmux send-keys -t "${SESSION}:navstate" "${navstate_cmd}" C-m

tmux new-window -t "${SESSION}:" -n "log"
tmux send-keys -t "${SESSION}:log" "${log_cmd}" C-m

tmux new-window -t "${SESSION}:" -n "node_info"
tmux send-keys -t "${SESSION}:node_info" "${shell_cmd}" C-m

tmux attach-session -t "${SESSION}"
