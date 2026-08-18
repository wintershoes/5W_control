#!/usr/bin/env bash
# Kuavo 5-W vision interface snapshot (read-only inspection).
#
# Run this script on the upper computer (leju_kuavo). It only reads system and
# ROS state. It does not launch nodes, publish topics, call services, or change
# parameters. The report is always overwritten at:
#   ~/robot_vision_interface_snapshot.txt

set -o pipefail

OUTPUT="$HOME/robot_vision_interface_snapshot.txt"

# Load known ROS environments when present. Sourcing setup files only changes
# this script process; it does not modify robot configuration.
source /opt/ros/noetic/setup.bash 2>/dev/null || true
if [ -f "$HOME/kuavo_ros_application/devel/setup.bash" ]; then
    source "$HOME/kuavo_ros_application/devel/setup.bash" 2>/dev/null || true
fi

section() {
    printf '\n===== %s =====\n' "$1"
}

command_path() {
    if command -v "$1" >/dev/null 2>&1; then
        command -v "$1"
    else
        echo "NOT FOUND"
    fi
}

topic_exists() {
    grep -Fxq "$1" <<<"$ROS_TOPICS"
}

show_topic() {
    local topic="$1"
    echo
    echo "--- TOPIC: $topic ---"
    if topic_exists "$topic"; then
        timeout 5 rostopic type "$topic" 2>&1
        timeout 5 rostopic info "$topic" 2>&1
    else
        echo "NOT PRESENT"
    fi
}

sample_topic() {
    local topic="$1"
    local seconds="${2:-5}"
    echo
    echo "--- SAMPLE: $topic (timeout ${seconds}s) ---"
    if topic_exists "$topic"; then
        timeout "$seconds" rostopic echo -n 1 "$topic" 2>&1 ||
            echo "NO MESSAGE WITHIN ${seconds}s"
    else
        echo "NOT PRESENT"
    fi
}

measure_topic() {
    local topic="$1"
    echo
    echo "--- RATE: $topic (4s sample) ---"
    if topic_exists "$topic"; then
        timeout 4 rostopic hz "$topic" 2>&1 || true
    else
        echo "NOT PRESENT"
    fi
}

show_tf() {
    local parent="$1"
    local child="$2"
    echo
    echo "--- TF: $parent -> $child (4s sample) ---"
    timeout 4 rosrun tf tf_echo "$parent" "$child" 2>&1 ||
        echo "TF NOT CONFIRMED"
}

{
    section "采集信息"
    date --iso-8601=seconds 2>/dev/null || date
    echo "user=$(whoami)"
    echo "hostname=$(hostname)"
    echo "output=$OUTPUT"
    echo "ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"
    echo "ROS_IP=${ROS_IP:-<unset>}"
    echo "ROS_HOSTNAME=${ROS_HOSTNAME:-<unset>}"
    echo "ROBOT_VERSION=${ROBOT_VERSION:-<unset>}"
    echo "HEAD_CAMERA_SERIAL_NO=${HEAD_CAMERA_SERIAL_NO:-<unset>}"
    echo "WAIST_CAMERA_SERIAL_NO=${WAIST_CAMERA_SERIAL_NO:-<unset>}"
    echo "LEFT_WRIST_CAMERA_SERIAL_NO=${LEFT_WRIST_CAMERA_SERIAL_NO:-<unset>}"
    echo "RIGHT_WRIST_CAMERA_SERIAL_NO=${RIGHT_WRIST_CAMERA_SERIAL_NO:-<unset>}"

    section "工具与ROS连接"
    echo "rostopic=$(command_path rostopic)"
    echo "rosnode=$(command_path rosnode)"
    echo "rospack=$(command_path rospack)"
    echo "rs-enumerate-devices=$(command_path rs-enumerate-devices)"
    ROS_CONNECT_OUTPUT="$(timeout 5 rosnode list 2>&1)"
    ROS_STATUS=$?
    if [ "$ROS_STATUS" -eq 0 ]; then
        echo "ROS master: reachable"
    else
        echo "ROS master: unavailable or timed out"
        sed -n '1,40p' <<<"$ROS_CONNECT_OUTPUT"
    fi

    section "USB与RealSense设备"
    if command -v lsusb >/dev/null 2>&1; then
        lsusb 2>&1 | grep -Ei 'Intel|RealSense|Orbbec|Gemini|camera|video' ||
            echo "No matching USB camera description"
    else
        echo "lsusb NOT FOUND"
    fi
    if command -v rs-enumerate-devices >/dev/null 2>&1; then
        timeout 8 rs-enumerate-devices -s 2>&1 ||
            echo "RealSense enumeration unavailable or timed out"
    fi
    echo
    echo "Video devices:"
    ls -l /dev/video* 2>&1 || true
    echo
    echo "Stable video links:"
    ls -l /dev/v4l/by-id /dev/v4l/by-path 2>&1 || true

    section "相机与检测进程"
    ps -ef | grep -Ei \
        'realsense|orbbec|gemini|camera_node|camera_driver|apriltag|tag_detection|ar_control|yolo' |
        grep -v grep || echo "No matching process"

    section "相关ROS包"
    for pkg in realsense2_camera orbbec_camera apriltag_ros detection_apriltag dynamic_biped; do
        printf '%-24s ' "$pkg"
        rospack find "$pkg" 2>&1 || true
    done

    section "相关ROS节点"
    ROS_NODES="$(timeout 5 rosnode list 2>/dev/null || true)"
    grep -Ei 'camera|realsense|orbbec|apriltag|tag|vision|yolo|ar_control' <<<"$ROS_NODES" ||
        echo "No matching ROS node"

    section "相机与AprilTag话题清单"
    ROS_TOPICS="$(timeout 5 rostopic list 2>/dev/null || true)"
    grep -Ei 'camera|image|depth|infra|tag|apriltag|robot_tag|camera_info' <<<"$ROS_TOPICS" ||
        echo "No matching ROS topic"

    section "关键话题连接"
    for topic in \
        /right_wrist_camera/color/image_raw \
        /right_wrist_camera/color/camera_info \
        /right_wrist_camera/aligned_depth_to_color/image_raw \
        /left_wrist_camera/color/image_raw \
        /left_wrist_camera/color/camera_info \
        /camera/color/image_raw \
        /camera/color/camera_info \
        /camera_1/color/image_raw \
        /camera_1/color/camera_info \
        /camera_2/color/image_raw \
        /camera_2/color/camera_info \
        /apriltag_cam_r/tag_detections \
        /tag_detections \
        /robot_tag_info \
        /robot_tag_info_odom; do
        show_topic "$topic"
    done

    section "图像频率"
    for topic in \
        /right_wrist_camera/color/image_raw \
        /left_wrist_camera/color/image_raw \
        /camera/color/image_raw \
        /camera_1/color/image_raw \
        /camera_2/color/image_raw; do
        measure_topic "$topic"
    done

    section "相机内参样本"
    sample_topic /right_wrist_camera/color/camera_info 5
    sample_topic /left_wrist_camera/color/camera_info 5
    sample_topic /camera/color/camera_info 5
    sample_topic /camera_1/color/camera_info 3
    sample_topic /camera_2/color/camera_info 3

    section "AprilTag样本"
    sample_topic /apriltag_cam_r/tag_detections 5
    sample_topic /tag_detections 5
    sample_topic /robot_tag_info 5
    sample_topic /robot_tag_info_odom 5

    section "腕部相机TF"
    show_tf base_link right_wrist_camera_color_optical_frame
    show_tf base_link right_wrist_camera_link
    show_tf base_link left_wrist_camera_color_optical_frame
    show_tf base_link left_wrist_camera_link

    section "TF话题连接"
    show_topic /tf
    show_topic /tf_static

    section "结论提示"
    echo "1. 右腕图像和camera_info必须同时存在，AprilTag才能可靠计算三维位姿。"
    echo "2. /tag_detections通常是相机坐标系；/robot_tag_info通常是机器人坐标系。"
    echo "3. 本报告只确认接口是否存在，不会启动或修改任何相机/检测节点。"
} >"$OUTPUT" 2>&1

echo "saved: $OUTPUT"
echo "lines: $(wc -l <"$OUTPUT")"
