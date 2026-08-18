#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W 右腕相机与 AprilTag 读取适配器。

本文件只读取视觉数据，不发布机械臂、底盘或夹爪控制指令。运行前必须在上位机
``leju_kuavo`` 的不同终端依次完成以下操作。

1. 启动头部及左右腕相机：

       cd ~/kuavo_ros_application
       source /opt/ros/noetic/setup.bash
       source devel/setup.bash
       roslaunch dynamic_biped load_robot_head.launch \
         use_orbbec:=true enable_wrist_camera:=true all_enable:=false

2. 启动右腕 AprilTag 检测器：

       cd ~/kuavo_ros_application
       source /opt/ros/noetic/setup.bash
       source devel/setup.bash
       ROS_NAMESPACE=/apriltag_cam_r \
       roslaunch apriltag_ros continuous_detection.launch \
         camera_name:=/right_wrist_camera/color image_topic:=image_raw

3. 运行本项目程序的终端也必须加载上位机工作空间，否则 Python 找不到
   ``apriltag_ros`` 消息：

       cd ~/5W_control
       source /opt/ros/noetic/setup.bash
       source ~/kuavo_ros_application/devel/setup.bash

现场已确认的接口：

* 彩色图像：``/right_wrist_camera/color/image_raw``，约 30 Hz，848x480；
* 相机内参：``/right_wrist_camera/color/camera_info``；
* 标签结果：``/apriltag_cam_r/tag_detections``；
* 消息类型：``apriltag_ros/AprilTagDetectionArray``；
* 标签 0：``tag36h11``，配置边长 0.042 m；
* 坐标系：``right_wrist_camera_color_optical_frame``。

本适配器直接返回标准相机光学坐标：x 向图像右侧、y 向图像下方、z 向相机
前方。旧机器的 ``CAMERA_ROTATION=90``、``CAMERA_INVERTED=True`` 只适用于旧
安装方式，不能在新机器人上继续使用。标签边长配置必须等于实物有效边长，否则
三维位置会按比例失真。
"""

import math
import statistics
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rosgraph
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo


RIGHT_IMAGE_TOPIC = "/right_wrist_camera/color/image_raw"
RIGHT_CAMERA_INFO_TOPIC = "/right_wrist_camera/color/camera_info"
RIGHT_TAG_TOPIC = "/apriltag_cam_r/tag_detections"
RIGHT_OPTICAL_FRAME = "right_wrist_camera_color_optical_frame"


@dataclass(frozen=True)
class TagPose:
    """一帧 AprilTag 位姿，位置单位为米，姿态为四元数 xyzw。"""

    tag_id: int
    tag_size: float
    frame_id: str
    stamp_sec: float
    sequence: int
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    received_monotonic: float

    def validate(self, expected_frame: Optional[str] = None) -> None:
        values = self.position + self.orientation + (self.tag_size, self.stamp_sec)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("tag pose contains a non-finite value")
        if self.tag_size <= 0.0:
            raise ValueError("tag size must be positive")
        if self.position[2] <= 0.0:
            raise ValueError("tag must be in front of the optical frame")
        norm = math.sqrt(sum(value * value for value in self.orientation))
        if norm <= 1e-9:
            raise ValueError("tag orientation quaternion is invalid")
        if expected_frame and self.frame_id != expected_frame:
            raise ValueError(
                "unexpected tag frame: {!r}, expected {!r}".format(
                    self.frame_id,
                    expected_frame,
                )
            )

    def age(self) -> float:
        """返回本进程收到该帧后经过的单调时钟时间。"""
        return max(0.0, time.monotonic() - self.received_monotonic)

    def as_dict(self) -> dict:
        return {
            "tag_id": self.tag_id,
            "tag_size": self.tag_size,
            "frame_id": self.frame_id,
            "stamp_sec": self.stamp_sec,
            "sequence": self.sequence,
            "position": {
                "x": self.position[0],
                "y": self.position[1],
                "z": self.position[2],
            },
            "orientation": {
                "x": self.orientation[0],
                "y": self.orientation[1],
                "z": self.orientation[2],
                "w": self.orientation[3],
            },
            "age_sec": self.age(),
        }


@dataclass(frozen=True)
class TagPoseEstimate:
    """多帧过滤后的标签位姿估计。"""

    tag_id: int
    tag_size: float
    frame_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    stddev: Tuple[float, float, float]
    sample_count: int
    inlier_count: int
    rejected_count: int

    def as_dict(self) -> dict:
        return {
            "tag_id": self.tag_id,
            "tag_size": self.tag_size,
            "frame_id": self.frame_id,
            "position": {
                "x": self.position[0],
                "y": self.position[1],
                "z": self.position[2],
            },
            "orientation": {
                "x": self.orientation[0],
                "y": self.orientation[1],
                "z": self.orientation[2],
                "w": self.orientation[3],
            },
            "stddev": {
                "x": self.stddev[0],
                "y": self.stddev[1],
                "z": self.stddev[2],
            },
            "sample_count": self.sample_count,
            "inlier_count": self.inlier_count,
            "rejected_count": self.rejected_count,
        }


class VisionAdapter:
    """右腕 AprilTag 只读接口，支持新鲜度检查和多帧鲁棒平均。"""

    def __init__(
        self,
        image_topic: str = RIGHT_IMAGE_TOPIC,
        camera_info_topic: str = RIGHT_CAMERA_INFO_TOPIC,
        tag_topic: str = RIGHT_TAG_TOPIC,
        expected_frame: str = RIGHT_OPTICAL_FRAME,
        max_data_age: float = 0.5,
    ):
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.tag_topic = tag_topic
        self.expected_frame = expected_frame
        self.max_data_age = max(0.01, float(max_data_age))

        self._condition = threading.Condition()
        self._latest_by_id: Dict[int, TagPose] = {}
        self._generation_by_id: Dict[int, int] = {}
        self._empty_message_count = 0
        self._message_count = 0

        self._subscriber = rospy.Subscriber(
            self.tag_topic,
            AprilTagDetectionArray,
            self._tag_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

    def _tag_callback(self, message: AprilTagDetectionArray) -> None:
        received_at = time.monotonic()
        with self._condition:
            self._message_count += 1
            if not message.detections:
                self._empty_message_count += 1

            for detection in message.detections:
                if not detection.id:
                    continue
                position = detection.pose.pose.pose.position
                orientation = detection.pose.pose.pose.orientation
                frame_id = detection.pose.header.frame_id or message.header.frame_id
                stamp = detection.pose.header.stamp
                if stamp.is_zero():
                    stamp = message.header.stamp

                for index, tag_id in enumerate(detection.id):
                    if detection.size:
                        size_index = min(index, len(detection.size) - 1)
                        tag_size = float(detection.size[size_index])
                    else:
                        tag_size = 0.0
                    pose = TagPose(
                        tag_id=int(tag_id),
                        tag_size=tag_size,
                        frame_id=frame_id,
                        stamp_sec=stamp.to_sec(),
                        sequence=int(detection.pose.header.seq),
                        position=(float(position.x), float(position.y), float(position.z)),
                        orientation=(
                            float(orientation.x),
                            float(orientation.y),
                            float(orientation.z),
                            float(orientation.w),
                        ),
                        received_monotonic=received_at,
                    )
                    try:
                        pose.validate(self.expected_frame)
                    except ValueError as exc:
                        rospy.logwarn_throttle(2.0, "忽略无效 AprilTag 位姿: %s", exc)
                        continue
                    self._latest_by_id[pose.tag_id] = pose
                    self._generation_by_id[pose.tag_id] = (
                        self._generation_by_id.get(pose.tag_id, 0) + 1
                    )
            self._condition.notify_all()

    def readiness(self) -> dict:
        """只读检查右腕图像、内参和 AprilTag 话题连接。"""
        try:
            master = rosgraph.Master(rospy.get_name())
            publishers, subscribers, _ = master.getSystemState()
            topic_types = dict(master.getTopicTypes())
        except Exception as exc:
            return {
                "ready": False,
                "reasons": ["ros_master_unavailable"],
                "error": repr(exc),
            }

        publisher_map = dict(publishers)
        subscriber_map = dict(subscribers)
        image_publishers = publisher_map.get(self.image_topic, [])
        camera_info_publishers = publisher_map.get(self.camera_info_topic, [])
        tag_publishers = publisher_map.get(self.tag_topic, [])
        tag_subscribers = subscriber_map.get(self.tag_topic, [])

        reasons = []
        if not image_publishers:
            reasons.append("right_wrist_image_has_no_publisher")
        if topic_types.get(self.image_topic) != "sensor_msgs/Image":
            reasons.append("right_wrist_image_type_mismatch")
        if not camera_info_publishers:
            reasons.append("right_wrist_camera_info_has_no_publisher")
        if topic_types.get(self.camera_info_topic) != "sensor_msgs/CameraInfo":
            reasons.append("right_wrist_camera_info_type_mismatch")
        if not tag_publishers:
            reasons.append("apriltag_detections_has_no_publisher")
        if topic_types.get(self.tag_topic) != "apriltag_ros/AprilTagDetectionArray":
            reasons.append("apriltag_detections_type_mismatch")
        if self._subscriber.get_num_connections() <= 0:
            reasons.append("apriltag_subscriber_not_connected")

        with self._condition:
            latest_ids = sorted(self._latest_by_id)
            message_count = self._message_count
            empty_message_count = self._empty_message_count

        return {
            "ready": not reasons,
            "reasons": reasons,
            "image_topic": self.image_topic,
            "image_type": topic_types.get(self.image_topic),
            "image_publishers": image_publishers,
            "camera_info_topic": self.camera_info_topic,
            "camera_info_type": topic_types.get(self.camera_info_topic),
            "camera_info_publishers": camera_info_publishers,
            "tag_topic": self.tag_topic,
            "tag_type": topic_types.get(self.tag_topic),
            "tag_publishers": tag_publishers,
            "tag_subscribers": tag_subscribers,
            "expected_frame": self.expected_frame,
            "message_count": message_count,
            "empty_message_count": empty_message_count,
            "latest_tag_ids": latest_ids,
        }

    def wait_until_ready(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while not rospy.is_shutdown():
            if self.readiness().get("ready"):
                return True
            if time.monotonic() >= deadline:
                return False
            rospy.sleep(0.1)
        return False

    def read_camera_info(self, timeout: float = 3.0) -> dict:
        """等待并返回一帧相机内参，不订阅高带宽原始图像。"""
        message = rospy.wait_for_message(
            self.camera_info_topic,
            CameraInfo,
            timeout=max(0.01, float(timeout)),
        )
        return {
            "frame_id": message.header.frame_id,
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": message.distortion_model,
            "D": list(message.D),
            "K": list(message.K),
            "R": list(message.R),
            "P": list(message.P),
        }

    def latest_tag(self, tag_id: int, max_age: Optional[float] = None) -> Optional[TagPose]:
        """返回指定ID的最新有效帧；过期或尚未检测到时返回 ``None``。"""
        age_limit = self.max_data_age if max_age is None else max(0.0, float(max_age))
        with self._condition:
            pose = self._latest_by_id.get(int(tag_id))
        if pose is None or pose.age() > age_limit:
            return None
        return pose

    def wait_for_tag(
        self,
        tag_id: int,
        timeout: float = 5.0,
        max_age: Optional[float] = None,
    ) -> Optional[TagPose]:
        """等待指定标签的一帧新鲜有效数据。"""
        deadline = time.monotonic() + max(0.0, float(timeout))
        tag_id = int(tag_id)
        with self._condition:
            while not rospy.is_shutdown():
                pose = self._latest_by_id.get(tag_id)
                age_limit = self.max_data_age if max_age is None else float(max_age)
                if pose is not None and pose.age() <= max(0.0, age_limit):
                    return pose
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(min(remaining, 0.2))
        return None

    def collect_samples(
        self,
        tag_id: int,
        count: int = 10,
        timeout: float = 5.0,
    ) -> List[TagPose]:
        """收集指定ID的连续新帧，确保同一帧不会被重复计数。"""
        if count <= 0:
            raise ValueError("sample count must be positive")

        tag_id = int(tag_id)
        deadline = time.monotonic() + max(0.0, float(timeout))
        samples: List[TagPose] = []
        with self._condition:
            generation = self._generation_by_id.get(tag_id, 0)
            while len(samples) < count and not rospy.is_shutdown():
                current_generation = self._generation_by_id.get(tag_id, 0)
                pose = self._latest_by_id.get(tag_id)
                if (
                    pose is not None
                    and current_generation > generation
                    and pose.age() <= self.max_data_age
                ):
                    samples.append(pose)
                    generation = current_generation
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(min(remaining, 0.2))
        return samples

    def estimate_tag_pose(
        self,
        tag_id: int,
        count: int = 10,
        timeout: float = 5.0,
        min_inliers: int = 3,
        minimum_outlier_radius: float = 0.001,
    ) -> TagPoseEstimate:
        """连续采样并以中位数/MAD剔除位置跳点，再平均位置和姿态。"""
        samples = self.collect_samples(tag_id=tag_id, count=count, timeout=timeout)
        if len(samples) < max(1, int(min_inliers)):
            raise TimeoutError(
                "only received {} valid samples for tag {}, need at least {}".format(
                    len(samples),
                    tag_id,
                    max(1, int(min_inliers)),
                )
            )

        medians = tuple(
            statistics.median(sample.position[axis] for sample in samples)
            for axis in range(3)
        )
        distances = [
            math.sqrt(
                sum(
                    (sample.position[axis] - medians[axis]) ** 2
                    for axis in range(3)
                )
            )
            for sample in samples
        ]
        median_distance = statistics.median(distances)
        mad = statistics.median(abs(value - median_distance) for value in distances)
        threshold = median_distance + max(
            3.0 * 1.4826 * mad,
            max(0.0, float(minimum_outlier_radius)),
        )
        inliers = [
            sample for sample, distance in zip(samples, distances) if distance <= threshold
        ]
        if len(inliers) < max(1, int(min_inliers)):
            raise ValueError(
                "too many position outliers: {} of {} samples remain".format(
                    len(inliers),
                    len(samples),
                )
            )

        position = tuple(
            statistics.fmean(sample.position[axis] for sample in inliers)
            for axis in range(3)
        )
        stddev = tuple(
            statistics.pstdev(sample.position[axis] for sample in inliers)
            if len(inliers) > 1
            else 0.0
            for axis in range(3)
        )
        orientation = self._average_quaternions(
            [sample.orientation for sample in inliers]
        )
        tag_size = statistics.median(sample.tag_size for sample in inliers)

        return TagPoseEstimate(
            tag_id=int(tag_id),
            tag_size=float(tag_size),
            frame_id=inliers[-1].frame_id,
            position=position,
            orientation=orientation,
            stddev=stddev,
            sample_count=len(samples),
            inlier_count=len(inliers),
            rejected_count=len(samples) - len(inliers),
        )

    @staticmethod
    def _average_quaternions(
        quaternions: Sequence[Tuple[float, float, float, float]],
    ) -> Tuple[float, float, float, float]:
        if not quaternions:
            raise ValueError("cannot average an empty quaternion sequence")
        reference = quaternions[0]
        aligned = []
        for quaternion in quaternions:
            dot = sum(a * b for a, b in zip(reference, quaternion))
            aligned.append(
                quaternion if dot >= 0.0 else tuple(-value for value in quaternion)
            )
        averaged = tuple(
            statistics.fmean(quaternion[axis] for quaternion in aligned)
            for axis in range(4)
        )
        norm = math.sqrt(sum(value * value for value in averaged))
        if norm <= 1e-9:
            raise ValueError("averaged quaternion is invalid")
        return tuple(value / norm for value in averaged)
