#!/usr/bin/env python3
"""Passively record short zero-command hardware trots after a policy enters."""

from bisect import bisect_right
from datetime import datetime
import math
from pathlib import Path
import sys
import time

import rclpy
from control_input_msgs.msg import Inputs
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage


PERIOD_S = 0.650
HALF_PERIOD_S = PERIOD_S / 2
START_DELAY_S = 0.5
DURATION_S = 10.0
ACTIVE_GAP_S = 0.25
MESSAGE_ORDER_GRACE_S = 0.08
STAMP_SKEW_NS = 20_000_000
SAMPLE_TIMES = (*(i * HALF_PERIOD_S for i in range(math.ceil(DURATION_S / HALF_PERIOD_S))), DURATION_S)
JOINTS = tuple(f"{leg}_{joint}" for leg in ("FL", "FR", "RL", "RR")
               for joint in ("hip", "thigh", "calf"))
HEADER = ("t_since_policy_s", "phase_approx", "inekf_x_cm", "inekf_y_cm", "pitch_deg", "stamp_skew_ms",
          *(f"{prefix}_{joint}_deg" for prefix in ("target", "actual") for joint in JOINTS))


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class HoldCapture(Node):
    def __init__(self, log_dir):
        super().__init__("sysu219_hybrid_hold_capture")
        self.log_dir = log_dir
        self.key_at = self.phase_origin = self.origin = None
        self.active_since = self.last_active = self.motion_at = -math.inf
        self.pose = None
        self.joints = {}
        self.next_sample = 0
        self.rows = []
        qos = qos_profile_sensor_data
        self.create_subscription(Inputs, "/control_input", self.on_control, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        self.create_subscription(JointState, "/joint_cmd_states",
                                 lambda msg: self.on_joints(msg, "target"), qos)
        self.create_subscription(JointState, "/joint_states",
                                 lambda msg: self.on_joints(msg, "actual"), qos)
        self.create_subscription(Empty, "/trotting_active", self.on_active, qos)
        self.create_subscription(Inputs, "/trotting_motion_input", self.on_motion, qos)
        self.create_timer(0.02, self.record)

    def on_control(self, msg):
        if msg.command in (1, 2):
            self.finish()

    def on_tf(self, msg):
        now = time.monotonic()
        if self.key_at is None and self.pose is not None and now - self.pose[1] < 0.04:
            return
        for tf in msg.transforms:
            if tf.header.frame_id != "world" or tf.child_frame_id != "base":
                continue
            q = tf.transform.rotation
            norm = math.hypot(q.w, q.x, q.y, q.z)
            p = tf.transform.translation
            if not math.isfinite(norm) or norm < 1e-6 or not all(map(math.isfinite, (p.x, p.y))):
                continue
            w, x, y, z = (value / norm for value in (q.w, q.x, q.y, q.z))
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            nose_up = math.asin(max(-1.0, min(1.0, 2 * (x * z - w * y))))
            self.pose = (stamp_ns(tf), now, (p.x, p.y, yaw, math.degrees(nose_up)))
            if self.key_at is not None and self.origin is None and now - self.key_at <= START_DELAY_S:
                self.origin = self.pose[2][:3]

    def on_joints(self, msg, kind):
        if self.key_at is None:
            return
        positions = dict(zip(msg.name, msg.position))
        values = tuple(positions.get(f"{joint}_joint", math.nan) for joint in JOINTS)
        if all(map(math.isfinite, values)):
            self.joints[kind] = (stamp_ns(msg), time.monotonic(), tuple(map(math.degrees, values)))

    def on_active(self, _msg):
        now = time.monotonic()
        if now - self.last_active > ACTIVE_GAP_S:
            self.active_since = now
            if self.key_at is None:
                self.key_at = now
                if self.pose is not None and now - self.pose[1] <= MESSAGE_ORDER_GRACE_S:
                    self.origin = self.pose[2][:3]
        self.last_active = now

    def on_motion(self, msg):
        if any(math.isfinite(axis) and axis != 0 for axis in (msg.ly, msg.rx)):
            self.motion_at = time.monotonic()

    def finish(self):
        rows, self.rows = self.rows, []
        self.key_at = self.phase_origin = self.origin = None
        self.next_sample = 0
        if not rows:
            return
        path = self.log_dir / f"hold_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')}.csv"
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            with path.open("x", encoding="utf-8") as output:
                output.write("\n".join((",".join(HEADER), *rows)) + "\n")
        except OSError as error:
            print(f"[PPO_HOLD] 保存失败 {error}", file=sys.stderr, flush=True)
        else:
            print(f"[PPO_HOLD] {path}（{len(rows)} 行）", flush=True)

    def record(self):
        if self.key_at is None:
            return
        now = time.monotonic()
        elapsed = now - self.key_at
        if self.motion_at >= self.key_at or elapsed > START_DELAY_S + DURATION_S + ACTIVE_GAP_S:
            self.finish()
            return
        active = self.last_active >= self.key_at - MESSAGE_ORDER_GRACE_S and now - self.last_active <= ACTIVE_GAP_S
        if not active:
            if self.phase_origin is not None or elapsed > START_DELAY_S + ACTIVE_GAP_S:
                self.finish()
            return
        if self.phase_origin is None:
            self.phase_origin = self.active_since
        sample_at = self.key_at + START_DELAY_S
        next_sample = bisect_right(SAMPLE_TIMES, now - sample_at)
        if next_sample <= self.next_sample:
            return
        target, actual = self.joints.get("target"), self.joints.get("actual")
        if self.pose is None or target is None or actual is None:
            return
        samples = (self.pose, target, actual)
        stamps = [sample[0] for sample in samples]
        skew = max(stamps) - min(stamps)
        if min(stamps) <= 0 or skew > STAMP_SKEW_NS or any(s[1] < sample_at or now - s[1] > 0.1 for s in samples):
            return
        x, y, yaw, pitch = self.pose[2]
        if self.origin is None:
            self.origin = x, y, yaw
        x0, y0, yaw0 = self.origin
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        values = (elapsed, (0.5 + (now - self.phase_origin) / PERIOD_S) % 1, 100 * (c * dx + s * dy),
                  100 * (-s * dx + c * dy), pitch, skew / 1e6, *target[2], *actual[2])
        self.rows.append(",".join(f"{value:.4g}" for value in values))
        self.next_sample = next_sample
        if next_sample == len(SAMPLE_TIMES):
            self.finish()


def workspace_log_dir(script):
    for parent in script.resolve().parents:
        if parent.name in ("src", "install"):
            workspace = parent.parent
            log_dir = workspace / "log" / "sysu219_hold"
            if not log_dir.resolve().is_relative_to(workspace):
                raise RuntimeError(f"拒绝写入工作区外的采集目录 {log_dir}")
            return log_dir
    raise RuntimeError(f"无法识别采集脚本所属的 ROS 2 工作区 {script}")


def main():
    log_dir = workspace_log_dir(Path(__file__))
    rclpy.init()
    node = HoldCapture(log_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
