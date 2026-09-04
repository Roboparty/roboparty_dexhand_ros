#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 RoboParty

"""Resident RP_Hand ROS 2 device node backed by ``roboparty_dexhand``.

Bridges the apt-provided ``dexhand_py`` SDK into the ROS2 graph so that the
hand follows the same lifecycle model as the motors: probe once, initialize
once, then stay resident and serve commands.

The vendor SDK restricts each OS process to one active hand instance
(process-wide RX callback).  Therefore one node instance serves exactly one
CAN interface; deploy two instances (e.g. ``dexhand_left`` and ``dexhand_right``)
for a two-hand robot.

Interface (namespaced by node name, e.g. ``/dexhand_left/...``):

Services (std_srvs/Trigger)
    ``init``   probe the bus, then create and enable the driver.
    ``reset``  re-enable and home the already-initialized hand (moves!).

Topics
    ``cmd`` (std_msgs/Int32MultiArray) -- six target positions in encoder
    counts (valid range 0..10000); applied to all six joints with the
    configured velocity.

Parameters
    ``interface``      SocketCAN interface name (default ``can0``).
    ``node_id``        CANopen node ID of the hand, 1..127 (default 1).
    ``velocity``       Position-mode velocity, counts/s (default 8000).
    ``auto_init``      Start background probe/init at startup (default true).
    ``home_on_init``   Home the hand during init (physical motion, default
                       true; pass false when the workspace is unverified).
"""

import subprocess
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from dexhand_py import HandDriver, HandModel

#: Encoder-count range accepted for ``cmd`` targets (vendor SDK convention).
POSITION_MIN = 0
POSITION_MAX = 10000
#: Seconds between bus probes while the hand is absent.
PROBE_PERIOD_S = 5.0
#: Seconds to wait for the SDO probe round trip.
PROBE_TIMEOUT_S = 1.0
#: Probe attempts per cycle (single shots miss on jittery motor chains).
PROBE_ATTEMPTS = 3


class DexHandNode(Node):
    """One resident driver instance for one CAN interface."""

    def __init__(self) -> None:
        super().__init__("dexhand_node")
        self.declare_parameter("interface", "can0")
        self.declare_parameter("node_id", 1)
        self.declare_parameter("velocity", 8000)
        self.declare_parameter("auto_init", True)
        self.declare_parameter("home_on_init", True)

        self._interface: str = self.get_parameter("interface").value
        self._node_id: int = int(self.get_parameter("node_id").value)
        self._velocity: int = int(self.get_parameter("velocity").value)
        self._home_on_init: bool = bool(
            self.get_parameter("home_on_init").value)

        self._hand: Optional[HandDriver] = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Private names follow the instance name: /dexhand_left/init, etc.
        # 双手两实例的服务/话题互不冲突.
        self.create_service(Trigger, "~/init", self._on_init)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_subscription(
            Int32MultiArray, "~/cmd", self._on_cmd,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.get_logger().info(
            f"interface={self._interface} node_id={self._node_id} "
            f"velocity={self._velocity} auto_init={self._auto_init_text()}")

        if bool(self.get_parameter("auto_init").value):
            threading.Thread(
                target=self._probe_loop, name="hand-probe",
                daemon=True).start()

    # ------------------------------------------------------------------ probe

    def _auto_init_text(self) -> str:
        return (f"true (home={self._home_on_init})" if
                bool(self.get_parameter("auto_init").value) else "false")

    def _probe(self) -> bool:
        """Return True iff the hand shows signs of life on the bus.

        Two-stage evidence, in order of strength:
        1. active SDO answer to a feedback-period read (best);
        2. presence of the vendor 0x481 realtime stream (any node), which
           only a powered hand firmware emits after an SDK init.

        SDO single shots drop out on jittery motor-chained buses (observed
        in the field), so stage 1 retries; stage 2 keeps the node from
        stalling forever when the bus is noisy but the hand is alive.
        """
        request = "6%02X#4401D201400000000" % self._node_id
        response_prefix = "58%02X" % self._node_id
        for _ in range(PROBE_ATTEMPTS):
            proc = None
            try:
                proc = subprocess.Popen(
                    ["timeout", str(PROBE_TIMEOUT_S), "candump", "-L",
                     self._interface],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                time.sleep(0.12)
                subprocess.run(
                    ["cansend", self._interface, request],
                    stderr=subprocess.DEVNULL, check=False)
                out, _ = proc.communicate(timeout=PROBE_TIMEOUT_S + 2.0)
                text = out.decode(errors="ignore")
                if response_prefix in text:
                    return True
            except Exception:  # noqa: BLE001 - probe must never crash node
                if proc is not None:
                    proc.kill()
            time.sleep(0.3)
        return self._stream_present()

    def _stream_present(self) -> bool:
        """Return True iff vendor 0x481 realtime frames appear on the bus."""
        try:
            out = subprocess.run(
                ["timeout", str(PROBE_TIMEOUT_S), "candump", "-L",
                 self._interface],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                timeout=PROBE_TIMEOUT_S + 2.0).stdout
            return " 481#" in out.decode(errors="ignore")
        except Exception:  # noqa: BLE001
            return False

    def _probe_loop(self) -> None:
        """Probe by attempting the vendor SDK init sequence itself.

        Field finding (RP1 28dof): after a power cycle the hand stays
        dormant - it neither streams nor answers raw SDO reads - until a
        full vendor SDK init sequence (0x501 configuration frames) wakes
        it.  When ``home_on_init`` is true, initialization also enables and
        homes the hand.  When false, it only wakes the communication path
        and leaves the hand disabled for an explicit ``~/init`` or
        ``~/reset`` request.
        """
        while not self._stop_event.is_set() and self._hand is None:
            ok, message = self._init_locked(
                self._home_on_init, enable=self._home_on_init)
            if ok:
                self.get_logger().info(
                    f"hand detected and initialized: {message}")
                return
            self.get_logger().warning(
                f"hand initialization attempt failed: {message}; retrying",
                throttle_duration_sec=5.0)
            self._stop_event.wait(PROBE_PERIOD_S)
        self.get_logger().warning("probe loop exited", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------- init

    def _reinit(self, enable: bool, home: bool) -> Tuple[bool, str]:
        """Recreate the driver so init_hand truly runs.

        The vendor-backed driver treats ``init_hand`` on an already-Ready
        instance as an idempotent no-op (returns true without sending
        enable/home), so enabling after a dormant wake-up requires a fresh
        instance.  This mirrors the create-init-drive sequence used by all
        verified standalone scripts.
        """
        with self._lock:
            if self._hand is not None:
                try:
                    self._hand.deinit_hand()
                except Exception:  # noqa: BLE001 - best effort teardown
                    pass
                self._hand = None
            try:
                hand = HandDriver.create_hand(
                    "RP_Hand", "canfd", self._interface,
                    HandModel.RP_HAND_6DOF, self._node_id)
                if not hand.init_hand(enable, home, 5.0):
                    hand.deinit_hand()
                    return False, "init_hand failed (hand enabled? bus ok?)"
                self._hand = hand
                return True, f"initialized (enable={enable}, home={home})"
            except Exception as exc:  # noqa: BLE001 - report, stay resident
                return False, f"re-init failed: {exc}"

    def _init_locked(self, home: bool,
                     enable: Optional[bool] = None) -> Tuple[bool, str]:
        """Create and initialize the driver under the instance lock.

        ``enable`` overrides the first ``init_hand`` argument. Automatic
        startup follows ``home_on_init``: true enables and homes; false only
        wakes communication without driving the hand.
        """
        with self._lock:
            if self._hand is not None:
                return True, "already initialized"
            if enable is None:
                enable = True
            try:
                hand = HandDriver.create_hand(
                    "RP_Hand", "canfd", self._interface,
                    HandModel.RP_HAND_6DOF, self._node_id)
            except Exception as exc:  # noqa: BLE001 - report, stay resident
                return False, f"create_hand failed: {exc}"
            if not hand.init_hand(enable, home, 5.0):
                hand.deinit_hand()
                return False, "init_hand failed (hand enabled? bus healthy?)"
            self._hand = hand
            message = f"initialized (enable={enable}, home={home})"
            if not enable:
                message += (
                    f"; call ~/{self.get_name()}/init for warm enable or "
                    f"~/{self.get_name()}/reset for enable+home")
            return True, message

    # --------------------------------------------------------------- services

    def _on_init(self, request, response) -> Trigger.Response:
        # 重建实例使 init_hand 真正执行(幂等空操作陷阱, 见 _reinit 注释),
        # 成功即进入使能待命状态.
        response.success, response.message = self._reinit(True, False)
        return response

    def _on_reset(self, request, response) -> Trigger.Response:
        # 重建实例 + 使能 + 回零(手会物理运动).
        response.success, response.message = self._reinit(True, True)
        response.message = ("reset: enabled and homed" if response.success
                            else f"reset failed: {response.message}")
        return response

    # ------------------------------------------------------------------ topic

    def _on_cmd(self, msg: Int32MultiArray) -> None:
        with self._lock:
            if self._hand is None:
                self.get_logger().warning(
                    "cmd ignored: hand not initialized",
                    throttle_duration_sec=5.0)
                return
            if len(msg.data) != 6:
                self.get_logger().warning(
                    f"cmd ignored: expected 6 positions, got {len(msg.data)}",
                    throttle_duration_sec=5.0)
                return
            if any(not POSITION_MIN <= v <= POSITION_MAX for v in msg.data):
                self.get_logger().warning(
                    f"cmd ignored: positions must be within "
                    f"[{POSITION_MIN}, {POSITION_MAX}]: {list(msg.data)}",
                    throttle_duration_sec=5.0)
                return
            for joint in range(1, 7):
                self._hand.set_target_position(joint, int(msg.data[joint - 1]))
                self._hand.set_position_velocity(joint, self._velocity)
            # 允许未完成回零时运动: joint 机械差异可能导致回零未完全到位,
            # 固件会静默拒绝运动指令(README 示例同样在运动前设置此标志).
            # 驱动只有在初始化成功后才会存入 self._hand；未使能的休眠实例
            # 仍会由固件拒绝运动，需先调用 ~/init 或 ~/reset。
            self._hand.set_move_no_home(1)
            self._hand.move_motors(0)

    # --------------------------------------------------------------- shutdown

    def shutdown(self) -> None:
        self._stop_event.set()
        with self._lock:
            if self._hand is not None:
                self._hand.deinit_hand()
                self._hand = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DexHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
