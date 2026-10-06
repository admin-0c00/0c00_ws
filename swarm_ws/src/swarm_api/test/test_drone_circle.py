# SPDX-License-Identifier: LGPL-3.0-only

import math
import threading
import time
import unittest
from types import SimpleNamespace

from swarm_api.drone import Drone, DroneError
from swarm_api.swarm import Swarm


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def run_blocking(drone, *args, method="circle", **kwargs):
    """在后台线程执行阻塞式 circle/circle_velocity()，返回结果容器。"""
    result = {}

    def worker():
        try:
            getattr(drone, method)(*args, **kwargs)
            result["ok"] = True
        except Exception as exc:  # noqa: BLE001
            result["err"] = exc

    thread = threading.Thread(target=worker)
    thread.start()
    return result, thread


class DroneCircleTest(unittest.TestCase):
    def setUp(self):
        self.drone = Drone.__new__(Drone)
        self.drone.ns = "uav_1"
        self.drone.pos = None
        self.drone.yaw = 0.0
        self.drone.offboard = True
        self.drone.failsafe = False
        self.drone._mode = "position"
        self.drone._pos_mode = "follow"
        self.drone._target = [0.0, 0.0, 0.0]
        self.drone._target_z = 1.5
        self.drone._sp_yaw = 0.0
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        self.drone._streaming = True
        self.drone._last_xy_reset = None
        self.drone._action_phase = None
        for field in ("cx", "cy", "radius", "omega", "angle",
                      "start_angle", "target_rad", "last_t"):
            setattr(self.drone, f"_circle_{field}", 0.0)
        self.drone._circle_velocity_mode = False
        self.drone._circle_smooth_mode = False
        self.drone._circle_k = 0.3
        self.drone._circle_ki = 0.1
        self.drone._circle_radial_int = 0.0
        self.drone._circle_t0 = 0.0
        self.drone.xy_valid = True
        self.drone.v_xy_valid = True
        self.drone.dead_reckoning = False
        self.drone.vz = 0.0
        self.drone._lock = threading.Lock()
        self.drone._pub_mode = Publisher()
        self.drone._pub_sp = Publisher()
        self.drone.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(error=lambda message: None))

    @staticmethod
    def local(timestamp_us, east, north, up, reset_counter=0,
              delta_ned=(0.0, 0.0), yaw=math.pi / 2):
        return SimpleNamespace(
            timestamp_sample=timestamp_us,
            x=north, y=east, z=-up, heading=math.pi / 2 - yaw,
            xy_reset_counter=reset_counter, delta_xy=list(delta_ned))

    def test_circle_sets_state_and_start_target_then_completes_to_follow(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=2,
            start_angle=0.0, timeout=5.0)
        time.sleep(0.2)
        self.assertEqual(self.drone._pos_mode, "circle")
        self.assertEqual(self.drone._circle_cx, 2.0)
        self.assertEqual(self.drone._circle_cy, 2.0)
        self.assertEqual(self.drone._circle_radius, 0.5)
        self.assertAlmostEqual(self.drone._circle_omega, 2.0)      # speed/radius
        self.assertAlmostEqual(self.drone._circle_target_rad, 4 * math.pi)
        self.assertAlmostEqual(self.drone._target[0], 2.5)         # 起点在圆上
        self.assertAlmostEqual(self.drone._target[1], 2.0)
        # 推进角度越过目标 -> circle() 返回并切 follow 悬停
        with self.drone._lock:
            self.drone._circle_angle = 4 * math.pi + 0.5
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))
        self.assertEqual(self.drone._pos_mode, "follow")

    def test_publish_setpoint_advances_target_along_circle_radius(self):
        self.drone._mode = "position"
        self.drone._pos_mode = "circle"
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 2.0
        self.drone._circle_angle = 0.0
        self.drone._circle_last_t = time.monotonic()
        self.drone._target = [1.5, 1.0, 1.0]
        time.sleep(0.05)
        self.drone._publish_setpoint()
        self.assertGreater(self.drone._circle_angle, 0.0)
        radius = math.hypot(self.drone._target[0] - 1.0,
                            self.drone._target[1] - 1.0)
        self.assertAlmostEqual(radius, 0.5, places=3)
        # 发布出去的 setpoint 是 NED
        setpoint = self.drone._pub_sp.messages[-1]
        self.assertAlmostEqual(setpoint.position[2], -1.0)

    def test_circle_ccw_false_gives_negative_omega(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=1,
            start_angle=0.0, ccw=False, timeout=5.0)
        time.sleep(0.2)
        self.assertLess(self.drone._circle_omega, 0.0)
        with self.drone._lock:
            self.drone._circle_angle = -2 * math.pi - 0.5
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))

    def test_circle_start_angle_defaults_to_current_azimuth(self):
        # 当前位置 (3,4)，圆心 (1,1) -> 方位角 atan2(3,2)
        self.drone._on_pos(self.local(1_000_000, 3.0, 4.0, 1.5))
        result, thread = run_blocking(
            self.drone, 1.0, 1.0, 1.5, 0.5, speed=1.0, laps=1, timeout=5.0)
        time.sleep(0.2)
        expected = math.atan2(3.0, 2.0)
        self.assertAlmostEqual(self.drone._circle_start_angle, expected)
        with self.drone._lock:
            self.drone._circle_angle = expected + 2 * math.pi + 0.5
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))

    def test_circle_rejects_invalid_arguments(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        cases = (
            (2.0, 2.0, 1.5, 0.0, 1.0, 1.0),    # radius = 0
            (2.0, 2.0, 1.5, 0.5, 0.0, 1.0),    # speed = 0
            (2.0, 2.0, 1.5, 0.5, 1.0, 0.0),    # laps = 0
            (2.0, 2.0, 1.5, 0.5, 1.0, -1.0),   # laps < 0
            (2.0, 2.0, 1.5, math.nan, 1.0, 1.0),  # radius = nan
        )
        for arguments in cases:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.drone.circle(*arguments)
        with self.assertRaisesRegex(ValueError, "有限数"):
            self.drone.circle(2.0, 2.0, math.inf, 0.5, speed=1.0)

    def test_circle_rejects_not_taken_off(self):
        self.drone._streaming = False
        with self.assertRaisesRegex(DroneError, "尚未 takeoff"):
            self.drone.circle(2.0, 2.0, 1.5, 0.5, speed=1.0)

    def test_circle_timeout_switches_to_follow_and_raises(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=2,
            start_angle=0.0, timeout=0.3)
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertIsInstance(result.get("err"), DroneError)
        self.assertIn("超时", str(result["err"]))
        self.assertEqual(self.drone._pos_mode, "follow")

    def test_circle_reset_translates_center_and_target(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        self.drone._mode = "position"
        self.drone._pos_mode = "circle"
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._target = [1.5, 1.0, 1.5]
        self.drone._on_pos(self.local(
            1_010_000, 5.0, 3.0, 1.5, reset_counter=1, delta_ned=(3.0, 1.0)))
        # NED delta(3,1) -> ENU delta(1,3)；圆心与目标水平分量都平移
        self.assertAlmostEqual(self.drone._circle_cx, 2.0)
        self.assertAlmostEqual(self.drone._circle_cy, 4.0)
        self.assertAlmostEqual(self.drone._target[0], 2.5)
        self.assertAlmostEqual(self.drone._target[1], 4.0)

    def test_circle_velocity_sets_velocity_mode_and_tangent(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=2,
            start_angle=0.0, timeout=5.0, method="circle_velocity")
        time.sleep(0.2)
        self.assertEqual(self.drone._mode, "velocity")
        self.assertTrue(self.drone._circle_velocity_mode)
        self.assertAlmostEqual(self.drone._circle_omega, 2.0)
        # 初始切向速度：angle=0 -> tangent≈(0,+1)，模长=speed=1.0
        mag = math.hypot(self.drone._vel[0], self.drone._vel[1])
        self.assertAlmostEqual(mag, 1.0, places=3)
        self.assertAlmostEqual(self.drone._vel[1], 1.0, places=3)
        # 完成判定：内部角度转够即返回（纯速度模式）
        with self.drone._lock:
            self.drone._circle_angle = 4 * math.pi + 0.5
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))
        self.assertEqual(self.drone._pos_mode, "follow")
        self.assertFalse(self.drone._circle_velocity_mode)

    def test_publish_setpoint_velocity_updates_tangent(self):
        self.drone._mode = "velocity"
        self.drone._circle_velocity_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 2.0   # speed = radius*omega = 1.0
        self.drone._circle_angle = 0.0
        self.drone._circle_last_t = time.monotonic()
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        time.sleep(0.05)
        self.drone._publish_setpoint()
        self.assertGreater(self.drone._circle_angle, 0.0)
        mag = math.hypot(self.drone._vel[0], self.drone._vel[1])
        self.assertAlmostEqual(mag, 1.0, places=3)          # 速度大小=speed
        setpoint = self.drone._pub_sp.messages[-1]
        self.assertTrue(math.isnan(setpoint.position[0]))   # 速度模式：位置 NaN
        self.assertAlmostEqual(math.hypot(setpoint.velocity[0],
                                          setpoint.velocity[1]), 1.0, places=3)
        mode = self.drone._pub_mode.messages[-1]
        self.assertTrue(mode.velocity)

    def test_circle_velocity_timeout_switches_to_follow(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=2,
            start_angle=0.0, timeout=0.3, method="circle_velocity")
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertIsInstance(result.get("err"), DroneError)
        self.assertIn("超时", str(result["err"]))
        self.assertEqual(self.drone._pos_mode, "follow")
        self.assertFalse(self.drone._circle_velocity_mode)

    def test_circle_velocity_rejects_invalid(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        for arguments in ((2.0, 2.0, 1.5, 0.0, 1.0, 1.0),
                          (2.0, 2.0, 1.5, 0.5, 0.0, 1.0),
                          (2.0, 2.0, 1.5, 0.5, 1.0, -1.0)):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.drone.circle_velocity(*arguments)
        with self.assertRaisesRegex(DroneError, "尚未 takeoff"):
            self.drone._streaming = False
            self.drone.circle_velocity(2.0, 2.0, 1.5, 0.5, speed=1.0)

    def test_circle_velocity_pure_tangent(self):
        # 纯速度：v = speed * (-sin a, cos a)，无位置反馈分量
        self.drone._mode = "velocity"
        self.drone._circle_velocity_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 2.0      # speed = 1.0
        self.drone._circle_angle = 0.0
        self.drone._circle_last_t = time.monotonic()
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        self.drone._on_pos(self.local(1_000_000, 0.5, 0.5, 1.5))
        time.sleep(0.05)
        self.drone._publish_setpoint()
        a = self.drone._circle_angle
        self.assertAlmostEqual(self.drone._vel[0], -math.sin(a), places=1)
        self.assertAlmostEqual(self.drone._vel[1], math.cos(a), places=1)
        self.assertAlmostEqual(
            math.hypot(self.drone._vel[0], self.drone._vel[1]), 1.0, places=2)

    def test_swarm_circle_velocity_all_forwards(self):
        calls = []

        def drone(namespace):
            return SimpleNamespace(
                ns=namespace, hover=lambda: None,
                circle_velocity=lambda *args, **kwargs: calls.append(
                    (namespace, args, kwargs)))

        swarm = Swarm.__new__(Swarm)
        swarm.drones = [drone("uav_1"), drone("uav_2")]
        swarm.circle_velocity_all(
            (1.0, 2.0, 1.5, 0.5, 0.5, 2.0), ccw=False, yaw_rate=0.2, timeout=5.0)
        self.assertEqual({call[1] for call in calls}, {
            (1.0, 2.0, 1.5, 0.5, 0.5, 2.0)})
        self.assertEqual({call[2]["ccw"] for call in calls}, {False})
        self.assertEqual({call[2]["yaw_rate"] for call in calls}, {0.2})
        with self.assertRaises(ValueError):
            swarm.circle_velocity_all([(1.0, 2.0, 1.5, 0.5, 0.5, 2.0)])

    def test_circle_smooth_sets_velocity_mode_and_completes_to_follow(self):
        self.drone._on_pos(self.local(1_000_000, 2.5, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=2,
            start_angle=0.0, k=0.3, timeout=5.0, method="circle_smooth")
        time.sleep(0.2)
        self.assertEqual(self.drone._mode, "velocity")
        self.assertTrue(self.drone._circle_smooth_mode)
        self.assertFalse(self.drone._circle_velocity_mode)
        self.assertAlmostEqual(self.drone._circle_omega, 2.0)
        self.assertAlmostEqual(self.drone._circle_k, 0.3)
        self.assertAlmostEqual(self.drone._circle_target_rad, 4 * math.pi)
        with self.drone._lock:
            self.drone._circle_angle = 4 * math.pi + 0.5
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))
        self.assertEqual(self.drone._pos_mode, "follow")
        self.assertFalse(self.drone._circle_smooth_mode)

    def test_circle_smooth_ramps_up_feedforward(self):
        # 起步 ramp：t0 时刻速度为 0，1s 后满速
        self.drone._mode = "velocity"
        self.drone._circle_smooth_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 2.0      # speed = 1.0
        self.drone._circle_angle = 0.0
        self.drone._circle_last_t = time.monotonic()
        self.drone._circle_t0 = self.drone._circle_last_t
        self.drone._circle_k = 0.3
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        self.drone._publish_setpoint()      # 立即发布：ramp≈0 -> 速度≈0
        mag0 = math.hypot(self.drone._vel[0], self.drone._vel[1])
        self.assertLess(mag0, 0.3)
        self.drone._circle_t0 -= 2.0        # 模拟已过 2s -> ramp=1
        self.drone._publish_setpoint()
        mag1 = math.hypot(self.drone._vel[0], self.drone._vel[1])
        self.assertGreater(mag1, 0.9)

    def test_circle_smooth_radial_correction_direction(self):
        # 无人机在圆内侧（r<R）-> 纠正速度应有离心（向外）分量
        self.drone._mode = "velocity"
        self.drone._circle_smooth_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 2.0
        self.drone._circle_angle = 0.0
        self.drone._circle_last_t = time.monotonic()
        self.drone._circle_t0 = self.drone._circle_last_t - 2.0   # ramp=1
        self.drone._circle_k = 0.3
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        # 无人机在圆心正东 0.3m（内侧，R=0.5），角度 0
        self.drone._on_pos(self.local(1_000_000, 1.3, 1.0, 1.5))
        self.drone._publish_setpoint()
        # 切向 FF 在 angle=0 处为 (0, +speed)；径向纠正向外(+x)；相位纠正忽略
        self.assertGreater(self.drone._vel[0], 0.05)   # 向外的径向分量
        # 改为外侧（正东 0.7m）-> 径向分量应向内（-x）
        self.drone._on_pos(self.local(1_000_000, 1.7, 1.0, 1.5))
        self.drone._publish_setpoint()
        self.assertLess(self.drone._vel[0], -0.05)

    def test_circle_smooth_phase_correction_direction(self):
        # 无人机角度落后于指令角度 -> 纠正速度应沿运动切向（加速追赶）
        self.drone._mode = "velocity"
        self.drone._circle_smooth_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 2.0
        self.drone._circle_angle = 0.3      # 指令角度领先 0.3 rad
        self.drone._circle_last_t = time.monotonic()
        self.drone._circle_t0 = self.drone._circle_last_t - 2.0
        self.drone._circle_k = 0.3
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        # 无人机在圆上角度 0 处（正东 R=0.5），落后于指令
        self.drone._on_pos(self.local(1_000_000, 1.5, 1.0, 1.5))
        self.drone._publish_setpoint()
        # 切向 FF 在 θ_cmd=0.3 处：(-sin0.3, cos0.3)*1.0；相位纠正沿 θ_d=0 处切向 (0,+1)
        # 合成后 vy 应大于纯 FF 在 0.3 处的 cos0.3
        self.assertGreater(self.drone._vel[1], math.cos(0.3) - 1e-6)

    def test_circle_smooth_reset_translates_center(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        self.drone._mode = "velocity"
        self.drone._circle_smooth_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._on_pos(self.local(
            1_010_000, 5.0, 3.0, 1.5, reset_counter=1, delta_ned=(3.0, 1.0)))
        # NED delta(3,1) -> ENU delta(1,3)；圆心平移
        self.assertAlmostEqual(self.drone._circle_cx, 2.0)
        self.assertAlmostEqual(self.drone._circle_cy, 4.0)

    def test_circle_smooth_radial_integral_grows_and_clamps(self):
        # 持续的内侧偏差 -> 径向积分逐渐增大（向内收口）；并验证限幅
        self.drone._mode = "velocity"
        self.drone._circle_smooth_mode = True
        self.drone._circle_cx = 1.0
        self.drone._circle_cy = 1.0
        self.drone._circle_radius = 0.5
        self.drone._circle_omega = 0.0      # 只看径向纠正，去掉切向/相位分量
        self.drone._circle_angle = 0.0
        self.drone._circle_t0 = time.monotonic() - 2.0
        self.drone._circle_k = 0.3
        self.drone._circle_ki = 0.1
        self.drone._circle_radial_int = 0.0
        self.drone._vel = [0.0, 0.0, 0.0]
        self.drone._yaw_rate = 0.0
        # 无人机在正东 0.9m（外侧 r>R）：err<0，积分应累积向内（-x）
        self.drone._on_pos(self.local(1_000_000, 1.9, 1.0, 1.5))
        ints = []
        for _ in range(5):
            self.drone._circle_last_t = time.monotonic() - 0.1   # 强制 dt=0.1s
            self.drone._publish_setpoint()
            ints.append(self.drone._circle_radial_int)
        self.assertTrue(all(b < a for a, b in zip(ints, ints[1:])))   # 单调累积
        self.assertLess(self.drone._vel[0], -0.1)                      # 向内
        # 限幅：err=-0.4, ki=0.1, dt=0.1 -> 每次 -0.004，模拟长时间后不超过 0.25
        self.drone._circle_radial_int = -0.249
        for _ in range(10):
            self.drone._circle_last_t = time.monotonic() - 0.1
            self.drone._publish_setpoint()
        self.assertGreaterEqual(self.drone._circle_radial_int, -0.25)

    def test_circle_smooth_rejects_invalid(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        for arguments in ((2.0, 2.0, 1.5, 0.0, 1.0, 1.0),
                          (2.0, 2.0, 1.5, 0.5, 0.0, 1.0),
                          (2.0, 2.0, 1.5, 0.5, 1.0, -1.0)):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.drone.circle_smooth(*arguments)
        with self.assertRaisesRegex(ValueError, "正数"):
            self.drone.circle_smooth(2.0, 2.0, 1.5, 0.5, 1.0, 1.0, k=0.0)
        with self.assertRaisesRegex(DroneError, "尚未 takeoff"):
            self.drone._streaming = False
            self.drone.circle_smooth(2.0, 2.0, 1.5, 0.5, speed=1.0)

    def test_circle_smooth_timeout_switches_to_follow(self):
        self.drone._on_pos(self.local(1_000_000, 2.0, 2.0, 1.5))
        result, thread = run_blocking(
            self.drone, 2.0, 2.0, 1.5, 0.5, speed=1.0, laps=2,
            start_angle=0.0, timeout=0.3, method="circle_smooth")
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertIsInstance(result.get("err"), DroneError)
        self.assertIn("超时", str(result["err"]))
        self.assertEqual(self.drone._pos_mode, "follow")
        self.assertFalse(self.drone._circle_smooth_mode)

    def test_swarm_circle_smooth_all_forwards(self):
        calls = []

        def drone(namespace):
            return SimpleNamespace(
                ns=namespace, hover=lambda: None,
                circle_smooth=lambda *args, **kwargs: calls.append(
                    (namespace, args, kwargs)))

        swarm = Swarm.__new__(Swarm)
        swarm.drones = [drone("uav_1"), drone("uav_2")]
        swarm.circle_smooth_all(
            (1.0, 2.0, 1.5, 0.5, 0.5, 2.0), ccw=False, k=0.25, timeout=5.0)
        self.assertEqual({call[1] for call in calls}, {
            (1.0, 2.0, 1.5, 0.5, 0.5, 2.0)})
        self.assertEqual({call[2]["ccw"] for call in calls}, {False})
        self.assertEqual({call[2]["k"] for call in calls}, {0.25})
        with self.assertRaises(ValueError):
            swarm.circle_smooth_all([(1.0, 2.0, 1.5, 0.5, 0.5, 2.0)])

    def test_swarm_circle_all_broadcasts_and_forwards_options(self):
        calls = []

        def drone(namespace):
            return SimpleNamespace(
                ns=namespace, hover=lambda: None,
                circle=lambda *args, **kwargs: calls.append(
                    (namespace, args, kwargs)))

        swarm = Swarm.__new__(Swarm)
        swarm.drones = [drone("uav_1"), drone("uav_2")]
        swarm.circle_all(
            (1.0, 2.0, 1.5, 0.5, 0.5, 2.0), ccw=False, yaw=0.1, timeout=5.0)
        self.assertEqual({call[1] for call in calls}, {
            (1.0, 2.0, 1.5, 0.5, 0.5, 2.0)})
        self.assertEqual({call[2]["ccw"] for call in calls}, {False})
        self.assertEqual({call[2]["timeout"] for call in calls}, {5.0})
        with self.assertRaises(ValueError):
            swarm.circle_all([(1.0, 2.0, 1.5, 0.5, 0.5, 2.0)])

    def test_hover_is_zero_velocity(self):
        self.drone.pos = (0.0, 0.0, 0.5)
        self.drone.yaw = 0.0
        self.drone.hover()
        self.assertEqual(self.drone._mode, "velocity")
        self.assertEqual(self.drone._vel, [0.0, 0.0, 0.0])

    def test_takeoff_waits_for_horizontal_settle(self):
        # 起飞：高度到位但水平速度大 -> 不应返回；水平速度收敛 -> 才返回
        self.drone.pos = (0.0, 0.0, 0.0)
        self.drone.yaw = 0.0
        self.drone.offboard = True
        self.drone.armed = True
        self.drone.vx = 0.0
        self.drone.vy = 0.0
        self.drone._command = lambda *args, **kwargs: None
        result = {}

        def worker():
            try:
                self.drone.takeoff(alt=1.0, timeout=3.0, settle_time=0.5)
                result["ok"] = True
            except Exception as exc:  # noqa: BLE001
                result["err"] = exc

        thread = threading.Thread(target=worker)
        thread.start()
        time.sleep(1.3)   # 过预发 1s + offboard/解锁(已满足)
        # 高度到位但水平速度 0.5 m/s -> 不应返回
        with self.drone._lock:
            self.drone.pos = (0.0, 0.0, 1.0)
            self.drone.vx = 0.5
        time.sleep(0.8)
        self.assertTrue(thread.is_alive(), "水平速度未收敛时 takeoff 不应返回")
        # 水平速度收敛 -> settle_time 后返回
        with self.drone._lock:
            self.drone.vx = 0.0
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))

    def test_takeoff_reclimbs_after_height_drift(self):
        # 到位后掉高 -> takeoff 应发出爬升修正（_vel[2]=vz），高度回位后才返回
        self.drone.pos = (0.0, 0.0, 0.0)
        self.drone.yaw = 0.0
        self.drone.offboard = True
        self.drone.armed = True
        self.drone.vx = 0.0
        self.drone.vy = 0.0
        self.drone._command = lambda *args, **kwargs: None
        result = {}

        def worker():
            try:
                self.drone.takeoff(alt=1.0, timeout=3.0, settle_time=0.3)
                result["ok"] = True
            except Exception as exc:  # noqa: BLE001
                result["err"] = exc

        thread = threading.Thread(target=worker)
        thread.start()
        time.sleep(1.3)
        with self.drone._lock:
            self.drone.pos = (0.0, 0.0, 1.0)   # 高度到位 → vz_zeroed
        time.sleep(0.2)
        with self.drone._lock:
            self.drone.pos = (0.0, 0.0, 0.5)   # 模拟掉高（出容差）
        time.sleep(0.3)
        self.assertGreater(self.drone._vel[2], 0.0)   # 已发出爬升修正
        with self.drone._lock:
            self.drone.pos = (0.0, 0.0, 1.0)   # 高度爬回目标
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result.get("ok"))

    def test_takeoff_times_out_when_horizontal_never_settles(self):
        self.drone.pos = (0.0, 0.0, 0.0)
        self.drone.yaw = 0.0
        self.drone.offboard = True
        self.drone.armed = True
        self.drone.vx = 0.5
        self.drone.vy = 0.0
        self.drone._command = lambda *args, **kwargs: None
        with self.assertRaises(DroneError) as ctx:
            # 高度直接到位但水平速度一直 0.5 -> 超时
            self.drone.pos = (0.0, 0.0, 1.0)
            self.drone.takeoff(alt=1.0, timeout=0.8, settle_time=0.5)
        self.assertIn("超时", str(ctx.exception))
        self.assertIn("水平速度", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
