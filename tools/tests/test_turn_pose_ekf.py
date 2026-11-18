"""ターンEKFの独立幾何・運動・観測喪失の数値検証。実機精度の試験ではない。"""
from pathlib import Path
import math
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"analysis"))
import turn_pose_ekf as ekf

SETTINGS_A = {"mouse": "A", "suction": {"enabled": False}}


def derivative(function, x, epsilon=1e-6):
    result = []
    for i in range(len(x)):
        step = np.zeros(len(x))
        step[i] = epsilon
        result.append((function(x+step)-function(x-step))/(2*epsilon))
    return np.asarray(result).T


def fixture():
    corners = [(-.2, -.2), (.5, -.2), (.5, .5), (-.2, .5)]
    walls = [dict(a_m=corners[i], b_m=corners[(i+1)%4]) for i in range(4)]
    sensors = [dict(name=str(i), offset_m=[.015, .005], angle_rad=angle,
                    min_m=.01, max_m=1.5, sigma_m=.003)
               for i, angle in enumerate((0., .96, -.96))]
    return walls, sensors


def simulation(use_walls):
    """既知の非ゼロ横速度、IMUバイアス、車輪速度誤差。乱数は固定。"""
    rng = np.random.default_rng(18)
    walls, sensors = fixture()
    f = ekf.TurnPoseEkf()
    f.state[2], f.state[3] = math.radians(2), .5
    truth = np.zeros(8)
    truth[3] = .5
    records = []
    dt, omega, duration = .005, 3., .8
    for i in range(1, 161):
        t = i*dt
        mid = t-.5*dt
        lateral = -.07*math.sin(math.pi*mid/duration)
        lateral_dot = -.07*math.pi/duration*math.cos(math.pi*mid/duration)
        # Independent body kinematics: ax=du/dt-omega*v, ay=dv/dt+omega*u.
        ax, ay = -omega*lateral, lateral_dot+omega*.5
        heading = omega*mid
        truth[:2] += np.array([.5*math.cos(heading)-lateral*math.sin(heading),
                               .5*math.sin(heading)+lateral*math.cos(heading)])*dt
        truth[2] = omega*t
        truth[3:5] = ekf.rotation(truth[2]) @ [.5, -.07*math.sin(math.pi*t/duration)]
        f.predict(omega+.015, [ax+.08, ay+.10], dt)
        f.observe_forward(.51, .02**2)
        if use_walls and t < .65:
            for sensor in sensors:
                observed = ekf.ray_prediction(truth, sensor, walls)
                if observed:
                    f.observe_wall(observed[0]+rng.normal(0, .001), sensor, walls, t)
        records.append(dict(time_s=t, truth_x_mm=truth[0]*1000, truth_y_mm=truth[1]*1000,
                            truth_yaw_deg=math.degrees(truth[2]),
                            truth_beta_deg=math.degrees(math.atan2(-.07*math.sin(math.pi*t/duration), .5)),
                            **f.estimate()))
    return f, records


class TurnPoseTests(unittest.TestCase):
    def replay_samples(self):
        count = 400
        active = (np.arange(count) >= 100) & (np.arange(count) < 300)
        pulses = active*.4/ekf.Config().mm_per_tick
        return pd.DataFrame({"time_ms": 1000+np.arange(count)*5, "sequence": np.arange(count),
                             "ideal.velo": active*.4, "ideal.rad_velo": 0., ekf.RIGHT: pulses,
                             ekf.LEFT: -pulses, "ego.rad_velo": .01, "ego.accel": .1,
                             "ego.horizon_accel.get": .02, "irq.period_us": 1000.})

    def test_replay_static_calibration_and_actual_mcu_time(self):
        frame, report = ekf.replay(self.replay_samples(), SETTINGS_A)
        self.assertEqual(report["rows"], 200)
        self.assertGreater(report["stationary_samples"], 30)
        self.assertEqual(report["wall_updates"], 0)
        np.testing.assert_allclose(report["initial_bias"], [.01, .1, .02])
        self.assertAlmostEqual(frame.x_mm.iloc[-1], .4*.005*199*1000, places=7)
        self.assertAlmostEqual(frame.beta_deg.iloc[-1], 0., places=7)

    def test_replay_rejects_gaps_missing_irq_unknown_mouse_and_uncalibrated_wall(self):
        cases = []
        data = self.replay_samples()
        data.loc[150, "sequence"] += 1
        cases.append(data)
        data = self.replay_samples()
        data.loc[150, "irq.period_us"] = np.nan
        cases.append(data)
        data = self.replay_samples()
        data.loc[150:, "time_ms"] += 30
        cases.append(data)
        cases.append(self.replay_samples().iloc[95:])
        cases.append(self.replay_samples().iloc[:301])
        for data in cases:
            with self.assertRaises(ValueError):
                ekf.replay(data, SETTINGS_A)
        with self.assertRaises(ValueError):
            ekf.replay(self.replay_samples(), {"mouse": "B"})
        with self.assertRaises(ValueError):
            ekf.replay(self.replay_samples(), SETTINGS_A, geometry={"mouse": "A", "calibrated": True})

    def test_suction_profile_uses_actual_switch_and_rejects_unknown_status(self):
        off = ekf.Config.for_suction(False)
        on = ekf.Config.for_suction(True)
        self.assertGreater(on.gyro_std, off.gyro_std*10)
        self.assertGreater(on.accel_std, off.accel_std)
        data = self.replay_samples()
        settings = {"mouse": "A", "parameters": {"velo": .5}, "suction": {"enabled": True}}
        _, report = ekf.replay(data, settings)
        self.assertEqual(report["noise_profile"], "suction_on")
        self.assertEqual(report["config"]["gyro_std"], on.gyro_std)
        for unknown in (None, 0, 1, "on"):
            with self.assertRaises(ValueError):
                ekf.Config.for_suction(unknown)

    def test_motion_and_input_jacobians(self):
        x = np.array([.1, .2, .7, .4, -.2, .01, .1, -.2])
        inputs = np.array([.7, 2., 3.])
        _, f, g = ekf.motion(x, inputs[2], inputs[:2], .013)
        numerical = derivative(lambda state: ekf.motion(state, inputs[2], inputs[:2], .013)[0], x)
        np.testing.assert_allclose(f, numerical, atol=1e-9)
        numerical = derivative(lambda u: ekf.motion(x, u[2], u[:2], .013)[0], inputs)
        np.testing.assert_allclose(g, numerical, atol=1e-9)

    def test_forward_observation_jacobian(self):
        x = np.array([0., 0., .7, .4, -.2, 0., 0., 0.])
        _, h = ekf.forward_measurement(x)
        np.testing.assert_allclose(h, derivative(lambda s: ekf.forward_measurement(s)[0], x), atol=1e-9)

    def test_finite_segment_ray_jacobian(self):
        walls, sensors = fixture()
        x = np.array([.04, .06, .7, .4, .2, 0., 0., 0.])
        value, h = ekf.ray_prediction(x, sensors[0], walls)
        self.assertGreater(value, 0)
        numerical = derivative(lambda s: ekf.ray_prediction(s, sensors[0], walls)[0], x)
        np.testing.assert_allclose(h, numerical, atol=1e-8)

    def test_circle_from_centripetal_acceleration_without_zero_slip_observation(self):
        f = ekf.TurnPoseEkf(bias=(.02, .1, .2))
        speed, omega, dt = .5, 3., .001
        f.state[3] = speed
        for _ in range(500):
            f.predict(omega+.02, [.1, speed*omega+.2], dt)
            f.observe_forward(speed, .04**2)
        angle = omega*.5
        np.testing.assert_allclose(f.state[:2], [speed/omega*math.sin(angle), speed/omega*(1-math.cos(angle))], atol=2e-5)
        self.assertAlmostEqual(f.estimate()["beta_deg"], 0., delta=.02)

    def test_lateral_velocity_is_allowed_and_walls_improve_known_pose(self):
        no_wall, a = simulation(False)
        with_wall, b = simulation(True)
        core = slice(40, 110)
        for key in ("x_mm", "y_mm", "yaw_deg"):
            rms = lambda rows: np.sqrt(np.mean([(r[key]-r["truth_"+key])**2 for r in rows[core]]))
            self.assertLess(rms(b), rms(a), key)
        self.assertGreater(with_wall.wall_updates, 30)
        self.assertLess(min(r["beta_deg"] for r in b), -3.)
        # Short wall coverage need not improve every realization of slip error.
        # Check the independent truth against the reported uncertainty instead.
        for r in b[core]:
            self.assertLess(abs(r["beta_deg"]-r["truth_beta_deg"]), 2*r["sigma_beta_deg"])
        for f in (no_wall, with_wall):
            self.assertGreaterEqual(np.linalg.eigvalsh(f.covariance).min(), -1e-12)
        # Last 150ms have no wall updates. Prediction continues, covariance grows.
        self.assertEqual(b[-1]["wall_updates"], b[130]["wall_updates"])
        self.assertGreater(b[-1]["sigma_x_mm"], b[130]["sigma_x_mm"])

    def test_global_yaw_and_translation_are_not_measured_by_wheels(self):
        a, b = ekf.TurnPoseEkf(), ekf.TurnPoseEkf()
        angle = .6
        transform = np.eye(8)
        transform[:2, :2] = transform[3:5, 3:5] = ekf.rotation(angle)
        a.state[3:5] = [.5, -.03]
        b.state = transform @ a.state
        b.state[:2], b.state[2] = [.2, -.1], angle
        b.covariance = transform @ a.covariance @ transform.T
        for _ in range(100):
            for f in (a, b):
                f.predict(2., [.1, .9], .005)
                f.observe_forward(.5, .04**2)
        expected = transform @ a.state
        expected[:2] += [.2, -.1]
        expected[2] += angle
        np.testing.assert_allclose(b.state, expected, atol=1e-12)
        np.testing.assert_allclose(b.covariance, transform @ a.covariance @ transform.T, atol=1e-12)

    def test_invalid_range_outlier_and_repeated_data_do_not_shrink_covariance(self):
        f = ekf.TurnPoseEkf()
        walls, sensors = fixture()
        sensor = sensors[0]
        value, _ = ekf.ray_prediction(f.state, sensor, walls)
        for invalid, valid in ((value, False), (float("nan"), True), (sensor["max_m"], True), (value+.1, True)):
            before = f.covariance.copy()
            self.assertFalse(f.observe_wall(invalid, sensor, walls, 0., valid))
            np.testing.assert_array_equal(before, f.covariance)
        self.assertTrue(f.observe_wall(value, sensor, walls, 0.))
        before = f.covariance.copy()
        self.assertFalse(f.observe_wall(value, sensor, walls, .001))
        np.testing.assert_array_equal(before, f.covariance)

    def test_ray_never_sees_through_invalid_near_wall(self):
        _, sensors = fixture()
        sensor = dict(sensors[0], offset_m=[0., 0.], min_m=.025)
        walls = [dict(a_m=[x, -.1], b_m=[x, .1]) for x in (.01, .1)]
        self.assertIsNone(ekf.ray_prediction(np.zeros(8), sensor, walls))

    def test_yaw_wrap_low_speed_and_gap(self):
        f = ekf.TurnPoseEkf()
        self.assertTrue(math.isnan(f.estimate()["beta_deg"]))
        f.state[2] = math.pi-.001
        f.predict(2., [0., 0.], .005)
        self.assertGreater(f.state[2], math.pi)
        for dt in (0., -.001, .03, float("nan")):
            with self.assertRaises(ValueError):
                f.predict(0., [0., 0.], dt)


if __name__ == "__main__":
    unittest.main()
