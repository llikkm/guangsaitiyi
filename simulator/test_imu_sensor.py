"""Direct checks of the six-axis sensor and real-record adapter."""
from dataclasses import FrozenInstanceError
import math
import random
from types import SimpleNamespace
import unittest

from imu_sensor import G, ImuSensor, imu_defaults, sample_from_raw


def car(heading=0., yaw=0.):
    return SimpleNamespace(body=SimpleNamespace(angle=heading, velocity=(0., 0.),
                                                angular_velocity=yaw))


def quiet(**overrides):
    options = dict(gyro_scale=(1.,)*3, accel_scale=(1.,)*3, mount_rpy_deg=(0.,)*3)
    for key in ("gyro_bias_dps", "accel_bias_g", "gyro_noise_dps", "accel_noise_g",
                "gyro_walk_dps_sqrt_s", "accel_walk_g_sqrt_s"):
        options[key] = (0.,)*3
    return options | overrides


class ImuTests(unittest.TestCase):
    def test_ideal_motion_and_mount(self):
        sensor = ImuSensor(dict(mount_rpy_deg=(0., 0., 90.)), "Ideal", random.Random(1))
        vehicle = car(heading=math.pi/2, yaw=2.)
        start = sensor.sample(vehicle, 0., 0.)
        self.assertEqual(start.accel_m_s2, (0., 0., G))
        vehicle.body.velocity = (0., 200.)
        moving = sensor.sample(vehicle, .1, .1)
        self.assertAlmostEqual(moving.accel_m_s2[0], 0.)
        self.assertAlmostEqual(moving.accel_m_s2[1], -2.)
        self.assertEqual(moving.yaw_rate, 2.)
        self.assertEqual(moving.time, moving.available_time)
        with self.assertRaises(FrozenInstanceError):
            moving.yaw_rate = 0.

    def test_roll_projection(self):
        sensor = ImuSensor(dict(mount_rpy_deg=(90., 0., 0.)), "Ideal", random.Random(1))
        sample = sensor.sample(car(yaw=-1.5), 0., 0.)
        self.assertAlmostEqual(sample.gyro_rad_s[1], -1.5)
        self.assertAlmostEqual(sample.accel_m_s2[1], G)
        self.assertAlmostEqual(sample.yaw_rate, -1.5)

    def test_seed_drifts_and_modes(self):
        def stream(seed, mode):
            sensor = ImuSensor(quiet(gyro_walk_dps_sqrt_s=(1.,)*3, latency_s=.02),
                               mode, random.Random(seed))
            return [sensor.sample(car(), i*.1, .1 if i else 0.) for i in range(6)]
        nominal = stream(5, "Nominal")
        self.assertEqual(nominal, stream(5, "Nominal"))
        self.assertNotEqual(nominal, stream(6, "Nominal"))
        stress = stream(5, "Stress")
        self.assertAlmostEqual(nominal[-1].available_time, .52)
        self.assertAlmostEqual(stress[-1].available_time, .56)
        self.assertAlmostEqual(stress[-1].yaw_rate, 3*nominal[-1].yaw_rate,
                               delta=math.radians(.00875)*2)
        self.assertNotEqual(nominal[1].gyro_rad_s, nominal[-1].gyro_rad_s)

    def test_adc_rounding_clipping_and_reset(self):
        sensor = ImuSensor(quiet(), "nominal", random.Random(1))
        vehicle = car(yaw=math.radians(.012))
        start = sensor.sample(vehicle, 0., 0.)
        self.assertAlmostEqual(math.degrees(start.yaw_rate), .00875)
        vehicle.body.angular_velocity = -100.
        vehicle.body.velocity = (50000., 0.)
        sample = sensor.sample(vehicle, .01, .01)
        self.assertAlmostEqual(math.degrees(sample.yaw_rate),
                               -math.floor(250/.00875)*.00875)
        self.assertAlmostEqual(sample.accel_m_s2[0]/G, 32767*.000061)
        self.assertEqual(sensor.sample(vehicle, 0., 0.).accel_m_s2[0], 0.)

    def test_raw_conversion_agrees_with_adc_output(self):
        sensor = ImuSensor(quiet(), "Nominal", random.Random(3))
        measurement = sensor.sample(car(yaw=.7), 1., 0.)
        row = dict(t_sample_us="1000000")
        options = imu_defaults()
        for idx, axis in enumerate("xyz"):
            row["g"+axis+"_raw"] = str(round(math.degrees(measurement.gyro_rad_s[idx]) /
                                             options["dps_per_count"]))
            row["a"+axis+"_raw"] = str(round(measurement.accel_m_s2[idx] / G / options["g_per_count"]))
        self.assertEqual(sample_from_raw(row, options, measurement.available_time, (0,0,0)),
                         measurement)


if __name__ == "__main__":
    unittest.main()
