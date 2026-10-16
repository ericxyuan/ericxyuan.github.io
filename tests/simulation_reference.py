"""Independent Python reference for the browser's one-axis yaw model.

Run: python tests/simulation_reference.py
Run: python tests/simulation_reference.py --json
No third-party packages. All inputs are SI; wheel speed is relative to body.
The model is open loop, begins at rest and is not a pitch/roll suspension model.
"""
import json
import math
import sys
import unittest

DEFAULTS = dict(targetDeg=30, durationS=1, torqueLimitNm=0.003,
                wheelMassKg=0.040, wheelRadiusM=0.024, totalMassKg=0.340,
                bodyRadiusGyrationM=0.026, inertiaFactor=1, maxWheelRpm=400,
                disturbanceNm=0, maxMotorAccelRadS2=500,
                torqueSpeedNoLoadRpm=None, currentLimitA=None,
                torqueConstantNmPerAmp=None, voltageV=None)
RPM = 60 / (2 * math.pi)
RAD = math.pi / 180


def validate_params(values=None):
    p = dict(DEFAULTS, **(values or {}))
    errors = []
    for key in ("targetDeg", "durationS", "torqueLimitNm", "wheelMassKg", "wheelRadiusM",
                "totalMassKg", "bodyRadiusGyrationM", "inertiaFactor", "disturbanceNm"):
        if not isinstance(p[key], (int, float)) or not math.isfinite(p[key]):
            errors.append(f"{key} must be finite")
    if errors:
        return dict(valid=False, errors=errors, params=p)
    for key in ("durationS", "torqueLimitNm", "wheelMassKg", "wheelRadiusM",
                "totalMassKg", "bodyRadiusGyrationM", "inertiaFactor"):
        if p[key] <= 0:
            errors.append(f"{key} must be positive")
    if p["wheelMassKg"] >= p["totalMassKg"]:
        errors.append("Flywheel mass must be below inclusive total mass")
    if p["inertiaFactor"] > 1:
        errors.append("Inertia factor must not exceed1")
    if abs(p["targetDeg"]) > 360 or not 0.001 <= p["durationS"] <= 120:
        errors.append("Target/duration outside demonstrator domain")
    for key in ("maxWheelRpm", "maxMotorAccelRadS2", "torqueSpeedNoLoadRpm",
                "currentLimitA", "torqueConstantNmPerAmp", "voltageV"):
        if p[key] is not None and (not isinstance(p[key], (float, int)) or
                                   not math.isfinite(p[key]) or p[key] <= 0):
            errors.append(f"{key} must be positive or None")
    if (p["currentLimitA"] is None) != (p["torqueConstantNmPerAmp"] is None):
        errors.append("Both current limit and output torque constant required")
    ib = (p["totalMassKg"] - p["wheelMassKg"]) * p["bodyRadiusGyrationM"] ** 2
    iw = p["inertiaFactor"] * p["wheelMassKg"] * p["wheelRadiusM"] ** 2
    if not math.isfinite(ib) or not math.isfinite(iw) or min(ib, iw) <= 0:
        errors.append("Derived inertias must be positive and finite")
    return dict(valid=not errors, errors=errors, params=p)


def _checked(values):
    v = validate_params(values)
    if not v["valid"]:
        raise ValueError("; ".join(v["errors"]))
    return v["params"]


def _hardware(p):
    ib = (p["totalMassKg"] - p["wheelMassKg"]) * p["bodyRadiusGyrationM"] ** 2
    iw = p["inertiaFactor"] * p["wheelMassKg"] * p["wheelRadiusM"] ** 2
    c = 1 / ib + 1 / iw
    current = math.inf if p["currentLimitA"] is None else p["currentLimitA"] * p["torqueConstantNmPerAmp"]
    cap = min(p["torqueLimitNm"], current)
    effective = min(cap, math.inf if p["maxMotorAccelRadS2"] is None else p["maxMotorAccelRadS2"] / c)
    speed = math.inf if p["maxWheelRpm"] is None else p["maxWheelRpm"] / RPM
    return ib, iw, c, cap, effective, speed


def _envelope(p, cap, relative):
    if p["torqueSpeedNoLoadRpm"] is None:
        return cap
    return cap * max(0, 1 - abs(relative) * RPM / p["torqueSpeedNoLoadRpm"])


def _capacity_angle(p, duration, steps=160):
    ib, iw, c, cap, _, speedcap = _hardware(p)
    speed = yaw = 0
    dt = duration / (2 * steps)
    for _ in range(steps):
        a = min(_envelope(p, cap, speed) * c,
                math.inf if p["maxMotorAccelRadS2"] is None else p["maxMotorAccelRadS2"])
        if p["torqueSpeedNoLoadRpm"] is not None:
            no_load = p["torqueSpeedNoLoadRpm"] / RPM
            a = min(a, cap * c * max(0, 1 - speed / no_load) / (1 + cap * c * dt / no_load))
        ramp = min(dt, max(0, (speedcap - speed) / a if a > 0 else math.inf))
        yaw += iw / (ib + iw) * (speed * dt + a * ramp * (dt - ramp / 2))
        speed += a * ramp
    return 2 * yaw


def calculate_metrics(values=None):
    p = _checked(values)
    ib, iw, coupling, cap, effective, speed = _hardware(p)
    theta, duration = abs(p["targetDeg"]) * RAD, p["durationS"]
    required = 4 * ib * theta / duration ** 2
    body_rate = 2 * theta / duration
    relative = body_rate * (ib + iw) / iw
    vmax = speed * iw / (ib + iw)
    amax = effective / ib
    torque_minimum = math.sqrt(4 * ib * theta / cap)
    curve_speed_time = (0 if p["torqueSpeedNoLoadRpm"] is None else
                        2 * theta * (ib + iw) / iw / (p["torqueSpeedNoLoadRpm"] / RPM))
    torque_speed_minimum = (0 if p["torqueSpeedNoLoadRpm"] is None else
                            (curve_speed_time + math.sqrt(curve_speed_time ** 2 + 4 * torque_minimum ** 2)) / 2)
    minimum = (2 * math.sqrt(theta / amax) if theta <= vmax ** 2 / amax
               else theta / vmax + vmax / amax)
    if p["torqueSpeedNoLoadRpm"] is not None and theta > 0:
        low, high = 0, max(minimum, 0.01)
        while _capacity_angle(p, high) < theta and high < 1e5:
            high *= 2
        for _ in range(36):
            mid = (low + high) / 2
            if _capacity_angle(p, mid) < theta:
                low = mid
            else:
                high = mid
        minimum = high
    torque_limited = required > cap * (1 + 1e-10)
    speed_limited = relative > speed * (1 + 1e-10)
    accel_limited = p["maxMotorAccelRadS2"] is not None and required * coupling > p["maxMotorAccelRadS2"] * (1 + 1e-10)
    envelope_limited = required > _envelope(p, cap, relative) * (1 + 1e-10)
    return dict(bodyInertia=ib, wheelInertia=iw, coupling=coupling,
                requiredTorqueNm=required, requiredPeakWheelRpm=relative * RPM,
                peakBodyRateDegS=body_rate / RAD,
                minimumDurationS=minimum,
                minimumBangBangDurationS=max(math.sqrt(4 * ib * theta / effective),
                                             0 if math.isinf(vmax) else 2 * theta / vmax, torque_speed_minimum),
                torqueLimited=torque_limited, speedLimited=speed_limited,
                accelerationLimited=accel_limited, torqueSpeedLimited=envelope_limited,
                feasible=not (torque_limited or speed_limited or accel_limited or envelope_limited))


def simulate_trajectory(values=None, steps=320):
    p = _checked(values)
    m = calculate_metrics(p)
    ib, iw, c, cap, _, speedcap = _hardware(p)
    steps = max(40, min(2400, 2 * round(steps / 2)))
    half, dt = steps // 2, p["durationS"] / steps
    direction = 1 if p["targetDeg"] > 0 else -1 if p["targetDeg"] < 0 else 0
    yaw = rate = relative = 0
    first, points = [], []
    disturbance_rate = p["disturbanceNm"] / ib
    for index in range(steps):
        available = _envelope(p, cap, relative)
        low, high = -available, available
        if p["maxMotorAccelRadS2"] is not None:
            low = max(low, (-p["maxMotorAccelRadS2"] + disturbance_rate) / c)
            high = min(high, (p["maxMotorAccelRadS2"] + disturbance_rate) / c)
        if math.isfinite(speedcap):
            sl = ((-speedcap - relative) / dt + disturbance_rate) / c
            sh = ((speedcap - relative) / dt + disturbance_rate) / c
            if max(low, sl) <= min(high, sh):
                low, high = max(low, sl), min(high, sh)
        desired = -direction * m["requiredTorqueNm"] if index < half else -first[steps - 1 - index]
        torque = min(high, max(low, desired)) if low <= high else min(available, max(-available, desired))
        if p["torqueSpeedNoLoadRpm"] is not None:
            def allowed(value):
                end = relative + (value * c - disturbance_rate) * dt
                return abs(value) <= _envelope(p, cap, max(abs(relative), abs(end))) + 1e-14
            if not allowed(torque):
                a, b = 0, 1
                for _ in range(30):
                    fraction = (a + b) / 2
                    if allowed(torque * fraction):
                        a = fraction
                    else:
                        b = fraction
                torque *= a
        if index < half:
            first.append(torque)
        alpha = (p["disturbanceNm"] - torque) / ib
        points.append(dict(t=index * dt, yawDeg=yaw / RAD, rateDegS=rate / RAD,
                           accelDegS2=alpha / RAD, wheelRpm=relative * RPM,
                           torqueNm=torque, momentum=(ib + iw) * rate + iw * relative))
        yaw += rate * dt + alpha * dt ** 2 / 2
        rate += alpha * dt
        relative += (torque * c - disturbance_rate) * dt
    points.append(dict(t=p["durationS"], yawDeg=yaw / RAD, rateDegS=rate / RAD,
                       accelDegS2=disturbance_rate / RAD, wheelRpm=relative * RPM,
                       torqueNm=0, momentum=(ib + iw) * rate + iw * relative))
    return dict(actualFinalDeg=yaw / RAD, finalBodyRateDegS=rate / RAD,
                finalWheelRpm=relative * RPM, points=points, metrics=m)


class ReferenceTests(unittest.TestCase):
    def test_baseline(self):
        m = calculate_metrics()
        self.assertAlmostEqual(m["bodyInertia"], 0.0002028)
        self.assertAlmostEqual(m["wheelInertia"], 0.00002304)
        self.assertAlmostEqual(m["requiredTorqueNm"] * 1000, 0.4247433267653401)
        self.assertAlmostEqual(m["requiredPeakWheelRpm"], 98.0208333333333)
        s = simulate_trajectory()
        self.assertAlmostEqual(s["points"][160]["yawDeg"], 15)
        self.assertAlmostEqual(s["actualFinalDeg"], 30)
        self.assertAlmostEqual(s["finalBodyRateDegS"], 0)
        self.assertAlmostEqual(s["finalWheelRpm"], 0)

    def test_inverse_square_and_torque_limit(self):
        m = calculate_metrics({"durationS": 0.1})
        self.assertAlmostEqual(m["requiredTorqueNm"] / calculate_metrics()["requiredTorqueNm"], 100)
        s = simulate_trajectory({"durationS": 0.1})
        self.assertFalse(m["feasible"])
        self.assertLess(s["actualFinalDeg"], 30)
        self.assertAlmostEqual(s["finalBodyRateDegS"], 0)

    def test_speed_limit(self):
        s = simulate_trajectory({"maxWheelRpm": 30})
        self.assertTrue(s["metrics"]["speedLimited"])
        self.assertLess(s["actualFinalDeg"], 30)
        self.assertLessEqual(max(abs(x["wheelRpm"]) for x in s["points"]), 30 + 1e-9)
        self.assertAlmostEqual(s["finalBodyRateDegS"], 0)

    def test_momentum_and_disturbance_impulse(self):
        for disturbance in [0, 0.0001, -0.0001]:
            s = simulate_trajectory({"disturbanceNm": disturbance})
            for point in s["points"]:
                self.assertAlmostEqual(point["momentum"], disturbance * point["t"], delta=2e-17)

    def test_inclusive_mass_radius_and_geometry(self):
        a, b = calculate_metrics(), calculate_metrics({"wheelMassKg": 0.08})
        self.assertLess(b["bodyInertia"], a["bodyInertia"])
        self.assertLess(b["requiredPeakWheelRpm"], a["requiredPeakWheelRpm"])
        self.assertAlmostEqual(calculate_metrics({"wheelRadiusM": 0.012})["wheelInertia"], a["wheelInertia"] / 4)

    def test_acceleration_and_current_limit(self):
        s = simulate_trajectory({"maxMotorAccelRadS2": 5})
        self.assertTrue(s["metrics"]["accelerationLimited"])
        s = simulate_trajectory({"currentLimitA": 0.1, "torqueConstantNmPerAmp": 0.001})
        self.assertLessEqual(max(abs(x["torqueNm"]) for x in s["points"]), 0.0001 + 1e-12)

    def test_zero_negative_and_invalid(self):
        self.assertEqual(simulate_trajectory({"targetDeg": 0})["actualFinalDeg"], 0)
        self.assertAlmostEqual(simulate_trajectory({"targetDeg": -30})["actualFinalDeg"], -30)
        for values in [{"wheelMassKg": 0.34}, {"targetDeg": math.nan}, {"durationS": 0}]:
            self.assertFalse(validate_params(values)["valid"])
            with self.assertRaises(ValueError):
                calculate_metrics(values)

    def test_linear_torque_speed_model(self):
        s = simulate_trajectory({"durationS": 0.3, "torqueSpeedNoLoadRpm": 150})
        for point in s["points"]:
            self.assertLessEqual(abs(point["torqueNm"]), 0.003 * max(0, 1 - abs(point["wheelRpm"]) / 150) + 2e-12)
        self.assertAlmostEqual(s["finalBodyRateDegS"], 0, delta=2e-6)
        m = calculate_metrics({"torqueSpeedNoLoadRpm": 150})
        at_minimum = calculate_metrics({"torqueSpeedNoLoadRpm": 150, "durationS": m["minimumBangBangDurationS"]})
        self.assertTrue(at_minimum["feasible"])
        self.assertFalse(calculate_metrics({"torqueSpeedNoLoadRpm": 150, "durationS": m["minimumBangBangDurationS"] * 0.999})["feasible"])


if __name__ == "__main__":
    if "--json" in sys.argv:
        cases = [{}, {"durationS": 0.1}, {"maxWheelRpm": 30}, {"disturbanceNm": 0.0001},
                 {"durationS": 0.3, "torqueSpeedNoLoadRpm": 150}]
        print(json.dumps([dict(parameters=p, metrics=calculate_metrics(p),
                               final=simulate_trajectory(p)["actualFinalDeg"]) for p in cases]))
    else:
        unittest.main()
