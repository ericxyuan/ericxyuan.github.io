import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
// A data URL keeps this standalone test independent of package.json module mode.
const source = await readFile(new URL('../website/simulation-physics.js', import.meta.url), 'utf8');
const physics = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const { DEFAULTS, validateParams, calculateMetrics, simulateTrajectory, simulateComparison, engineeringFeedback } = physics;
const near = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);

test('baseline SI inertia, bang-bang torque, relative wheel speed and half angle', () => {
  const m = calculateMetrics();
  near(m.bodyInertia, 0.0002028); near(m.wheelInertia, 0.00002304);
  near(m.requiredTorqueNm, 4 * 0.0002028 * Math.PI / 6);
  near(m.requiredPeakWheelRpm, 98.02083333333333, 1e-8);
  near(m.peakBodyRateDegS, 60);
  const s = simulateTrajectory();
  near(s.points[0].yawDeg, 0); near(s.points[0].wheelRpm, 0);
  near(s.points[s.points.length >> 1].yawDeg, 15, 1e-10);
  near(s.actualFinalDeg, 30, 1e-10); near(s.finalBodyRateDegS, 0, 1e-10); near(s.finalWheelRpm, 0, 1e-10);
  assert.ok(s.feasible); assert.ok(s.reachedTarget);
});

test('a small request uses required torque, never automatically the full limit', () => {
  const m = calculateMetrics(); const s = simulateTrajectory();
  near(s.actualPeakTorqueNm, m.requiredTorqueNm);
  assert.ok(s.actualPeakTorqueNm < DEFAULTS.torqueLimitNm / 2.5);
  assert.equal(engineeringFeedback().primary.title, 'Ample torque available');
});

test('short duration scales demand as inverse time squared and saturates honestly', () => {
  const m = calculateMetrics({ durationS: 0.1 });
  near(m.requiredTorqueNm / calculateMetrics().requiredTorqueNm, 100);
  const s = simulateTrajectory({ durationS: 0.1 });
  assert.equal(s.feasible, false); assert.ok(s.actualFinalDeg < 30);
  near(s.actualFinalDeg, DEFAULTS.torqueLimitNm * 0.1 ** 2 / (4 * m.bodyInertia) * 180 / Math.PI, 1e-9);
  near(s.finalBodyRateDegS, 0, 1e-8); assert.ok(s.actualPeakTorqueNm <= DEFAULTS.torqueLimitNm);
});

test('mass is inclusive; radius squared and inertia factor affect actual RPM', () => {
  const a = calculateMetrics(), b = calculateMetrics({ wheelMassKg: 0.080 });
  assert.ok(b.bodyInertia < a.bodyInertia); assert.ok(b.wheelInertia > a.wheelInertia);
  assert.ok(b.requiredPeakWheelRpm < a.requiredPeakWheelRpm);
  near(calculateMetrics({ wheelRadiusM: 0.012 }).wheelInertia, a.wheelInertia / 4);
  near(calculateMetrics({ inertiaFactor: 0.5 }).wheelInertia, a.wheelInertia / 2);
});

test('speed cap modifies torque, produces coast and final rest without clipping state', () => {
  const p = { maxWheelRpm: 30 }; const s = simulateTrajectory(p);
  assert.equal(s.metrics.speedLimited, true); assert.equal(s.feasible, false);
  assert.ok(s.actualFinalDeg < 30); assert.ok(s.actualPeakWheelRpm <= 30 + 1e-9);
  near(s.finalBodyRateDegS, 0, 1e-8); near(s.finalWheelRpm, 0, 1e-8);
  assert.ok(s.points.some(x => x.t < 0.5 && x.torqueNm === 0));
  const h = s.metrics; const dt = s.points[1].t;
  for (let i = 1; i < s.points.length; i += 1) {
    const previous = s.points[i - 1], current = s.points[i];
    near((current.wheelRpm - previous.wheelRpm) * 2 * Math.PI / 60 / dt,
      previous.torqueNm * (1 / h.bodyInertia + 1 / h.wheelInertia), 1e-8);
  }
});

test('all sampled states conserve momentum; disturbance changes it by angular impulse', () => {
  for (const disturbanceNm of [0, 0.0001, -0.0001]) {
    const s = simulateTrajectory({ disturbanceNm });
    for (const x of s.points) near(x.momentum, disturbanceNm * x.t, 2e-17);
    if (disturbanceNm) assert.ok(Math.abs(s.finalBodyRateDegS) > 0.1);
  }
});

test('negative yaw has mirrored signs, including wheel-relative RPM and motor torque', () => {
  const a = simulateTrajectory(), b = simulateTrajectory({ targetDeg: -30 });
  for (let i = 0; i < a.points.length; i += 1) {
    near(a.points[i].yawDeg, -b.points[i].yawDeg);
    near(a.points[i].wheelRpm, -b.points[i].wheelRpm);
    near(a.points[i].torqueNm, -b.points[i].torqueNm);
  }
});

test('relative motor acceleration and current torque constraints are enforced', () => {
  const a = simulateTrajectory({ maxMotorAccelRadS2: 5 });
  assert.ok(a.metrics.accelerationLimited); assert.ok(a.actualFinalDeg < 30);
  assert.ok(a.actualPeakTorqueNm * a.metrics.coupling <= 5 + 1e-10);
  const b = simulateTrajectory({ currentLimitA: 0.1, torqueConstantNmPerAmp: 0.001 });
  assert.ok(b.actualPeakTorqueNm <= 0.0001 + 1e-12);
});

test('combined torque/speed minimum uses trapezoid physics; bang-bang minimum is separate', () => {
  const m = calculateMetrics({ maxWheelRpm: 30 });
  const theta = Math.PI / 6, vmax = m.bodySpeedCap, a = m.effectiveTorqueCap / m.bodyInertia;
  near(m.minimumDurationS, theta / vmax + vmax / a);
  near(m.minimumBangBangDurationS, 2 * theta / vmax);
  assert.ok(m.minimumDurationS < m.minimumBangBangDurationS);
});

test('optional torque-speed envelope is applied throughout and preserves rest', () => {
  const p = { durationS: 0.3, torqueSpeedNoLoadRpm: 150 };
  const s = simulateTrajectory(p);
  assert.ok(s.metrics.torqueSpeedLimited);
  for (const x of s.points) {
    assert.ok(Math.abs(x.torqueNm) <= DEFAULTS.torqueLimitNm * Math.max(0, 1 - Math.abs(x.wheelRpm) / 150) + 2e-12);
  }
  near(s.finalBodyRateDegS, 0, 2e-6); near(s.finalWheelRpm, 0, 2e-6);
  const m = calculateMetrics({ torqueSpeedNoLoadRpm: 150 });
  assert.ok(calculateMetrics({ torqueSpeedNoLoadRpm: 150, durationS: m.minimumBangBangDurationS }).feasible);
  assert.ok(!calculateMetrics({ torqueSpeedNoLoadRpm: 150, durationS: m.minimumBangBangDurationS * 0.999 }).feasible);
});

test('zero target and all disabled optional limits are valid; nonfinite/physical invalid rejected', () => {
  near(simulateTrajectory({ targetDeg: 0 }).actualFinalDeg, 0);
  assert.ok(validateParams({ maxWheelRpm: null, maxMotorAccelRadS2: null }).valid);
  for (const p of [{ wheelMassKg: 0.34 }, { durationS: 0 }, { targetDeg: NaN },
    { maxWheelRpm: 0 }, { inertiaFactor: 1.5 }, { currentLimitA: 0.5 }]) {
    assert.equal(validateParams(p).valid, false); assert.throws(() => calculateMetrics(p), RangeError);
    assert.equal(engineeringFeedback(p).primary.title, 'Invalid physical configuration');
  }
});

test('comparison includes selected target; continuous increasing time, finite values and state updates', () => {
  const all = simulateComparison({ targetDeg: 37 });
  assert.equal(all.length, 8); assert.ok(all.some(x => x.targetDeg === 37));
  for (const s of all) for (let i = 0; i < s.points.length; i += 1) {
    assert.ok(Object.values(s.points[i]).every(Number.isFinite));
    if (i) assert.ok(s.points[i].t > s.points[i - 1].t);
  }
  assert.notEqual(simulateTrajectory({ durationS: 0.1 }).actualFinalDeg, simulateTrajectory().actualFinalDeg);
  assert.ok(calculateMetrics({ targetDeg: 45 }).requiredTorqueNm > calculateMetrics().requiredTorqueNm);
});

test('Python reference independently matches baseline, limits, disturbance and motor curve', () => {
  const result = spawnSync(process.env.PYTHON || 'python', [fileURLToPath(new URL('simulation_reference.py', import.meta.url)), '--json'], { encoding: 'utf8' });
  assert.equal(result.status, 0, result.stderr);
  for (const reference of JSON.parse(result.stdout)) {
    const m = calculateMetrics(reference.parameters);
    for (const key of ['bodyInertia', 'wheelInertia', 'requiredTorqueNm', 'requiredPeakWheelRpm',
      'minimumDurationS', 'minimumBangBangDurationS']) near(m[key], reference.metrics[key], 1e-10);
    near(simulateTrajectory(reference.parameters).actualFinalDeg, reference.final, 1e-10);
  }
});
