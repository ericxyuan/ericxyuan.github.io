/** Single-axis open-loop momentum exchange. All internal quantities use SI units.
 * +yaw is body rotation. Positive motor torque accelerates the wheel inertially,
 * so Ib*bodyAcceleration = disturbance - motorTorque. Wheel RPM is RELATIVE
 * to the body. Total mass includes the rotor. This is not parachute pitch/roll ADCS.
 */
export const DEFAULTS = Object.freeze({
  targetDeg: 30, durationS: 1, torqueLimitNm: 0.003,
  wheelMassKg: 0.040, wheelRadiusM: 0.024, totalMassKg: 0.340,
  bodyRadiusGyrationM: 0.026, inertiaFactor: 1,
  maxWheelRpm: 400, disturbanceNm: 0, maxMotorAccelRadS2: 500,
  torqueSpeedNoLoadRpm: null, currentLimitA: null,
  torqueConstantNmPerAmp: null, voltageV: null,
});
const PI = Math.PI;
const RAD = PI / 180;
const RPM = 60 / (2 * PI);
const finite = Number.isFinite;
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export function validateParams(input = {}) {
  const params = { ...DEFAULTS, ...input };
  const errors = [];
  for (const key of ['targetDeg', 'durationS', 'torqueLimitNm', 'wheelMassKg',
    'wheelRadiusM', 'totalMassKg', 'bodyRadiusGyrationM', 'inertiaFactor',
    'disturbanceNm']) {
    if (!finite(params[key])) errors.push(`${key} must be finite.`);
  }
  for (const key of ['durationS', 'torqueLimitNm', 'wheelMassKg', 'wheelRadiusM',
    'totalMassKg', 'bodyRadiusGyrationM', 'inertiaFactor']) {
    if (finite(params[key]) && params[key] <= 0) errors.push(`${key} must be positive.`);
  }
  if (params.wheelMassKg >= params.totalMassKg) errors.push('Flywheel mass must be below the inclusive total mass.');
  if (params.inertiaFactor > 1) errors.push('Inertia distribution factor must be <=1 for the specified outer/effective radius.');
  if (Math.abs(params.targetDeg) > 360) errors.push('Target magnitude must be <=360 degrees for this demonstrator.');
  if (params.durationS > 120 || params.durationS < 0.001) errors.push('Duration must be between 0.001 and 120 seconds.');
  for (const key of ['maxWheelRpm', 'maxMotorAccelRadS2', 'torqueSpeedNoLoadRpm',
    'currentLimitA', 'torqueConstantNmPerAmp', 'voltageV']) {
    if (params[key] !== null && (!finite(params[key]) || params[key] <= 0)) errors.push(`${key} must be positive or null (not configured).`);
  }
  if ((params.currentLimitA === null) !== (params.torqueConstantNmPerAmp === null)) {
    errors.push('Current limiting requires both current limit and output-shaft torque constant.');
  }
  const ib = (params.totalMassKg - params.wheelMassKg) * params.bodyRadiusGyrationM ** 2;
  const iw = params.inertiaFactor * params.wheelMassKg * params.wheelRadiusM ** 2;
  if (!finite(ib) || !finite(iw) || ib <= 0 || iw <= 0) errors.push('Derived inertias must be positive and finite.');
  return { valid: errors.length === 0, errors, params };
}

function checked(input) {
  const v = validateParams(input);
  if (!v.valid) throw new RangeError(v.errors.join(' '));
  return v.params;
}

function hardware(p) {
  const bodyInertia = (p.totalMassKg - p.wheelMassKg) * p.bodyRadiusGyrationM ** 2;
  const wheelInertia = p.inertiaFactor * p.wheelMassKg * p.wheelRadiusM ** 2;
  const coupling = 1 / bodyInertia + 1 / wheelInertia;
  const currentTorque = p.currentLimitA === null ? Infinity : p.currentLimitA * p.torqueConstantNmPerAmp;
  const torqueCap = Math.min(p.torqueLimitNm, currentTorque);
  const accelTorque = p.maxMotorAccelRadS2 === null ? Infinity : p.maxMotorAccelRadS2 / coupling;
  const effectiveTorqueCap = Math.min(torqueCap, accelTorque);
  const relativeSpeedCap = p.maxWheelRpm === null ? Infinity : p.maxWheelRpm / RPM;
  return { bodyInertia, wheelInertia, coupling, torqueCap, effectiveTorqueCap,
    relativeSpeedCap, bodySpeedCap: relativeSpeedCap * wheelInertia / (bodyInertia + wheelInertia) };
}

function torqueEnvelope(p, h, wheelRelativeSpeed) {
  const speedFactor = p.torqueSpeedNoLoadRpm === null ? 1
    : Math.max(0, 1 - Math.abs(wheelRelativeSpeed) * RPM / p.torqueSpeedNoLoadRpm);
  return h.torqueCap * speedFactor;
}

// Maximum undisturbed rest-to-rest angle within T with symmetric acceleration,
// optional speed plateau and a conservative symmetric linear torque-speed model.
function capacityAngle(p, h, duration, steps = 160) {
  let speed = 0, yawHalf = 0;
  const dt = duration / (2 * steps);
  const bodyPerRelative = h.wheelInertia / (h.bodyInertia + h.wheelInertia);
  for (let i = 0; i < steps; i += 1) {
    const cap = torqueEnvelope(p, h, speed);
    let relAccel = Math.min(cap * h.coupling,
      p.maxMotorAccelRadS2 === null ? Infinity : p.maxMotorAccelRadS2);
    if (p.torqueSpeedNoLoadRpm !== null) {
      const noLoad = p.torqueSpeedNoLoadRpm / RPM;
      relAccel = Math.min(relAccel, h.torqueCap * h.coupling * Math.max(0, 1 - speed / noLoad)
        / (1 + h.torqueCap * h.coupling * dt / noLoad));
    }
    const reachTime = relAccel > 0 ? (h.relativeSpeedCap - speed) / relAccel : Infinity;
    const rampTime = clamp(reachTime, 0, dt);
    yawHalf += bodyPerRelative * (speed * dt + relAccel * rampTime * (dt - rampTime / 2));
    speed += relAccel * rampTime;
  }
  return 2 * yawHalf;
}

export function calculateMetrics(input = {}) {
  const p = checked(input), h = hardware(p);
  const theta = Math.abs(p.targetDeg) * RAD, T = p.durationS;
  const requiredTorqueNm = 4 * h.bodyInertia * theta / T ** 2;
  const peakBodyRateRadS = 2 * theta / T;
  const requiredRelativeSpeed = peakBodyRateRadS * (h.bodyInertia + h.wheelInertia) / h.wheelInertia;
  const requiredPeakWheelRpm = requiredRelativeSpeed * RPM;
  const requiredMotorAccelRadS2 = requiredTorqueNm * h.coupling;
  const torqueMinimumDurationS = Math.sqrt(4 * h.bodyInertia * theta / h.torqueCap);
  const accelerationMinimumDurationS = Math.sqrt(4 * h.bodyInertia * theta / h.effectiveTorqueCap);
  const speedMinimumDurationS = finite(h.bodySpeedCap) ? 2 * theta / h.bodySpeedCap : 0;
  // A fixed-torque bang-bang reference must fit the torque-speed envelope at
  // its peak speed. Solve C/T² <= tau0*(1-D/(T*omega0)) exactly for positive T.
  const curveSpeedTime = p.torqueSpeedNoLoadRpm === null ? 0
    : 2 * theta * (h.bodyInertia + h.wheelInertia) / h.wheelInertia
      / (p.torqueSpeedNoLoadRpm / RPM);
  const torqueSpeedMinimumDurationS = p.torqueSpeedNoLoadRpm === null ? 0
    : (curveSpeedTime + Math.sqrt(curveSpeedTime ** 2 + 4 * torqueMinimumDurationS ** 2)) / 2;
  const aMax = h.effectiveTorqueCap / h.bodyInertia;
  let minimumDurationS = theta === 0 ? 0 : (theta <= h.bodySpeedCap ** 2 / aMax
    ? 2 * Math.sqrt(theta / aMax) : theta / h.bodySpeedCap + h.bodySpeedCap / aMax);
  // With a torque-speed curve there is no constant-acceleration closed form.
  if (p.torqueSpeedNoLoadRpm !== null && theta > 0) {
    let low = 0, high = Math.max(minimumDurationS, 0.01);
    while (capacityAngle(p, h, high) < theta && high < 1e5) high *= 2;
    for (let i = 0; i < 36; i += 1) {
      const middle = (low + high) / 2;
      if (capacityAngle(p, h, middle) < theta) low = middle; else high = middle;
    }
    minimumDurationS = high;
  }
  const torqueLimited = requiredTorqueNm > h.torqueCap * (1 + 1e-10);
  const speedLimited = requiredRelativeSpeed > h.relativeSpeedCap * (1 + 1e-10);
  const accelerationLimited = p.maxMotorAccelRadS2 !== null && requiredMotorAccelRadS2 > p.maxMotorAccelRadS2 * (1 + 1e-10);
  const torqueSpeedLimited = requiredTorqueNm > torqueEnvelope(p, h, requiredRelativeSpeed) * (1 + 1e-10);
  return {
    ...h, requiredTorqueNm, requiredPeakWheelRpm, requiredMotorAccelRadS2,
    peakBodyRateDegS: peakBodyRateRadS / RAD,
    requiredBodyAccelDegS2: 4 * theta / (T ** 2 * RAD),
    accelerationLimitDegS2: h.torqueCap / h.bodyInertia / RAD,
    torqueMinimumDurationS, speedMinimumDurationS, accelerationMinimumDurationS, torqueSpeedMinimumDurationS,
    minimumDurationS, minimumBangBangDurationS: Math.max(accelerationMinimumDurationS, speedMinimumDurationS, torqueSpeedMinimumDurationS),
    torqueRatio: requiredTorqueNm === 0 ? Infinity : h.torqueCap / requiredTorqueNm,
    torqueUsage: requiredTorqueNm / h.torqueCap,
    torqueMarginNm: h.torqueCap - requiredTorqueNm,
    remainingMassKg: p.totalMassKg - p.wheelMassKg,
    rotorMassFraction: p.wheelMassKg / p.totalMassKg,
    torqueLimited, speedLimited, accelerationLimited, torqueSpeedLimited,
    feasible: !(torqueLimited || speedLimited || accelerationLimited || torqueSpeedLimited),
    disturbed: p.disturbanceNm !== 0,
    minimumDurationAssumption: 'Undisturbed time-optimal bang-coast-bang; requested reference remains symmetric bang-bang.',
  };
}

function requestedAt(theta, duration, time) {
  const a = 4 * theta / duration ** 2;
  if (time >= duration - 1e-12) return { yaw: theta, rate: 0, acceleration: 0 };
  if (time < duration / 2) return { yaw: a * time ** 2 / 2, rate: a * time, acceleration: a };
  const remaining = duration - time;
  return { yaw: theta - a * remaining ** 2 / 2, rate: a * remaining, acceleration: -a };
}

/** Returns sampled actual and requested trajectories. Constant-torque steps use
 * exact kinematics. Limits act on applied torque/acceleration; wheel speed is
 * never overwritten/clipped. In the undisturbed case, mirrored braking preserves
 * final rest even when speed saturation produces a coast segment.
 */
export function simulateTrajectory(input = {}, targetDeg = undefined, requestedSteps = 320) {
  const p = checked(targetDeg === undefined ? input : { ...input, targetDeg });
  const metrics = calculateMetrics(p), h = hardware(p);
  const steps = Math.max(40, Math.min(2400, 2 * Math.round(requestedSteps / 2)));
  const dt = p.durationS / steps, half = steps / 2;
  const theta = p.targetDeg * RAD, direction = Math.sign(theta);
  const demand = metrics.requiredTorqueNm;
  let yaw = 0, rate = 0, relativeSpeed = 0;
  let peakWheelRpm = 0, peakRateDegS = 0, peakTorqueNm = 0;
  let speedConstraintUnavoidable = false;
  const points = [], firstHalf = [];
  function sample(t, torque, alpha) {
    const reference = requestedAt(theta, p.durationS, t);
    const momentum = (h.bodyInertia + h.wheelInertia) * rate + h.wheelInertia * relativeSpeed;
    points.push({ t, yawDeg: yaw / RAD, rateDegS: rate / RAD, accelDegS2: alpha / RAD,
      wheelRpm: relativeSpeed * RPM, torqueNm: torque,
      requestedYawDeg: reference.yaw / RAD, requestedRateDegS: reference.rate / RAD,
      requestedAccelDegS2: reference.acceleration / RAD,
      requestedTorqueNm: -h.bodyInertia * reference.acceleration,
      momentum, expectedMomentum: p.disturbanceNm * t });
    peakWheelRpm = Math.max(peakWheelRpm, Math.abs(relativeSpeed * RPM));
    peakRateDegS = Math.max(peakRateDegS, Math.abs(rate / RAD));
    peakTorqueNm = Math.max(peakTorqueNm, Math.abs(torque));
  }
  for (let i = 0; i < steps; i += 1) {
    const cap = torqueEnvelope(p, h, relativeSpeed);
    let low = -cap, high = cap;
    const disturbanceRate = p.disturbanceNm / h.bodyInertia;
    if (p.maxMotorAccelRadS2 !== null) {
      low = Math.max(low, (-p.maxMotorAccelRadS2 + disturbanceRate) / h.coupling);
      high = Math.min(high, (p.maxMotorAccelRadS2 + disturbanceRate) / h.coupling);
    }
    // Endpoint speed bounds constrain torque, not the integrated state. If a
    // disturbance makes the interval unreachable, retain physical actuator limits.
    if (finite(h.relativeSpeedCap)) {
      const speedLow = ((-h.relativeSpeedCap - relativeSpeed) / dt + disturbanceRate) / h.coupling;
      const speedHigh = ((h.relativeSpeedCap - relativeSpeed) / dt + disturbanceRate) / h.coupling;
      if (Math.max(low, speedLow) <= Math.min(high, speedHigh)) {
        low = Math.max(low, speedLow); high = Math.min(high, speedHigh);
      } else speedConstraintUnavoidable = true;
    }
    let desired = i < half ? -direction * demand : -firstHalf[steps - 1 - i];
    let torque = low <= high ? clamp(desired, low, high) : clamp(desired, -cap, cap);
    // Require the optional linear motor envelope at BOTH step endpoints. The
    // resulting piecewise-constant integration is conservative under the curve.
    if (p.torqueSpeedNoLoadRpm !== null) {
      const endpointAllowed = value => {
        const endSpeed = relativeSpeed + (value * h.coupling - disturbanceRate) * dt;
        return Math.abs(value) <= torqueEnvelope(p, h, Math.max(Math.abs(relativeSpeed), Math.abs(endSpeed))) + 1e-14;
      };
      if (!endpointAllowed(torque)) {
        let a = 0, b = 1;
        for (let iteration = 0; iteration < 30; iteration += 1) {
          const factor = (a + b) / 2;
          if (endpointAllowed(torque * factor)) a = factor; else b = factor;
        }
        torque *= a;
      }
    }
    if (i < half) firstHalf.push(torque);
    const alpha = (p.disturbanceNm - torque) / h.bodyInertia;
    sample(i * dt, torque, alpha);
    yaw += rate * dt + 0.5 * alpha * dt ** 2;
    rate += alpha * dt;
    relativeSpeed += (torque * h.coupling - disturbanceRate) * dt;
  }
  sample(p.durationS, 0, p.disturbanceNm / h.bodyInertia);
  const finalErrorDeg = p.targetDeg - yaw / RAD;
  return { targetDeg: p.targetDeg, durationS: p.durationS, feasible: metrics.feasible,
    metrics, points, actualFinalDeg: yaw / RAD, finalErrorDeg,
    finalBodyRateDegS: rate / RAD, finalWheelRpm: relativeSpeed * RPM,
    actualPeakWheelRpm: peakWheelRpm, actualPeakBodyRateDegS: peakRateDegS,
    actualPeakTorqueNm: peakTorqueNm, speedConstraintUnavoidable,
    reachedTarget: Math.abs(finalErrorDeg) < 0.02 && Math.abs(rate / RAD) < 0.02,
    model: 'Open-loop single-axis yaw, zero initial state, constant body disturbance; relative wheel RPM.',
  };
}

export function simulateComparison(input = {}, angles = [10, 15, 20, 30, 45, 60, 90]) {
  const p = checked(input);
  const targets = [...new Set([...angles, p.targetDeg])].sort((a, b) => a - b);
  return targets.map(target => simulateTrajectory(p, target));
}

/** One primary status and at most one secondary recommendation. */
export function engineeringFeedback(input = {}, suppliedMetrics = null, trajectory = null) {
  const validation = validateParams(input);
  if (!validation.valid) return { primary: { severity: 'error', title: 'Invalid physical configuration', message: validation.errors[0] }, secondary: null };
  const p = validation.params, m = suppliedMetrics || calculateMetrics(p);
  let primary, secondary = null;
  if (m.torqueLimited || m.torqueSpeedLimited) {
    primary = { severity: 'warning', title: m.torqueLimited ? 'Torque limit exceeded' : 'Torque-speed envelope exceeded',
      message: `The requested manoeuvre needs ${(m.requiredTorqueNm * 1000).toFixed(3)} mN·m. The graph shows the bounded response.` };
    secondary = `The undisturbed torque-and-speed minimum is ${m.minimumDurationS.toFixed(3)} s; the requested bang-bang profile may require longer.`;
  } else if (m.speedLimited) {
    primary = { severity: 'warning', title: 'Flywheel speed limit exceeded',
      message: `The requested profile needs ${m.requiredPeakWheelRpm.toFixed(1)} RPM; its actual response includes a coast segment.` };
    secondary = `Allow at least ${m.minimumBangBangDurationS.toFixed(3)} s for the symmetric bang-bang profile, or re-plan a constrained profile.`;
  } else if (m.accelerationLimited) {
    primary = { severity: 'warning', title: 'Motor acceleration limit exceeded', message: `Required relative-wheel acceleration is ${m.requiredMotorAccelRadS2.toFixed(1)} rad/s².` };
    secondary = `Increase the duration to at least ${m.minimumBangBangDurationS.toFixed(3)} s in the undisturbed model.`;
  } else if (p.disturbanceNm !== 0) {
    primary = { severity: 'notice', title: 'External torque enabled', message: `A constant ${Math.abs(p.disturbanceNm * 1000).toFixed(3)} mN·m acts in the ${p.disturbanceNm >= 0 ? 'positive' : 'negative'} yaw direction. This open-loop response will drift.` };
    secondary = 'Feasibility figures describe the undisturbed reference; this model has no feedback disturbance rejection.';
  } else if (m.torqueUsage >= 0.75) {
    primary = { severity: 'notice', title: 'Limited torque margin', message: `This manoeuvre requires ${(100 * m.torqueUsage).toFixed(0)}% of configured torque.` };
    secondary = 'Allow more time to improve control margin.';
  } else if (m.rotorMassFraction >= 0.20) {
    primary = { severity: 'notice', title: 'High rotor mass allocation', message: `The rotating wheel alone is ${(100 * m.rotorMassFraction).toFixed(1)}% of inclusive CanSat mass.` };
    secondary = 'Remaining mass is an allocation, not a verified structure, motor, battery and parachute budget.';
  } else if (m.torqueRatio >= 2.5) {
    primary = { severity: 'success', title: 'Ample torque available', message: `The configured motor provides ${finite(m.torqueRatio) ? m.torqueRatio.toFixed(1) : 'unlimited relative'}× the requested torque.` };
    secondary = 'A shorter rotation may be possible, subject to wheel-speed, acceleration and power limits.';
  } else {
    primary = { severity: 'success', title: 'Feasible idealised manoeuvre', message: 'The undisturbed requested profile is within the configured torque, acceleration and wheel-speed bounds.' };
  }
  if (trajectory?.speedConstraintUnavoidable) {
    primary = { severity: 'warning', title: 'Disturbance exceeds speed-hold authority', message: 'Available motor torque cannot maintain the configured wheel-speed bound; the graph retains the physical response.' };
    secondary = 'Reduce disturbance exposure or change actuator authority; wheel speed has not been clipped.';
  }
  return { primary, secondary };
}
