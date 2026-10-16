import { DEFAULTS, validateParams, calculateMetrics, simulateComparison, engineeringFeedback } from './simulation-physics.js';

const primary = [
  ['targetDeg', 'Target yaw rotation', '°', 5, 90, 1, 1],
  ['durationS', 'Manoeuvre duration', 's', .1, 5, .05, 1],
  ['torqueLimitNm', 'Motor torque limit', 'mN·m', .1, 20, .1, 1000],
  ['wheelMassKg', 'Flywheel mass', 'g', 5, 80, 1, 1000],
  ['wheelRadiusM', 'Flywheel effective radius', 'mm', 10, 28, .5, 1000],
  ['totalMassKg', 'Total CanSat mass', 'g', 300, 350, 1, 1000],
];
const advanced = [
  ['bodyRadiusGyrationM', 'Body radius of gyration', 'mm', 10, 33, .5, 1000],
  ['inertiaFactor', 'Wheel inertia distribution factor', '', .5, 1, .01, 1],
  ['maxWheelRpm', 'Maximum permitted wheel speed', 'RPM', 10, 5000, 10, 1],
  ['disturbanceNm', 'Constant external yaw torque', 'mN·m', -.1, .1, .005, 1000],
  ['maxMotorAccelRadS2', 'Maximum relative motor acceleration', 'rad/s²', 10, 10000, 10, 1],
];
const modes = {
  yawDeg: ['Yaw angle', 'Yaw angle (°)', '°', 1],
  rateDegS: ['Angular velocity', 'Angular velocity (°/s)', '°/s', 1],
  accelDegS2: ['Angular acceleration', 'Angular acceleration (°/s²)', '°/s²', 1],
  wheelRpm: ['Flywheel RPM', 'Relative wheel speed (RPM)', 'RPM', 1],
  torqueNm: ['Motor torque', 'Motor torque (mN·m)', 'mN·m', 1000],
};
const state = { ...DEFAULTS };
const hiddenTargets = new Set();
const charts = [];
let trajectories = [];
let metrics;
let selected;
let hoveredTarget = null;
let inspectionTime = null;
let renderPending = false;
let announceTimer;
const $ = (id) => document.getElementById(id);
const pretty = (n, digits = 2) => Number.isFinite(n) ? Number(n.toFixed(digits)).toLocaleString('en-GB', { maximumFractionDigits: digits }) : '—';

function addControl(container, spec) {
  const [key, label, unit, min, max, step, scale] = spec;
  const wrapper = document.createElement('div');
  wrapper.className = 'sim-control';
  const precision = Math.min(3, (String(step).split('.')[1] || '').length);
  wrapper.innerHTML = `<div class="sim-control__head"><label for="${key}-range">${label}</label><div class="sim-control__value"><input class="sim-number" id="${key}-number" type="number" min="${min}" max="${max}" step="${step}" aria-label="${label} in ${unit || 'dimensionless units'}"><span>${unit}</span></div></div><input id="${key}-range" type="range" min="${min}" max="${max}" step="${step}" aria-label="${label}" aria-describedby="${key}-ends"><div class="sim-control__ends" id="${key}-ends"><span>${min} ${unit}</span><span>${max} ${unit}</span></div>`;
  container.append(wrapper);
  const range = $(key + '-range');
  const number = $(key + '-number');
  const setValue = (value) => {
    state[key] = value / scale;
    range.value = String(value);
    number.value = String(Number(value.toFixed(precision)));
    range.setAttribute('aria-valuetext', `${number.value} ${unit}`);
    number.removeAttribute('aria-invalid');
    document.querySelectorAll('[data-preset]').forEach((b) => b.setAttribute('aria-pressed', 'false'));
    requestRender();
  };
  range.addEventListener('input', () => setValue(Number(range.value)));
  number.addEventListener('input', () => {
    if (!number.value || !number.validity.valid) { number.setAttribute('aria-invalid', 'true'); return; }
    setValue(Number(number.value));
  });
  number.addEventListener('change', () => {
    if (!number.value || !number.validity.valid) setValue(state[key] * scale);
  });
}
primary.forEach((s) => addControl($('primary-controls'), s));
advanced.forEach((s) => addControl($('advanced-controls'), s));
addControl($('torque-curve-controls'), ['torqueSpeedNoLoadRpm', 'Motor no-load speed', 'RPM', 100, 10000, 100, 1]);

function syncInputs() {
  [...primary, ...advanced].forEach(([key, , unit, , , , scale]) => {
    const value = state[key] * scale;
    $(key + '-range').value = String(value);
    $(key + '-number').value = String(Number(value.toFixed(4)));
    $(key + '-range').setAttribute('aria-valuetext', `${pretty(value, 3)} ${unit}`);
  });
  const curve = state.torqueSpeedNoLoadRpm != null;
  $('enable-torque-curve').checked = curve;
  $('torque-curve-controls').hidden = !curve;
  $('torqueSpeedNoLoadRpm-range').value = String(state.torqueSpeedNoLoadRpm || 2000);
  $('torqueSpeedNoLoadRpm-number').value = String(state.torqueSpeedNoLoadRpm || 2000);
}
$('enable-torque-curve').addEventListener('change', (e) => {
  state.torqueSpeedNoLoadRpm = e.target.checked ? Number($('torqueSpeedNoLoadRpm-number').value || 2000) : null;
  $('torque-curve-controls').hidden = !e.target.checked;
  requestRender();
});
const presets = {
  standard: { targetDeg: 30, durationS: 1, wheelMassKg: .04, torqueLimitNm: .003 },
  fast: { targetDeg: 30, durationS: .3, wheelMassKg: .04, torqueLimitNm: .003 },
  large: { targetDeg: 90, durationS: 1.5, wheelMassKg: .04, torqueLimitNm: .003 },
  light: { targetDeg: 30, durationS: 1, wheelMassKg: .015, torqueLimitNm: .003 },
};
document.querySelectorAll('[data-preset]').forEach((button) => button.addEventListener('click', () => {
  Object.assign(state, DEFAULTS, presets[button.dataset.preset]);
  document.querySelectorAll('[data-preset]').forEach((b) => b.setAttribute('aria-pressed', String(b === button)));
  hiddenTargets.clear(); syncInputs(); requestRender();
}));
$('reset-simulation').addEventListener('click', () => {
  Object.assign(state, DEFAULTS); hiddenTargets.clear(); inspectionTime = null;
  syncInputs(); document.querySelectorAll('[data-preset]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.preset === 'standard'))); requestRender();
});

function interpolate(points, t, key) {
  if (t <= points[0].t) return points[0][key];
  if (t >= points.at(-1).t) return points.at(-1)[key];
  let low = 0; let high = points.length - 1;
  while (high - low > 1) { const mid = (low + high) >> 1; if (points[mid].t <= t) low = mid; else high = mid; }
  const a = points[low]; const b = points[high];
  // Acceleration and torque are piecewise constant between integration events.
  if (key === 'accelDegS2' || key === 'torqueNm') return a[key];
  return a[key] + (b[key] - a[key]) * (t - a.t) / (b.t - a.t);
}
const crosshair = {
  id: 'engineering-crosshair',
  afterEvent(chart, { event, inChartArea }) {
    if (event.type === 'mouseout') { hoveredTarget = null; applyHighlight(); return; }
    if (!inChartArea) return;
    inspectionTime = Math.max(0, Math.min(chart.scales.x.max, chart.scales.x.getValueForPixel(event.x)));
    let nearest = null; let distance = 14;
    const [, , , scale] = modes[chart.$quantity];
    trajectories.forEach((trajectory) => {
      if (hiddenTargets.has(trajectory.targetDeg)) return;
      const value = interpolate(trajectory.points, inspectionTime, chart.$quantity) * scale;
      const d = Math.abs(chart.scales.y.getPixelForValue(value) - event.y);
      if (d < distance) { nearest = trajectory.targetDeg; distance = d; }
    });
    if (hoveredTarget !== nearest) { hoveredTarget = nearest; applyHighlight(); }
    drawInspection();
  },
  afterDraw(chart) {
    if (inspectionTime == null || !trajectories.length) return;
    const x = chart.scales.x.getPixelForValue(inspectionTime);
    const { top, bottom, left, right } = chart.chartArea;
    if (x < left || x > right) return;
    const ctx = chart.ctx;
    ctx.save(); ctx.strokeStyle = '#668696'; ctx.lineWidth = 1; ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, bottom); ctx.stroke(); ctx.setLineDash([]);
    trajectories.forEach((trajectory) => {
      if (hiddenTargets.has(trajectory.targetDeg)) return;
      const value = interpolate(trajectory.points, inspectionTime, chart.$quantity) * modes[chart.$quantity][3];
      const y = chart.scales.y.getPixelForValue(value);
      if (y < top || y > bottom) return;
      ctx.fillStyle = trajectory.targetDeg === state.targetDeg ? '#0788cf' : '#6e8b9d';
      ctx.beginPath(); ctx.arc(x, y, trajectory.targetDeg === state.targetDeg ? 4 : 2.5, 0, Math.PI * 2); ctx.fill();
    });
    ctx.restore();
  },
};

function makeChart(id, quantity) {
  const chart = new Chart($(id), {
    type: 'line', data: { datasets: [] }, plugins: [crosshair],
    options: {
      responsive: true, maintainAspectRatio: false, animation: false, parsing: false, normalized: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { display: false }, tooltip: { enabled: false } },
      elements: { point: { radius: 0, hitRadius: 8 }, line: { tension: 0, borderWidth: 1.5 } },
      scales: {
        x: { type: 'linear', min: 0, title: { display: true, text: 'Time (s)', color: '#40535e' }, grid: { color: '#14283108' }, ticks: { maxTicksLimit: 7, color: '#4e626f' } },
        y: { title: { display: true, text: modes[quantity][1], color: '#40535e' }, grid: { color: '#14283110' }, ticks: { maxTicksLimit: 6, color: '#4e626f' }, grace: '8%' },
      },
    },
  });
  chart.$quantity = quantity; charts.push(chart); return chart;
}
const yawChart = makeChart('yaw-chart', 'yawDeg');
const detailChart = makeChart('detail-chart', 'accelDegS2');

function updateChart(chart) {
  const key = chart.$quantity; const scale = modes[key][3];
  chart.data.datasets = trajectories.map((trajectory) => ({
    label: `${trajectory.targetDeg}°${trajectory.feasible ? '' : ' · limited'}`,
    targetDeg: trajectory.targetDeg, hidden: hiddenTargets.has(trajectory.targetDeg),
    data: trajectory.points.map((point) => ({ x: point.t, y: point[key] * scale })),
    borderColor: '#718b9c80', borderWidth: 1.3, order: 2,
    stepped: key === 'accelDegS2' || key === 'torqueNm' ? 'after' : false,
  }));
  const end = Math.max(...selected.points.map((p) => p.t));
  if ((!selected.feasible || state.disturbanceNm !== 0) && (key === 'yawDeg' || key === 'accelDegS2')) {
    const requestedKey = key === 'yawDeg' ? 'requestedYawDeg' : 'requestedAccelDegS2';
    chart.data.datasets.push({ label: 'Selected ideal request', data: selected.points.map((p) => ({ x: p.t, y: p[requestedKey] })), borderColor: '#167eae80', borderDash: [6, 5], borderWidth: 1.4, order: 3, stepped: key === 'accelDegS2' ? 'after' : false });
  }
  if (key === 'accelDegS2') {
    for (const sign of [-1, 1]) {
      const accelerationLimit = (state.disturbanceNm + sign * state.torqueLimitNm) / metrics.bodyInertia * 180 / Math.PI;
      chart.data.datasets.push({ label: `${sign > 0 ? '+' : '−'} motor limit including disturbance`, data: [{ x: 0, y: accelerationLimit }, { x: end, y: accelerationLimit }], borderColor: '#a0784e85', borderDash: [5, 5], borderWidth: 1, order: 4 });
    }
  }
  chart.options.scales.x.max = end;
  chart.options.scales.y.title.text = modes[key][1];
  chart.canvas.setAttribute('aria-label', `${modes[key][1]} versus time for ${trajectories.length} target manoeuvres. ${$('chart-accessible-summary').textContent}`);
  chart.update('none');
}
function applyHighlight() {
  charts.forEach((chart) => {
    chart.data.datasets.forEach((dataset) => {
      if (dataset.targetDeg == null) return;
      const active = dataset.targetDeg === state.targetDeg;
      const hover = dataset.targetDeg === hoveredTarget;
      dataset.borderColor = active ? '#0788cf' : hover ? '#315e78' : '#708b9d75';
      dataset.borderWidth = active ? 3 : hover ? 2.5 : 1.3;
      dataset.order = active ? 0 : hover ? 1 : 2;
    });
    chart.update('none');
  });
}
function drawInspection() {
  if (inspectionTime == null) return;
  $('inspect-time').value = String(inspectionTime);
  $('inspect-time-label').textContent = `${inspectionTime.toFixed(2)} s`;
  charts.forEach((chart, i) => {
    const key = chart.$quantity; const [, , unit, scale] = modes[key];
    const tooltip = $(i === 0 ? 'yaw-tooltip' : 'detail-tooltip');
    tooltip.hidden = false;
    tooltip.replaceChildren();
    const time = document.createElement('strong'); time.textContent = `${inspectionTime.toFixed(3)} s`; tooltip.append(time);
    trajectories.forEach((trajectory) => {
      if (hiddenTargets.has(trajectory.targetDeg)) return;
      const row = document.createElement('div');
      if (trajectory.targetDeg === state.targetDeg) row.className = 'selected';
      const name = document.createElement('span'); name.textContent = `${trajectory.targetDeg}°${trajectory.feasible ? '' : ' · limited'}`;
      const value = document.createElement('span'); value.textContent = `${pretty(interpolate(trajectory.points, inspectionTime, key) * scale, key === 'torqueNm' ? 4 : 2)} ${unit}`;
      row.append(name, value); tooltip.append(row);
    });
    chart.draw();
  });
}
$('inspect-time').addEventListener('input', (e) => {
  inspectionTime = Number(e.target.value); drawInspection();
  $('chart-accessible-summary').textContent = `At ${inspectionTime.toFixed(2)} s, the selected ${state.targetDeg}° target reaches ${pretty(interpolate(selected.points, inspectionTime, 'yawDeg'))}° yaw and ${pretty(interpolate(selected.points, inspectionTime, 'rateDegS'))}°/s.`;
});
$('graph-mode').addEventListener('change', (e) => {
  detailChart.$quantity = e.target.value;
  $('detail-chart-title').textContent = modes[e.target.value][0];
  $('detail-chart-note').textContent = e.target.value === 'accelDegS2' ? 'Positive acceleration in the first half; braking in the second. Dashed boundaries show the motor torque limit. Limited manoeuvres may share the same actual profile.' : e.target.value === 'wheelRpm' ? 'Signed speed relative to the body. Negative wheel motion produces positive body yaw.' : e.target.value === 'torqueNm' ? 'Signed torque on the wheel. The body receives equal and opposite internal torque.' : 'Actual simulated response under the configured motor and wheel limits.';
  updateChart(detailChart); applyHighlight(); drawInspection();
});

function makeLegend() {
  const oldFocus = document.activeElement?.dataset?.target;
  $('trajectory-legend').replaceChildren();
  trajectories.forEach((trajectory) => {
    const button = document.createElement('button'); button.type = 'button'; button.dataset.target = trajectory.targetDeg;
    button.textContent = `${trajectory.targetDeg}°${trajectory.feasible ? '' : ' · limited'}`;
    button.setAttribute('aria-pressed', String(!hiddenTargets.has(trajectory.targetDeg)));
    button.setAttribute('aria-label', `Show ${trajectory.targetDeg} degree target${trajectory.feasible ? '' : ', limited by configured hardware'}`);
    if (trajectory.targetDeg === state.targetDeg) button.className = 'is-selected';
    button.addEventListener('click', () => {
      if (hiddenTargets.has(trajectory.targetDeg)) hiddenTargets.delete(trajectory.targetDeg); else hiddenTargets.add(trajectory.targetDeg);
      button.setAttribute('aria-pressed', String(!hiddenTargets.has(trajectory.targetDeg)));
      charts.forEach(updateChart); applyHighlight(); drawInspection();
    });
    button.addEventListener('pointerenter', () => { hoveredTarget = trajectory.targetDeg; applyHighlight(); });
    button.addEventListener('pointerleave', () => { hoveredTarget = null; applyHighlight(); });
    $('trajectory-legend').append(button);
    if (String(trajectory.targetDeg) === oldFocus) button.focus();
  });
}
function requestRender() { if (!renderPending) { renderPending = true; requestAnimationFrame(render); } }
function render() {
  renderPending = false;
  const validation = validateParams(state);
  $('simulation-error').hidden = validation.valid;
  if (!validation.valid) { $('simulation-error').textContent = validation.errors.join(' '); return; }
  metrics = calculateMetrics(state);
  trajectories = simulateComparison(state);
  selected = trajectories.find((t) => t.targetDeg === state.targetDeg);
  hiddenTargets.delete(state.targetDeg);
  $('result-rpm').textContent = pretty(metrics.requiredPeakWheelRpm, 1);
  $('result-torque').textContent = pretty(metrics.requiredTorqueNm * 1000, 4);
  $('result-rate').textContent = pretty(Math.max(...selected.points.map((p) => Math.abs(p.rateDegS))), 1);
  // This value preserves the exact requested equal-half bang-bang profile.
  $('result-duration').textContent = pretty(metrics.minimumBangBangDurationS ?? metrics.minimumDurationS, 3);
  $('wheel-mass-label').textContent = `${pretty(state.wheelMassKg * 1000, 1)} g`;
  $('body-mass-label').textContent = `${pretty((state.totalMassKg - state.wheelMassKg) * 1000, 1)} g`;
  $('wheel-mass-bar').style.width = `${state.wheelMassKg / state.totalMassKg * 100}%`;
  const torquePercent = 100 * metrics.requiredTorqueNm / state.torqueLimitNm;
  $('secondary-results').replaceChildren();
  for (const label of [`Body inertia ${metrics.bodyInertia.toExponential(3)} kg·m²`, `Wheel inertia ${metrics.wheelInertia.toExponential(3)} kg·m²`, `${pretty(torquePercent, 1)}% of motor torque`, `Actual final yaw ${pretty(selected.actualFinalDeg, 2)}°`]) {
    const span = document.createElement('span'); span.textContent = label; $('secondary-results').append(span);
  }
  const feedback = engineeringFeedback(state, metrics, selected);
  $('feedback-primary').textContent = feedback.primary.title;
  $('feedback-secondary').textContent = [feedback.primary.message, feedback.secondary].filter(Boolean).join(' ');
  document.querySelector('.sim-feedback').dataset.tone = !selected.feasible || torquePercent > 75 ? 'warning' : 'normal';
  $('inspect-time').max = String(selected.points.at(-1).t);
  if (inspectionTime != null) inspectionTime = Math.min(inspectionTime, selected.points.at(-1).t);
  clearTimeout(announceTimer);
  announceTimer = setTimeout(() => {
    $('chart-accessible-summary').textContent = `Selected target ${state.targetDeg}° in ${state.durationS.toFixed(2)} s. Actual yaw at command end ${pretty(selected.actualFinalDeg)}°. Required torque ${pretty(metrics.requiredTorqueNm * 1000, 4)} mN·m. Required relative wheel speed ${pretty(metrics.requiredPeakWheelRpm, 1)} RPM. ${feedback.primary.title}.`;
    charts.forEach((chart) => chart.canvas.setAttribute('aria-label', `${modes[chart.$quantity][1]} versus time. ${$('chart-accessible-summary').textContent}`));
  }, 180);
  makeLegend(); charts.forEach(updateChart); applyHighlight(); drawInspection();
}
syncInputs(); render();
