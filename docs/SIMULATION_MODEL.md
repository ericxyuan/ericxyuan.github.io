# Reaction-wheel yaw simulator: model and validation

The browser simulator is a portable engineering model for **single-axis, open-loop yaw**. It runs entirely in the client: no Python service, hardware connection, accounts or external chart requests are needed. The existing static website remains buildless. Chart.js 4.5.1 is self-hosted under `website/vendor/`, together with its MIT licence.

## Conventions and default calculation

Internal units are kilograms, metres, seconds, radians and newton-metres. Display units convert to grams, millimetres, degrees, RPM and mN·m. Positive motor torque acts on the wheel; positive body yaw therefore requires negative motor torque. Wheel speed is **relative to the rotating body**, not inertial speed.

The total mass includes the flywheel. For total mass `M`, rotating flywheel mass `mw`, body radius of gyration `kb`, wheel radius `rw` and inertia distribution factor `f`:

```text
Ib = (M - mw) kb²
Iw = f mw rw²
Ib alpha_body = disturbance - motor_torque
alpha_wheel_relative = motor_torque (1/Iw + 1/Ib) - disturbance/Ib
H = (Ib + Iw) omega_body + Iw omega_wheel_relative
```

`H` remains zero without external torque and grows by `disturbance × time` under the configured constant external torque. A thin rim has `f=1`; a uniform disc has `f=0.5` when `rw` is its outer radius. The input must represent a rotating rotor, rather than a combined motor/mount assembly. Remaining body mass is only an allocation: it does not establish that real hardware fits or meets the complete mass budget.

At the default 340 g total mass, 40 g flywheel, 24 mm radius, 26 mm body radius of gyration and 30° / 1 s command:

| Quantity | Predicted value |
|---|---:|
| Body inertia | 0.0002028 kg·m² |
| Wheel inertia | 0.00002304 kg·m² |
| Required motor torque | 0.424743 mN·m |
| Peak relative wheel speed | 98.0208 RPM |
| Peak body angular velocity | 60°/s |
| Yaw at 0.5 s | 15° |
| Yaw at 1 s | 30° |
| Final body and relative-wheel speed | Zero, within numerical precision |

Increasing rotor mass reduces remaining body mass because total mass stays inclusive. Increasing radius changes wheel inertia quadratically. Neither change establishes rotor strength, safe RPM, electrical endurance or flight performance.

## Requested and achievable motion

The ideal request is a rest-to-rest manoeuvre from zero initial yaw, body rate and relative wheel speed. It uses equal acceleration/deceleration halves:

```text
theta = target_degrees × pi/180
required_torque = 4 Ib abs(theta) / T²
peak_body_rate = 2 abs(theta) / T
peak_relative_wheel_rate = 2 abs(theta) (Ib + Iw) / (T Iw)
```

When demand is below the limit, the model applies the required torque rather than automatically applying the maximum. Actual motion is sampled using exact constant-acceleration kinematics within each integration step. Torque, optional relative-wheel acceleration and wheel-speed constraints modify the torque that is physically applied. The integration never overwrites or clips a wheel-speed state. A speed-limited undisturbed manoeuvre has a coast segment and mirrored braking, so it ends at rest but can fall short of the requested angle.

Requested yaw/acceleration and actual yaw/acceleration are retained separately. The main RPM and required-torque outputs describe the ideal request; actual peak body rate and final yaw come from the constrained trajectory. Identical saturated acceleration paths overlap naturally. No visual separation is invented where the physical response is the same.

The optional torque-speed curve assumes available torque falls linearly from the configured zero-speed intercept to zero at the entered no-load relative speed. Both integration-step endpoints must remain below this assumed envelope, making its discrete application conservative. Real motor curves, braking asymmetry, supply sag, friction and thermal limits require measurements. Optional current limiting in the engine requires both a current limit and an output-shaft torque constant. A voltage value alone cannot establish torque or electrical power.

## Two different minimum durations

The public result uses `minimumBangBangDurationS`: the earliest feasible **equal-half, fixed-torque reference profile**. It combines the torque, relative-wheel acceleration, wheel-speed and optional torque-speed limits. Torque-only minimum is `sqrt(4 Ib abs(theta)/tau_max)`. The speed-only minimum for that profile is `2 abs(theta)/Vmax`, where `Vmax=omega_wheel_limit × Iw/(Ib+Iw)`.

For the linear torque-speed curve, let `C=4 Ib abs(theta)` and `D=2 abs(theta)(Ib+Iw)/Iw`. The required profile must satisfy `C/T² <= tau0(1-D/(T omega0))`. The positive quadratic root is combined with the other minimum-duration constraints. Tests verify that a profile at this boundary is feasible and a slightly shorter profile is not.

The engine also exposes `minimumDurationS`, an undisturbed time-optimal **bang-coast-bang** bound that can be shorter than the equal-half reference minimum. With constant maximum acceleration `A` and body speed limit `V`, it is `2sqrt(abs(theta)/A)` when the speed cap is not reached; otherwise it is `abs(theta)/V + V/A`. An optional motor curve uses a conservative numerical capacity integration and root search. This alternative duration does not silently replace the requested torque schedule or promise that the original request reaches its target.

## Disturbances and limitations

External torque is constant in the body yaw equation, with sign stated in the feedback. The schedule is open loop and does not reject disturbance using sensor feedback. Total angular momentum changes consistently with external angular impulse. Nonzero disturbance can leave final rotation and tracking error. Minimum-duration and feasibility calculations describe the undisturbed reference; the actual disturbed trajectory is shown separately.

This model omits pitch/roll suspension restoring torque, parachute and line dynamics, off-axis wheel placement, three-axis gyroscopic coupling, bearings, gearbox backlash, friction, flexible structure, launch motion and measured actuator dynamics. Body radius of gyration is an estimate, not a CAD-derived inertia tensor. Maximum RPM is an analysis input and does not certify rotor containment. There is no hardware-in-the-loop, physical control test, flight validation or flight-readiness claim in these numerical results.

## Reproduce the checks

From the website repository root, run:

```sh
node --test tests/simulation-physics.test.mjs
python tests/simulation_reference.py
```

The Node suite also calls the independent standard-library Python reference to compare baseline, torque-limited, speed-limited, disturbed and optional motor-curve outputs. Python 3 and a Node version supporting `node:test` are required; set the `PYTHON` environment variable if the Python executable is named differently. No third-party Python dependencies are required.

Validated 16 October 2026: **13 Node tests and 8 Python tests pass**, both with exit 0. Tests cover units/inertias, inclusive mass, radius squared, bang-bang equations and midpoint, torque/acceleration/current/speed limits, motor envelope, inverse-square duration demand, momentum conservation and external impulse, final rest, signed yaw, invalid/nonfinite inputs, continuous sampling, comparison targets and state changes.

There is no configured package-manager build, lint or typecheck workflow in this static HTML/CSS/JavaScript repository; no such check is represented as having run. Source syntax, numerical tests and browser interaction/responsiveness checks are separate forms of evidence. Browser review must verify the actual published interface and should not be inferred from the calculation tests alone.

## Source layout

- `website/simulation-physics.js`: validation, independent SI calculations, constrained trajectories and concise feedback.
- `website/simulations.js`: shared input state, presets, result presentation and interactive charts.
- `website/simulations.html` and `website/simulations.css`: accessible responsive integration with the existing website.
- `tests/simulation-physics.test.mjs`: browser-engine numerical invariants and Python cross-comparison.
- `tests/simulation_reference.py`: matching independent Python equations, integration and executable unit tests.
