# Slice 2 aerodynamic model

The runtime now uses a fixed-step six-degree-of-freedom rigid-body model. It does not inject control torques or use a single aircraft-level lift formula. Forces are evaluated at the scale-accurate wing, aileron, stabiliser, elevator, fin and rudder reference panels in `aircraft_model.cpp`; each panel sees translational flow plus the local `omega x r` rotational velocity. The resulting forces and moments are summed before the inertia tensor and attitude quaternion are integrated.

## Data and equations

- Wing: NACA 2412 low-Reynolds-number lift and drag anchors from the CC BY 4.0 [Airfoil 360 v2022 wind-tunnel dataset](https://data.mendeley.com/datasets/dz4bv26ncd/1), interpolated between Re 50,000 and 100,000. The important measured behavior is retained: `CLmax` is approximately 1.06–1.21 near 11–12 degrees, followed by a steep drag rise.
- Finite wing: aspect ratio 7.48, lifting-line slope correction and `CL^2/(pi e AR)` induced drag with `e=0.72`, following the low-Re finite-wing treatment in [Ananda, Sukumar and Selig](https://m-selig.ae.illinois.edu/pubs/AnandaSukumarSelig-2015-AeroSciTech-LowReWings.pdf).
- Post-stall: measured table behavior through 45 degrees, then a smooth flat-plate/Viterna-style full-angle continuation. Forces remain finite and continuous rather than being clipped at stall. The canonical NASA post-stall source is [Viterna and Janetzke](https://ntrs.nasa.gov/api/citations/20090001311/downloads/20090001311.pdf).
- Tail and fin: symmetric finite-surface slope, nonlinear stall continuation and drag buildup. Elevator, aileron and rudder inputs change local panel incidence; they do not add artificial moments.
- Propeller: the 14 x 7 inch geometry uses the UIUC Volume 4 APC 14x7 Thin Electric [5963 rpm wind-tunnel table](https://m-selig.ae.illinois.edu/props/volume-4/data/apce_14x7_1012od_5963.txt) through `J=0.3356`, the matching [6038 rpm run](https://m-selig.web.engr.illinois.edu/props/volume-4/data/apce_14x7_1013od_6038.txt) through `J=0.6098`, and the complete measured RPM sweep from the [static table](https://m-selig.ae.illinois.edu/props/volume-4/data/apce_14x7_static_1006od.txt). Runtime interpolation uses `T=CT rho n^2 D^4` and `Q=CP/(2 pi) rho n^2 D^5`; it does not replace the table with a single fitted line.
- Rigid body: force, moment, full diagonal inertia, gyroscopic `omega x I omega`, and normalized quaternion integration at 120 Hz.

## Weather boundary

`AtmosphereSample` is an explicit solver input with density, pressure, temperature, dynamic viscosity, world wind, deterministic gust, rain rate and turbulence intensity. Every surface uses air-relative velocity:

`Vrelative = Vaircraft - Vwind - Vgust + omega x r`

This is ready for a later live-weather provider without coupling networking or locale lookup into flight physics. Rain and turbulence fields are present but deliberately have no speculative aerodynamic correction yet; those effects need a defined weather source and validation dataset.

## Validation

`RCTrainer.exe --self-test` checks deterministic fixed-step behavior plus an aerodynamic suite: coefficient finiteness from -45 to +45 degrees, dynamic-pressure square-law scaling, low-Re stall shape, elevator/aileron/rudder moment signs, stationary-aircraft headwind response, Galilean wind invariance, takeoff, collision/model contracts and replay looping. The ground suite additionally checks individual tyre-force directions, opposite nose-steering response, zero airborne contact, braking versus coasting distance, soft-touchdown settling and bounded ground effect. Results are written to `self-test.txt`.

## Landing gear, tyres and ground effect

`src/ground_dynamics.*` evaluates three aircraft-defined wheel contacts. Each tyre receives the point velocity `Vcg + omega x r`, a spring/damper normal load, longitudinal rolling/braking force and bounded lateral force. The nose wheel rotates with rudder command; yaw emerges from the contact force at its actual mount rather than an injected steering moment. `B` supplies an explicit brake command. Normal and friction forces are bounded, and the 120 Hz solver retains a 35 mm suspension travel stop to prevent hard-step tunnelling.

Airframe collision proxies remain separate from wheel support. Small overlap from conservative rectangular fuselage proxies is tolerated during normal suspension motion and rotation; deep airframe contact, invalid gear attitude and excessive flown touchdown speed remain crash conditions.

Ground effect uses wing height divided by span to reduce induced flow, induced angle and the induced-drag term continuously near the runway. The HUD reports the resulting effect percentage. This is a bounded lifting-line approximation, not validation against a full-airframe ground-effect wind-tunnel dataset.

## Calibration boundary

This is a research-backed engineering flight model, not a claim of certified Cessna flight-test equivalence. The visual aircraft is a 1:6.395 C172 replica, while exact RC mass distribution, control hinge geometry, motor/ESC constants, thrust-stand curves and full-airframe wind-tunnel data do not exist for this particular virtual build. The constants are isolated and documented so measured hardware data can replace them without redesigning the solver.

The same boundary applies to tyre friction, gear stiffness/damping and braking coefficients: current values are stable model-scale starting points pending measurements. See [FLIGHT_TEST_PLAN.md](FLIGHT_TEST_PLAN.md) for the evidence ladder and expert flight-test programme.

## Trainer assistance

The default mode is **REALISTIC** and exposes the full motor reaction and gyroscopic moments. F4 enables **ASSISTED**, which cancels the propulsion roll reaction at centred aileron while airborne. Runway tyre contact carries the static shaft reaction in either mode. The high-wing bank-restoring term remains a low-order airframe stability approximation; it is not propeller torque removal. Fin force opposes lateral flow to restore heading and damp yaw rate.

## Reusable electric propulsion system

`src/propulsion.*` is independent of the Cessna solver. Each installed unit carries its own mount point and axis, spin direction, diameter, pitch identity, shaft inertia, motor Kv and resistance, no-load current, ESC current limit, battery open-circuit voltage and resistance, drivetrain efficiency, dynamic `CT/CP` map, and RPM-dependent static map. `AircraftModel` owns a validated list of these components, so later aircraft can install tractor, pusher, twin, or counter-rotating systems without adding another hard-coded force block. A per-unit command boundary supports later differential-throttle and engine-out mixers.

The fixed-step state contains actual shaft angular velocity and phase. RPM emerges from the balance

`I d(omega)/dt = Qmotor - Qprop`

rather than from a throttle-to-RPM time constant. The average-value BLDC/ESC calculation uses back EMF, winding resistance, battery sag and current limiting. The airframe receives the equal-and-opposite motor reaction, mount-offset moment, and fixed-axis gyroscopic moment. The same integrated phase rotates the propeller blade mesh and is saved in replay data.

For positive thrust, actuator-disk momentum theory estimates induced velocity. A contracted, radially smoothed slipstream is sampled at each aerodynamic panel, so elevator, rudder and tail response can exist at low forward speed. The implementation remains allocation-free inside the fixed-step solver.

Telemetry exposes voltage, current, shaft power, RPM, advance ratio, efficiency and whether the current point lies inside the measured propeller map. The HUD warns `OUTSIDE MEASURED PROP MAP`; clamped coefficients outside that map are a bounded fallback, not validation evidence.

The configured 4S / 620 Kv motor, resistance, battery resistance, drivetrain efficiency and inertia are plausible replaceable calibration values because no manufacturer motor file, battery discharge test or complete-aircraft thrust-stand record has been supplied. Normal-operation propeller coefficients are measured. Stopped-prop windmilling, regeneration, reverse rotation, high-incidence disc loading and full off-axis blade-element behaviour require dedicated datasets before they can be claimed as validated. Pitch is metadata identifying a pitch-specific measured map; changing that number alone deliberately cannot fabricate another propeller's aerodynamics.

The propulsion regression suite checks density scaling, advance-ratio thrust reduction, reaction-torque direction, counter-rotating twin cancellation, current-limited shaft convergence, phase bounds, slipstream coverage and deterministic integration. The current static coupled solution converges near 7423 rpm; that is a solver result, not a commanded target.
