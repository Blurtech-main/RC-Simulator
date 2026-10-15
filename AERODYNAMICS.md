# Slice 2 aerodynamic model

The runtime now uses a fixed-step six-degree-of-freedom rigid-body model. It does not inject control torques or use a single aircraft-level lift formula. Forces are evaluated at the scale-accurate wing, aileron, stabiliser, elevator, fin and rudder reference panels in `aircraft_model.cpp`; each panel sees translational flow plus the local `omega x r` rotational velocity. The resulting forces and moments are summed before the inertia tensor and attitude quaternion are integrated.

## Data and equations

- Wing: NACA 2412 low-Reynolds-number lift and drag anchors from the CC BY 4.0 [Airfoil 360 v2022 wind-tunnel dataset](https://data.mendeley.com/datasets/dz4bv26ncd/1), interpolated between Re 50,000 and 100,000. The important measured behavior is retained: `CLmax` is approximately 1.06–1.21 near 11–12 degrees, followed by a steep drag rise.
- Finite wing: aspect ratio 7.48, lifting-line slope correction and `CL^2/(pi e AR)` induced drag with `e=0.72`, following the low-Re finite-wing treatment in [Ananda, Sukumar and Selig](https://m-selig.ae.illinois.edu/pubs/AnandaSukumarSelig-2015-AeroSciTech-LowReWings.pdf).
- Post-stall: measured table behavior through 45 degrees, then a smooth flat-plate/Viterna-style full-angle continuation. Forces remain finite and continuous rather than being clipped at stall. The canonical NASA post-stall source is [Viterna and Janetzke](https://ntrs.nasa.gov/api/citations/20090001311/downloads/20090001311.pdf).
- Tail and fin: symmetric finite-surface slope, nonlinear stall continuation and drag buildup. Elevator, aileron and rudder inputs change local panel incidence; they do not add artificial moments.
- Propeller: the 14 x 7 inch geometry uses the UIUC Volume 4 APC 14x7 Thin Electric measurements. The [5963 rpm wind-tunnel table](https://m-selig.ae.illinois.edu/props/volume-4/data/apce_14x7_1012od_5963.txt) supplies `CT(J)`, and the [static table](https://m-selig.ae.illinois.edu/props/volume-4/data/apce_14x7_static_1006od.txt) supplies the static `CT` and `CP`. Runtime thrust and torque use `T=CT rho n^2 D^4` and `Q=CQ rho n^2 D^5`, with `CQ=CP/(2 pi)`.
- Rigid body: force, moment, full diagonal inertia, gyroscopic `omega x I omega`, and normalized quaternion integration at 120 Hz.

## Weather boundary

`AtmosphereSample` is an explicit solver input with density, pressure, temperature, dynamic viscosity, world wind, deterministic gust, rain rate and turbulence intensity. Every surface uses air-relative velocity:

`Vrelative = Vaircraft - Vwind - Vgust + omega x r`

This is ready for a later live-weather provider without coupling networking or locale lookup into flight physics. Rain and turbulence fields are present but deliberately have no speculative aerodynamic correction yet; those effects need a defined weather source and validation dataset.

## Validation

`RCTrainer.exe --self-test` checks deterministic fixed-step behavior plus an aerodynamic suite: coefficient finiteness from -45 to +45 degrees, dynamic-pressure square-law scaling, low-Re stall shape, elevator/aileron/rudder moment signs, stationary-aircraft headwind response, Galilean wind invariance, takeoff, collision/model contracts and replay looping. Results are written to `self-test.txt`.

## Calibration boundary

This is a research-backed engineering flight model, not a claim of certified Cessna flight-test equivalence. The visual aircraft is a 1:6.395 C172 replica, while exact RC mass distribution, control hinge geometry, motor/ESC constants, thrust-stand curves and full-airframe wind-tunnel data do not exist for this particular virtual build. The constants are isolated and documented so measured hardware data can replace them without redesigning the solver.
