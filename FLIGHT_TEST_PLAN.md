# RC flight-test and world-class validation plan

## Purpose and evidence boundary

The simulator will be developed as if every physics change must pass an engineering flight-test programme. This process builds strong RC flight-test expertise into the project even when an experienced local RC pilot is unavailable. It does not invent first-hand stick time or call automated results pilot certification. Human RC-pilot acceptance remains a separate final evidence layer.

The operating rule is: **one declared change, one test card, repeatable telemetry, explicit abort limits, and no realism claim beyond the evidence.**

## Built-in virtual test-pilot report

`RCTrainer.exe --test-pilot` runs a closed-loop takeoff/climb card plus matched-state manoeuvre pairs. Each elevator, aileron and rudder command is compared with an identical neutral aircraft, which prevents existing momentum or a previous command from producing a false PASS. A direct attitude/AoA/world-vertical-force invariant checks that nose-down reduces aerodynamic support and nose-up increases it. It generates `reports/virtual-test-pilot.html`, telemetry CSV and `virtual-test-pilot-oracle.csv`; every failed expected corridor includes a likely subsystem to inspect. Mandatory gates cover finite state, crash state, the complete 30-second trajectory, altitude and lateral corridors, control signs and lift direction.

The chase camera follows horizontal flight-path heading through a rate-limited damper and renders the aircraft relative to that heading. Short yaw oscillations remain visible on the model but no longer rotate the entire ground plane from side to side.

An F5 flight recording uses the same reporter when **Stop** is pressed and writes `reports/transmitter-latest.html` and `.csv`. For meaningful channel-sign screening, fly one control input at a time above 8 m/s and include both directions. Mixed correction inputs can weaken or confound correlation, so the report labels unexercised and inconclusive axes instead of inventing a verdict.

## Flight-test knowledge programme

The project maintainer and AI collaborator must use these primary references as the shared curriculum and convert each relevant subject into simulator test cards:

- FAA [Pilot's Handbook of Aeronautical Knowledge](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/phak): principles of flight, stability, controls, weight and balance, performance, and weather.
- FAA [Airplane Flying Handbook](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/airplane_handbook): ground operations, basic manoeuvres, energy management, takeoffs, circuits, approaches, landings, and upset recovery.
- FAA [AC 90-89C Amateur-Built Aircraft and Ultralight Flight Testing Handbook](https://www.faa.gov/airports/resources/advisory_circulars/index.cfm/go/document.information/documentID/1041650): phased test planning, cards, risk controls, envelope expansion, and recording results.
- NASA [Introduction to Flight Test Engineering — Handling Qualities](https://ntrs.nasa.gov/citations/20050210000): stability/control response, pilot workload, and safe operational-envelope assessment.

For RC-specific competence, every lesson must be translated to transmitter conventions, model-scale Reynolds number, electric propulsion, short visual range, lack of onboard bodily cues, and the simulator's `REALISTIC` versus `ASSISTED` modes. Each physics change must record which reference concept it implements, what is measured, and what remains an approximation.

## Test-card format

Every card records:

1. build/hash, aircraft definition and physics mode;
2. mass, CG, inertia, control throws, propulsion and battery configuration;
3. initial position, attitude, velocity, trim, atmosphere and runway state;
4. exact control schedule and fixed timestep;
5. measured outputs and numeric pass/fail limits;
6. abort/crash conditions;
7. deterministic replay identifier and telemetry file;
8. observer/pilot notes, controller path and confidence level.

Automated cards run at least twice from identical state. Any nondeterministic result, NaN, sign reversal, unexplained energy gain, runway tunnelling, or control-direction disagreement is a failure.

## Ground-physics slice

Implemented foundation:

- three independent landing-gear contacts with spring and damping forces;
- bounded longitudinal rolling resistance and commanded wheel braking;
- bounded lateral tyre force and rudder-linked nose-wheel steering;
- force-at-contact moments rather than global velocity damping;
- separate gear support, suspension travel stop, and airframe-strike crash logic;
- span/height-based reduction of induced flow near the ground;
- HUD gear-contact, brake and ground-effect indications;
- deterministic gates for force directions, steering symmetry, airborne isolation, coast-versus-brake distance, soft-touchdown settling and ground effect.

Current calibration values are engineering starting points, not measured tyre or oleo data. Their acceptance class is **deterministic model behaviour**, pending scale-aircraft measurements and pilot judgement.

## Ground and landing envelope to complete

| Card | Required measurements | Gate before release claim |
|---|---|---|
| Parked settle | wheel compression/load, CG motion, angular rates | finite, stable support; no drift or energy growth |
| Straight taxi | centerline error, heading, wheel loads, speed | no uncommanded swerve or wheel lift |
| Steering S-turn | signed yaw/lateral response, roll, radius | opposite commands produce bounded opposite turns |
| Coast and brake | stopping distance/time, reverse chatter | braking is shorter and cannot accelerate backwards |
| Crosswind taxi | signed wind, counter-steer, centerline error | response reverses with wind; controllable runway tracking |
| Soft landing | sink rate, peak load/compression, rebound | settles without crash, tunnelling or sustained bounce |
| Hard landing | sink-rate increments, strike/crash reason | documented boundary triggers consistently |
| Ground effect | height/span, lift, total and induced drag terms | continuous, bounded, fades to free air with height |
| Takeoff/rollout | distance, rotation speed, heading, wheel unloading | repeatable, controllable, no hidden corrective input |

## Expertise and competitive-quality ladder

1. **Theory gate** — maintain a source-linked model notebook for aerodynamics, propulsion, atmosphere, ground mechanics, stability/control and pilot technique. Unknowns become named calibration fields, never hidden constants.
2. **Instrumentation gate** — log state, controls, atmosphere, per-surface forces, per-wheel forces/slip, propulsion, energy, contacts and crash reason at deterministic intervals.
3. **Bench gate** — replace assumptions with measured mass/CG, pendulum inertia, control throws, thrust/current/RPM, battery sag, propeller and tyre data.
4. **Envelope gate** — expand from parked/taxi to takeoff, climb, trim, turns, stalls, spins, glide, approach, landing, rollout and abnormal cases. Calm/nominal cards pass before wind or failure cases.
5. **Competitive benchmark gate** — fly matched aircraft/configuration/test cards in established RC simulators and compare direction, rates, distances and qualitative workload without copying undocumented behaviour.
6. **Human gate** — when pilots become available, use at least three experienced fixed-wing RC pilots where practical. Run blinded, repeated cards and collect direction correctness, predictability, workload, stall cues, landing feel and confidence ratings plus free-form defects.
7. **Release gate** — keep source checks, deterministic simulation, Windows/package operation, real transmitter/device operation, comparative benchmarking and human handling acceptance as separate evidence rows.

Until the human gate exists, the correct claim is **research-backed and systematically flight-tested simulation**, not “pilot-proven” or “fully real.”

## Next physics sequence

1. finish the ground envelope above, including crosswind taxi and incremental hard landings;
2. derive density and viscosity consistently from pressure/temperature/altitude and add deterministic gusts, turbulence and shear;
3. replace assumed airframe, motor, battery and gear constants with measurements;
4. increase wing/tail spanwise resolution and validate adverse yaw, tip stall, spin entry/recovery and stalled-control behaviour;
5. add full energy-accounting and timestep-sensitivity gates before expanding aircraft types.
