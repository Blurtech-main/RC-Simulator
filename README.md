# RC Simulator — Complete Patch Notes

This document is the all-in-one development record for **RC Simulator / RC Trainer**. It covers the complete repository history currently available: the Windows-native trainer foundation and every evidenced fix in the aerodynamic Cessna update.

Build instructions, controls, and the concise project overview are preserved in **[PROJECT_GUIDE.md](PROJECT_GUIDE.md)**.

> **Repository boundary:** these notes describe `Blurtech-main/RC-Simulator` only. They do not describe Semantic Resonance Engine, Sigil, PhoenixRC, SimEmu, or any other project.

## Status key

| Label | Meaning |
|---|---|
| **Released foundation** | Present in commit [`3b3d4e8`](https://github.com/Blurtech-main/RC-Simulator/commit/3b3d4e8d6b00663fa0bdc863b389058c0901d649). |
| **Current update** | Implemented on the current `main` branch but not represented by a packaged release. |
| **Recorded validation** | Supported by an existing generated diagnostic or self-test result. It is not a substitute for human flying, visual acceptance, or real-transmitter acceptance. |

---

## Development patch chronology

The smaller steps below are reconstructed from the surviving source, diffs, diagnostics, and worktree snapshots. They are listed in development order without pretending that every step was a separate release.

1. **Project and controller foundation** — established the native Win32 trainer, DirectInput discovery and hot-plug retry, keyboard fallback, persistent calibration/mapping, fixed-step simulation, replay CSV, self-test entry point, and standalone PPM probe.
2. **PPM usability cleanup** — removed intrusive signal-hit dialogue behaviour in favour of the persistent HUD state and timestamped hit log; added automatic preference for Native Instruments Komplete Audio hardware while retaining manual F2 source cycling and fallback selection.
3. **Crash-reset and grass-range correction** — made crash recovery restart the aircraft and lesson cleanly after two seconds, reset the countdown when the crash condition clears, kept calibration/replay state outside that reset, and extended/thinned the procedural grass and grid rather than letting the field visibly terminate near 120 m.
4. **Aircraft pitch-visibility correction** — stopped drawing an aircraft cue that communicated bank alone; derived pitch from the real forward vector so elevator motion remained visible even after banking.
5. **Flight-school second slice** — added the staged takeoff/climb/cruise/glide/landing lesson, objective/timer/scoring state, recording controls, looping playback, manual override, clear behaviour, and the first deterministic takeoff/replay tests.
6. **Ground-performance and fresh-launch patch** — replaced the dense grid and roughly 90,000 candidate grass segments with the solid horizon field, runway wedge, sixteen moving distance marks, and two field boundaries; stopped auto-playing the last replay so a crash-ending recording could not make the next launch appear instantly broken.
7. **Keyboard over-control and focus-safety patch** — moved flight input from global polling to trainer-window messages, cleared held keys on focus loss, foreground-gated the mouse wheel, eased keyboard aileron to 35%, added temporary up/down elevator control, and exposed a clear 11.5 m/s keyboard rotation cue. These changes target unintended background flight and the full-deflection diagonal/corkscrew behaviour of digital keys.
8. **Scale-accurate Cessna model patch** — added the attributed original GLB, repeatable converter, compact runtime mesh, exact 1:6.395 dimensions, seven collision proxies, eight aerodynamic reference surfaces, landing gear, and model-based ground clearance.
9. **Textured-aircraft orientation and stability patch** — added the depth-tested software renderer, corrected the imported model’s visual forward direction at render time, preserved the physics axis, corrected vanishing canopy glass, displayed asset-load errors, and added runway-contact angular damping/prop-torque reaction to prevent parked tilt.
10. **Research-backed aerodynamic slice** — replaced the prototype force approximation with measured low-Re NACA 2412 data, local surface forces, finite-wing and post-stall behaviour, measured propeller coefficients, six-degree-of-freedom integration, an explicit atmosphere boundary, richer telemetry, and the expanded aerodynamic regression suite.
11. **Mouse-wheel elevator recentering** — changed the keyboard fallback's wheel input from permanent trim to a brief, smoothly recentring elevator pulse. A short scroll can no longer hold full elevator indefinitely, bleed all forward speed in a stall, and make the powered aircraft appear to strike an invisible curved wall before rotating in place.
12. **Unmasked takeoff roll-stability correction** — added a neutral-aileron takeoff regression after discovering both older takeoff checks continuously applied hidden aileron correction. Added the previously omitted dynamic-pressure-scaled high-wing/dihedral restoring moment, preventing measured propeller reaction torque from accumulating bank forever and making an ordinary throttle-and-elevator takeoff inevitably keel diagonally into the ground.
13. **Trainer elevator-authority correction** — reduced the elevator's exaggerated hinge effectiveness after measuring roughly 1 N·m of pitching-moment change at only 25% stick. The live-input takeoff regression now uses a finite keyboard-style rotation pulse with neutral aileron rather than hidden roll correction or indefinitely held elevator, covering the reported throttle-plus-pitch sequence without forcing a stall.
14. **Live-style takeoff regression patch** — added a second automated takeoff using the same gradual throttle ramp and post-11.5 m/s elevator rotation expected from keyboard operation, and made it a mandatory aerodynamic-suite gate. The current recorded self-test includes and passes this path.
15. **Controls-on-demand overlay patch** — changed the aerodynamic controls/help overlay from visible-by-default to hidden on launch. It now appears only when the pilot presses F1, leaving the runway view clear at startup.

---

## Current main update — Aerodynamic Cessna overhaul

Development snapshot: **16 October 2026**
Status: **implemented on `main`; not yet packaged as a release**

### Flight model — complete six-degree-of-freedom replacement

- Replaced the prototype single-aircraft lift approximation with a fixed-step, six-degree-of-freedom rigid-body solver.
- Removed direct, artificial elevator/aileron/rudder torque injection. Control inputs now alter the incidence of their corresponding physical reference surfaces.
- Split the airframe into eight named aerodynamic panels: left wing, right wing, left aileron, right aileron, stabiliser, elevator, fin, and rudder.
- Evaluated every panel at its own location on the aircraft instead of at one global force point.
- Added surface-local rotational flow through `omega × r`, so rotation changes the airflow seen by each panel.
- Summed each panel’s force and moment before integration, retaining the leverage produced by its actual position.
- Added a full diagonal inertia tensor rather than treating angular motion as a generic damped vector.
- Added the rigid-body gyroscopic term `omega × I omega`.
- Normalised the attitude quaternion during every 120 Hz physics update to prevent accumulated orientation drift.
- Increased the simulated aircraft definition to a 3.65 kg, 1:6.395-scale Cessna 172 replica.
- Isolated vector, quaternion, transform, clamp, cross-product, and normalisation primitives in `src/flight_math.h` so the physics code no longer depends on private helpers embedded in the window application.

### Wing aerodynamics and stall behaviour

- Added digitised NACA 2412 lift and drag anchors from the CC BY 4.0 Airfoil 360 v2022 low-Reynolds-number wind-tunnel dataset.
- Added interpolation between Reynolds numbers 50,000 and 100,000 rather than using a single fixed coefficient curve.
- Preserved the measured low-Re stall onset and drag rise instead of clipping lift at one arbitrary angle.
- Modelled the cambered section around its approximately -2° zero-lift angle, including the negative-angle side of the curve.
- Added finite-wing lift-slope correction for the model’s 7.48 aspect ratio.
- Added induced drag using `CL² / (pi e AR)` with span efficiency `e = 0.72`.
- Retained measured polar behaviour through 45° angle of attack.
- Added a smooth flat-plate/Viterna-style continuation outside the measured range so coefficients remain finite and continuous at extreme attitudes.
- Added explicit stall fraction telemetry instead of leaving stall state implicit in a clamped force value.

### Tail, fin, and control surfaces

- Added a symmetric finite-surface model for the horizontal tail, elevator, fin, and rudder.
- Added nonlinear tail and fin stall continuation and drag growth.
- Added wing-induced angle/downwash for the main lifting surfaces.
- Added tail downwash derived from the wing lift coefficient.
- Made elevator input change the elevator panel’s local aerodynamic incidence and pitching moment.
- Made aileron input change the left and right aileron incidences in opposing directions.
- Made rudder input change the rudder’s local side-force incidence and yawing moment.
- Corrected and locked conventional control signs with automated tests: pull-up elevator produces the expected pitch direction, positive aileron the expected roll direction, and positive rudder the expected yaw direction.

### Propeller, motor response, and torque

- Replaced constant thrust with a 14 × 7 inch propeller model.
- Anchored thrust coefficient behaviour to UIUC Volume 4 APC 14x7 Thin Electric wind-tunnel measurements.
- Added advance ratio so thrust falls as forward airspeed rises.
- Added static thrust and power coefficients from the measured static dataset.
- Added propeller torque using `CQ = CP / (2 pi)` rather than modelling thrust alone.
- Added separate propeller spool-up and spool-down response instead of instantaneous RPM changes.
- Added live propeller RPM telemetry.
- Added a landing-gear reaction for propeller torque while the aircraft is on the runway, preventing the motor from unrealistically rolling the parked airframe through the ground.

### Atmosphere and future weather boundary

- Added an explicit `AtmosphereSample` input containing density, pressure, temperature, dynamic viscosity, world wind, deterministic gust, rain rate, and turbulence intensity.
- Changed every aerodynamic surface to use air-relative velocity: aircraft velocity minus wind and gust, plus the surface’s local rotational velocity.
- Added stationary-aircraft headwind handling, so a parked aircraft in wind now sees aerodynamic flow.
- Added Galilean-invariance coverage, ensuring the same relative airflow produces the same aerodynamic force when aircraft and air masses share a velocity offset.
- Kept networking, location lookup, and live-weather providers outside the physics solver.
- Reserved rain and turbulence inputs without inventing unvalidated aerodynamic corrections for them. They are explicit future boundaries, not claimed live features.

### Ground contact, landing gear, and crash handling

- Replaced the old fixed `y = 0.45 m` ground assumption with orientation-aware collision geometry.
- Added seven stable collision proxies: fuselage, main wing, tailplane, fin, left main wheel, right main wheel, and nose wheel.
- Separated collision proxies from the high-detail visual mesh so physics remains stable and inspectable.
- Added an orientation-aware lowest-collision-point calculation for upright, banked, pitched, and inverted contact.
- Set the scale-correct ground clearance to 0.20 m.
- Updated the lesson’s landing completion threshold to use the aircraft model’s ground clearance instead of a hard-coded height.
- Added `hasFlown` tracking so ordinary contact during the takeoff roll does not count as an airborne crash.
- Kept the hard-impact crash threshold at 4.5 m/s after the aircraft has flown.
- Added attitude-based crash detection so non-gear contact or an invalid landing attitude is recognised even without a large vertical impact.
- Reduced ground bounce and retained rolling friction during contact resolution.
- Added angular damping from the three-wheel support footprint, fixing the parked aircraft tipping or rotating under propeller reaction torque before the aerodynamic controls have authority.
- Added a dedicated parked-stability regression test that holds full elevator for 2,400 physics frames.

### Exact-scale visual aircraft

- Replaced the yellow screen-space aircraft silhouette with the texture-mapped “Plane (Cessna 172)” model by osmosikum.
- Preserved the original 33 MB GLB unchanged beside the runtime derivative.
- Added full CC BY 4.0 attribution, original asset links, redistribution source, licence link, and dimensional source.
- Baked the authored scene transforms into a compact runtime mesh.
- Reoriented source coordinates to the simulator’s `+X` right wing, `+Y` up, `+Z` forward convention.
- Calibrated the source to the official 11.00 m wingspan, 8.28 m length, and 2.72 m height, then scaled it to 1:6.39535.
- Produced a 1.720 m wingspan, 1.294 m length, and 0.425 m height runtime aircraft.
- Retained all six 2048 × 2048 base-colour maps at full resolution and embedded them as RGBA data in the runtime asset.
- Added a repeatable GLB-to-`RCTMESH1` conversion tool that parses GLB 2.0, bakes node transforms, remaps axes, normalises dimensions, preserves UVs and normals, and packages material pixels.
- Added defensive mesh loading: magic/version checks, count limits, texture-size validation, material-index validation, and truncated-file detection.
- Added a visible in-simulator error message when the aircraft asset cannot be loaded.
- Added build-time asset copying so the complete `assets` directory is placed beside `RCTrainer.exe`.

### Software renderer

- Added a dependency-free textured aircraft renderer using the Windows GDI path already used by the simulator.
- Added a dedicated 480 × 340 aircraft render surface.
- Added a depth buffer so closer triangles correctly obscure farther geometry.
- Added separate opaque and transparent rendering passes.
- Added bilinear texture filtering rather than nearest-pixel sampling.
- Added lightweight normal-based illumination to retain visible aircraft form.
- Added wrapped UV sampling for material coordinates outside the base interval.
- Corrected the imported model’s visual-only forward axis so a fresh flight points down the runway instead of back toward the camera.
- Strengthened the source glass alpha and added restrained blue-grey absorption/reflection, fixing the canopy disappearing into the blue sky and exposing an overly pale cabin interior.
- Kept the model-axis correction visual-only so it cannot silently invert the flight model or its aerodynamic axes.

### Field, runway, and performance rendering

- Removed the previous dense perspective grid and the approximately 90,000 candidate grass-blade segments evaluated during a frame.
- Replaced them with one solid field polygon extending cleanly to the horizon.
- Added a single runway wedge whose apparent width and lateral offset respond to altitude and aircraft displacement.
- Added sixteen broad, moving cross-field distance bands to communicate forward travel and height at very low draw cost.
- Added two converging field boundaries to make lateral drift visible without recreating a dense grid.
- Removed the finite-looking 120 m grid cutoff and far-field grass fade.
- Based the horizon on the aircraft’s actual forward vector rather than raw quaternion component signs, avoiding visual flips from equivalent quaternion sign changes.
- Based all ground-plane projections on aircraft altitude.
- Limited the camera pitch cue so steep pitch does not push the entire field off-screen and make ground contact look like an unexplained mid-air crash.
- Derived visible aircraft pitch from the real forward vector, retaining stable pitch feedback after the aircraft banks.
- Updated above-ground-level calculations to use the aircraft model’s 0.20 m ground clearance.

### Keyboard and focus safety fixes

- Replaced global `GetAsyncKeyState` flight polling with key-down and key-up messages delivered to the trainer window.
- Fixed ordinary typing in other applications moving or crashing the aircraft while RC Trainer was not focused.
- Added focus-loss handling that clears all held flight keys, preventing stuck aileron, rudder, or throttle commands after task switching.
- Restricted mouse-wheel elevator input to the foreground trainer window.
- Changed keyboard aileron from immediate full deflection to a modest 35% target with easing, making the digital fallback behave more like a trainer input.
- Added held up/down-arrow elevator control: Down represents pulling the stick back for nose-up rotation, while Up commands nose-down elevator.
- Retained the mouse wheel as a persistent elevator input/trim and combined it safely with the temporary arrow-key elevator command.
- Centralised reset handling so F3, Space, and delayed crash recovery also clear keyboard throttle, elevator, aileron, held-key state, timer state, frame count, aircraft state, and lesson state.
- Prevented held key repeat from repeatedly toggling one-shot commands such as help or replay controls.

### Replay and fresh-launch safety

- Removed automatic replay loading during application startup.
- Fixed a saved replay that ended in a crash making a new session appear to crash immediately on launch.
- Kept the saved `replays/latest.csv` file intact until the pilot deliberately clears it.
- Made F6 the explicit action that loads and loops the saved flight.
- Preserved immediate Manual Override hand-back to the live controller.
- Preserved exact interval capture from **Record Flight** to **Stop**.
- Preserved deterministic frame-by-frame CSV state and input storage.
- Preserved replay looping and added it to the expanded regression suite.

### HUD and diagnostics

- Renamed speed telemetry to TAS to clarify that the displayed value is true airspeed through the current atmosphere.
- Added sideslip angle (`Beta`) beside pitch and angle of attack.
- Added Reynolds number in thousands.
- Added live propeller RPM.
- Added stall percentage.
- Kept lift, drag, and thrust visible together in new `L/D/T` telemetry.
- Added live aileron, elevator, throttle, and rudder values to the flight-school HUD, making the actual input reaching the simulation visible.
- Added an explicit **ROTATE** prompt after 11.5 m/s during the takeoff roll, including the Down-arrow pull-back instruction for keyboard pilots.
- Expanded `controller-diagnostic.txt` with aircraft-model identity, textured-asset load status, collision-proxy count, and aerodynamic-surface count.
- Preserved audio-capture, DirectInput/fallback, and ground-renderer diagnostics.
- Preserved the live eight-channel raw-axis/PPM inspector, audio meter, PPM lock/hit state, flight-school objective, score, timer, crash countdown, selected mapping, and calibration prompt.

### Build and source organisation

- Added dedicated flight-math, flight-dynamics, aircraft-model, and textured-renderer translation units.
- Linked `msimg32` for alpha compositing of the rendered aircraft.
- Added post-build recursive asset copying beside the Release executable.
- Kept the separate `PpmProbe` diagnostic target.
- Kept C++20, Unicode, DirectInput 8, strict MSVC conformance, warnings, exception handling, and UTF-8 compilation settings.

### Expanded automated regression suite

- Preserved the 2,400-frame deterministic simulation hash comparison.
- Preserved finite-position checks.
- Added parked-aircraft stability under sustained control input.
- Expanded takeoff validation to record peak altitude, speed, distance, orientation, angular rate, and crash state.
- Added coefficient-finiteness coverage from -45° through +45° angle of attack.
- Added dynamic-pressure square-law checks at 10 m/s and 20 m/s for both lift and drag.
- Added low-Re stall-shape checks comparing pre-stall 11° behaviour with post-stall 25° behaviour.
- Added elevator, aileron, and rudder moment-sign checks.
- Added stationary-aircraft headwind response.
- Added Galilean wind invariance.
- Preserved replay-loop validation.
- Added aircraft-model contract checks for upright and inverted bounds, seven collision proxies, and eight aerodynamic panels.
- Added a second takeoff path that mirrors live keyboard operation: gradual throttle ramp, modest roll correction, and elevator rotation after 11.5 m/s.
- Made that live-input-path takeoff a required part of the overall aerodynamic suite rather than reporting it as optional telemetry.

### Recorded validation snapshot

The current `self-test.txt` records **PASS** with:

| Check | Recorded result |
|---|---:|
| Deterministic fixed-step hash | `cf3cdfd5012b741a` |
| Simulated deterministic frames | 2,400 |
| Finite and repeatable state | Pass |
| Parked stability | Pass |
| Automated takeoff peak | 3.01361 m |
| Automated takeoff speed | 15.7618 m/s |
| Automated takeoff distance | 57.0246 m |
| Takeoff crash state | No crash |
| Gradual-throttle live-input takeoff | Pass |
| Gradual-throttle live-input peak | 3.01719 m |
| Gradual-throttle live-input speed | 15.6629 m/s |
| Coefficients finite | Pass |
| Lift dynamic-pressure ratio | 4.08377 |
| Drag dynamic-pressure ratio | 4.02744 |
| Low-Re stall shape | Pass |
| `CL` at 11° | 0.957237 |
| `CL` at 25° | 0.63438 |
| Control moment signs | Pass |
| Wind-relative response | Pass |
| Galilean invariance | Pass |
| Replay loop | Pass |
| Aircraft model contract | Pass |
| Collision proxies / aero surfaces | 7 / 8 |

The current `controller-diagnostic.txt` additionally records successful Windows audio capture and textured-aircraft loading. No DirectInput device was present for that diagnostic run, so it records keyboard fallback rather than proving transmitter acceptance.

### Known boundaries in this update

- The automated suite proves deterministic software invariants; it does not prove that the aircraft feels correct to a human RC pilot.
- The current diagnostic proves that the audio capture device opened; it does not by itself prove a complete live DX6i flight.
- No current evidence establishes visual acceptance across all window sizes, monitors, or display-scaling settings.
- The current recorded self-test includes the live-input-path takeoff gate (`live_takeoff=1`); it is a simulated keyboard-style input path, not evidence of human controller acceptance.
- This is a research-backed engineering model, not certified Cessna flight-test equivalence.
- Exact RC mass distribution, hinge geometry, motor/ESC constants, thrust-stand curves, and full-airframe wind-tunnel data remain replaceable calibration inputs because measurements for this specific virtual airframe are not available.
- Rain and turbulence exist as solver inputs but deliberately have no claimed aerodynamic effect yet.
- The in-app F1 help overlay still contains the old statement that a saved flight starts automatically. Runtime and README behaviour are the opposite: every launch starts live, and F6 loads the replay. This wording remains to be corrected in a later code patch.
- Lesson scoring still exists in the lesson state, but the current HUD line displays live control values and elapsed time instead of the score. README wording that says the HUD presents the score is therefore stale.
- The direct-build fallback command predates the split source files and asset renderer; the CMake/`build.ps1` path is the authoritative build route for this snapshot.

---

## 0.1.0 — Released trainer foundation

Date: **15 October 2026**
Commit: [`3b3d4e8` — Initial RC Simulator project](https://github.com/Blurtech-main/RC-Simulator/commit/3b3d4e8d6b00663fa0bdc863b389058c0901d649)
Status: **released foundation on `main`**

### Native Windows application

- Created a clean-room, Windows-native RC flight trainer in C++20.
- Added a lightweight Win32/GDI application with no game-engine runtime dependency.
- Added a high-resolution timer, 120 Hz fixed-step simulation, and approximately 60 Hz repaint pacing.
- Capped an individual elapsed-time update at 100 ms to prevent an extreme catch-up burst after a stall or breakpoint.
- Added double-buffered painting to avoid drawing directly into the visible window.
- Added a Visual Studio/CMake Release build path producing `build/Release/RCTrainer.exe`.
- Added a direct MSVC fallback build command for the original single-source application.
- Added a separate command-line PPM probe utility.
- Added repository ignores for build products, runtime profiles, replays, diagnostics, PPM captures, temporary work, and intermediate objects.

### DirectInput controller support

- Added DirectInput 8 controller discovery.
- Added enumeration of attached game controllers.
- Added eight-axis input capture.
- Added cooperative background/non-exclusive device access.
- Added automatic reacquisition after a lost input device.
- Added periodic hot-plug rediscovery when no device is present, avoiding an application restart after connecting an adapter.
- Added keyboard fallback when DirectInput hardware is unavailable.
- Added the connected controller name to the HUD.

### Controller calibration and mapping

- Added live raw-axis bars for all eight inputs.
- Added guided calibration that captures minimum and maximum travel for every used axis.
- Added centred-value capture when calibration is completed.
- Added per-axis asymmetric normalisation around the captured centre.
- Added selectable aileron, elevator, throttle, and rudder functions.
- Added previous/next physical-axis assignment for each function.
- Added per-function reversing.
- Added adjustable dead zones from 0% to 25% in 0.5% steps.
- Added endpoint persistence, centre persistence, dead-zone persistence, mapping persistence, and reversal persistence in `profiles/default.ini`.
- Added a safe default range if an axis was not moved during calibration.

### DX6i trainer-port PPM audio input

- Added direct capture of a DX6i trainer-port PPM signal through a Windows microphone or line input.
- Added 48 kHz, 16-bit, mono Windows WaveIn capture.
- Added four recycled 2,048-sample input buffers for continuous capture.
- Added multi-threshold and dual-polarity edge candidates so the decoder can adapt to differently scaled or inverted audio signals.
- Added sync-gap detection and 4–12 channel frame acceptance.
- Added pulse-width mapping for approximately 0.8–2.2 ms channel intervals.
- Added repeated-good-frame locking rather than accepting one isolated candidate frame.
- Added lock timeout so stale PPM does not remain active after the signal disappears.
- Added an audio peak meter.
- Added PPM hit logging with timestamp, threshold, polarity, channel count, and frame duration.
- Replaced intrusive hit notifications with persistent HUD feedback and log evidence, so candidate pulses no longer interrupt the simulator.
- Added a visible **PPM HIT** state before full lock and **PPM LOCKED** after stable decoding.
- Added F2 cycling through available Windows audio capture sources.
- Added automatic preference for a connected Native Instruments Komplete Audio interface.
- Added fallback to the last-listed Windows audio input when a Komplete interface is not found.
- Added light exponential channel smoothing.
- Added a 6.5% centred dead-zone/remap for trainer-port channels to reduce noise around stick centre.
- Added the selected audio-device name to the HUD.

### Original trainer flight model

- Added a tuneable high-wing electric trainer prototype.
- Added position, velocity, angular velocity, and quaternion attitude state.
- Added gravity, lift, drag, thrust, and control authority.
- Added airspeed-dependent control authority.
- Added a simple stall/post-stall reduction rather than unlimited linear lift.
- Added ground contact, bounce reduction, rolling friction, and crash detection.
- Added live angle-of-attack, airspeed, lift, drag, thrust, and altitude telemetry.

### Flight-school loop

- Added a five-stage lesson: takeoff, climb, cruise, power-off glide, and controlled landing.
- Added a live objective for the current stage.
- Added lesson elapsed time.
- Added a 1,000-point starting score.
- Added a 350-point crash penalty.
- Added completion scoring based on elapsed time.
- Added a visible crash countdown.
- Added automatic aircraft and lesson restart two seconds after a crash.
- Kept controller calibration and saved replay data intact across a crash restart.
- Added F3 lesson restart and Space runway reset.

### Replay system

- Added deterministic CSV flight recording to `replays/latest.csv`.
- Recorded frame number; aileron, elevator, throttle, and rudder; position; velocity; quaternion; angular velocity; crash state; angle of attack; airspeed; lift; drag; and thrust.
- Added **Record Flight**, **Stop**, **Clear**, and **Manual Override** controls.
- Added F5 start/stop recording, F6 replay loading, and F7 playback stop.
- Added replay parsing that skips malformed rows instead of poisoning the complete load.
- Added continuous replay looping.
- Added immediate Manual Override from playback to the live input source.
- Added Clear behaviour that stops playback, clears loaded frames, and removes the saved CSV.
- Originally auto-loaded a saved replay on launch; the current update above removes that behaviour after it caused crash-ending recordings to make a fresh launch appear broken.

### Keyboard fallback

- Mapped W/S to gradual left-stick vertical movement for throttle.
- Mapped A/D to left-stick horizontal movement for rudder.
- Mapped the mouse wheel to persistent right-stick vertical movement for elevator in 10% steps.
- Mapped left/right arrows to right-stick horizontal movement for aileron.
- Added Escape to close the trainer.
- Added F1 help-overlay toggling.
- The original fallback polled global key state; the current focus-safety update above replaces it with window-scoped input.

### Original field, camera, and aircraft cues

- Added a procedural sky and grass field without a bitmap-rendering dependency.
- Extended the original perspective grid well beyond the first 120 m so it no longer ended abruptly during the circuit.
- Extended grass cues into the far field while reducing their distant density.
- Added deterministic irregular grass positioning so the terrain conveyed motion without shimmering randomly between frames.
- Added camera pitch and altitude to field projection.
- Based the horizon on the aircraft’s forward direction rather than a raw Euler-angle shortcut.
- Added a banked yellow aircraft silhouette.
- Added an explicit pitch cue to the silhouette so elevator/pitch motion remained visible rather than showing bank alone.
- The current rendering update above replaces the dense grass/grid and silhouette with the low-cost runway field and textured Cessna.

### HUD and usability

- Added clear `LIVE`, `RECORDING`, and `PLAYBACK` mode labels.
- Added input-source identity.
- Added audio-source identity and cycling hint.
- Added audio signal level and PPM lock state.
- Added flight telemetry, lesson stage, objective, score, and time.
- Added eight live input bars.
- Added a four-function mapping panel showing the selected function, assigned axis, and reversed state.
- Added an on-screen calibration instruction.
- Added an F1 help panel covering replay, manual override, audio source selection, lesson restart, calibration, runway reset, exit, and keyboard fallback.

### Foundation self-tests and diagnostics

- Added a command-line `--self-test` mode.
- Added a deterministic hash comparison across two identical 2,400-frame simulations.
- Added finite-state validation.
- Added an automated takeoff gate.
- Added replay-loop validation.
- Wrote results to `self-test.txt` for inspectable build evidence.
- Wrote audio-capture state, DirectInput presence/fallback, and ground-renderer identity to `controller-diagnostic.txt` at startup.

---

## Patch ledger by repository area

This ledger is intentionally redundant: it makes small patches discoverable even when someone knows the file but not the release section.

| Area | Patch record |
|---|---|
| `src/main.cpp` | Win32 shell, DirectInput, calibration, WaveIn PPM, lessons, replay, input-focus fix, fresh-launch replay fix, collision integration, low-cost field, telemetry, and expanded self-tests. |
| `src/flight_math.h` | Shared vector and quaternion mathematics extracted from the original application. |
| `src/flight_dynamics.*` | NACA 2412 data, finite-wing correction, panel forces, post-stall continuation, tail/fin/control response, propeller coefficients, atmosphere-relative flow, rigid-body integration, and telemetry. |
| `src/aircraft_model.*` | Scale-correct dimensions, seven collision proxies, eight aerodynamic surfaces, landing gear, and orientation-aware ground bounds. |
| `src/textured_aircraft_renderer.*` | Validated runtime asset loading, two-pass depth-tested rasterisation, bilinear texture sampling, lighting, canopy correction, and alpha compositing. |
| `tools/ppm_probe.cpp` | Standalone Windows audio capture and evidence tool for locating likely PPM signal thresholds and frame intervals. |
| `tools/convert_glb_aircraft.py` | Reproducible GLB conversion, axis/dimension normalisation, transform baking, full-resolution material embedding, and runtime-mesh output. |
| `assets/grass-tile.png` | Original tracked grass asset; the active ground renderer is procedural. |
| `assets/aircraft/cessna172` | Original attributed GLB, compact runtime derivative, six embedded material maps, and complete licence/modification record. |
| `profiles/default.ini` | Persistent controller endpoints, centres, dead zones, function mapping, and reversal defaults; runtime-generated and intentionally ignored. |
| `replays/latest.csv` | Deterministic flight record; runtime-generated and intentionally ignored. |
| `CMakeLists.txt` | RCTrainer and PpmProbe targets, strict compile options, required Windows libraries, new module integration, and post-build asset packaging. |
| `build.ps1` | Visual Studio discovery, CMake configure/build, Release output, and optional self-test execution. |
| `build-direct.cmd` | Original single-source MSVC fallback; retained for history but not authoritative for the current multi-file renderer build. |
| `AERODYNAMICS.md` | Equations, research sources, validation gates, live-weather boundary, and calibration limitations for the current solver. |
| `self-test.txt` | Latest automated deterministic, takeoff, aerodynamic, wind, replay, and aircraft-contract result; runtime-generated and intentionally ignored. |
| `controller-diagnostic.txt` | Latest local device, capture, renderer, asset, collision, and aerodynamic-surface startup snapshot; runtime-generated and intentionally ignored. |
| `ppm-hits.log` | Timestamped PPM candidate evidence used for signal diagnosis; runtime-generated and intentionally ignored. |

---

## Acceptance record

| Acceptance layer | Current evidence |
|---|---|
| Source implementation | Present on `main` for the aerodynamic/C172 update. |
| Repository history | Foundation commit `3b3d4e8` and the aerodynamic/C172 update are recorded on `main`. |
| Automated simulation | Latest recorded self-test is PASS. |
| Asset loading | Latest recorded diagnostic says `TEXTURED_ASSET=LOADED`. |
| Windows audio capture | Latest recorded diagnostic says `AUDIO_PPM_CAPTURE=OPEN`. |
| DirectInput hardware | Not established by the latest diagnostic; it records `DIRECTINPUT=NONE`. |
| Live DX6i flight | Not established by the automated artifacts alone. |
| Human handling / musical-feel equivalent | Not yet recorded; requires an actual pilot session. |
| Visual acceptance | Not yet recorded; requires human inspection of the running simulator. |
| Release/package acceptance | Not yet recorded for the aerodynamic/C172 update. |

That separation is deliberate: passing code and telemetry checks are evidence of implementation, not a claim that a human has accepted the aircraft’s handling, visuals, controller chain, or packaged release.
