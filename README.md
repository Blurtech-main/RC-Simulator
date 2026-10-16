# RC Simulator — Complete Patch Notes

This document is the all-in-one development record for **RC Simulator / RC Trainer**. It covers the complete repository history currently available: the Windows-native trainer foundation and every evidenced fix in the aerodynamic Cessna update.

Build instructions, controls, and the concise project overview are preserved in **[PROJECT_GUIDE.md](PROJECT_GUIDE.md)**.

> **Repository boundary:** these notes describe `Blurtech-main/RC-Simulator` only. They do not describe Semantic Resonance Engine, Sigil, PhoenixRC, SimEmu, or any other project.

<details>
<summary><strong>Initial development (unversioned)</strong></summary>

Everything in this section predates formal versioning. Its historical labels describe development state and evidence only; they are not release version numbers.

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
16. **Virtual RC test-pilot and camera-truth patch** — added deterministic and real-transmitter HTML/CSV flight reports covering received controls, X/Y/Z motion, pitch/bank/heading, vertical speed, airspeed, angle of attack, stall and signed control response. Removed the pitch-coupled aircraft screen offset that made nose-up look like downward travel and nose-down look like upward travel in the chase view.

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
- Replaced the screen-space field with a world-locked perspective plane sampled from a compact grass bitmap.
- Built a mip pyramid at load time, selected it from the projected pixel footprint and bilinearly sampled it to prevent distant shimmer and chunky near-field texels.
- Integrated the mown grass runway and boundary strips in world coordinates so they turn, translate and converge with the terrain rather than behaving like a screen-space wedge.
- Rendered stochastic ground at half resolution and performed one filtered GDI upscale, retaining full-resolution aircraft and HUD rendering.
- Kept the field effectively unbounded through wrapped world coordinates; distance mip filtering supplies antialiasing without a synthetic grass-fog colour.
- Added a second 170 m grass-variation scale so distant terrain retains broad natural colour/detail instead of collapsing into a flat average-green band.
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
- Added a longer takeoff/pull/push card through `--test-pilot`, with explicit finite-state, crash, climb, lateral-stability and signed pitch-rate gates.
- Added a self-contained inline-SVG report and full-resolution CSV, requiring no plotting package, network service, or browser runtime inside the simulator.
- Made `build.ps1 -RunTests` run both the conventional suite and the virtual test-pilot card; `-RunTestPilot` runs the latter independently.

### Recorded validation snapshot

The current `self-test.txt` records **PASS** with:

| Check | Recorded result |
|---|---:|
| Deterministic fixed-step hash | `b22c21c8944b4c01` |
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
| Virtual test-pilot takeoff/climb | Pass |
| Virtual test-pilot pull/push response | Pass / Pass |

The current `controller-diagnostic.txt` additionally records successful Windows audio capture and textured-aircraft loading. No DirectInput device was present for that diagnostic run, so it records keyboard fallback rather than proving transmitter acceptance.

### Known boundaries in this update

- The automated suite proves deterministic software invariants; it does not prove that the aircraft feels correct to a human RC pilot.
- The current diagnostic proves that the audio capture device opened; it does not by itself prove a complete live DX6i flight.
- No current evidence establishes visual acceptance across all window sizes, monitors, or display-scaling settings.
- The current recorded self-test includes the live-input-path takeoff gate (`live_takeoff=1`); it is a simulated keyboard-style input path, not evidence of human controller acceptance.
- A transmitter report analyses the exact controls received by the simulator, but a clean one-control-at-a-time recording is still required before treating its correlation warning as a channel-reversal diagnosis.
- This is a research-backed engineering model, not certified Cessna flight-test equivalence.
- Exact RC mass distribution, hinge geometry, motor/ESC constants, thrust-stand curves, and full-airframe wind-tunnel data remain replaceable calibration inputs because measurements for this specific virtual airframe are not available.
- Rain and turbulence exist as solver inputs but deliberately have no claimed aerodynamic effect yet.
- The in-app F1 help overlay still contains the old statement that a saved flight starts automatically. Runtime and README behaviour are the opposite: every launch starts live, and F6 loads the replay. This wording remains to be corrected in a later code patch.
- Lesson scoring still exists in the lesson state, but the current HUD line displays live control values and elapsed time instead of the score. README wording that says the HUD presents the score is therefore stale.
- The direct-build fallback command predates the split source files and asset renderer; the CMake/`build.ps1` path is the authoritative build route for this snapshot.

---

## Released trainer foundation (pre-versioning)

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
| `src/flight_test_report.*` | Dependency-free virtual test-pilot CSV/HTML generation, X/Y/Z and attitude plots, control-response screening, and deterministic test-card verdicts. |
| `src/ground_dynamics.*` | Per-wheel suspension, rolling/braking resistance, lateral tyre grip, nose-wheel steering, force-at-contact moments, and contact telemetry. |
| `src/ground_renderer.*` | Perspective-correct world ground, fine and macro bitmap scales, mip-filtered bilinear sampling, mown runway integration, and half-resolution shading without synthetic distance fog. |
| `src/environment_renderer.*` | Cached baked sky and alpha-premultiplied hills/treeline layers, panoramic heading movement, and transparent ground overlap. |
| `src/aircraft_model.*` | Scale-correct dimensions, seven collision proxies, eight aerodynamic surfaces, landing gear, and orientation-aware ground bounds. |
| `src/textured_aircraft_renderer.*` | Validated runtime asset loading, two-pass depth-tested rasterisation, bilinear texture sampling, lighting, canopy correction, and alpha compositing. |
| `tools/ppm_probe.cpp` | Standalone Windows audio capture and evidence tool for locating likely PPM signal thresholds and frame intervals. |
| `tools/convert_glb_aircraft.py` | Reproducible GLB conversion, axis/dimension normalisation, transform baking, full-resolution material embedding, and runtime-mesh output. |
| `assets/grass-field-cheap.bmp` | Compact runtime grass surface loaded from the executable's packaged asset directory. |
| `assets/grass-tile.png` | Original high-resolution source grass asset retained for future asset regeneration. |
| `assets/environment/sky-rural-v1.png` | Baked rural daylight sky panorama with asymmetric photographic clouds. |
| `assets/environment/hills-treeline-v2.png` | Baked rolling hills, hedgerows and tree clusters with real transparency above and a feathered irregular lower edge. |
| `assets/aircraft/cessna172` | Original attributed GLB, compact runtime derivative, six embedded material maps, and complete licence/modification record. |
| `profiles/default.ini` | Persistent controller endpoints, centres, dead zones, function mapping, and reversal defaults; runtime-generated and intentionally ignored. |
| `replays/latest.csv` | Deterministic flight record; runtime-generated and intentionally ignored. |
| `reports/*.html`, `reports/*.csv` | Scripted or real-transmitter virtual test-pilot evidence; runtime-generated and intentionally ignored. |
| `CMakeLists.txt` | RCTrainer and PpmProbe targets, strict compile options, required Windows libraries, new module integration, and post-build asset packaging. |
| `build.ps1` | Visual Studio discovery, CMake configure/build, Release output, conventional self-tests, and virtual test-pilot execution. |
| `build-direct.cmd` | Original single-source MSVC fallback; retained for history but not authoritative for the current multi-file renderer build. |
| `AERODYNAMICS.md` | Equations, research sources, validation gates, live-weather boundary, and calibration limitations for the current solver. |
| `FLIGHT_TEST_PLAN.md` | Expert-development curriculum, test-card format, envelope gates, competitive benchmarks, instrumentation and human-pilot acceptance boundary. |
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
| Automated takeoff/pitch report | Latest deterministic virtual test-pilot card is PASS. |
| Asset loading | Latest recorded diagnostic says `TEXTURED_ASSET=LOADED`. |
| Windows audio capture | Latest recorded diagnostic says `AUDIO_PPM_CAPTURE=OPEN`. |
| DirectInput hardware | Not established by the latest diagnostic; it records `DIRECTINPUT=NONE`. |
| Live DX6i flight | Not established by the automated artifacts alone. |
| Human handling / musical-feel equivalent | Not yet recorded; requires an actual pilot session. |
| Visual acceptance | Not yet recorded; requires human inspection of the running simulator. |
| Release/package acceptance | Not yet recorded for the aerodynamic/C172 update. |

That separation is deliberate: passing code and telemetry checks are evidence of implementation, not a claim that a human has accepted the aircraft’s handling, visuals, controller chain, or packaged release.

## 2026-10-16: throttle-only turning correction

Corrected the fin side-force sign so sideways flow produces restoring yaw and yaw-rate damping. Rudder steering direction is preserved. Added centred-aileron propeller torque trim, fading with aileron input, to prevent steady bank and continuous turning; ground compensation uses the same residual torque. This is deliberate trainer assistance.

Regression tests cover both signs of sideslip and yaw rate, plus 60 simulated seconds each of keyboard throttle ramp and immediate full throttle with centred controls. Original solver: FAIL, maximum heading departure 127.759 degrees. Corrected solver: PASS, zero heading departure. Existing tests pass. Live controller acceptance remains unverified.

## 2026-10-16: reusable coupled propulsion system

Replaced the artificial throttle-to-target-RPM spool with a reusable electric propulsion component. Each aircraft definition can now install one or more independently configured motor, ESC, battery and fixed-pitch propeller units with arbitrary body mount, shaft axis and rotation direction. The current Cessna uses merged measured UIUC APC 14x7 runs through advance ratio 0.6098 plus the complete measured static RPM sweep.

Motor back EMF, winding resistance, ESC current limiting, battery sag, drivetrain efficiency, propeller load and combined shaft inertia now determine RPM. The airframe receives actual mount-offset force, motor reaction and gyroscopic moments. Propwash reaches aerodynamic panels through a low-order momentum-theory wake. Shaft phase drives the separately tagged propeller mesh and is recorded in replay data.

The simulator starts in `REALISTIC` mode with full propulsion reaction. F4 explicitly enables the old centred-aileron trainer compensation. The HUD now reports volts, amps, shaft watts and advance ratio, and warns when operation leaves the measured propeller envelope.

The self-test now gates measured-map interpolation, definition validation, density scaling, forward-speed unloading, reaction sign, counter-rotating twin cancellation, current-limited shaft convergence, phase integration and slipstream sampling. The build script now waits for the Windows GUI self-test process and enforces its real exit code. Build and automated tests pass; physical thrust-stand, battery-discharge, motor-manufacturer and expert RC pilot validation remain open acceptance layers.

## 2026-10-16: individual landing gear and flight-test discipline

Replaced the global ground velocity/angular damping shortcut with three force-producing wheel contacts. Each aircraft-defined gear leg now has wheel position/radius, spring rate, damping, rolling resistance, lateral grip, braking grip and steering range. Contact velocity includes rigid-body rotation, tyre forces are friction-bounded, the nose wheel follows rudder, and `B` commands wheel braking. Gear support is separated from conservative airframe collision proxies; a bounded suspension stop prevents runway tunnelling while deep airframe contact, invalid attitude and excessive touchdown speed remain crash conditions.

Added a wing-height/span ground-effect approximation that continuously reduces induced flow and induced drag near the runway. The HUD exposes wheel-contact count, brake state and ground-effect percentage.

The expanded deterministic suite passes individual force-direction and steering-sign checks, airborne isolation, braking, soft-touchdown settling and bounded ground effect while retaining the earlier takeoff, yaw, stall, wind, propulsion and replay gates. The recorded brake scenario travels 4.28 m versus 20.37 m while coasting from the same initial state; this is a regression result, not measured tyre validation.

Added [FLIGHT_TEST_PLAN.md](FLIGHT_TEST_PLAN.md), which turns authoritative flight-theory and flight-test references into project test cards, instrumentation requirements, an envelope-expansion order, competitive simulator comparisons and a future multi-pilot handling gate. Until experienced RC pilots participate, the project will describe the result as research-backed and systematically tested—not pilot-proven or fully real.

## 2026-10-16: virtual RC test pilot and truthful chase-camera motion

Added a built-in `--test-pilot` card that performs a gradual-throttle takeoff, finite rotation, deliberate pull, and deliberate push at the same fixed 120 Hz step used by the simulator. It writes a portable `reports/virtual-test-pilot.html` plus complete CSV telemetry. The report overlays the controls actually received with X/Y/Z position, vertical speed, pitch, bank, heading, body rates, airspeed, angle of attack and stall percentage, then issues finite-state, crash, climb, lateral-stability and signed elevator-response verdicts. A correct push is judged first by reduction of nose-up angular rate, because real rotational inertia can let geometric pitch continue rising briefly.

Stopping an ordinary F5 transmitter recording now produces `reports/transmitter-latest.html` and `.csv` through the same analysis path. The control-sign correlation is deliberately a screening tool: a pilot should move one control at a time at flying speed before treating a warning as proof of a reversed channel.

Corrected the chase-camera presentation independently of the physics. The old aircraft screen centre was attached to the pitch-shifted horizon, so nose-up moved the entire aircraft downward and nose-down moved it upward by an amount comparable to several metres of apparent altitude. The aircraft now remains on a stable chase-camera anchor; world-locked ground perspective and the AGL telemetry show real altitude change. The latest conventional suite and the new scripted report both pass. This does not yet constitute a real-transmitter handling acceptance run.

Replaced the open-loop pull/push sequence after it was shown to pass a nose-down command while the aircraft was still pitched roughly 41 degrees up. The virtual pilot now flies a closed-loop 30-second takeoff/climb trajectory and runs isolated matched-state elevator, aileron and rudder pairs. It also checks nose attitude, angle of attack and world-vertical aerodynamic force directly. The concise oracle CSV records expected corridors, observations and likely fault areas; the detailed HTML retains the complete plots.

Stabilized the chase camera against short-period aircraft yaw. Camera heading now follows the horizontal flight path through a damped 22-degree-per-second limiter, and the aircraft is rendered relative to that camera yaw. This removes the false impression that the whole ground swings left and right while preserving actual heading and cross-track motion in telemetry.

</details>

---

## Version 0.1.0 — Realistic ground, propulsion, environment, and flight-test release

Release date: **16 October 2026**
Status: **first formally versioned program state; accepted by the project owner**

Version 0.1.0 establishes the current simulator as the first numbered checkpoint. It gathers every implementation change made since the previous patch-notes request into one release: physically coupled electric propulsion, individual landing gear and ground effect, a world-locked textured airfield, composited rural scenery, a steadier chase camera, deterministic virtual test-pilot reporting, corrected controller mapping and yaw behaviour, expanded diagnostics, and a hardened Windows build/test path.

### Flight dynamics and trainer behaviour

- Corrected the vertical-fin side-force sign so sideslip produces a restoring force and aft-fin yaw-rate damping instead of reinforcing a turn.
- Preserved conventional rudder steering after the fin correction; positive and negative rudder inputs still generate their expected opposite yaw responses.
- Added explicit regressions for both sideslip signs and both yaw-rate signs rather than validating only one direction.
- Added two 60-second throttle-only trials with centred flight controls: one using a keyboard-style throttle ramp and one beginning at full throttle.
- Added optional centred-aileron trainer compensation for residual propulsion roll torque. The compensation fades away as the pilot commands aileron, so it cannot conceal deliberate roll input.
- Changed startup to the unassisted `REALISTIC` path. The complete motor/propeller reaction is therefore active by default; F4 explicitly toggles trainer assistance.
- Moved mass, inertia and aerodynamic constants into the aircraft definition so the solver is no longer tied to hidden Cessna-specific literals.
- Added configuration-owned reference chord, aspect ratio, span efficiency, wing incidence, dihedral, wing area, roll stability, tail lift slope/stall angle and fuselage drag area.
- Retained the fixed 120 Hz six-degree-of-freedom integration, panel-local airflow, atmosphere-relative velocity, NACA 2412 interpolation, post-stall continuation and rigid-body gyroscopic term.
- Added a wing-height/span ground-effect approximation that continuously reduces induced flow and induced drag near the runway and fades to zero with height.
- Added direct tests of nose attitude, angle of attack and world-vertical aerodynamic force so a correct response cannot pass merely because inertial altitude continues in the old direction for a moment.
- Added a pitch-integrator sign check and matched-state positive/negative elevator, aileron and rudder manoeuvres.
- Corrected the PPM stick-channel test mapping and now gate elevator, aileron and rudder signs independently.

### Reusable electric propulsion system

- Replaced the artificial throttle-to-target-RPM spool with a reusable, definition-driven electric propulsion component in `src/propulsion.*`.
- Aircraft definitions can install one or more propulsion units, each with its own mount point, shaft axis and rotation direction.
- Added replaceable motor, ESC, battery, drivetrain and propeller calibration values instead of embedding one opaque throttle curve in the flight solver.
- Modelled motor back EMF, winding resistance, ESC current limiting, battery internal resistance/sag, drivetrain efficiency, combined shaft inertia and propeller aerodynamic load.
- Merged the measured UIUC APC 14x7 Thin Electric dynamic coefficient series through advance ratio 0.6098 with the complete measured static RPM sweep.
- Interpolated thrust and power coefficients inside the measured tables and reported whether operation remains within their covered envelope.
- Calculated thrust, propeller torque, motor reaction torque and gyroscopic moment from the evolving shaft state.
- Applied thrust at the configured mount rather than at the centre of gravity, preserving moment from an offset installation.
- Added momentum-theory propwash sampling at aerodynamic panels, including spatial rejection outside the simplified wake.
- Added separate shaft angle integration for visual propeller rotation and deterministic replay state.
- Added live battery voltage, motor current, shaft power, advance ratio, propulsive efficiency and measured-curve coverage telemetry.
- Added validation for coefficient ordering, density scaling, forward-speed unloading, reaction sign, current limiting, shaft convergence and phase wrapping.
- Added a counter-rotating twin-engine symmetry test to verify cancellation of paired reaction moments.

### Individual landing gear, braking, and runway contact

- Replaced the global ground velocity/angular damping shortcut with three aircraft-defined, force-producing wheel contacts.
- Added left-main, right-main and nose-wheel definitions with position, radius, spring rate, damping, rolling resistance, lateral grip, braking grip and steering range.
- Calculated each tyre's contact velocity from translation plus rigid-body rotation.
- Applied suspension and tyre forces at the actual contact points so their pitch, roll and yaw moments emerge from geometry.
- Bounded longitudinal and lateral tyre forces by available friction instead of allowing unlimited corrective forces.
- Connected nose-wheel steering to rudder input and added `B` as the explicit wheel-brake command.
- Preserved unbraked rolling resistance while making braking a separate, measurable input.
- Separated wheel support from conservative airframe collision proxies: ordinary gear compression supports the aircraft, while deep fuselage/wing/tail contact still produces a crash.
- Added a bounded suspension stop to prevent runway tunnelling without turning a soft touchdown into an artificial bounce.
- Kept excessive touchdown speed, invalid attitude and non-gear airframe contact as crash conditions.
- Added wheel-contact count, brake state, maximum gear compression and ground-effect percentage to telemetry/HUD diagnostics.
- Added deterministic tests for force direction, opposite steering signs, airborne isolation, braking effectiveness, soft-touchdown settling and bounded ground effect.
- Recorded the braking regression from the same initial state: **4.28 m with braking versus 20.37 m while coasting**. This is a software regression result, not a measured tyre claim.

### World-locked grass field and runway

- Replaced the screen-space/procedural field treatment with a world-locked perspective ground plane in `src/ground_renderer.*`.
- Added the compact `assets/grass-field-cheap.bmp` runtime surface while retaining the original higher-resolution grass source for future regeneration.
- Built a mip pyramid at load time and selected levels from projected pixel footprint to reduce distant shimmer and oversized near-field texels.
- Added bilinear sampling and wrapped world coordinates so the terrain remains effectively unbounded without a visible texture edge.
- Added a second broad 170 m variation scale so distant terrain retains large natural colour variation instead of collapsing into one flat average green.
- Integrated the mown runway and boundary strips in world space, allowing them to translate, rotate and converge with the terrain perspective.
- Rendered stochastic ground at half resolution and used one filtered GDI upscale, leaving the aircraft, scenery silhouettes and HUD at full resolution.
- Avoided synthetic distance-fog colour; the terrain's distance behaviour comes from mip filtering and world scale.
- Added ground-renderer asset/load state to startup diagnostics.

### Rural sky, hills, and treeline environment

- Added `src/environment_renderer.*` and linked GDI+ for cached PNG decoding and compositing.
- Added a baked rural daylight panorama with deliberately asymmetric photographic cloud structure.
- Added a separate rolling-hills, hedgerow and tree-cluster layer with real transparency above the skyline.
- Retained both the first hills/treeline iteration and the refined v2 asset as source-history evidence; v2 is the documented active scene layer.
- Preserved an irregular feathered lower edge so the landscape blends into the rendered ground rather than ending as a rectangular strip.
- Used alpha-premultiplied cached layers instead of re-decoding or repeatedly transforming source images during every frame.
- Moved the panorama with camera heading so turns read as movement through a world rather than a static screen backdrop.
- Added environment assets to the normal post-build asset copy, keeping the executable's packaged runtime tree self-contained.

### Chase camera and visual truthfulness

- Removed the pitch-coupled aircraft screen offset that made nose-up flight appear to move downward and nose-down flight appear to climb.
- Anchored the aircraft to a stable chase-camera point while leaving ground perspective and AGL telemetry responsible for communicating real vertical movement.
- Added a reusable chase-camera heading limiter in `src/chase_camera.h`.
- Made camera heading follow horizontal flight path through a damped **22 degrees/second** limit instead of snapping to short-period aircraft yaw.
- Rendered aircraft attitude relative to camera yaw, preventing the entire ground from appearing to swing during small yaw oscillations.
- Added an automated maximum-camera-step gate and a yaw-relative attitude check.

### Virtual RC test pilot and reports

- Added `src/flight_test_report.*` as a dependency-free reporting path shared by automated and recorded-controller flights.
- Added `--test-pilot`, which uses the simulator's real fixed 120 Hz step for a closed-loop 30-second takeoff/climb trajectory.
- Added isolated matched-state pull/push, left/right aileron and left/right rudder comparisons after the trajectory.
- Replaced the earlier open-loop pitch sequence after it could pass a nose-down command while the aircraft remained at a severely nose-up attitude.
- Added finite-state, no-crash, climb, runway/lateral corridor, pitch, roll, yaw, angle-of-attack and vertical-force verdicts.
- Added `reports/virtual-test-pilot.html` with self-contained inline SVG plots and no browser package, plotting library or network dependency.
- Added full-resolution `reports/virtual-test-pilot.csv` telemetry plus the concise `virtual-test-pilot-oracle.csv` containing expected corridors, observations and likely fault areas.
- Plotted the controls actually received beside X/Y/Z position, vertical speed, pitch, bank, heading, body rates, airspeed, angle of attack and stall percentage.
- Changed correct push evaluation to consider reduction of nose-up angular rate and direct aerodynamic response, respecting real rotational inertia.
- Made stopping an ordinary F5 transmitter recording generate matching `transmitter-latest.html` and `.csv` analysis.
- Kept channel correlation explicitly diagnostic: a clean one-control-at-a-time recording at flying speed is still required before calling a transmitter channel reversed.

### Test plan and acceptance discipline

- Added `FLIGHT_TEST_PLAN.md` with an expert-development curriculum, formal test-card structure, instrumentation requirements and envelope-expansion order.
- Added competitive simulator comparison targets and a future multi-pilot handling gate.
- Separated deterministic simulation evidence from physical thrust-stand, battery-discharge, motor-manufacturer, real-transmitter, visual and expert-pilot acceptance.
- Preserved the claim boundary: this release is research-backed and systematically tested, but not yet expert-pilot-proven or hardware-calibrated.

### Windows application, resources, and packaging

- Added `src/rc_trainer.rc` plus privately loaded B612 Mono regular/bold font resources and their OFL licence for the native Windows application.
- Updated `CMakeLists.txt` to compile the propulsion, landing-gear, ground, environment and flight-report modules.
- Added the chase-camera helper and Windows resource script to the RCTrainer target.
- Added the GDI+ link dependency required by the cached environment renderer.
- Kept the post-build copy of the full `assets` tree so aircraft, ground and environment art travel beside `RCTrainer.exe`.
- Expanded `.gitignore` for generated reports and other runtime evidence while keeping source assets and authored documents tracked.
- Updated the GLB conversion and textured-aircraft renderer paths for separately tagged propeller geometry and shaft-driven propeller presentation.

### Build and automated verification

- Updated `build.ps1 -RunTests` to wait for the GUI self-test process and propagate its real exit code instead of returning before completion.
- Made `-RunTests` execute both the conventional deterministic suite and the virtual test-pilot card.
- Added `-RunTestPilot` for independently rebuilding and running the virtual test-pilot path.
- Expanded the conventional suite across yaw stability, throttle-only heading, propulsion, individual wheel contact, braking, touchdown, ground effect, chase camera, control mapping and the prior aerodynamic/replay/model gates.
- Preserved deterministic repeatability, finite-state, parked stability, takeoff, live-style takeoff, hands-off stability, pressure scaling, stall shape, wind-relative flow, Galilean invariance, replay loop and model-contract checks.
- Generated human-readable evidence in `self-test.txt` and the report/oracle files while keeping generated results outside version control.

### Known limitations at 0.1.0

- No physical thrust-stand or battery-discharge measurements have calibrated this exact virtual powertrain.
- No current manufacturer data has independently validated the chosen replaceable motor and battery constants.
- The ground model is deliberately low order; its brake-distance result is a regression corridor, not validated tyre/runway performance.
- The environment is a baked visual scene, not live weather, geospatial terrain or time-of-day simulation.
- The transmitter report cannot diagnose channel direction reliably unless the pilot records clean, isolated control movements.
- Automated flight and rendering checks do not replace human visual acceptance, real-transmitter acceptance or experienced RC-pilot evaluation.
