# RC Trainer — aerodynamic vertical slice

## Complete development record

The repository keeps a deliberately exhaustive record of every RC Simulator feature, correction, physics revision, input fix, rendering change, diagnostic, validation result, and known boundary in **[README.md](README.md)**.

The record separates the committed `3b3d4e8` trainer foundation from the current aerodynamic Cessna update and distinguishes source validation from packaged-release or human acceptance.

A clean-room, Windows-native RC flight trainer. The first slice deliberately stays small:

- DirectInput controller discovery with a keyboard fallback
- automatic PS5 DualSense Mode 2 mapping (left stick throttle/rudder, right stick elevator/aileron) with an isolated controller profile
- automatic hot-plug discovery (no restart needed when an adapter appears)
- direct DX6i trainer-port PPM decoding through a Windows microphone/line input
- automatically prefers a connected Native Instruments Komplete Audio interface; otherwise uses the usual last-listed Windows audio input
- live raw-axis inspector, calibration, channel assignment, reversing, endpoints and dead zones
- a 3.65 kg, 1:6.395 high-wing electric C172 replica and one simple grass field
- a texture-mapped Cessna 172 asset at exact 1:6.395 scale (1.72 m wingspan), with landing gear and component collision hulls
- fixed-step telemetry plus deterministic CSV record/replay
- embedded Airbus/PolarSys B612 Mono cockpit readout (including bold heading face), with no system-font installation required

The second slice replaces the prototype force approximation with local-surface, low-Reynolds-number aerodynamics and a six-degree-of-freedom rigid body. Wing stall and drag rise are anchored to measured NACA 2412 data; finite-wing correction, induced drag, tail downwash, surface-local rotational flow, control-surface incidence, propeller advance ratio, measured APC 14x7 coefficients, prop torque and the inertia tensor all participate in the result. The HUD exposes AoA, sideslip, Reynolds number, propeller RPM and stall fraction. See [AERODYNAMICS.md](AERODYNAMICS.md) for equations, research sources, validation gates and the explicit future live-weather boundary.

Propulsion is an aircraft-installed component rather than a Cessna-specific force. The current electric unit couples motor back EMF, ESC/current limits, battery sag, shaft inertia and measured propeller load; it supplies force, reaction/gyroscopic moment, propwash, telemetry and visual shaft phase. New aircraft should provide mass/inertia, aero panels, collision proxies and a list of propulsion definitions. `U` switches between default `REALISTIC` reaction torque and the explicitly labelled trainer assist.

The flight-school loop remains restartable: takeoff, climb, cruise, power-off glide, and a controlled landing. The HUD presents the current objective, elapsed time, and score; a crash returns the aircraft and lesson to the start after two seconds without disturbing input calibration or replay data. The ground is a world-locked, perspective-correct textured plane with fine and large-scale grass layers, distance mip filtering and a mown grass runway embedded in the same projection. The macro layer preserves broad field variation when the fine texture averages at altitude. Ground is shaded at half resolution and filtered once into the frame, avoiding grass geometry, a dense grid and screen-space texture tiles. Cached baked sky, rolling-hill and photographic treeline PNG layers move panoramically with heading; the transparent irregular lower feather blends the distant landscape into the ground rather than restoring a straight horizon seam. No synthetic ground-fog colour is applied. Trainer-port PPM input receives light center dead-zone filtering and smoothing.

Ground motion now comes from three individual wheel contacts with suspension, tyre grip, rolling resistance, rudder-linked nose-wheel steering and explicit braking. Ground effect varies continuously with wing height/span. The complete test-card, expertise and competitive-validation programme is in [FLIGHT_TEST_PLAN.md](FLIGHT_TEST_PLAN.md).

The flight recorder stores the complete interval from **Record Flight** to **Stop** in `replays/latest.csv`. Stopping a recording also creates `reports/transmitter-latest.html`: a self-contained virtual test-pilot report with the received controls, X/Y/Z position, pitch/bank/heading, vertical speed, airspeed, angle of attack, stall state, and control-sign screening. Launch always begins as a fresh live flight; press **O** to load and loop a saved flight. **Manual Override** immediately ends playback and hands the aircraft back to the live controller; **Clear** stops playback and removes the saved flight.

Trajectory validation is available during every flight. With no route selected, the cyan line is a two-second held-control forecast produced by cloning the current aircraft and stepping the same physics solver; the one-second prediction is later compared with the real aircraft. Fixed routes instead show a cyan reference path, colour the actual trace green/amber/red by deviation, and draw perpendicular error whiskers. `G` opens the friendly validation dashboard and cycles top, side, and simplified Wind Tunnel views. It includes clickable route cards, close-follow/overview and layer controls, deviation/anomaly cards, synchronized telemetry plots, record/export controls, and buttons to mark raising, bumpiness, sluggish response, or another problem at the exact flight frame.

Codex troubleshooting is live while the simulator runs. `reports/troubleshooting/live-flight-snapshot.json` is atomically refreshed at 10 Hz, `current.ptr` identifies the active append-only JSONL archive, and every launch creates a unique historical session under `reports/troubleshooting/archive/`. The feed records applied controls, full motion/attitude, aero/propulsion/ground state, trajectory error, derived acceleration/jerk, anomaly scores, lifecycle changes, and user issue markers. Run `tools/inspect_live_flight.ps1` for a compact recent-window diagnosis; the raw archive remains available when the user asks about the previous flight.

Choosing any fixed trajectory is an active test command, not just a drawing option. The simulator snaps to that route's repeatable start state, selects padded overview, and hands the four normal RC channels to `RoutePilot`. The pilot is deliberately authority-limited and uses the same physics step as the user, so the aircraft may deviate; it is never placed directly on later route points. `STOP`, Manual Override, or the held-control-prediction card returns control to the user. Main flight view retains the world-space route layer and adds a whole-route deviation inset.

The visual aircraft is the 29,700-triangle CC BY 4.0 “Plane (Cessna 172)” asset by osmosikum, rendered with its texture maps by the built-in depth-tested software renderer. The original GLB and full attribution are packaged under `assets/aircraft/cessna172`. Collision primitives and named aerodynamic panels remain separate data in `src/aircraft_model.*`; `src/flight_dynamics.*` evaluates those panels.

## Controls

| Key | Action |
|---|---|
| `R` | show/hide help |
| `T` | cycle through Windows audio inputs for trainer-port PPM |
| `Y` | restart the flight-school lesson and aircraft |
| `U` | switch between realistic propulsion reaction and trainer assistance |
| `B` | hold wheel brakes |
| `C` | start calibration; move every used stick through its full travel, then press `Enter` |
| `1`–`4` | select a function: aileron, elevator, throttle, rudder |
| `[` / `]` | assign the previous/next physical axis to the selected function |
| `L` | reverse the selected function |
| `-` / `=` | decrease/increase selected function dead zone |
| `I` | start/stop recording to `replays/latest.csv` |
| `O` | play `replays/latest.csv` |
| `P` | stop playback |
| `F` | cycle no-route prediction and the fixed expert-test routes |
| `G` | cycle flight, top dashboard, side dashboard, and simplified Wind Tunnel view |
| `H` | show/hide the trajectory layer |
| `J` | switch diagnostic views between close follow and route overview |
| `K` | export `reports/trajectory-latest.csv` |
| `Space` | reset aircraft on the runway |
| `Esc` | exit |

Keyboard fallback follows the transmitter sticks: hold `W/S` for throttle, `A/D` for rudder, left/right arrows for aileron, and briefly hold the down arrow to pull the elevator back for nose-up rotation (up arrow pushes nose-down). Each mouse-wheel notch adds a 10% elevator pulse, holds it briefly, then springs it smoothly back to centre so a short scroll cannot leave the aircraft in a permanent stall. On takeoff, build at least 11.5 m/s, briefly hold the down arrow to rotate, then release it as the aircraft climbs. Keyboard flight input is accepted only while the trainer window has focus, and held directional controls clear when it loses focus.

## Build

Run `build.ps1`. The executable is written to `build/Release/RCTrainer.exe`.

Run `build.ps1 -RunTests` for the conventional regression suite plus the deterministic flight-envelope oracle. It writes `reports/virtual-test-pilot.html`, full telemetry in `virtual-test-pilot.csv`, and one concise row per expected corridor in `virtual-test-pilot-oracle.csv`. To run only the flight-envelope card after compiling, use `build.ps1 -RunTestPilot`.

## Cockpit readout typeface

The in-app flight readout uses **B612 Mono**, a typeface designed and tested for aircraft cockpit screens. Its regular and bold faces are compiled directly into `RCTrainer.exe`; the application loads them privately at runtime, so the visual design is retained on every user's PC. The source fonts and their SIL Open Font License 1.1 are redistributed in `assets/fonts/` beside the executable.
