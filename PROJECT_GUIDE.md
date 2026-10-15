# RC Trainer — aerodynamic vertical slice

## Complete development record

The repository keeps a deliberately exhaustive record of every RC Simulator feature, correction, physics revision, input fix, rendering change, diagnostic, validation result, and known boundary in **[README.md](README.md)**.

The record separates the committed `3b3d4e8` trainer foundation from the current aerodynamic Cessna update and distinguishes source validation from packaged-release or human acceptance.

A clean-room, Windows-native RC flight trainer. The first slice deliberately stays small:

- DirectInput controller discovery with a keyboard fallback
- automatic hot-plug discovery (no restart needed when an adapter appears)
- direct DX6i trainer-port PPM decoding through a Windows microphone/line input
- automatically prefers a connected Native Instruments Komplete Audio interface; otherwise uses the usual last-listed Windows audio input
- live raw-axis inspector, calibration, channel assignment, reversing, endpoints and dead zones
- a 3.65 kg, 1:6.395 high-wing electric C172 replica and one simple grass field
- a texture-mapped Cessna 172 asset at exact 1:6.395 scale (1.72 m wingspan), with landing gear and component collision hulls
- fixed-step telemetry plus deterministic CSV record/replay

The second slice replaces the prototype force approximation with local-surface, low-Reynolds-number aerodynamics and a six-degree-of-freedom rigid body. Wing stall and drag rise are anchored to measured NACA 2412 data; finite-wing correction, induced drag, tail downwash, surface-local rotational flow, control-surface incidence, propeller advance ratio, measured APC 14x7 coefficients, prop torque and the inertia tensor all participate in the result. The HUD exposes AoA, sideslip, Reynolds number, propeller RPM and stall fraction. See [AERODYNAMICS.md](AERODYNAMICS.md) for equations, research sources, validation gates and the explicit future live-weather boundary.

The flight-school loop remains restartable: takeoff, climb, cruise, power-off glide, and a controlled landing. The HUD presents the current objective, elapsed time, and score; a crash returns the aircraft and lesson to the start after two seconds without disturbing input calibration or replay data. The ground is intentionally a single stretched field polygon that reaches the horizon. One runway wedge changes apparent width with altitude, while sixteen broad moving distance bands and two boundary lines provide cheap scale and movement cues—no grass blades or dense grid. Trainer-port PPM input receives light center dead-zone filtering and smoothing.

The flight recorder stores the complete interval from **Record Flight** to **Stop** in `replays/latest.csv`. Launch always begins as a fresh live flight; press **F6** to load and loop a saved flight. **Manual Override** immediately ends playback and hands the aircraft back to the live controller; **Clear** stops playback and removes the saved flight.

The visual aircraft is the 29,700-triangle CC BY 4.0 “Plane (Cessna 172)” asset by osmosikum, rendered with its texture maps by the built-in depth-tested software renderer. The original GLB and full attribution are packaged under `assets/aircraft/cessna172`. Collision primitives and named aerodynamic panels remain separate data in `src/aircraft_model.*`; `src/flight_dynamics.*` evaluates those panels.

## Controls

| Key | Action |
|---|---|
| `F1` | show/hide help |
| `F2` | cycle through Windows audio inputs for trainer-port PPM |
| `F3` | restart the flight-school lesson and aircraft |
| `C` | start calibration; move every used stick through its full travel, then press `Enter` |
| `1`–`4` | select a function: aileron, elevator, throttle, rudder |
| `[` / `]` | assign the previous/next physical axis to the selected function |
| `R` | reverse the selected function |
| `-` / `=` | decrease/increase selected function dead zone |
| `F5` | start/stop recording to `replays/latest.csv` |
| `F6` | play `replays/latest.csv` |
| `F7` | stop playback |
| `Space` | reset aircraft on the runway |
| `Esc` | exit |

Keyboard fallback follows the transmitter sticks: hold `W/S` for throttle, `A/D` for rudder, left/right arrows for aileron, and briefly hold the down arrow to pull the elevator back for nose-up rotation (up arrow pushes nose-down). Each mouse-wheel notch adds a 10% elevator pulse, holds it briefly, then springs it smoothly back to centre so a short scroll cannot leave the aircraft in a permanent stall. On takeoff, build at least 11.5 m/s, briefly hold the down arrow to rotate, then release it as the aircraft climbs. Keyboard flight input is accepted only while the trainer window has focus, and held directional controls clear when it loses focus.

## Build

Run `build.ps1`. The executable is written to `build/Release/RCTrainer.exe`.
