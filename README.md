# RC Trainer — second vertical slice

A clean-room, Windows-native RC flight trainer. The first slice deliberately stays small:

- DirectInput controller discovery with a keyboard fallback
- automatic hot-plug discovery (no restart needed when an adapter appears)
- direct DX6i trainer-port PPM decoding through a Windows microphone/line input
- automatically prefers a connected Native Instruments Komplete Audio interface; otherwise uses the usual last-listed Windows audio input
- live raw-axis inspector, calibration, channel assignment, reversing, endpoints and dead zones
- one tuneable high-wing electric trainer and one simple grass field
- fixed-step telemetry plus deterministic CSV record/replay

The second slice adds a complete, restartable flight-school loop on top of that foundation: takeoff, climb, cruise, power-off glide, and a controlled landing. The HUD presents the current objective, elapsed time, and score; a crash returns the aircraft and lesson to the start after two seconds without disturbing input calibration or replay data. A lightweight procedural grass grid and long, irregular two-line grass blades use camera pitch and altitude to show lift, speed, and distance without bitmap-rendering overhead. The blades extend far into the field while thinning sharply with distance. Trainer-port PPM input receives light center dead-zone filtering and smoothing.

The flight recorder stores the complete interval from **Record Flight** to **Stop** in `replays/latest.csv`. A saved flight starts automatically on the next launch and loops forever. **Manual Override** immediately ends playback and hands the aircraft back to the live controller; **Clear** stops playback and removes the saved flight so nothing starts on the next launch.

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

Keyboard fallback follows the transmitter sticks: hold `W/S` to raise/lower the left stick (throttle), hold `A/D` to move it left/right (rudder), use the mouse wheel to move the right stick up/down (elevator), and use the left/right arrow keys for its horizontal axis (aileron). Each mouse-wheel notch moves the right stick vertically by 10%; wheel forward moves it up.

## Build

Run `build.ps1`. The executable is written to `build/Release/RCTrainer.exe`.

