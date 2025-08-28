# OpenMotion

## 6-DOF Motion Simulator (Stewart Platform)

OpenMotion is a real-time 6 Degrees of Freedom (6-DOF) motion simulator that drives a Stewart platform across X, Y, Z, roll (φ), pitch (θ), and yaw (ψ). The desktop app (Qt + ImGui/ImPlot3D) computes inverse kinematics and streams servo commands to an Arduino-based motion base; it can be driven manually via sliders or live from X-Plane Flight Simulator over UDP.

If you scroll down, you will find some information on the journey I went through to put together the physical platform. But thanks to SolidWorks and some 3D printing, I can show you the first full system test with a Flight Simulator (X-Plane). If possible, try not to look at how I wired the power lines :P. Here's the clip:



https://github.com/user-attachments/assets/6e2c3a48-fd71-4478-bf3f-1aa16c9fb139

Here is the test for the new and updated GUI that includes a platform simulation independent of any physical platform:



https://github.com/user-attachments/assets/462b11cf-cf5f-43e0-a33d-44034f19c2b5



---

## What’s new

- **Physical platform assembled** and online (servo-driven).
- **Inverse kinematics** hooked directly to Arduino; end-to-end motion path working.
- **X-Plane integration**: live telemetry wired in; platform can be driven “From Sim”.
- **3D visualization**: ImGuiGLWidget + ImPlot3D render base/top/legs at ~60 Hz.
- **Serial I/O**: auto-detects Arduino, retries if unplugged, streams CSV angles.
- **Telemetry Dock**: live roll/pitch/yaw, body rates, and accelerations.
- **Safety/robustness**: stale-sim watchdog decays pose toward zero; home/zero capture.

> Geometry changed slightly after build; the Qt/ImPlot3D model is being updated to match the final hardware layout.

---

## Past Demos and Test for fun

<!-- Hardware prototype -->
<video controls playsinline muted loop width="640"
       src="https://github.com/user-attachments/assets/c51cb468-61b1-43b8-a83f-750a5365476e">
  <a href="https://github.com/user-attachments/assets/c51cb468-61b1-43b8-a83f-750a5365476e">Watch video</a>
</video>

<!-- Servo control -->
<video controls playsinline muted loop width="640"
       src="https://github.com/user-attachments/assets/a1f8d8f1-394a-4851-b28d-f788cad15bbf">
  <a href="https://github.com/user-attachments/assets/a1f8d8f1-394a-4851-b28d-f788cad15bbf">Watch video</a>
</video>


---

## Features

### Current
- Manual mode (sliders) and **From Sim** mode (X-Plane UDP).
- **Inverse kinematics** → per-servo angles with pairing/inversion and clamps.
- **Real-time rendering** (~60 Hz) with fixed/auto world bounds.
- **Serial protocol**: `a,b,c,d,e,f\n` (0–180°), `HOME`, `PING`, echo replies.
- **Home/zero** from current sim pose; watchdog decays pose when telemetry stalls.

### Planned
- Motion cueing filters (washout, gains, limits).
- Geometry/rig editor and calibration tools.
- Smoother servo drives (rate/accel limiting, deadband, microseconds mode).
- Logging, plots, and replay.

---


# OpenMotion — Quick Start (Linux)

This project is built and run via **Qt Creator** using the provided `OpenMotion.pro`. No manual `make` steps needed.

---

## Dependencies

**Required**
- **Qt** 6.x (or 5.15+) — Widgets + OpenGL
- **C++17** toolchain (g++, clang, etc.)
- **Eigen** (header-only linear algebra)
- **Dear ImGui** (core + OpenGL3 backend)
- **ImPlot3D** (already vendored in this repo: `implot3d.*`)

**Recommended (Debian/Ubuntu/Kubuntu)**
```bash
sudo apt update
sudo apt install qt6-base-dev build-essential cmake git libeigen3-dev

## Notes
- **ImPlot3D** is included; no extra install required.
- **Eigen** is header-only. If `libeigen3-dev` isn’t available, vendor headers into `Libraries/eigen/` and add that to your include path.
- **Dear ImGui** must be available in your include paths along with the OpenGL3 backend (`backends/imgui_impl_opengl3.*`). If you don’t already have it vendored:
```bash
git submodule add https://github.com/ocornut/imgui Libraries/imgui
```
- Then ensure your `.pro` includes the ImGui sources and backend.

## Build & Run (Qt Creator)
1. Launch **Qt Creator**.
2. **Open Project** → select `OpenMotion.pro`.
3. Select a **Kit** (Qt 6 preferred).
4. **Build** → **Run**.

### If Creator complains about missing headers
- Ensure Eigen is installed system-wide (`/usr/include/eigen3`) or vendored at `Libraries/eigen/`.
- Ensure `Libraries/imgui/` exists (if you added it) and that the `.pro` file includes ImGui sources and `backends/imgui_impl_opengl3.cpp`.
- `implot3d.*` is part of this repo and is compiled with the app.

## Configure X-Plane Input
1. In X-Plane, enable **UDP Data Output** and set the port to match `XP_UDP_REC_PORT` (see `config.h`).
2. Enable channels for roll/pitch/yaw (deg), p/q/r (rad/s), and accelerations (m/s²).
3. In OpenMotion, toggle **Drive From Sim** to feed the platform from X-Plane.

## Arduino
- Sketch: `stewart_platform_arduino_code.ino`  
  Pins: `3, 5, 6, 9, 10, 11` • Baud: `115200`
- Protocol:
  - CSV: `90,90,90,90,90,90\n`
  - Text: `HOME`, `PING`
- Odd channels can be inverted; per-channel trims/clamps are available in the sketch.
- **Power**: Use an external 5–6 V PSU for servos and **common GND** with the Arduino.

> **Safety**: Verify mechanical limits offline; keep a power cut-off within reach.

## Configuration Highlights
- `config.h` — network ports, update rates, UI defaults  
- `xplaneudpreceiver.h` — minimal UDP receiver for X-Plane telemetry  
- `ImGuiGLWidget.*` — OpenGL + Dear ImGui + ImPlot3D renderer  
- `mainwindow.*` — UI, IK, serial, telemetry dock, drive modes

## Current Status
- Hardware assembled and servo-driven platform online  
- IK engine driving servos directly (unfiltered signals)  
- Platform motion synchronized with X-Plane via OpenMotion  
- Geometry adjustments identified for Qt/ImPlot3D model refinement

## Next steps
- Update Qt visualization to match revised geometry  
- Tune outputs for smoother, stable motion and alignment  
- Implement motion cueing filters (washout, gains, limits)

## License
GPL-3.0
