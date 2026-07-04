# SO-101 Viewer

The [So101Viewer](https://timrobot.github.io/SOBle/) serves as the central control dashboard and digital twin visualization interface for the **SO-101** robotics platform. It integrates a 3D WebGL viewport powered by Three.js with hardware interface hooks for real-time teleoperation, calibration, and servo diagnostics.

---

## 📊 Core Features & Capabilities

### 1. 3D Digital Twin & Scene Visualization
* **Dynamic MuJoCo Model Parsing:** Utilizes a custom `MuJoCoSceneBuilder` and STLLoader assembly to dynamically parse and render structural meshes for both the mobile platform chassis and dual arm systems (Leader and Follower tracking configurations).
* **High-DPI Viewport Stabilization:** Implements automatic viewport, scissor testing, and rendering boundary resets mapped to high-DPI (`devicePixelRatio`) displays, eliminating projection skew and layout shifts on 4K monitors.
* **Gimbal HUD Overlay:** Integrates a real-time orientation HUD overlay (`GimbalHud`) to track spatial changes and alignment relative to the layout.
* **Fiducial Tag Tracking:** Incorporates an AprilTag computer vision overlay system (`AprilTagOverlay`) that receives camera detection strings, transforming length coordinates into real-world meter projections to render tag spatial locations in the scene.

### 2. Hardware Teleoperation & Connectivity
* **Web Bluetooth Low Energy (BLE):** Remotely pairs with the physical `SO101Platform` over BLE. Includes automatic connection health monitoring, link state filtering, and diagnostic readouts tracking onboarding status (Wi-Fi network state and Raspberry Pi compute vitality).
* **Keyboard-Driven Mobile Chassis Control:** Captures low-level key events (`W`, `A`, `S`, `D`) to compute live throttle and steering vectors, dispatching differential drive telemetry directly to the robot while reporting hardware wheel position feedback via dual dial encoder gauges.
* **Web Serial Teleoperation Bus:** Hooks into physical master controller arms via a dedicated Web Serial interface (`LeaderSerial`). Automatically polls raw hardware encoder arrays from the Leader device to drive the kinematic virtual scene and mirror inputs to the follower.

### 3. Kinematic Joint Mapping
* **Proportional Linear Remapping:** Features an explicit vector-mapping algorithm (`vectorMap`) that scales joint inputs proportionally. It converts raw master encoder bounds seamlessly across custom joint ranges to yield safe target positions for the secondary Follower servos.
* **Dual-Range Manual Positioners:** Provides interactive dual-range UI sliders for individual degrees of freedom ($J_1$ to $J_6$), enabling manual joint tuning when the master serial bus is disconnected.
* **Actuation Safety interlock:** Implements a global control enablement toggle that locks out positioning pipelines until an authenticated kinematic calibration configuration profile is active.

---

## 🛠️ Diagnostics, Flashing, & Calibration (Settings Dialog)

The app communicates lower-level diagnostic and structural alignment routines to the hardware layer through a child `SettingsDialog` view element:

### 🎛️ Kinematic Arm Calibration
* **Live Range Extremum Tracking:** Registers absolute minimum (`min`) and maximum (`max`) raw encoder endpoints as physical joints are moved across their functional limits.
* **Configuration Packaging:** Compiles registered extreme bounds into specialized configuration maps via the `CalibrationConfig` runtime utility.
* **Profile Persistence:** Supports saving generated JSON structural map profiles directly to local disks or hot-loading pre-configured parameters to override default axis boundaries.

### ⚡ Servo Diagnostics & Configuration
* **ID Reprogramming (`writeServoId`):** Provisions individual hardware node adjustments by directly updating specific serial-bus servo identifiers across the active bus connection.
* **Home Axis Alignment (`centerServo`):** Dispatches low-level centering frames to individual servo nodes, commanding specific IDs to snap back to factory default home alignment notches ($2048$ / mid-point).