# Control Award Submission - Team 18338 StarTech

## 1. Software Philosophy: The Robot as a Partner

Our software philosophy is centered around three core principles: **Robustness, Intelligence, and Driver-Enhancement**. We believe the most effective robot is one that can reliably execute complex tasks, intelligently adapt to the dynamic nature of the game, and seamlessly augment the capabilities of our human drivers. Our control system is designed from the ground up to be a true partner on the field, not just a tool.

---

## 2. Dynamic, Vision-Based Autonomous (`AutonomusStarTech.java`)

Our autonomous program is not a single, hard-coded path. It is a dynamic state machine that uses computer vision to react to the randomized game state and **establish true field-centric awareness from the very first second.**

*   **Absolute Pose Calculation at Init:** Instead of assuming a (0,0) start, our `init` phase features an interactive setup routine. The robot uses the known absolute position of the initial AprilTag (`(72, 144)`) and its own relative measurements (`range`, `bearing`, `yaw`) to perform trigonometric calculations and determine its **true starting Pose (X, Y, and Heading)** on the field. This calculated pose is then used to initialize the pathing system, ensuring all subsequent movements are in the absolute field coordinate system.
*   **Interactive Setup Assistance:** The software actively assists the human operator during setup. Based on the robot's distance from the tag, it determines the required starting angle (45 or 90 degrees). If manual adjustment is needed, the telemetry provides clear, real-time instructions like **"Turn Robot LEFT"** or **"Turn Robot RIGHT"** until the alignment is perfect. This human-robot collaboration drastically improves the consistency and reliability of our autonomous runs.
*   **Dynamic Positioning:** Using a PID-like control loop, the robot performs "visual-servoing" to precisely position itself relative to both the initial tag and the scoring goal tag (ID 20 or 24). This vision-based navigation is highly resistant to odometry drift and minor setup inconsistencies.
*   **Vision-Assisted Park:** The final parking maneuver is not a "blind" move. The robot keeps the goal AprilTag in view, executing a precise lateral strafe to a calculated offset. This ensures a consistent and accurate final position, every time.

---

## 3. Seamless Transition & Driver-Enhancing TeleOp (`TeleOpStarTech.java`)

Our `TeleOpStarTech.java` program continues the philosophy of intelligent control, empowering our drivers.

### 3.1. Robust Autonomous-to-TeleOp Data Persistence
To maintain field awareness, we implemented a **fail-safe, hybrid data persistence system**.
*   **Primary (File Storage):** At the end of autonomous, the robot's final `Pose` and starting `Side` are saved to a persistent file (`last_pose.txt`). This method survives an application restart.
*   **Backup (Static Variable):** The same data is simultaneously saved to a `static` variable in memory (`OpModeData`). This data persists between OpModes if the app is not fully closed.
*   **Intelligent Loading:** TeleOp first tries to load from the file. If that fails, it seamlessly falls back to the static backup. This two-tiered system makes our robot exceptionally resilient to errors, ensuring the driver **always** starts with an accurate field-centric pose.

### 3.2. Context-Aware Driver Assistance
*   **"Smart" Go-To-Point Navigation:** A single button press (`dpad_left`) sends the robot to a key position. The destination is chosen intelligently based on the starting side loaded from the autonomous period (`(24, 24)` for left, `(84, 84)` for right).
*   **Real-Time Scoring Assist:** During manual control, the vision system remains active. When aiming, the robot detects the goal AprilTag and automatically scales the outtake motor power based on the measured distance, ensuring high scoring consistency.

### 3.3. Real-Time "Auto-Aim" & Firing Interlock
The pinnacle of our driver-assist features is the "Auto-Aim" mode, activated via the `start` button. This transforms the robot into a semi-autonomous scoring platform:
*   **Persistent Target Lock:** Once activated, the software takes full control of the drivetrain. It continuously uses real-time vision data to maintain a perfect distance (`GOAL_TAG_DISTANCE`) and alignment to the target AprilTag. If the robot is pushed or bumped, it **autonomously corrects its position** to re-acquire the lock.
*   **Firing Interlock System:** To prevent missed shots, the firing servos are **completely disabled** while the robot is searching for the tag or adjusting its position. Only when the alignment error is within our strict `POSITIONING_TOLERANCE` do the servo controls become active, signaled to the driver with a "Locked On! Ready to fire." telemetry message. This ensures that an artifact can only be launched when a successful shot is guaranteed.

This system allows the driver to focus solely on the timing of the shot, confident that the robot's alignment is perfect.

## 4. Foundation of Quality Code

These advanced features are built upon a foundation of professional software engineering practices, including modular helper classes (`PoseStorage`), a strict adherence to readable and maintainable code, and robust error handling to ensure peak reliability during competition.
