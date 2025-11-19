# Control Award Submission - Final - Team 18338 StarTech

## 1. Software Philosophy: The Robot as a Partner

Our software philosophy is centered around three core principles: **Robustness, Intelligence, and Driver-Enhancement**. We believe the most effective robot is one that can reliably execute complex tasks, intelligently adapt to the dynamic nature of the game, and seamlessly augment the capabilities of our human drivers. Our control system is designed from the ground up to be a true partner on the field, not just a tool.

---

## 2. The "Super" Autonomous: A Multi-Phase, Dynamic Strategy

Our flagship autonomous program, `AutonomusSuper.java`, is not a single, hard-coded path. It is a sophisticated, multi-phase state machine that leverages advanced sensor fusion to execute a complete, high-scoring routine.

### Phase 1: Precision First Score
*   **Vision-Based Start:** The routine begins by identifying one of three randomized AprilTags (ID 21-23), instantly determining the game state. Based on the tag's position, the robot determines its starting side (left or right), which dictates all subsequent strategic decisions.
*   **Dynamic Positioning:** Using a PID-like control loop, the robot performs "visual-servoing" to precisely position itself relative to both the initial tag and the scoring goal tag (ID 20 or 24). This vision-based navigation is highly resistant to odometry drift.
*   **First Strike:** Once in position, the robot scores its pre-loaded artifacts.

### Phase 2: Autonomous Ball Collection
This is where our robot truly demonstrates its intelligence. Instead of parking, it transitions into a collection phase.
*   **Dual Vision Processing:** Our VisionPortal runs two processors simultaneously: `AprilTagProcessor` and our custom `BallDetectionProcessor`. This allows the robot to search for colored balls (purple and green) without losing its ability to see AprilTags.
*   **Search-and-Acquire Sub-routine:** The robot enters a sub-state machine dedicated to collection. It actively scans its surroundings, and upon detecting a ball, it centers on it, estimates distance based on the contour's size, and initiates an intake sequence.
*   **Sensor Fusion for Confirmation:** A distance sensor inside the intake provides definitive confirmation of a successful collection, making the process highly reliable. The robot repeats this cycle until it has collected the required number of new balls.

### Phase 3: Second Score and Intelligent Parking
*   **Re-acquisition and Scoring:** With new balls collected, the robot re-acquires the goal AprilTag, navigates back to the optimal scoring position, and launches its second volley.
*   **Vision-Assisted Park:** The final parking maneuver is not a "blind" move. The robot keeps the goal AprilTag in view, executing a precise lateral strafe to a calculated offset. This ensures a consistent and accurate final position, every time.

---

## 3. Seamless Transition & Driver-Enhancing TeleOp

Our `TeleOpStarTech.java` program continues the philosophy of intelligent control, empowering our drivers.

### 3.1. Robust Autonomous-to-TeleOp Data Persistence
To maintain field awareness, we implemented a **fail-safe, hybrid data persistence system**.
*   **Primary (File Storage):** At the end of autonomous, the robot's final `Pose` and starting `Side` are saved to a persistent file.
*   **Backup (Static Variable):** The same data is simultaneously saved to a `static` variable in memory (`OpModeData`).
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

These advanced features are built upon a foundation of professional software engineering practices, including modular helper classes (`PoseStorage`, `BallDetectionProcessor`), a strict adherence to readable and maintainable code, and robust error handling to ensure peak reliability during competition.
