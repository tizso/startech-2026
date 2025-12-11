package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareBox;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This is the main TeleOp program for the StarTech team's 2026 season robot.
 * <p>
 * It features:
 * - A field-centric drive system using the Pedro Pathing library.
 * - Seamless pose transition from the Autonomous period.
 * - Driver-assist automation for navigating to preset field locations.
 * - A sophisticated, physics-based, dynamic shooter velocity calculation.
 * - A robust, sensor-driven automatic separator mechanism with manual override.
 * - An innovative shot detection algorithm based on shooter velocity drop.
 */
@Configurable
@TeleOp(name = "TeleOp StarTech New - 2026", group = "00-TeleOp")

public class TeleOpStarTechNew extends OpMode {

    // Core Robot and Pathing Objects
    HardwareBox robot;
    private Follower follower;

    // Driving Control Variables
    private boolean slowMode = false;

    private double SLOW_DOWN_FACTOR = 1.0;

    // Mechanism State Toggles
    private boolean intake = false;
    private boolean outtake = false;
    private boolean reverse = false;
    private boolean sep = false;
    private boolean manualSeparatorMode = false;

    // Sensor and Ball Counter States
    private boolean ballWasPresent = false; // Used for rising-edge detection of a new ball
    private int ballCount = 0; // Tracks the number of balls currently held by the robot

    // Shot Detection State Machine
    private enum ShotDetectorState {READY, COOLDOWN} // States to prevent a single shot from being counted multiple times

    private ShotDetectorState shotDetectorState = ShotDetectorState.READY;
    private final ElapsedTime shotCooldownTimer = new ElapsedTime();
    private double lastShooterVelocity = 0.0; // Stores the previous loop's velocity for drop detection
    private static final double SHOT_COOLDOWN_MS = 500; // Cooldown period in milliseconds after detecting a shot
    private static final double SHOT_VELOCITY_DROP_PERCENT = 0.15; // Velocity drop threshold (15%) to register a shot

    // Gamepad State Variables for one-shot button presses
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    // Vision Components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- Constants ---
    private static final double SHOOTER_RPM = 4800; // Default RPM if no AprilTag is visible
    private static final double TICKS_PER_REV = 28; // From REV HD Hex Motor datasheet
    private static final double BASKET_HEIGHT_M = 1.0; // Height of the target basket in meters
    private static final double ROBOT_SHOOT_HEIGHT_M = 0.381; // 15 inches, height of the shooter from the ground
    private static final double SHOOT_ANGLE_DEG = 45; // The fixed physical angle of the shooter
    private static final double G = 9.81; // Acceleration due to gravity (m/s^2)
    private static final double WHEEL_DIAMETER_M = 0.096; // Diameter of the shooter wheels in meters
    private static final double SLIP_FACTOR = 1.3; // Compensation factor for ball slipping on the flywheel
    private static final double BALL_DETECTION_THRESHOLD_INCH = 2.0; // Proximity sensor threshold for detecting a ball
    private static final double MIN_RPM = 3600; // Minimum RPM to prevent weak shots when too close
    private static final double MAX_RPM = 5000; // Maximum RPM to prevent overpowering shots when far away
    private int autoStartingSide = 0; // Stores the starting side determined in Autonomous (-1 for Red, 1 for Blue)

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        initVision();

        // --- Hybrid Pose Loading Logic ---
        // This ensures a seamless transition from the Autonomous period by loading the robot's last known pose.
        Pose startingPose;
        String loadSource;
        PoseStorage.StoredPose storedPoseFromFile = PoseStorage.loadPoseFromFile();
        boolean isFilePoseDefault = storedPoseFromFile.pose.getX() == 0 && storedPoseFromFile.pose.getY() == 0 && storedPoseFromFile.pose.getHeading() == 0;

        // Use the static OpModeData as a backup if the file is empty or uninitialized, but a pose exists from the same session.
        if (isFilePoseDefault && OpModeData.lastPose != null) {
            startingPose = OpModeData.lastPose;
            autoStartingSide = OpModeData.initialSide;
            loadSource = "Static Backup";
        } else {
            // Default to loading from the persistent file.
            startingPose = storedPoseFromFile.pose;
            autoStartingSide = storedPoseFromFile.initialSide;
            loadSource = "File";
        }

        follower.setStartingPose(startingPose);
        follower.update();
        robot = new HardwareBox();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pose Load Source", loadSource);
        telemetry.addData("Loaded Pose", "X: %.2f, Y: %.2f, H: %.2f", startingPose.getX(), startingPose.getY(), Math.toDegrees(startingPose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Engages TeleOp driving mode in the follower.
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update all subsystems in each loop iteration
        follower.update(); // Updates odometry and path following state
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        handleManualDrive();
        handleGamepadControls();
        handleShooter();

        updateTelemetry();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void handleManualDrive() {
        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        // Pass joystick values to the follower. The library internally handles
        // whether to drive manually or follow a path, ignoring this command if a path is active.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                true // Use field-centric drive
        );
    }

    private void handleGamepadControls() {
        // --- Driver-Assist Automation Trigger ---
        if (currentGamepad1.start && !previousGamepad1.start) {
            // On START button press, navigate to a preset position based on the starting side.
            Pose currentPose = follower.getPose();
            Pose targetPose = (autoStartingSide > 0) ?
                    new Pose(105, 34, Math.toRadians(90)) : // Blue side target
                    new Pose(39, 34, Math.toRadians(90));   // Red side target

            // This builds a two-stage path: first move to the location, then turn to the final heading.
            // This ensures the final heading is always correct, regardless of the starting orientation.
            Pose intermediatePose = new Pose(targetPose.getX(), targetPose.getY(), currentPose.getHeading());

            PathChain pathToTarget = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, intermediatePose)) // 1. Go to correct X,Y
                    .addPath(new BezierLine(intermediatePose, targetPose)) // 2. Turn to correct heading
                    .build();

            follower.followPath(pathToTarget);
        }

        // --- Mechanism Toggles ---
        if (currentGamepad1.a && !previousGamepad1.a) slowMode = !slowMode;
        if (currentGamepad1.b && !previousGamepad1.b) intake = !intake;
        if (currentGamepad1.x && !previousGamepad1.x) outtake = !outtake;
        if (currentGamepad1.y && !previousGamepad1.y) reverse = !reverse;

        // --- Separator Manual Override ---
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            manualSeparatorMode = false;
        } // Return to AUTO mode
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            manualSeparatorMode = true;  // Activate MANUAL mode
            sep = !sep; // Manually toggle separator position
        }

        // --- Auto-Aim to Backdrop ---
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            int goalTagId = (autoStartingSide > 0) ? 20 : 24; // Blue: 20, Red: 24
            AprilTagDetection goalTag = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == goalTagId) {
                    goalTag = detection;
                    break;
                }
            }

            if (goalTag != null && goalTag.ftcPose != null) {
                // We have a visible tag, let's align to it by both strafing and turning.
                Pose currentPose = follower.getPose();
                double currentHeading = currentPose.getHeading();

                // ftcPose.x is the lateral error in inches. A positive value means the tag is to the robot's right.
                // We need to strafe right by this amount.
                double strafeCorrectionInches = goalTag.ftcPose.x;

                // ftcPose.bearing is the rotational error in degrees. A positive value means the tag is to the left (CCW).
                // We need to turn left by this amount.
                double headingCorrectionRad = Math.toRadians(goalTag.ftcPose.bearing);

                // Calculate the target field position by applying the strafe correction.
                // A right strafe vector in field coordinates is (sin(H), -cos(H)).
                double targetX = currentPose.getX() + (strafeCorrectionInches * Math.sin(currentHeading));
                double targetY = currentPose.getY() - (strafeCorrectionInches * Math.cos(currentHeading));

                // Calculate the target heading by applying the bearing correction.
                double targetHeading = currentHeading + headingCorrectionRad;

                // Create a new target pose that both strafes and turns.
                Pose targetPose = new Pose(targetX, targetY, targetHeading);

                // Build a path to execute the alignment maneuver.
                PathChain alignPath = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetPose))
                        .build();

                follower.followPath(alignPath);
            }
        }

        // --- Direct Servo/Motor Control ---
        robot.servoInR.setPower(currentGamepad1.right_bumper ? 1.0 : 0.0);
        robot.servoInL.setPower(currentGamepad1.left_bumper ? 1.0 : 0.0);
        robot.intake.setPower(intake ? 0.9 : 0.0);

        // --- Automatic/Manual Separator Logic ---
        if (!manualSeparatorMode && intake) {
            boolean ballIsCurrentlyPresent = robot.sensorDistance.getDistance(DistanceUnit.INCH) < BALL_DETECTION_THRESHOLD_INCH;
            // Rising-edge detector: triggers only on the frame a new ball is seen
            if (ballIsCurrentlyPresent && !ballWasPresent) {
                if (ballCount < 3) ballCount++; // Increment ball count, max 3
                // Core logic: only toggle the separator for the VERY FIRST ball
                if (ballCount == 1) sep = !sep;
            }
            ballWasPresent = ballIsCurrentlyPresent; // Store current state for next loop's edge detection
        } else if (!intake) {
            ballWasPresent = false; // Reset detector when intake is off
        }
        robot.separator.setPosition(sep ? 0.8 : 0.3);

        // --- Reverse Mode Logic ---
        if (reverse) {
            robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInR.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInL.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInR.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    private void handleShooter() {
        if (!outtake) {
            robot.outtakeLeft.setPower(0);
            robot.outtakeRight.setPower(0);
            lastShooterVelocity = 0;
            return;
        }

        double targetRpm = SHOOTER_RPM;
        int goalTagId = (autoStartingSide > 0) ? 20 : 24; // Determine which backdrop tag to look for
        AprilTagDetection goalTag = null;

        // Find the correct goal tag from the camera's detections
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == goalTagId) {
                goalTag = detection;
                break;
            }
        }

        // If a tag is visible, perform the ballistic calculation
        if (goalTag != null) {
            double distance_m = goalTag.ftcPose.range * 0.0254;
            double theta_rad = Math.toRadians(SHOOT_ANGLE_DEG);
            double y = BASKET_HEIGHT_M - ROBOT_SHOOT_HEIGHT_M;
            double cos_theta = Math.cos(theta_rad);

            // Solves the projectile motion equation for the required initial velocity
            double denominator = 2 * cos_theta * cos_theta * (distance_m * Math.tan(theta_rad) - y);

            if (denominator > 0) {
                double v_ball = Math.sqrt((G * distance_m * distance_m) / denominator);
                double v_wheel = v_ball * SLIP_FACTOR; // Compensate for slip
                double omega_wheel = v_wheel / (WHEEL_DIAMETER_M / 2.0); // rad/s
                double calculatedRpm = omega_wheel * 60.0 / (2.0 * Math.PI);

                targetRpm = Math.max(MIN_RPM, Math.min(calculatedRpm, MAX_RPM));
            }
        }

        double targetVelocity = (targetRpm / 60.0) * TICKS_PER_REV;
        robot.outtakeLeft.setVelocity(targetVelocity);
        robot.outtakeRight.setVelocity(targetVelocity);

        // --- Shot Detection Logic ---
        // Manages the cooldown state to prevent multiple detections for a single shot
        if (shotDetectorState == ShotDetectorState.COOLDOWN && shotCooldownTimer.milliseconds() > SHOT_COOLDOWN_MS) {
            shotDetectorState = ShotDetectorState.READY;
        }

        double currentVelocity = (robot.outtakeLeft.getVelocity() + robot.outtakeRight.getVelocity()) / 2.0;
        // A shot is detected if the velocity suddenly drops below a threshold
        if (lastShooterVelocity > 0 && shotDetectorState == ShotDetectorState.READY) {
            double velocityDrop = lastShooterVelocity - currentVelocity;
            double dropThreshold = lastShooterVelocity * SHOT_VELOCITY_DROP_PERCENT;

            if (velocityDrop > dropThreshold && ballCount > 0) {
                ballCount--; // Decrement the ball counter
                shotDetectorState = ShotDetectorState.COOLDOWN; // Enter cooldown state
                shotCooldownTimer.reset();
            }
        }
        lastShooterVelocity = currentVelocity; // Store velocity for the next cycle
    }

    private void updateTelemetry() {
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Shooter Target RPM", outtake ? String.format("%.0f", (lastShooterVelocity * 60 / TICKS_PER_REV)) : "OFF");
        telemetry.addData("Shooter Actual Vel L|R", "%.1f | %.1f", robot.outtakeLeft.getVelocity(), robot.outtakeRight.getVelocity());
        telemetry.addData("Separator Mode", manualSeparatorMode ? "MANUAL" : "AUTO");
        telemetry.addData("Ball Count", ballCount);
        telemetry.update();
    }
}
