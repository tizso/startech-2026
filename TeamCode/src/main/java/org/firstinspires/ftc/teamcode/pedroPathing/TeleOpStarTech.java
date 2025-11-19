package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareBox;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "TeleOp StarTech - 2026", group="00-TeleOp")

public class TeleOpStarTech extends OpMode {

    HardwareBox robot;
    private Follower follower;
    private boolean automatedDrive = false; // For dpad_left path
    private boolean autoAiming = false;     // For start button aiming
    private boolean slowMode = false;
    private double SLOW_DOWN_FACTOR = 1.0;

    private boolean intake = false;
    private boolean outtake = false;

    private boolean reverse = false;
    private boolean sep = false;

    // AprilTag detection
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Positioning Constants
    private static final double GOAL_TAG_DISTANCE = 35.0; // inches, as in AutonomusStarTech

    /**
     * Proportional gain for turning correction. The larger the bearing error to the target,
     * the more this number amplifies the corrective turning speed.
     * A higher value means faster, but potentially overshooting, correction.
     */
    private static final double TURN_GAIN = 0.025;

    /**
     * Proportional gain for forward/backward correction. The larger the range error to the
     * target, the more this number amplifies the corrective driving speed.
     * A higher value means faster correction.
     */
    private static final double FORWARD_GAIN = 0.04;
    private static final double MAX_TURN_POWER = 0.4;

    /**
     * Positioning tolerance (in inches and degrees). Determines how much error is
     * acceptable when aiming. If both the range and bearing errors are within this
     * value, the robot considers itself "locked on" target.
     */
    private static final double POSITIONING_TOLERANCE = 1.5;

    // Variable to store the starting side from autonomous
    private int autoStartingSide = 0; // -1 for left, 1 for right, 0 for unknown

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // --- Hybrid Pose Loading Logic ---
        Pose startingPose;
        String loadSource;
        PoseStorage.StoredPose storedPoseFromFile = PoseStorage.loadPoseFromFile();
        boolean isFilePoseDefault = storedPoseFromFile.pose.getX() == 0 && storedPoseFromFile.pose.getY() == 0 && storedPoseFromFile.pose.getHeading() == 0;
        if (isFilePoseDefault && OpModeData.lastPose != null) {
            startingPose = OpModeData.lastPose;
            autoStartingSide = OpModeData.initialSide;
            loadSource = "Static Backup";
        } else {
            startingPose = storedPoseFromFile.pose;
            autoStartingSide = storedPoseFromFile.initialSide;
            loadSource = "File";
        }

        follower.setStartingPose(startingPose);
        follower.update();

        robot = new HardwareBox();
        robot.init(hardwareMap);
        initAprilTag();

        telemetry.addData("Status", "Initialized (18338)");
        telemetry.addData("Pose Load Source", loadSource);
        telemetry.addData("Loaded Pose", "X: %.2f, Y: %.2f, H: %.2f", startingPose.getX(), startingPose.getY(), startingPose.getHeading());
        telemetry.addData("Loaded Auto Side", (autoStartingSide < 0 ? "Left" : (autoStartingSide > 0 ? "Right" : "Unknown")));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // --- Mode Switching --- //
        handleModeSwitches();

        // --- Main Control Logic --- //
        if (autoAiming) {
            handleAutoAim();
        } else if (automatedDrive) {
            // Path following from dpad_left is active
            if (!follower.isBusy() || (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)) {
                follower.startTeleopDrive(); // Cancel path
                automatedDrive = false;
            }
        } else {
            // Manual driver control
            handleManualDrive();
        }

        // --- Other Robot Mechanisms --- //
        handleMechanisms();
        handleAprilTagPowerScaling();

        // --- Telemetry --- //
        updateTelemetry();
    }

    private void handleModeSwitches() {
        // Toggle Auto-Aim mode
        if (currentGamepad1.start && !previousGamepad1.start) {
            autoAiming = !autoAiming;
            if (!autoAiming) {
                follower.startTeleopDrive(); // Cancel any auto-aim movement
                robot.servoInR.setPower(0.0);
                robot.servoInL.setPower(0.0);
            }
        }

        // Start automated path to a fixed point
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && !autoAiming && !automatedDrive) {
            Pose currentPose = follower.getPose();
            Pose parkingPose = (autoStartingSide < 0) ? new Pose(105, 34, Math.toRadians(90)) : new Pose(39, 34, Math.toRadians(90));
            telemetry.addData("Auto-Drive Target", parkingPose.toString());
            follower.followPath(new Path(new BezierLine(currentPose, parkingPose)));
            automatedDrive = true;
        }

        // Toggle slow mode
        if (currentGamepad1.a && !previousGamepad1.a) {
            slowMode = !slowMode;
        }
    }

    private void handleAutoAim() {
        int targetTagId = (autoStartingSide < 0) ? 20 : 24; // Blue or Red goal tag
        AprilTagDetection targetTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetTagId) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag != null) {
            double errorRange = targetTag.ftcPose.range - GOAL_TAG_DISTANCE;
            double errorBearing = targetTag.ftcPose.bearing;

            if (Math.abs(errorRange) < POSITIONING_TOLERANCE && Math.abs(errorBearing) < POSITIONING_TOLERANCE) {
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetry.addData("Auto-Aim", "Locked On! Ready to fire.");
                // Servos are only active when locked on
                robot.servoInR.setPower(currentGamepad1.right_bumper ? 1.0 : 0.0);
                robot.servoInL.setPower(currentGamepad1.left_bumper ? 1.0 : 0.0);
            } else {
                double forwardPower = -FORWARD_GAIN * errorRange;
                double turnPower = -TURN_GAIN * errorBearing;
                follower.setTeleOpDrive(forwardPower, 0, turnPower, true);
                telemetry.addData("Auto-Aim", "Engaged, Adjusting to Tag %d", targetTagId);
                // Servos are inactive while adjusting
                robot.servoInR.setPower(0.0);
                robot.servoInL.setPower(0.0);
            }
        } else {
            int turnDirection = (autoStartingSide < 0) ? -1 : 1;
            follower.setTeleOpDrive(0, 0, MAX_TURN_POWER * turnDirection, true);
            telemetry.addData("Auto-Aim", "Engaged, Searching for Tag %d", targetTagId);
            // Servos are inactive while searching
            robot.servoInR.setPower(0.0);
            robot.servoInL.setPower(0.0);
        }
    }

    private void handleManualDrive() {
        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                true // Robot Centric
        );
    }

    private void handleMechanisms() {
        // Servo controls are now in handleAutoAim()
        if (currentGamepad1.b && !previousGamepad1.b) intake = !intake;
        if (currentGamepad1.x && !previousGamepad1.x) outtake = !outtake;
        if (currentGamepad1.y && !previousGamepad1.y) reverse = !reverse;
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) sep = !sep;

        robot.intake.setPower(intake ? 0.9 : 0.0);
        robot.separator.setPosition(sep ? 0 : 1);

        if (reverse) {
            robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            // Assuming servo reversal is needed. If not, this can be removed.
            robot.servoInL.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInR.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInR.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void handleAprilTagPowerScaling() {
        if (autoAiming) {
            // When auto-aiming, the outtake power is controlled manually by the 'x' button.
            // This allows the driver to decide when to shoot after the robot is in position.
            robot.outtake.setPower(outtake ? 0.7 : 0.0);
            return;
        }

        AprilTagDetection visibleTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                visibleTag = detection;
                break;
            }
        }

        if (visibleTag != null) {
            double range = visibleTag.ftcPose.range;
            double minRange = 30.0, maxRange = 100.0, minPower = 0.6, maxPower = 1.0;
            double power = minPower + (range - minRange) * (maxPower - minPower) / (maxRange - minRange);
            power = Math.max(minPower, Math.min(power, maxPower)); // Clamp power
            robot.outtake.setPower(power);
            telemetry.addData("Auto Power", "%.2f at %.1f in", power, range);
        } else {
            robot.outtake.setPower(outtake ? 0.7 : 0.0);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Mode", autoAiming ? "Auto-Aim" : (automatedDrive ? "Auto-Path" : "Manual"));
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
