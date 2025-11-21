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

/**
 * This is the main TeleOp program for the 2026 season.
 * It includes several advanced features:
 * - Hybrid pose loading from Autonomous to maintain field-centric driving.
 * - Context-aware automated navigation to predefined points.
 * - Automatic outtake power scaling based on distance to the target.
 */
@Configurable
@TeleOp(name = "TeleOp StarTech - 2026", group="00-TeleOp")

public class TeleOpStarTech extends OpMode {

    HardwareBox robot;
    private Follower follower;
    private boolean automatedDrive = false;
    private boolean slowMode = false;
    private double SLOW_DOWN_FACTOR = 1.0;

    private boolean intake = false;
    private boolean outtake = false;

    private boolean reverse = false;
    private boolean sep = false;

    // Vision components
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Variable to store the starting side from autonomous
    private int autoStartingSide = 0; // -1 for left, 1 for right, 0 for unknown

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    /**
     * Initializes the robot, follower, and vision systems.
     * Implements the hybrid pose loading logic to ensure a seamless transition from Autonomous.
     */
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
        initVision();

        telemetry.addData("Status", "Initialized (18338)");
        telemetry.addData("Pose Load Source", loadSource);
        telemetry.addData("Loaded Pose", "X: %.2f, Y: %.2f, H: %.2f", startingPose.getX(), startingPose.getY(), startingPose.getHeading());
        telemetry.addData("Loaded Auto Side", (autoStartingSide < 0 ? "Left" : (autoStartingSide > 0 ? "Right" : "Unknown")));
        telemetry.update();
    }

    /**
     * Starts the teleop driving mode for the follower.
     */
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * Main loop of the OpMode. Handles driver input and automated behaviors.
     */
    @Override
    public void loop() {
        follower.update();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // The robot is either in manual drive, or automated path following.
        if (!automatedDrive) {
            handleManualDrive();
        }

        // Handle all gamepad inputs for mechanisms and mode changes.
        handleGamepadControls();
        
        // In manual mode, scale outtake power based on vision.
        handleAprilTagPowerScaling();

        // Check if the automated path should be cancelled.
        if (automatedDrive && (!follower.isBusy() || (currentGamepad1.dpad_down && !previousGamepad1.dpad_down))) {
            follower.startTeleopDrive(); // Re-enables manual control
            automatedDrive = false;
        }

        updateTelemetry();
    }

    /**
     * Initializes the vision portal and AprilTag processor.
     */
    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Handles manual, driver-controlled robot movement.
     */
    private void handleManualDrive() {
        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                true // Robot Centric
        );
    }

    /**
     * Handles all gamepad button presses for mechanisms and mode changes.
     */
    private void handleGamepadControls() {
        if (currentGamepad1.a && !previousGamepad1.a) {
            slowMode = !slowMode;
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            intake = !intake;
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            outtake = !outtake;
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            reverse = !reverse;
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            sep = !sep;
        }

        if (currentGamepad1.right_bumper) {
            robot.servoInR.setPower(1.0);
        } else {
            robot.servoInR.setPower(0.0);
        }

        if (currentGamepad1.left_bumper) {
            robot.servoInL.setPower(1.0);
        } else {
            robot.servoInL.setPower(0.0);
        }

        // Context-aware automated parking
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && !automatedDrive) {
            Pose currentPose = follower.getPose();
            Pose parkingPose = (autoStartingSide < 0) ? new Pose(105, 34, Math.toRadians(90)) : new Pose(39, 34, Math.toRadians(90));
            telemetry.addData("Auto-Drive Target", parkingPose.toString());
            follower.followPath(new Path(new BezierLine(currentPose, parkingPose)));
            automatedDrive = true;
        }

        robot.intake.setPower(intake ? 0.9 : 0.0);
        robot.separator.setPosition(sep ? 0 : 1);

        if (reverse) {
            robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInL.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInR.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInR.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Automatically adjusts outtake power based on distance to the goal tag.
     */
    private void handleAprilTagPowerScaling() {
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
            power = Math.max(minPower, Math.min(power, maxPower));
            robot.outtake.setPower(power);
            telemetry.addData("Auto Power", "%.2f at %.1f in", power, range);
        } else {
            robot.outtake.setPower(outtake ? 0.7 : 0.0);
        }
    }

    /**
     * Updates the driver station telemetry with key information.
     */
    private void updateTelemetry() {
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Mode", automatedDrive ? "Auto-Path" : "Manual");
        telemetry.update();
    }
}
