package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareBox;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.ShooterManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Main TeleOp program, now using a modular ShooterManager and central RobotConstants.
 */
@Configurable
@TeleOp(name = "TeleOp StarTech - 2026", group="00-TeleOp")
@Disabled
public class TeleOpStarTech extends OpMode {

    // --- Core Robot Components ---
    HardwareBox robot;
    private Follower follower;
    private ShooterManager shooterManager;

    // --- Control Flags ---
    private boolean slowMode = false;
    private double SLOW_DOWN_FACTOR = 1.0;
    private boolean intake = false;
    private boolean outtake = false;
    private boolean reverse = false;
    private boolean sep = false;

    // --- Gamepad State ---
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    // --- Vision Components ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double lastKnownDistance = 0;
    private double xPos = 0;
    private double yPos = 0;
    private double yawPos = 0;
    private double braringPos = 0;

    // --- Automation Variables ---
    private int autoStartingSide = 0; // -1 for right (Red), 1 for left (Blue), 0 for unknown

    @Override
    public void init() {
        // --- Follower & Vision Init ---
        follower = Constants.createFollower(hardwareMap);
        initVision();

        // --- Hybrid Pose Loading ---
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

        // --- Hardware & Shooter Manager Init ---
        robot = new HardwareBox();
        robot.init(hardwareMap);
        shooterManager = new ShooterManager(robot.outtakeLeft, robot.outtakeRight, hardwareMap.voltageSensor.iterator().next());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter Mode", ShooterManager.USE_ENCODER_FOR_SHOOTER ? "VELOCITY" : "POWER");
        telemetry.addData("Pose Load Source", loadSource);
        telemetry.addData("Loaded Pose", "X: %.2f, Y: %.2f, H: %.2f", startingPose.getX(), startingPose.getY(), startingPose.getHeading());
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

        handleManualDrive();
        handleGamepadControls();
        handleShooter();
    }
    
    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(886.3435, 886.3871, 327.3383, 250.4806) // Based on your camera calibration
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void handleManualDrive() {
        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                true
        );
    }

    private void handleGamepadControls() {
        if (currentGamepad1.start && !previousGamepad1.start) {
            Pose targetPose = (autoStartingSide > 0) ? 
                                new Pose(105, 34, Math.toRadians(90)) : // Blue side target
                                new Pose(39, 34, Math.toRadians(90));   // Red side target

            PathChain pathToTarget = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), targetPose)).build();
            follower.followPath(pathToTarget);
        }

        // --- Auto-Aim Feature ---
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            int goalTagId = (autoStartingSide > 0) ? RobotConstants.BLUE_GOAL_TAG_ID : RobotConstants.RED_GOAL_TAG_ID;
            AprilTagDetection goalTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == goalTagId) {
                    goalTag = detection;
                    break;
                }
            }

            Pose currentPose = follower.getPose();
            if (goalTag != null && currentPose != null) {
                double angleToTurnRad = Math.toRadians(goalTag.ftcPose.bearing);
                double desiredHeading = currentPose.getHeading() + angleToTurnRad;

                Pose targetTurnPose = new Pose(currentPose.getX(), currentPose.getY(), desiredHeading);
                PathChain turnPath = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, targetTurnPose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), desiredHeading)
                        .build();
                follower.followPath(turnPath);
            }
        }

        if (currentGamepad1.a && !previousGamepad1.a) slowMode = !slowMode;
        if (currentGamepad1.b && !previousGamepad1.b) intake = !intake;
        if (currentGamepad1.x && !previousGamepad1.x) outtake = !outtake;
        if (currentGamepad1.y && !previousGamepad1.y) reverse = !reverse;
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) sep = !sep;

        robot.servoInR.setPower(currentGamepad1.right_bumper ? 1.0 : 0.0);
        robot.servoInL.setPower(currentGamepad1.left_bumper ? 1.0 : 0.0);
        robot.intake.setPower(intake ? 0.9 : 0.0);
        robot.separator.setPosition(sep ? 0.8 : 0.3);

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
    int goalTagId = 0;
    private void handleShooter() {
        if (!outtake) {
            shooterManager.stop();
            return;
        }

        goalTagId = (autoStartingSide > 0) ? RobotConstants.BLUE_GOAL_TAG_ID : RobotConstants.RED_GOAL_TAG_ID;
        AprilTagDetection goalTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == goalTagId) {
                goalTag = detection;
                break; 
            }
        }

        if (goalTag != null) {
            lastKnownDistance = goalTag.ftcPose.range;
            xPos = goalTag.ftcPose.x;
            yPos = goalTag.ftcPose.y;
            yawPos = goalTag.ftcPose.yaw;
            braringPos = goalTag.ftcPose.bearing;
        }
        
        // Set shooter speed based on the last known distance, or 0 if never seen.
        shooterManager.setSpeedFromDistance(lastKnownDistance);

        // --- Telemetry ---
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Shooter", shooterManager.getTelemetryData());
        telemetry.addData("Tag Distance", "%.2f in", lastKnownDistance);
        telemetry.addData("x Distance", "%.2f in", xPos);
        telemetry.addData("y Distance", "%.2f in", yPos);
        telemetry.addData("yaw Distance", "%.2f in", yawPos);
        telemetry.addData("bearing", "%.2f in", braringPos);
        telemetry.addData("goalTagId", goalTagId);
        telemetry.update();
    }
}
