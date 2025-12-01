package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Autonomous NextGen", group = "Opmode")
public class AutonomousNextGen extends LinearOpMode {

    // Core components
    HardwareBox robot = new HardwareBox();
    private Follower follower;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection detectedTag = null;

    private enum State {
        SETUP,
        START_MOVE,
        WAIT_FOR_MOVE,
        FINAL_APPROACH,             // New state for precision docking
        WAIT_FOR_FINAL_APPROACH,    // New state to wait for docking to finish
        CHECK_SPEED_FOR_SHOT_1,
        SHOOT_1,
        CHECK_SPEED_FOR_SHOT_2,
        SHOOT_2,
        WAIT_FOR_INTAKE,
        CHECK_SPEED_FOR_SHOT_3,
        SHOOT_3,
        FINAL_WAIT,
        PARK,
        WAIT_FOR_PARK,
        END
    }
    private State currentState = State.SETUP;

    // --- Constants ---
    private static final Pose BLUE_BACKDROP_POSE = new Pose(16.01, 133.28, Math.toRadians(-126.0));
    private static final Pose RED_BACKDROP_POSE = new Pose(127.99, 133.28, Math.toRadians(-54.0));
    private static final double SHOOTING_X_OFFSET = 25.0; 
    private static final double SHOOTING_Y_OFFSET = 50.0; 
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double AUTON_START_DELAY_SEC = 5.0;
    private static final double AIM_TIMEOUT_SEC = 3.0;
    private static final double SHOOTER_SPEED_TIMEOUT_SEC = 2.0;

    // --- Robot & Field Dimensions ---
    private static final double ROBOT_WIDTH = 18.0;
    private static final double ROBOT_LENGTH = 18.0;
    private static final double CAMERA_FORWARD_OFFSET = (ROBOT_LENGTH / 2.0) - 2.0; // Camera is 2 inches back from the front
    private static final double BLUE_SIDE_MAX_X = 70.0; // Furthest X for blue side
    private static final double RED_SIDE_MIN_X = 74.0;  // Furthest X for red side

    // --- Shooter Constants ---
    private static final double SHOOTER_RPM = 3600;
    private static final double MAX_RPM = 6000;
    private static final double TICKS_PER_REV = 28;

    double maxTicksPerSec = TICKS_PER_REV * (MAX_RPM / 60.0);

    private static final double SHOOTER_VELOCITY_TICKS_PER_SEC = (SHOOTER_RPM / 60.0) * TICKS_PER_REV;

    private static final double BASKET_HEIGHT_M = 1.0;
    private static final double SHOOT_ANGLE_DEG = 45;
    private static final double G = 9.81;
    private static final double WHEEL_DIAMETER_M = 0.096;
    private static final double SLIP_FACTOR = 1.3;

    // --- Logic Flow Variables ---
    private int initialSide = 0;
    private int foundID = -1;
    private Pose finalShootingPose = null; // To store the calculated precise shooting pose

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.servoInR.setDirection(DcMotorSimple.Direction.FORWARD);
        follower = Constants.createFollower(hardwareMap);

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double f = 12.0/maxTicksPerSec;
        double i = 0.0005;
        double d = 0.0001;
        double p = 0.002 * (SHOOTER_RPM / 1000) * 0.5;

        initVision();

        while (opModeInInit()) {
            handleSetup();
            telemetry.update();
        }

        waitForStart();

        if (currentState == State.SETUP) {
            telemetry.addData("ERROR", "Could not set starting pose. OpMode will not run.");
            telemetry.update();
            sleep(5000);
            return;
        }
        sleep((long) (AUTON_START_DELAY_SEC * 1000));
        while (opModeIsActive() && !isStopRequested()) {

            follower.update();
            updateDetectedTagForState();
            switch (currentState) {
                case START_MOVE:
                    Pose startPose = follower.getPose();
                    double shout = initialSide > 0 ? 25 : 10;
                    Pose goalPose = (initialSide > 0) ? BLUE_BACKDROP_POSE : RED_BACKDROP_POSE;
                    double shootPoseX = goalPose.getX() - (initialSide * shout);
                    double shootPoseY = goalPose.getY() - SHOOTING_Y_OFFSET;
                    double targetHeadingRad = startPose.getHeading() + Math.toRadians(45.0 * initialSide);
                    Pose shootingPose = new Pose(shootPoseX, shootPoseY, targetHeadingRad);

                    telemetry.addData("startPose.getHeading", startPose.getHeading());
                    telemetry.addData("targetHeadingRad", targetHeadingRad);

                    robot.outtakeLeft.setVelocityPIDFCoefficients(p, i, d, f);
                    robot.outtakeRight.setVelocityPIDFCoefficients(p, i, d, f);

                    robot.outtakeLeft.setVelocity(SHOOTER_VELOCITY_TICKS_PER_SEC);
                    robot.outtakeRight.setVelocity(SHOOTER_VELOCITY_TICKS_PER_SEC);

                    PathChain pathToShoot = follower.pathBuilder()
                            .addPath(new BezierLine(startPose, shootingPose))
                            .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                            .build();

                    follower.followPath(pathToShoot);
                    currentState = State.WAIT_FOR_MOVE;
                    break;

                case WAIT_FOR_MOVE:
                    if (!follower.isBusy()) {
                        follower.startTeleopDrive();
                        stateTimer.reset();
                        currentState = State.FINAL_APPROACH;
                    }
                    break;

                case FINAL_APPROACH:
                    handleFinalApproach();
                    break;

                case WAIT_FOR_FINAL_APPROACH:
                    if (!follower.isBusy()) {
                        follower.startTeleopDrive();
                        stateTimer.reset();
                        currentState = State.CHECK_SPEED_FOR_SHOT_1;
                    }
                    break;

                case CHECK_SPEED_FOR_SHOT_1:
                    handleCheckShooterSpeed(State.SHOOT_1);
                    break;

                case SHOOT_1:
                    if (foundID == 21) shutGreenArtifact(); else shutPurpleArtifact();
                    stateTimer.reset();
                    currentState = State.CHECK_SPEED_FOR_SHOT_2;
                    break;

                case CHECK_SPEED_FOR_SHOT_2:
                    handleCheckShooterSpeed(State.SHOOT_2);
                    break;

                case SHOOT_2:
                    if (foundID == 22) shutGreenArtifact(); else shutPurpleArtifact();
                    robot.intake.setPower(0.8);
                    stateTimer.reset();
                    currentState = State.WAIT_FOR_INTAKE;
                    break;
                
                case WAIT_FOR_INTAKE:
                    if (stateTimer.seconds() > 2) {
                        robot.intake.setPower(0);
                        stateTimer.reset();
                        currentState = State.CHECK_SPEED_FOR_SHOT_3;
                    }
                    break;

                case CHECK_SPEED_FOR_SHOT_3:
                    handleCheckShooterSpeed(State.SHOOT_3);
                    break;

                case SHOOT_3:
                    if (foundID == 23) shutGreenArtifact(); else shutPurpleArtifact();
                    stateTimer.reset();
                    currentState = State.FINAL_WAIT;
                    break;

                case FINAL_WAIT:
                     if (stateTimer.seconds() > 0.5) {
                        robot.outtakeLeft.setPower(0);
                        robot.outtakeRight.setPower(0);
                        currentState = State.PARK;
                    }
                    break;

                case PARK:
                    Pose currentPose = follower.getPose();
                    double additionalTurnRad = Math.toRadians(45.0 * initialSide);
                    double finalHeadingRad = currentPose.getHeading() + additionalTurnRad;
                    double strafeDistance = 25.0 * initialSide;

                    double sideVecX = -Math.sin(currentPose.getHeading());
                    double sideVecY = Math.cos(currentPose.getHeading());
                    double dispX = strafeDistance * sideVecX;
                    double dispY = strafeDistance * sideVecY;

                    Pose targetPose = new Pose(currentPose.getX() + dispX, currentPose.getY() + dispY, finalHeadingRad);

                    PathChain parkPath = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, targetPose))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                            .build();

                    follower.followPath(parkPath);
                    currentState = State.WAIT_FOR_PARK;
                    break;

                case WAIT_FOR_PARK:
                    if (!follower.isBusy()) {
                        currentState = State.END;
                    }
                    break;

                case END:
                    handleEnd();
                    break;
            }
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }

        handleEnd(); 
        visionPortal.close();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void handleSetup() {
        telemetry.addData("Status", "SETUP: Looking for tags...");
        AprilTagDetection obeliskTag = null;
        AprilTagDetection backdropTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id >= 21 && detection.id <= 23) {
                    obeliskTag = detection;
                } else if (detection.id == BLUE_GOAL_TAG_ID || detection.id == RED_GOAL_TAG_ID) {
                    backdropTag = detection;
                }
            }
        }


        if (obeliskTag != null) {
            foundID = obeliskTag.id;
            initialSide = (obeliskTag.ftcPose.x < 0) ? -1 : 1;
            telemetry.addData("Strategy", "Obelisk found! Side: " + (initialSide == 1 ? "Blue" : "Red") + ", Sequence: " + foundID);

            Pose landmarkPose = null;
            if (backdropTag != null) {
                if (backdropTag.id == BLUE_GOAL_TAG_ID) {
                    landmarkPose = BLUE_BACKDROP_POSE;
                } else if (backdropTag.id == RED_GOAL_TAG_ID) {
                    landmarkPose = RED_BACKDROP_POSE;
                }
            }

            if (landmarkPose != null) {
                double robotHeadingRad = landmarkPose.getHeading() - Math.toRadians(backdropTag.ftcPose.yaw);

                double relX = backdropTag.ftcPose.x;
                double relY = backdropTag.ftcPose.y;

                double worldVectorX = relX * Math.cos(robotHeadingRad) - relY * Math.sin(robotHeadingRad);
                double worldVectorY = relX * Math.sin(robotHeadingRad) + relY * Math.cos(robotHeadingRad);

                double cameraX = landmarkPose.getX() - worldVectorX;
                double cameraY = landmarkPose.getY() - worldVectorY;
                
                double robotX = cameraX + CAMERA_FORWARD_OFFSET * Math.sin(robotHeadingRad);
                double robotY = cameraY - CAMERA_FORWARD_OFFSET * Math.cos(robotHeadingRad);

                Pose startingPose = new Pose(robotX, robotY, robotHeadingRad);
                follower.setStartingPose(startingPose);

                telemetry.addData("Status", "LOCKED! Pose calculated from Backdrop Tag ID: " + backdropTag.id);
                telemetry.addData("Calculated Start", "X: %.2f, Y: %.2f, H: %.1f", robotX, robotY, Math.toDegrees(robotHeadingRad));
                telemetry.addData(">>>", "Ready to Start!");
                currentState = State.START_MOVE;

            } else {
                currentState = State.SETUP;
                telemetry.addData("Status", "Localization Failed! Cannot see ANY Backdrop Tag.");
                telemetry.addData("ACTION", "<--- PAN to find a Backdrop Tag (20 or 24) --->");
            }

        } else {
            currentState = State.SETUP;
            telemetry.addData("Status", "Waiting for Obelisk Tag (21-23) for strategy...");
        }
    }

    private void handleFinalApproach() {
        telemetry.addData("Status", "FINAL_APPROACH: Looking for goal tag...");

        int goalTagId = (initialSide > 0) ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
        AprilTagDetection backdropTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == goalTagId) {
                backdropTag = detection;
                break;
            }
        }

        if (backdropTag == null) {
            telemetry.addData("Status", "FAILSAFE: Cannot see goal tag " + goalTagId + ". Shooting from current pos.");
            currentState = State.CHECK_SPEED_FOR_SHOT_1; 
            return;
        }

        Pose landmarkPose = (initialSide > 0) ? BLUE_BACKDROP_POSE : RED_BACKDROP_POSE;

        double targetX = landmarkPose.getX() - (initialSide * SHOOTING_X_OFFSET);
        double targetY = landmarkPose.getY() - SHOOTING_Y_OFFSET;

        double halfRobotWidth = ROBOT_WIDTH / 2.0;
        if (initialSide > 0) { // Blue side
            targetX = Math.max(targetX, halfRobotWidth);
            targetX = Math.min(targetX, BLUE_SIDE_MAX_X - halfRobotWidth);
        } else { // Red side
            targetX = Math.min(targetX, 144 - halfRobotWidth);
            targetX = Math.max(targetX, RED_SIDE_MIN_X + halfRobotWidth);
        }

        double deltaX = landmarkPose.getX() - targetX;
        double deltaY = landmarkPose.getY() - targetY;
        double targetHeading = Math.atan2(deltaX, deltaY);

        finalShootingPose = new Pose(targetX, targetY, targetHeading);
        telemetry.addData("Status", "Final Approach: Moving to calculated pose.");
        telemetry.addData("Final Target", finalShootingPose);

        PathChain pathToFinalPose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), finalShootingPose))
                .build();
        
        follower.followPath(pathToFinalPose);
        currentState = State.WAIT_FOR_FINAL_APPROACH;
    }

    private void handleCheckShooterSpeed(State nextState) {
        double velocityLeft = robot.outtakeLeft.getVelocity();
        double velocityRight = robot.outtakeRight.getVelocity();
        double tolerance = 0.95; // 95% of target speed

        boolean isAtSpeed = (velocityLeft >= SHOOTER_VELOCITY_TICKS_PER_SEC * tolerance) && (velocityRight >= SHOOTER_VELOCITY_TICKS_PER_SEC * tolerance);

        telemetry.addData("Shooter Status", "Waiting for speed...");
        telemetry.addData("Target Vel", "%.1f", SHOOTER_VELOCITY_TICKS_PER_SEC);
        telemetry.addData("Actual Vel L|R", "%.1f | %.1f", velocityLeft, velocityRight);

        if (isAtSpeed || stateTimer.seconds() > SHOOTER_SPEED_TIMEOUT_SEC) { 
            telemetry.addData("Shooter Status", isAtSpeed ? "At speed!" : "Timeout!");
            currentState = nextState;
        }
    }

    private void shutGreenArtifact(){
        robot.servoInL.setPower(1);
        robot.safeWaitSeconds(0.5);
        robot.servoInL.setPower(0);
    }

    private void shutPurpleArtifact(){
        robot.servoInR.setPower(1);
        robot.safeWaitSeconds(0.5);
        robot.servoInR.setPower(0);
    }
    
    private void updateDetectedTagForState() {
        detectedTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) return;

        int[] targetIds;
        if (currentState == State.FINAL_APPROACH) { // Adjusted from ADJUST_AIM
            targetIds = new int[]{(initialSide > 0) ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID};
        } else {
            targetIds = new int[]{};
        }

        double minRange = Double.MAX_VALUE;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (int id : targetIds) {
                    if (detection.id == id && detection.ftcPose.range < minRange) {
                        minRange = detection.ftcPose.range;
                        detectedTag = detection;
                    }
                }
            }
        }
    }

    private void handleEnd() {
        if (currentState != State.END) { // Prevent multiple saves
            robot.outtakeLeft.setPower(0);
            robot.outtakeRight.setPower(0);
            Pose finalPose = follower.getPose();
            if(finalShootingPose != null) { // Save the precise pose if calculated
                PoseStorage.savePoseToFile(finalShootingPose, initialSide);
                OpModeData.lastPose = finalShootingPose;
            } else {
                PoseStorage.savePoseToFile(finalPose, initialSide);
                OpModeData.lastPose = finalPose;
            }
            OpModeData.initialSide = initialSide;
            currentState = State.END;
            if (opModeIsActive()) {
                requestOpModeStop();
            }
        }
    }
}
