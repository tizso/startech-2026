package org.firstinspires.ftc.teamcode;

// Autonomous NexGen – Now using a modular ShooterManager and central RobotConstants

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

@Autonomous(name = "Autonomous StarTech V2", group = "Opmode")
public class AutonomousStarTechNewV2 extends LinearOpMode {

    // --- Core Robot Components ---
    HardwareBox robot = new HardwareBox();
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ShooterManager shooterManager;

    // --- State machine ---
    private enum State { 
        SETUP, 
        MOVE_TO_SHOOT, WAIT_FOR_MOVE, 
        FINE_AIM, WAIT_FOR_FINE_AIM, 
        CHECK_SPEED, FIRE, NEXT_SHOT, 
        MOVE_TO_BALL, WAIT_FOR_BALL_MOVE, TILT_CAMERA, AIM_AT_BALL, WAIT_FOR_BALL_AIM, // New states for ball alignment
        PARK, WAIT_FOR_PARK, 
        END 
    }
    private State currentState = State.SETUP;
    private final ElapsedTime timer = new ElapsedTime();

    // --- Autonomous Strategy ---
    private boolean stayAndTurnMode = false;
    private double computedDistanceInch = 0.0;
    private int initialSide = 1;
    private Pose startPose = null;
    private Pose shootPose = null;
    private int foundID = -1;
    private int backdropId = -1;
    private char[] sequence = {'P','P','P'};
    private int shotIndex = 0;
    private PathChain pathToShoot = null;
    private PathChain pathToPark  = null;
    private Pose targetBallPose = null;

    // --- Constants for Ball Alignment ---
    private static final double CAMERA_FORWARD_OFFSET = 9.0; // User specified 9in
    private static final double CAMERA_MOUNT_HEIGHT_INCHES = 12.0; // Camera height from the ground (TUNE THIS)
    private static final double BALL_DIAMETER_INCHES = 5.0;
    private static final double ROBOT_ALIGN_TO_BALL_DIST_IN = 9.0; // How far from the ball the robot should stop

    // Ball positions (own half)
    private static final Pose[] BALLS_BLUE3 = new Pose[] {
            new Pose(19.0, 84.0, 0.0),
            new Pose(24.0, 84.0, 0.0),
            new Pose(29.0, 84.0, 0.0)
    };
    private static final Pose[] BALLS_BLUE2 = new Pose[] {
            new Pose(19.0, 60.0, 0.0),
            new Pose(24.0, 60.0, 0.0),
            new Pose(29.0, 60.0, 0.0)
    };
    private static final Pose[] BALLS_BLUE1 = new Pose[] {
            new Pose(19.0, 36.0, 0.0),
            new Pose(24.0, 36.0, 0.0),
            new Pose(29.0, 36.0, 0.0)
    };
    private static final Pose[] BALLS_RED3  = new Pose[] {
            new Pose(115.0, 84.0, 0.0),
            new Pose(120.0, 84.0, 0.0),
            new Pose(125.0, 84.0, 0.0)
    };
    private static final Pose[] BALLS_RED2  = new Pose[] {
            new Pose(115.0, 60.0, 0.0),
            new Pose(120.0, 60.0, 0.0),
            new Pose(125.0, 60.0, 0.0)
    };
    private static final Pose[] BALLS_RED1  = new Pose[] {
            new Pose(115.0, 36.0, 0.0),
            new Pose(120.0, 36.0, 0.0),
            new Pose(125.0, 36.0, 0.0)
    };

    private static Pose[] SELECTED_SET = new Pose[]{};

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialization ---
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        shooterManager = new ShooterManager(robot.outtakeLeft, robot.outtakeRight, hardwareMap.voltageSensor.iterator().next());

        initVision();

        // --- INIT Loop ---
        while (opModeInInit()) {
            handleSetup();
            if (startPose != null) {
                follower.setStartingPose(startPose);
            }
            if (startPose != null && shootPose != null) {
                double dx = shootPose.getX() - startPose.getX();
                double dy = shootPose.getY() - startPose.getY();
                computedDistanceInch = Math.sqrt(dx*dx + dy*dy);
                if (pathToShoot == null) {
                    pathToShoot = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading())
                            .build();
                }
            }
            displayInitTelemetry();
        }

        // --- START ---
        waitForStart();
        shooterManager.setSpeedFromDistance(computedDistanceInch);

        if (stayAndTurnMode) {
            currentState = State.FINE_AIM;
        } else if (pathToShoot != null) {
            follower.followPath(pathToShoot);
            currentState = State.WAIT_FOR_MOVE;
        } else {
            currentState = State.CHECK_SPEED;
        }

        timer.reset();

        // --- Main Loop ---
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            shooterManager.updateBoostState(computedDistanceInch);
            runStateMachine();
            displayRuntimeTelemetry();
        }

        if (visionPortal != null) visionPortal.close();
    }

    private void runStateMachine() {
        switch (currentState) {
            case WAIT_FOR_MOVE:
                if (!follower.isBusy()) {
                    currentState = State.FINE_AIM;
                    timer.reset();
                }
                break;
            case FINE_AIM:
                startFineAim();
                currentState = State.WAIT_FOR_FINE_AIM;
                timer.reset();
                break;
            case WAIT_FOR_FINE_AIM:
                if (!follower.isBusy() || timer.seconds() > RobotConstants.FINE_AIM_TIMEOUT_SEC) {
                    currentState = State.CHECK_SPEED;
                    timer.reset();
                }
                break;
            case CHECK_SPEED:
                if (shooterManager.isReady() || timer.seconds() > RobotConstants.SHOOTER_READY_TIMEOUT_SEC) {
                    shooterManager.applyBoost();
                    currentState = State.FIRE;
                    timer.reset();
                }
                break;
            case FIRE:
                fireBall(sequence[shotIndex]);
                currentState = State.NEXT_SHOT;
                timer.reset();
                break;
            case NEXT_SHOT:
                if (timer.seconds() > (RobotConstants.SHOOTING_SERVO_RUN_TIME_SEC + RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC)) {
                    shotIndex++;
                    if (shotIndex < sequence.length) {
                        currentState = State.CHECK_SPEED;
                    } else {
                        // All shots fired, now move to the ball
                        goToBall();
                        currentState = State.WAIT_FOR_BALL_MOVE;
                        timer.reset();
                    }
                }
                break;

            // --- New States for Ball Alignment ---
            case WAIT_FOR_BALL_MOVE:
                if (!follower.isBusy()) {
                    currentState = State.TILT_CAMERA;
                    timer.reset();
                }
                break;
            
            case TILT_CAMERA:
                calculateAndSetCameraTilt();
                sleep(500); // Wait for servo to move
                currentState = State.AIM_AT_BALL;
                timer.reset();
                break;

            case AIM_AT_BALL:
                aimAtBall();
                currentState = State.WAIT_FOR_BALL_AIM;
                timer.reset();
                break;

            case WAIT_FOR_BALL_AIM:
                if (!follower.isBusy()) {
                    currentState = State.END; // End of autonomous after aligning to ball
                }
                break;

            case PARK:
                 // This state is now effectively bypassed, but left for potential future use.
                if (!follower.isBusy() || timer.seconds() > 3.0) {
                    currentState = State.END;
                }
                break;
            case END:
                shooterManager.stop();
                Pose finalPose = follower.getPose();
                if (finalPose != null) {
                    PoseStorage.savePoseToFile(finalPose, initialSide);
                    OpModeData.lastPose = finalPose;
                    OpModeData.initialSide = initialSide;
                }
                requestOpModeStop();
                break;
        }
    }

    private void initVision() {
         try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(886.3435, 886.3871, 327.3383, 250.4806) // Based on your camera calibration
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            aprilTag = null; visionPortal = null;
        }
    }

    private void handleSetup() {
        List<AprilTagDetection> dets = (aprilTag != null) ? aprilTag.getDetections() : Collections.emptyList();
        AprilTagDetection obelisk = null;
        AprilTagDetection backdrop = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection d : dets) {
            if (d == null || d.metadata == null || d.ftcPose == null) continue;
            if (d.id >= 21 && d.id <= 23) obelisk = d;
            if (d.id == RobotConstants.BLUE_GOAL_TAG_ID || d.id == RobotConstants.RED_GOAL_TAG_ID) {
                if (d.ftcPose.range < bestRange) {
                    bestRange = d.ftcPose.range;
                    backdrop = d;
                }
            }
        }

        if (obelisk == null || backdrop == null) return;

        initialSide = (obelisk.ftcPose.x < 0) ? -1 : 1;
        foundID = obelisk.id;
        backdropId = backdrop.id;
        sequence = getSequenceForID(foundID);
        stayAndTurnMode = Math.abs(obelisk.ftcPose.yaw) < RobotConstants.OBELISK_YAW_THRESHOLD_DEG;

        Pose landmarkPose = (backdrop.id == RobotConstants.BLUE_GOAL_TAG_ID) ? RobotConstants.BLUE_BACKDROP_POSE : RobotConstants.RED_BACKDROP_POSE;
        double robotHeadingRad = landmarkPose.getHeading() - Math.toRadians(backdrop.ftcPose.yaw);
        double angleToTag_world = robotHeadingRad + Math.toRadians(backdrop.ftcPose.bearing);
        double cameraX = landmarkPose.getX() - backdrop.ftcPose.range * Math.cos(angleToTag_world);
        double cameraY = landmarkPose.getY() - backdrop.ftcPose.range * Math.sin(angleToTag_world);
        double robotX = cameraX - CAMERA_FORWARD_OFFSET * Math.cos(robotHeadingRad);
        double robotY = cameraY - CAMERA_FORWARD_OFFSET * Math.sin(robotHeadingRad);
        startPose = new Pose(robotX, robotY, robotHeadingRad);

        Pose correctLandmark = (initialSide > 0) ? RobotConstants.BLUE_BACKDROP_POSE : RobotConstants.RED_BACKDROP_POSE;
        Pose baseLandmark = (Math.abs(backdrop.ftcPose.yaw) > RobotConstants.YAW_THRESHOLD_DEG) ? correctLandmark : landmarkPose;
        shootPose = new Pose(baseLandmark.getX() - initialSide * RobotConstants.X_OFFSET_IN, baseLandmark.getY() - RobotConstants.Y_OFFSET_IN, baseLandmark.getHeading());
    }

    private void startFineAim() {
        Pose current = follower.getPose();
        if (current == null) return;

        double desiredHeading;
        AprilTagDetection backdrop = getClosestBackdrop();
        if (backdrop != null && backdrop.ftcPose != null) {
            desiredHeading = current.getHeading() + Math.toRadians(backdrop.ftcPose.bearing);
        } else {
            desiredHeading = (initialSide > 0) ? RobotConstants.BLUE_BACKDROP_POSE.getHeading() : RobotConstants.RED_BACKDROP_POSE.getHeading(); // Fallback
        }

        if (RobotConstants.UPDATE_SPEED_ON_FINE_AIM) {
            Pose landmark = (initialSide > 0) ? RobotConstants.BLUE_BACKDROP_POSE : RobotConstants.RED_BACKDROP_POSE;
            double targetX = landmark.getX() - initialSide * RobotConstants.X_OFFSET_IN;
            double targetY = landmark.getY() - RobotConstants.Y_OFFSET_IN;
            computedDistanceInch = Math.hypot(targetX - current.getX(), targetY - current.getY());
            shooterManager.setSpeedFromDistance(computedDistanceInch);
        }
        double esp = 0.1;

        Pose target = new Pose(
                current.getX() + esp * Math.cos(desiredHeading),
                current.getY() + esp * Math.sin(desiredHeading),
                desiredHeading);
        PathChain fine = follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), desiredHeading)
                .build();
        follower.followPath(fine);
    }
    
    private void fireBall(char type) {
        if (type == 'G') {
            robot.servoInL.setPower(1.0);
        } else {
            robot.servoInR.setPower(1.0);
        }
        sleep((long)(RobotConstants.SHOOTING_SERVO_RUN_TIME_SEC * 1000));
        robot.servoInL.setPower(0.0);
        robot.servoInR.setPower(0.0);
        sleep((long)(RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC * 1000));
    }

    /**
     * Finds the closest ball, calculates a path to it, and starts the follower.
     */
    private void goToBall() {
        Pose currentPose = follower.getPose();
        targetBallPose = findClosestBall(currentPose);

        if (targetBallPose == null) {
            currentState = State.END; // No ball found, end the routine
            return;
        }

        // Calculate a stopping point in front of the ball, facing it
        double angleToBall = Math.atan2(targetBallPose.getY() - currentPose.getY(), targetBallPose.getX() - currentPose.getX());
        double stopPointX = targetBallPose.getX() - ROBOT_ALIGN_TO_BALL_DIST_IN * Math.cos(angleToBall);
        double stopPointY = targetBallPose.getY() - ROBOT_ALIGN_TO_BALL_DIST_IN * Math.sin(angleToBall);

        Pose goToBallPose = new Pose(stopPointX, stopPointY, angleToBall);

        PathChain pathToBall = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, goToBallPose))
                .build();
        follower.followPath(pathToBall);
    }

    /**
     * Dynamically calculates the required camera tilt to see the target ball and sets the servo.
     */
    private void calculateAndSetCameraTilt() {
        if (targetBallPose == null) return;
        Pose currentPose = follower.getPose();
        if (currentPose == null) return;

        double distanceToBall = Math.hypot(currentPose.getX() - targetBallPose.getX(), currentPose.getY() - targetBallPose.getY());
        double heightDifference = CAMERA_MOUNT_HEIGHT_INCHES - (BALL_DIAMETER_INCHES / 2.0);

        // Calculate the required angle using arctangent
        double requiredAngleRad = Math.atan2(heightDifference, distanceToBall);
        double requiredAngleDeg = Math.toDegrees(requiredAngleRad);

        // Convert angle to servo position (0-1). Assumes 0 deg = 0.0 and 90 deg = 1.0
        // This mapping may need to be tuned based on your specific servo setup.
        double servoPosition = requiredAngleDeg / 90.0;
        servoPosition = Math.max(0.0, Math.min(servoPosition, 1.0)); // Clamp between 0 and 1

        // IMPORTANT: Make sure "cameraTilt" servo is configured in HardwareBox.java
        robot.cameraTilt.setPosition(servoPosition);
        telemetry.addData("Camera Tilt", "Calculated: %.2f for %.1f deg", servoPosition, requiredAngleDeg);
    }

    /**
     * Performs a final turn-in-place to aim perfectly at the target ball.
     */
    private void aimAtBall() {
        if (targetBallPose == null) return;
        Pose currentPose = follower.getPose();
        if (currentPose == null) return;

        double dx = targetBallPose.getX() - currentPose.getX();
        double dy = targetBallPose.getY() - currentPose.getY();
        double desiredHeading = Math.atan2(dy, dx);

        // Use a zero-distance path to execute a turn-in-place
        Pose targetHeadingPose = new Pose(currentPose.getX(), currentPose.getY(), desiredHeading);

        PathChain aimPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetHeadingPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), desiredHeading)
                .build();
        follower.followPath(aimPath);
    }

    /**
     * Finds the closest ball from the relevant group based on the alliance side.
     * @param currentPose The robot's current position on the field.
     * @return The Pose of the closest ball, or null if none are available.
     */
    private Pose findClosestBall(Pose currentPose) {
        if(currentPose.getY() > 72){
            SELECTED_SET = (initialSide > 0) ? BALLS_BLUE3 : BALLS_RED3;
        } else {
            SELECTED_SET = (initialSide > 0) ? BALLS_BLUE1 : BALLS_RED1;
        }
        Pose[] ballGroup = SELECTED_SET;
        Pose closestBall = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose ball : ballGroup) {
            double distance = Math.hypot(currentPose.getX() - ball.getX(), currentPose.getY() - ball.getY());
            if (distance < minDistance) {
                minDistance = distance;
                closestBall = ball;
            }
        }
        return closestBall;
    }

    private AprilTagDetection getClosestBackdrop() {
        if (aprilTag == null) return null;
        List<AprilTagDetection> dets = aprilTag.getDetections();
        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection d : dets) {
            if (d != null && d.metadata != null && (d.id == RobotConstants.BLUE_GOAL_TAG_ID || d.id == RobotConstants.RED_GOAL_TAG_ID)) {
                if (d.ftcPose != null && d.ftcPose.range < bestRange) {
                    bestRange = d.ftcPose.range;
                    best = d;
                }
            }
        }
        return best;
    }

    private char[] getSequenceForID(int id) {
        switch (id) {
            case 21: return new char[]{'G','P','P'};
            case 22: return new char[]{'P','G','P'};
            default: return new char[]{'P','P','G'};
        }
    }

    private void displayInitTelemetry() {
        telemetry.addLine("=== INIT PRECOMPUTE ===");
        telemetry.addData("Shooter Mode", ShooterManager.USE_ENCODER_FOR_SHOOTER ? "VELOCITY" : "POWER");
        telemetry.addData("Stay and Turn Mode", stayAndTurnMode ? "ACTIVE" : "Inactive");
        telemetry.addData("ObeliskID", foundID);
        telemetry.addData("BackdropId", backdropId);
        telemetry.addData("StartPose", startPose);
        telemetry.addData("ShootPose", shootPose);
        telemetry.addData("Distance (in)", computedDistanceInch);
        telemetry.update();
    }

    private void displayRuntimeTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Shooter", shooterManager.getTelemetryData());
        telemetry.addData("Target Ball", targetBallPose);
        telemetry.update();
    }
}
