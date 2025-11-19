package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Rect;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "Autonomus Super", group = "Opmode")
@Configurable
@Disabled
@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class AutonomusSuper extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    HardwareBox robot = new HardwareBox();
    private Follower follower;

    private enum MainState {
        SETUP,
        SEARCH_INITIAL_TAG,
        ADJUST_TO_INITIAL_TAG,
        TURN_TO_GOAL_TAG,
        ADJUST_TO_GOAL_TAG,
        SHOOT_PRELOADED,
        START_BALL_COLLECTION,
        COLLECTING_BALLS,
        RETURN_TO_GOAL_TAG,
        ADJUST_FOR_SECOND_SHOT,
        SHOOT_COLLECTED,
        PARK,
        END
    }
    private MainState mainState = MainState.SETUP;

    private enum CollectionState {
        SEARCHING, CENTERING, MOVING_TO_BALL, INTAKING, CONFIRMING_CAPTURE
    }
    private CollectionState collectionState = CollectionState.SEARCHING;

    // --- Constants ---
    private static final double AUTON_START_DELAY_SEC = 0.0;
    private static final double AUTON_TIMEOUT_SEC = 30.0;
    private static final Pose INITIAL_TAG_POSE = new Pose(72, 144, Math.toRadians(-90));
    private static final double HEADING_DECISION_RANGE = 72.0;
    private static final double LATERAL_OFFSET_INITIAL = 24.0;
    private static final double DISTANCE_FROM_APRILTAG = 40.0;
    private static final double GOAL_TAG_DISTANCE = 35.0;
    private static final double PARK_DISTANCE = 20.0;
    private static final int BALLS_TO_COLLECT = 3;
    private static final double BALL_CAPTURED_DISTANCE_CM = 5.0;
    private static final double BALL_TARGET_HEIGHT = 100;
    private static final int CAMERA_WIDTH = 640;
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double TURN_GAIN = 0.025;
    private static final double FORWARD_GAIN = 0.04;
    private static final double STRAFE_GAIN = 0.04;
    private static final double MAX_TURN_POWER = 0.4;
    private static final double MAX_DRIVE_POWER = 0.75;
    private static final double POSITIONING_TOLERANCE = 1.5;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private BallDetectionProcessor ballProcessor;
    private AprilTagDetection detectedTag = null;

    // --- Logic Flow ---
    private int foundID = -1;
    private int initialSide = 0;
    private boolean isSpinningUp = false;
    private int ballsCollected = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        initVision();

        while (opModeInInit()) {
            if (handleSetup()) break;
        }

        telemetry.addData("Status", "Setup complete. Press START.");
        telemetry.update();
        waitForStart();

        if (mainState == MainState.SETUP) {
            telemetry.addData("ERROR", "Could not set starting pose. Shutting down.");
            telemetry.update();
            sleep(5000);
            return;
        }

        sleep((long) (AUTON_START_DELAY_SEC * 1000));
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            if (runtime.seconds() > AUTON_TIMEOUT_SEC) mainState = MainState.END;

            follower.update();
            updateDetectedTagForState();

            switch (mainState) {
                case SEARCH_INITIAL_TAG: handleSearchInitialTag(); break;
                case ADJUST_TO_INITIAL_TAG: handleAdjustToInitialTag(); break;
                case TURN_TO_GOAL_TAG: handleTurnToGoalTag(); break;
                case ADJUST_TO_GOAL_TAG: handleAdjustToGoalTag(MainState.SHOOT_PRELOADED); break;
                case SHOOT_PRELOADED: handleShoot(MainState.START_BALL_COLLECTION); break;
                case START_BALL_COLLECTION: handleStartBallCollection(); break;
                case COLLECTING_BALLS: handleCollectingBalls(); break;
                case RETURN_TO_GOAL_TAG: handleTurnToGoalTag(); break;
                case ADJUST_FOR_SECOND_SHOT: handleAdjustToGoalTag(MainState.SHOOT_COLLECTED); break;
                case SHOOT_COLLECTED: handleShoot(MainState.PARK); break;
                case PARK: handlePark(); break;
                case END: handleEnd(); break;
                default: mainState = MainState.END;
            }

            telemetry.addData("Time", "%.1f / %.1f", runtime.seconds(), AUTON_TIMEOUT_SEC);
            telemetry.addData("State", mainState.toString());
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }
        handleEnd();
    }

    private boolean handleSetup() {
        telemetry.addData("Status", "SETUP: Waiting for AprilTag...");
        AprilTagDetection initialTag = null;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id >= 21 && detection.id <= 23) {
                initialTag = detection;
                break;
            }
        }

        if (initialTag != null) {
            double robotHeadingRad;
            if (initialTag.ftcPose.range > HEADING_DECISION_RANGE) {
                robotHeadingRad = Math.toRadians(90.0);
                telemetry.addData("Target Heading", "90 deg (Far)").addData("Heading Status", "OK");
            } else {
                double targetHeadingRad = Math.toRadians(45.0);
                double currentActualHeadingRad = INITIAL_TAG_POSE.getHeading() - Math.toRadians(initialTag.ftcPose.yaw);
                double errorDeg = Math.toDegrees(targetHeadingRad - currentActualHeadingRad);

                telemetry.addData("Target Heading", "45 deg (Close)").addData("Current Heading", "%.1f deg", Math.toDegrees(currentActualHeadingRad));

                if (Math.abs(errorDeg) < 2.5) {
                    telemetry.addData("Heading Status", "OK!");
                } else if (errorDeg > 0) {
                    telemetry.addData("Heading Status", "Turn Robot LEFT");
                } else {
                    telemetry.addData("Heading Status", "Turn Robot RIGHT");
                }
                robotHeadingRad = currentActualHeadingRad;
            }

            double relX = initialTag.ftcPose.x;
            double relY = initialTag.ftcPose.y;
            double worldVectorX = relX * Math.cos(robotHeadingRad) - relY * Math.sin(robotHeadingRad);
            double worldVectorY = relX * Math.sin(robotHeadingRad) + relY * Math.cos(robotHeadingRad);
            double robotX = INITIAL_TAG_POSE.getX() - worldVectorX;
            double robotY = INITIAL_TAG_POSE.getY() - worldVectorY;
            
            Pose startingPose = new Pose(robotX, robotY, robotHeadingRad);
            follower.setStartingPose(startingPose);

            telemetry.addData("Status", "Pose Ready!").addData("Calculated Start", startingPose).addData(">", "Ready to Start!");
            mainState = MainState.SEARCH_INITIAL_TAG;
            return true;
        } else {
            telemetry.addData("Status", "NO APRILTAG VISIBLE").addData(">", "Adjust robot to see a tag (21-23)");
        }
        telemetry.update();
        sleep(50);
        return false;
    }

    private void handleSearchInitialTag() {
        if (detectedTag != null && (detectedTag.id >= 21 && detectedTag.id <= 23)) {
            foundID = detectedTag.id;
            initialSide = (int) Math.signum(detectedTag.ftcPose.x);
            mainState = MainState.ADJUST_TO_INITIAL_TAG;
        } else {
             follower.setTeleOpDrive(0,0,0.2, true);
        }
    }

    private void handleAdjustToInitialTag() {
        if (detectedTag == null) { mainState = MainState.SEARCH_INITIAL_TAG; return; }
        double targetX = initialSide * LATERAL_OFFSET_INITIAL;
        double errorX = detectedTag.ftcPose.x - targetX;
        double errorY = detectedTag.ftcPose.y - DISTANCE_FROM_APRILTAG;
        if (Math.abs(errorX) < POSITIONING_TOLERANCE && Math.abs(errorY) < POSITIONING_TOLERANCE) {
            follower.setTeleOpDrive(0, 0, 0, true);
            mainState = MainState.TURN_TO_GOAL_TAG;
        } else {
            double forwardPower = -FORWARD_GAIN * errorY;
            double strafePower = STRAFE_GAIN * errorX;
            follower.setTeleOpDrive(clip(forwardPower), clip(strafePower), 0, true);
        }
    }

    private void handleTurnToGoalTag() {
        double turnDirection = initialSide < 0 ? -1.0 : 1.0;
        follower.setTeleOpDrive(0, 0, MAX_TURN_POWER * turnDirection, true);
        if (detectedTag != null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            mainState = (ballsCollected < BALLS_TO_COLLECT) ? MainState.ADJUST_TO_GOAL_TAG : MainState.ADJUST_FOR_SECOND_SHOT;
        }
    }

    private void handleAdjustToGoalTag(MainState nextState) {
        if (detectedTag == null) {
            if (isSpinningUp) { robot.setOuttake(0.0); isSpinningUp = false; }
            mainState = (nextState == MainState.SHOOT_PRELOADED) ? MainState.TURN_TO_GOAL_TAG : MainState.RETURN_TO_GOAL_TAG;
            return;
        }
        double errorRange = detectedTag.ftcPose.range - GOAL_TAG_DISTANCE;
        double errorBearing = detectedTag.ftcPose.bearing;
        if (Math.abs(errorRange) < POSITIONING_TOLERANCE * 3 && Math.abs(errorBearing) < POSITIONING_TOLERANCE * 3 && !isSpinningUp) {
            robot.setOuttake(1.0);
            isSpinningUp = true;
        }
        if (Math.abs(errorRange) < POSITIONING_TOLERANCE && Math.abs(errorBearing) < POSITIONING_TOLERANCE) {
            follower.setTeleOpDrive(0, 0, 0, true);
            mainState = nextState;
        } else {
            double forwardPower = -FORWARD_GAIN * errorRange;
            double turnPower = -TURN_GAIN * errorBearing;
            follower.setTeleOpDrive(clip(forwardPower), 0, clip(turnPower), true);
        }
    }

    private void handleShoot(MainState nextState) {
        shootArtifacts();
        mainState = nextState;
    }

    private void handleStartBallCollection() {
        Pose currentPose = follower.getPose();
        Pose turnPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + Math.toRadians(180));
        follower.followPath(new Path(new BezierLine(currentPose, turnPose)));
        collectionState = CollectionState.SEARCHING;
        mainState = MainState.COLLECTING_BALLS;
    }

    private void handleCollectingBalls() {
        if (ballsCollected >= BALLS_TO_COLLECT) {
            mainState = MainState.RETURN_TO_GOAL_TAG;
            return;
        }
        Rect targetBall = ballProcessor.getBestPurpleContourRect() != null ? ballProcessor.getBestPurpleContourRect() : ballProcessor.getBestGreenContourRect();
        switch (collectionState) {
            case SEARCHING:
                follower.setTeleOpDrive(0, 0, -MAX_TURN_POWER, true);
                if (targetBall != null) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    collectionState = CollectionState.CENTERING;
                }
                break;
            case CENTERING:
                if (targetBall == null) { collectionState = CollectionState.SEARCHING; break; }
                double error = (targetBall.x + (double) targetBall.width / 2) - ((double) CAMERA_WIDTH / 2);
                if (Math.abs(error) > POSITIONING_TOLERANCE * 2) {
                    follower.setTeleOpDrive(0, 0, -TURN_GAIN * error, true);
                } else {
                    collectionState = CollectionState.MOVING_TO_BALL;
                }
                break;
            case MOVING_TO_BALL:
                if (targetBall == null) { collectionState = CollectionState.SEARCHING; break; }
                if (targetBall.height < BALL_TARGET_HEIGHT) {
                    follower.setTeleOpDrive(MAX_DRIVE_POWER * 0.6, 0, 0, true);
                } else {
                    collectionState = CollectionState.INTAKING;
                }
                break;
            case INTAKING:
                robot.intake.setPower(0.9);
                follower.setTeleOpDrive(MAX_DRIVE_POWER * 0.4, 0, 0, true);
                stateTimer.reset();
                collectionState = CollectionState.CONFIRMING_CAPTURE;
                break;
            case CONFIRMING_CAPTURE:
                if (robot.sensorDistance.getDistance(DistanceUnit.CM) < BALL_CAPTURED_DISTANCE_CM) {
                    ballsCollected++;
                    robot.intake.setPower(0);
                    collectionState = CollectionState.SEARCHING;
                } else if (stateTimer.seconds() > 2.0) {
                    robot.intake.setPower(0);
                    collectionState = CollectionState.SEARCHING;
                }
                break;
        }
    }

    private void handlePark() {
        if (detectedTag == null) { mainState = MainState.END; return; }
        double targetX = initialSide * PARK_DISTANCE;
        double errorX = detectedTag.ftcPose.x - targetX;
        double errorY = detectedTag.ftcPose.y - GOAL_TAG_DISTANCE;
        double errorBearing = detectedTag.ftcPose.bearing;
        if (Math.abs(errorX) < POSITIONING_TOLERANCE) {
            follower.setTeleOpDrive(0, 0, 0, true);
            mainState = MainState.END;
        } else {
            double strafePower = STRAFE_GAIN * errorX;
            double forwardPower = -FORWARD_GAIN * errorY;
            double turnPower = -TURN_GAIN * errorBearing;
            follower.setTeleOpDrive(clip(forwardPower), clip(strafePower), clip(turnPower), true);
        }
    }
    
    private void handleEnd() {
        Pose finalPose = follower.getPose();
        PoseStorage.savePoseToFile(finalPose, initialSide);
        OpModeData.lastPose = finalPose;
        OpModeData.initialSide = initialSide;
        if (opModeIsActive()) requestOpModeStop();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        ballProcessor = new BallDetectionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag, ballProcessor)
                .build();
    }

    private void updateDetectedTagForState() {
        detectedTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) return;
        int[] targetIds;
        switch (mainState) {
            case SETUP:
            case SEARCH_INITIAL_TAG:
            case ADJUST_TO_INITIAL_TAG:
                targetIds = new int[]{21, 22, 23};
                break;
            case TURN_TO_GOAL_TAG:
            case ADJUST_TO_GOAL_TAG:
            case RETURN_TO_GOAL_TAG:
            case ADJUST_FOR_SECOND_SHOT:
            case PARK:
                targetIds = new int[]{(initialSide < 0) ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID};
                break;
            default:
                targetIds = new int[]{}; // For collection and other states, we don't need tags
                break;
        }
        for (AprilTagDetection detection : currentDetections) {
            for (int id : targetIds) {
                if (detection.id == id) {
                    detectedTag = detection;
                    return;
                }
            }
        }
    }

    public void shootArtifacts() {
        robot.setOuttake(1.0);
        if (foundID == 21) {
            robot.shutGreenArtifact();
            robot.shutPurpleArtifact();
            robot.shutPurpleArtifact();
        } else if (foundID == 22) {
            robot.shutPurpleArtifact();
            robot.shutGreenArtifact();
            robot.shutPurpleArtifact();
        } else if (foundID == 23) {
            robot.shutPurpleArtifact();
            robot.shutPurpleArtifact();
            robot.shutGreenArtifact();
        }
        sleep(1000);
        robot.setOuttake(0.0);
        isSpinningUp = false;
    }

    private double clip(double value) {
        return Math.max(-MAX_DRIVE_POWER, Math.min(value, MAX_DRIVE_POWER));
    }
}
