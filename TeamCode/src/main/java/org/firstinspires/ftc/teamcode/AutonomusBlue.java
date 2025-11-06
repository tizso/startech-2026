package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Rect;

import java.util.List;

@Autonomous(name = "AutonomusBlue", group = "Opmode")
@Configurable
@SuppressWarnings("FieldCanBeLocal")
public class AutonomusBlue extends LinearOpMode {
    HardwareBox robot = new HardwareBox();
    private final ElapsedTime runtime = new ElapsedTime();

    // Poses and Paths for AprilTag navigation
    private final Pose startPose = new Pose(72, 120, Math.toRadians(90));
    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115));
    private final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0));
    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0));
    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0));
    private PathChain currentPath;

    // Vision variables
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private BallDetectionProcessor ballProcessor;

    // Follower and state variables
    private Follower follower;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Main state machine enum
    private enum MainState {
        FIND_APRILTAG_FIRST,
        GO_TO_SCORE_FIRST,
        SHOOT_PRELOADED_BALLS,
        START_COLLECTION,
        COLLECTING_BALLS,
        FIND_APRILTAG_SECOND,
        GO_TO_SCORE_SECOND,
        SHOOT_COLLECTED_BALLS,
        END
    }
    private MainState mainState = MainState.FIND_APRILTAG_FIRST;

    // Ball collection sub-state machine enum
    private enum CollectionState {
        SEARCHING,
        CENTERING,
        MOVING_TO_BALL,
        INTAKING,
        CONFIRMING_CAPTURE
    }
    private CollectionState collectionState = CollectionState.SEARCHING;

    private int ballsCollected = 0;

    // Constants
    private static final double BALL_CAPTURED_DISTANCE_CM = 5.0;
    private static final int CAMERA_WIDTH = 640;
    private static final double TARGET_BALL_CENTER_ERROR = 15.0; // pixels
    private static final double FORWARD_POWER = 0.4;
    private static final double TURN_POWER = 0.35;
    private static final double TURN_KP = 0.006;

    @Override
    public void runOpMode() {
        // Initialization
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        initVision();

        telemetry.addData("Status", "Initialized and ready.");
        telemetry.addData("-", "Waiting for START");
        telemetry.update();

        waitForStart();
        runtime.reset();
        follower.startTeleopDrive(); // Use teleop drive for manual motor control in states

        // Main autonomous loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            switch (mainState) {
                case FIND_APRILTAG_FIRST:
                    telemetry.addData("Main State", "Finding AprilTag (First)");
                    if (findAprilTagAndBuildPath()) {
                        mainState = MainState.GO_TO_SCORE_FIRST;
                    } else {
                        follower.setTeleOpDrive(0, 0, TURN_POWER, true); // Rotate to find tag
                    }
                    break;

                case GO_TO_SCORE_FIRST:
                    telemetry.addData("Main State", "Going to Score (First)");
                    follower.followPath(currentPath);
                    mainState = MainState.SHOOT_PRELOADED_BALLS;
                    break;

                case SHOOT_PRELOADED_BALLS:
                    telemetry.addData("Main State", "Shooting Pre-loaded Balls");
                    if (!follower.isBusy()) {
                        shootBalls(3); // Shoot 3 balls
                        mainState = MainState.START_COLLECTION;
                    }
                    break;

                case START_COLLECTION:
                    telemetry.addData("Main State", "Starting Ball Collection");
                    ballsCollected = 0;
                    collectionState = CollectionState.SEARCHING;
                    mainState = MainState.COLLECTING_BALLS;
                    break;

                case COLLECTING_BALLS:
                    telemetry.addData("Main State", "Collecting Balls");
                    updateBallCollectionStateMachine();
                    if (ballsCollected >= 3) {
                        mainState = MainState.FIND_APRILTAG_SECOND;
                    }
                    break;

                case FIND_APRILTAG_SECOND:
                    telemetry.addData("Main State", "Finding AprilTag (Second)");
                    if (findAprilTagAndBuildPath()) {
                        mainState = MainState.GO_TO_SCORE_SECOND;
                    } else {
                        follower.setTeleOpDrive(0, 0, TURN_POWER, true);
                    }
                    break;

                case GO_TO_SCORE_SECOND:
                    telemetry.addData("Main State", "Going to Score (Second)");
                    follower.followPath(currentPath);
                    mainState = MainState.SHOOT_COLLECTED_BALLS;
                    break;

                case SHOOT_COLLECTED_BALLS:
                    telemetry.addData("Main State", "Shooting Collected Balls");
                    if (!follower.isBusy()) {
                        shootBalls(3);
                        mainState = MainState.END;
                    }
                    break;

                case END:
                default:
                    telemetry.addData("Main State", "END");
                    follower.setTeleOpDrive(0, 0, 0, true);
                    requestOpModeStop(); // End the OpMode
                    break;
            }
            telemetry.update();
        }
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private boolean findAprilTagAndBuildPath() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Pose goalPose = null;
                if (detection.id == GPP_TAG_ID) goalPose = GPPPose;
                else if (detection.id == PGP_TAG_ID) goalPose = PGPPose;
                else if (detection.id == PPG_TAG_ID) goalPose = PPGPose;

                if (goalPose != null) {
                    currentPath = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), scorePose))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    follower.setTeleOpDrive(0, 0, 0, true); // Stop turning
                    return true;
                }
            }
        }
        return false;
    }

    private void shootBalls(int count) {
        // This is a simplified shooting mechanism. Run outtake for a duration.
        robot.outtake.setPower(1.0);
        robot.servoInR.setPower(1.0);
        sleep(count * 1000L); // Assume 1 second per ball
        robot.outtake.setPower(0);
        robot.servoInR.setPower(0);
    }

    private void updateBallCollectionStateMachine() {
        Rect targetBall = ballProcessor.getBestPurpleContourRect() != null ? ballProcessor.getBestPurpleContourRect() : ballProcessor.getBestGreenContourRect();

        telemetry.addData("Collection State", collectionState.toString());
        telemetry.addData("Balls Collected", ballsCollected);
        telemetry.addData("Ball Target", targetBall == null ? "None" : "Acquired");

        switch (collectionState) {
            case SEARCHING:
                robot.intake.setPower(0);
                follower.setTeleOpDrive(0, 0, TURN_POWER, true); // Rotate to find a ball
                if (targetBall != null) {
                    follower.setTeleOpDrive(0, 0, 0, true); // Stop turning
                    collectionState = CollectionState.CENTERING;
                }
                break;

            case CENTERING:
                if (targetBall == null) { collectionState = CollectionState.SEARCHING; break; }
                double error = (targetBall.x + (double)targetBall.width / 2) - ((double)CAMERA_WIDTH / 2);
                if (Math.abs(error) > TARGET_BALL_CENTER_ERROR) {
                    follower.setTeleOpDrive(0, 0, -TURN_KP * error, true);
                } else {
                    collectionState = CollectionState.MOVING_TO_BALL;
                }
                break;

            case MOVING_TO_BALL:
                if (targetBall == null) { collectionState = CollectionState.SEARCHING; break; }
                double distanceEstimate = 2000 / targetBall.height; // simple inverse proportion
                if (distanceEstimate > 10) { // arbitrary distance threshold
                    follower.setTeleOpDrive(FORWARD_POWER, 0, 0, true);
                } else {
                    collectionState = CollectionState.INTAKING;
                }
                break;

            case INTAKING:
                follower.setTeleOpDrive(FORWARD_POWER * 0.5, 0, 0, true);
                robot.intake.setPower(0.9);
                stateTimer.reset();
                collectionState = CollectionState.CONFIRMING_CAPTURE;
                break;

            case CONFIRMING_CAPTURE:
                if (robot.sensorDistance.getDistance(DistanceUnit.CM) < BALL_CAPTURED_DISTANCE_CM) {
                    ballsCollected++;
                    robot.intake.setPower(0);
                    collectionState = CollectionState.SEARCHING;
                } else if (stateTimer.seconds() > 3.0) { // Timeout
                    robot.intake.setPower(0);
                    collectionState = CollectionState.SEARCHING;
                }
                break;
        }
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        ballProcessor = new BallDetectionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag, ballProcessor)
                .build();
    }
}
