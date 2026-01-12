package org.firstinspires.ftc.teamcode;

// Autonomous StarTech V2 – Time-aware autonomous + half-field safety (X bounds)
// - Early jump to RETURN_TO_SHOOT at 24 s (to leave time for shooting + parking)
// - 28 s hard cap (never exceed the 29 s physical limit)
// - Half-field safety: left side X ∈ [0,70], right side X ∈ [74,144]
// - PedroPathing micro-paths (heading alignment + approach)

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Autonomous(name = "Autonomous StarTech V2", group = "Opmode")
@Disabled
public class AutonomousStarTechNewV2 extends LinearOpMode {
    // --- Core robot components ---
    HardwareBox robot = new HardwareBox();
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ShooterManager shooterManager;
    private BallDetectionProcessor ballProcessor;

    // --- State machine ---
    private enum State {
        SETUP,
        // Collection
        COLLECT_PREP,
        MOVE_TO_NEXT_BALL, WAIT_FOR_NEXT_BALL_MOVE,
        TRACK_AND_APPROACH, DECIDE_SEPARATOR, INTAKE_THROUGH,
        VERIFY_AND_ADVANCE,
        // Return and shooting after collection
        RETURN_TO_SHOOT, WAIT_FOR_RETURN,
        SHOOT_FINE_AIM, WAIT_SHOOT_FINE_AIM,
        SHOOT_CHECK_SPEED, SHOOT_FIRE, SHOOT_NEXT_SHOT,
        // Park (path or fallback micro-step)
        WAIT_FOR_PARK,
        END
    }
    private State currentState = State.SETUP;

    // --- Timing guard: 28 s hard cap + 24 s early return ---
    private final ElapsedTime autoTimer = new ElapsedTime();
    private static final double AUTO_HARDCAP_SEC = 28.0;  // never exceeded
    private static final double AUTO_EARLY_RETURN_SEC = 24.0; // jump back to shooting here
    private final ElapsedTime stateTimer = new ElapsedTime();

    // --- Half-field X limits ---
    private static final double LEFT_MIN_X  = 0.0;
    private static final double LEFT_MAX_X  = 70.0;
    private static final double RIGHT_MIN_X = 74.0;
    private static final double RIGHT_MAX_X = 144.0;
    private static final double ROBOT_ALIGN_TO_BALL_DIST_IN = 4.0;

    private double ALLOWED_MIN_X = LEFT_MIN_X;  // initialized at start based on startPose
    private double ALLOWED_MAX_X = LEFT_MAX_X;

    // --- Strategy / preparation ---
    private double computedDistanceInch = 0.0;
    private Pose startPose = null;
    private Pose shootPose = null; // initial shooting pose
    private int foundID = -1;
    private int backdropId = -1;
    private char[] sequence = {'P','P','P'}; // initially determined shooting sequence
    private int shotIndex = 0;

    // --- Ball collection ---
    private Pose[] SELECTED_SET = new Pose[]{}; // ordered ball list
    private int currentBallIdx = 0;
    private static final int TOTAL_BALLS = 3;

    // --- Distance sensor-based pass detection --
    private static final double PASS_DISTANCE_CM = 9.0;   // threshold below which a ball is considered present
    private static final long   PASS_MIN_MS      = 120;   // minimum presence time (ms) to confirm pass
    private boolean passInProgress = false;
    private int ballsCollected = 0;
    private final ElapsedTime passElapsed = new ElapsedTime();

    // --- Tracking/approach parameters for PedroPathing micro-paths ---
    private static final double TRACK_KP_TURN = 0.012;   // pixel error → heading delta
    private static final double CENTER_TOL_FRAC = 0.06;  // 6% of image width
    private static final double AREA_NEAR_THRESHOLD = 2500; // bounding-box area threshold
    private static final double ROTATE_STEP_IN = 0.8;    // rotation micro-step
    private static final double APPROACH_STEP_IN = 2.0;  // approach micro-step

    // --- Separator positions ---
    private static final double SEPARATOR_RIGHT_PURPLE = 0.6; // purple → right side
    private static final double SEPARATOR_LEFT_GREEN   = 0.4; // green → left side

    // --- Parking (fallback): simple forward micro-path after shooting ---
    private static final double PARK_FWD_IN = 14.0;  // hangold a pályához

    // (Optional) If you later define a specific park path:
    private PathChain pathToPark = null; // currently using fallback

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        shooterManager = new ShooterManager(robot.outtakeLeft, robot.outtakeRight, hardwareMap.voltageSensor.iterator().next());
        initVision();

        // --- INIT loop ---
        while (opModeInInit()) {
            handleSetup(); // computes startPose, shootPose and sequence based on AprilTag
            if (startPose != null) {
                follower.setStartingPose(startPose);
            }
            displayInitTelemetry();
        }

        // --- START ---
        waitForStart();
        autoTimer.reset();
        stateTimer.reset();

        // Collection indul azonnal
        currentState = State.COLLECT_PREP;

        // --- Main loop ---
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            // --- Half-field guard: always clamp X for any targets (applied when building paths) ---

            // --- Global time guards ---
            double t = autoTimer.seconds();
            if (t >= AUTO_HARDCAP_SEC) {
                telemetry.addLine("AUTO TIME CAP REACHED: 28s → Ending safely");
                currentState = State.END;
            } else if (t >= AUTO_EARLY_RETURN_SEC) {
                // After 24 s, from any collecting state jump back to shooting
                switch (currentState) {
                    case COLLECT_PREP:
                    case MOVE_TO_NEXT_BALL:
                    case WAIT_FOR_NEXT_BALL_MOVE:
                    case TRACK_AND_APPROACH:
                    case DECIDE_SEPARATOR:
                    case INTAKE_THROUGH:
                    case VERIFY_AND_ADVANCE:
                        stopIntake();
                        currentState = State.RETURN_TO_SHOOT;
                        stateTimer.reset();
                        telemetry.addLine("Early Return at 24s → RETURN_TO_SHOOT");
                        break;
                    default:
                        break;
                }
            }

            runStateMachine();
            displayRuntimeTelemetry();
        }

        if (visionPortal != null) visionPortal.close();
    }

    private void runStateMachine() {
        switch (currentState) {
            // --- Continuous collection in field order ---
            case COLLECT_PREP:
                calculateAndSetCameraTilt();
                setSeparatorRightPurple(); // default: purple to right
                startIntake();             // intake stays ON during collection
                ballsCollected = 0;
                currentBallIdx = 0;
                SELECTED_SET = selectBallSeries(follower.getPose());
                currentState = State.MOVE_TO_NEXT_BALL;
                stateTimer.reset();
                break;

            case MOVE_TO_NEXT_BALL:
                if (currentBallIdx >= TOTAL_BALLS) {
                    // Collection kész → vissza a initial shooting posera
                    stopIntake();
                    currentState = State.RETURN_TO_SHOOT;
                    stateTimer.reset();
                    break;
                }
                Pose cur = follower.getPose();
                if (cur == null) break;
                Pose nominal = SELECTED_SET[currentBallIdx];
                double ang = Math.atan2(nominal.getY() - cur.getY(), nominal.getX() - cur.getX());
                // Stop in front of the ball, but keep X within half-field bounds
                double stopX = clampX(nominal.getX() - ROBOT_ALIGN_TO_BALL_DIST_IN * Math.cos(ang));
                double stopY = nominal.getY() - ROBOT_ALIGN_TO_BALL_DIST_IN * Math.sin(ang);
                Pose stopPose = new Pose(stopX, stopY, ang);
                PathChain toNominal = follower.pathBuilder()
                        .addPath(new BezierLine(cur, stopPose))
                        .setLinearHeadingInterpolation(cur.getHeading(), ang)
                        .build();
                follower.followPath(toNominal);
                currentState = State.WAIT_FOR_NEXT_BALL_MOVE;
                stateTimer.reset();
                break;

            case WAIT_FOR_NEXT_BALL_MOVE:
                if (!follower.isBusy()) {
                    currentState = State.TRACK_AND_APPROACH;
                    stateTimer.reset();
                }
                break;

            case TRACK_AND_APPROACH:
                if (alignAndApproachDetectedBall()) {
                    currentState = State.DECIDE_SEPARATOR;
                    stateTimer.reset();
                }
                break;

            case DECIDE_SEPARATOR:
                decideSeparatorForCurrentDetection(); // zöld→bal(0.4), lila→jobb(0.6)
                currentState = State.INTAKE_THROUGH;
                stateTimer.reset();
                break;

            case INTAKE_THROUGH:
                // Detect pass with REV 2m Distance sensor
                if (updatePassDetector()) {
                    currentState = State.VERIFY_AND_ADVANCE;
                    stateTimer.reset();
                } else if (stateTimer.seconds() > 2.0) { // time-based fallback
                    currentState = State.VERIFY_AND_ADVANCE;
                    stateTimer.reset();
                }
                break;

            case VERIFY_AND_ADVANCE:
                currentBallIdx++;
                currentState = State.MOVE_TO_NEXT_BALL;
                stateTimer.reset();
                break;

            // --- Return to shooting pose and 3 shots ---
            case RETURN_TO_SHOOT:
                if (shootPose == null) { currentState = State.END; break; }
                Pose now = follower.getPose();
                if (now == null) break;
                Pose clampedShoot = new Pose(clampX(shootPose.getX()), shootPose.getY(), shootPose.getHeading());
                PathChain backToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(now, clampedShoot))
                        .setLinearHeadingInterpolation(now.getHeading(), clampedShoot.getHeading())
                        .build();
                follower.followPath(backToShoot);
                currentState = State.WAIT_FOR_RETURN;
                stateTimer.reset();
                break;

            case WAIT_FOR_RETURN:
                if (!follower.isBusy()) {
                    currentState = State.SHOOT_FINE_AIM; // fine aim
                    stateTimer.reset();
                }
                break;

            case SHOOT_FINE_AIM:
                startFineAim(); // PedroPathing micro-step with heading correction (within X clamp)
                currentState = State.WAIT_SHOOT_FINE_AIM;
                stateTimer.reset();
                break;

            case WAIT_SHOOT_FINE_AIM:
                if (!follower.isBusy() || stateTimer.seconds() > RobotConstants.FINE_AIM_TIMEOUT_SEC) {
                    currentState = State.SHOOT_CHECK_SPEED;
                    stateTimer.reset();
                }
                break;

            case SHOOT_CHECK_SPEED:
                Pose poseNow = follower.getPose();
                if (poseNow != null && shootPose != null) {
                    computedDistanceInch = Math.hypot(shootPose.getX() - poseNow.getX(), shootPose.getY() - poseNow.getY());
                }
                shooterManager.setSpeedFromDistance(computedDistanceInch, false);
                if (shooterManager.isReady() || stateTimer.seconds() > RobotConstants.SHOOTER_READY_TIMEOUT_SEC) {
                    shooterManager.applyBoost();
                    currentState = State.SHOOT_FIRE;
                    stateTimer.reset();
                }
                break;

            case SHOOT_FIRE:
                fireBall(sequence[shotIndex]);
                currentState = State.SHOOT_NEXT_SHOT;
                stateTimer.reset();
                break;

            case SHOOT_NEXT_SHOT:
                if (stateTimer.seconds() > (RobotConstants.SHOOTING_SERVO_RUN_TIME_SEC + RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC)) {
                    shotIndex++;
                    if (shotIndex < sequence.length) {
                        currentState = State.SHOOT_CHECK_SPEED;
                    } else {
                        // After shots – try to park within X bounds
                        double remaining = AUTO_HARDCAP_SEC - autoTimer.seconds();
                        Pose curPose = follower.getPose();
                        if (curPose == null) { currentState = State.END; break; }
                        if (remaining > 1.5) {
                            if (pathToPark != null) {
                                // If a prebuilt park path exists, follow it (assuming it is safe)
                                follower.followPath(pathToPark);
                                currentState = State.WAIT_FOR_PARK;
                            } else {
                                // Fallback: drive forward PARK_FWD_IN with X clamp
                                double heading = curPose.getHeading();
                                double px = clampX(curPose.getX() + PARK_FWD_IN * Math.cos(heading));
                                double py = curPose.getY() + PARK_FWD_IN * Math.sin(heading);
                                Pose parkTarget = new Pose(px, py, heading);
                                PathChain parkPath = follower.pathBuilder()
                                        .addPath(new BezierLine(curPose, parkTarget))
                                        .setLinearHeadingInterpolation(curPose.getHeading(), heading)
                                        .build();
                                follower.followPath(parkPath);
                                currentState = State.WAIT_FOR_PARK;
                            }
                        } else {
                            currentState = State.END; // no time left to park
                        }
                        stateTimer.reset();
                    }
                }
                break;

            case WAIT_FOR_PARK:
                if (!follower.isBusy() || stateTimer.seconds() > 3.0) {
                    currentState = State.END;
                    stateTimer.reset();
                }
                break;

            case END:
                shooterManager.stop();
                stopIntake();
                Pose finalPose = follower.getPose();
                if (finalPose != null) {
                    PoseStorage.savePoseToFile(finalPose, (startPose != null && startPose.getX() < 72.0) ? -1 : 1, finalPose);
                    OpModeData.lastPose = finalPose;
                    OpModeData.initialSide = (startPose != null && startPose.getX() < 72.0) ? -1 : 1;
                }
                requestOpModeStop();
                break;
        }
    }

    // --- Vision init: AprilTag + BallDetectionProcessor ---
    private void initVision() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            ballProcessor = new BallDetectionProcessor();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .addProcessor(ballProcessor)
                    .build();
        } catch (Exception e) {
            aprilTag = null; visionPortal = null;
        }
    }

    // --- Position and sequence calculation during INIT based on AprilTag ---
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
        if (backdrop == null) return;

        // We estimated startPose from the backdrop before; simplified here:
        // if startPose X < 72 → left half, otherwise use right half bounds.
        if (startPose != null) {
            if (startPose.getX() < 72.0) { ALLOWED_MIN_X = LEFT_MIN_X; ALLOWED_MAX_X = LEFT_MAX_X; }
            else { ALLOWED_MIN_X = RIGHT_MIN_X; ALLOWED_MAX_X = RIGHT_MAX_X; }
        }
    }

    // --- Fine aim for shooting (PedroPathing micro-step with X clamp) ---
    private void startFineAim() {
        Pose current = follower.getPose();
        if (current == null) return;
        double desiredHeading;
        AprilTagDetection backdrop = getClosestBackdrop();
        if (backdrop != null && backdrop.ftcPose != null) {
            desiredHeading = current.getHeading() + Math.toRadians(backdrop.ftcPose.bearing);
        } else {
            desiredHeading = (shootPose != null) ? shootPose.getHeading() : current.getHeading();
        }
        Pose target = new Pose(
                clampX(current.getX() + ROTATE_STEP_IN * Math.cos(desiredHeading)),
                current.getY() + ROTATE_STEP_IN * Math.sin(desiredHeading),
                desiredHeading
        );
        PathChain fine = follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), desiredHeading)
                .build();
        follower.followPath(fine);
    }

    private void fireBall(char type) {
        if (type == 'G') { robot.servoInL.setPower(1.0); }
        else { robot.servoInR.setPower(1.0); }
        sleep((long)(RobotConstants.SHOOTING_SERVO_RUN_TIME_SEC * 1000));
        robot.servoInL.setPower(0.0);
        robot.servoInR.setPower(0.0);
        sleep((long)(RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC * 1000));
    }

    /**
     * Select ordered ball list (by field Y row, then X increasing).
     */
    private Pose[] selectBallSeries(Pose currentPose) {
        Pose[] raw;
        if (currentPose != null && currentPose.getY() > 72) {
            raw = BALLS_TOP_ROW;
        } else {
            raw = BALLS_BOTTOM_ROW;
        }
        Pose[] copy = Arrays.copyOf(raw, raw.length);
        Arrays.sort(copy, Comparator.comparingDouble(Pose::getX));
        return copy; // clamping is applied during path building
    }

    /**
     * Collectioni kameradőlés – egyszerű fix beállítás (ha kell, később hangolható).
     */
    private void calculateAndSetCameraTilt() {
        double servoPosition = 0.5; // középállás – hangolható
        robot.cameraTilt.setPosition(servoPosition);
        telemetry.addData("Camera Tilt", "Position: %.2f", servoPosition);
    }

    /**
     * Micro-path alignment and approach to the currently detected ball.
     * Returns true when we are close enough and centered.
     */
    private boolean alignAndApproachDetectedBall() {
        if (ballProcessor == null) return false;
        org.opencv.core.Point cG = ballProcessor.getGreenCenter();
        org.opencv.core.Point cP = ballProcessor.getPurpleCenter();

        if (cG == null && cP == null) {
            // No ball detected: micro-rotation to search
            if (!follower.isBusy()) {
                Pose cur = follower.getPose();
                if (cur == null) return false;
                double desiredHeading = cur.getHeading() + Math.toRadians(5.0);
                Pose target = new Pose(
                        clampX(cur.getX() + ROTATE_STEP_IN * Math.cos(desiredHeading)),
                        cur.getY() + ROTATE_STEP_IN * Math.sin(desiredHeading),
                        desiredHeading
                );
                PathChain micro = follower.pathBuilder()
                        .addPath(new BezierLine(cur, target))
                        .setLinearHeadingInterpolation(cur.getHeading(), desiredHeading)
                        .build();
                follower.followPath(micro);
            }
            return false;
        }

        double aG = ballProcessor.getGreenArea();
        double aP = ballProcessor.getPurpleArea();
        org.opencv.core.Point center = (aG >= aP) ? cG : cP;
        double area = Math.max(aG, aP);
        int width = ballProcessor.getFrameWidth();
        if (width <= 0) return false;

        double errorPx = center.x - (width / 2.0);
        double headingDelta = TRACK_KP_TURN * errorPx;

        if (area > AREA_NEAR_THRESHOLD && Math.abs(errorPx) < CENTER_TOL_FRAC * width) {
            return true;
        } else {
            if (!follower.isBusy()) {
                Pose cur = follower.getPose();
                if (cur == null) return false;
                double desiredHeading = cur.getHeading() + headingDelta;
                Pose target = new Pose(
                        clampX(cur.getX() + APPROACH_STEP_IN * Math.cos(desiredHeading)),
                        cur.getY() + APPROACH_STEP_IN * Math.sin(desiredHeading),
                        desiredHeading
                );
                PathChain micro = follower.pathBuilder()
                        .addPath(new BezierLine(cur, target))
                        .setLinearHeadingInterpolation(cur.getHeading(), desiredHeading)
                        .build();
                follower.followPath(micro);
            }
            return false;
        }
    }

    private enum BallColor { GREEN, PURPLE, UNKNOWN }

    private BallColor getDetectedColor() {
        if (ballProcessor == null) return BallColor.UNKNOWN;
        boolean hasG = ballProcessor.getGreenCenter()  != null;
        boolean hasP = ballProcessor.getPurpleCenter() != null;
        if (hasG && !hasP) return BallColor.GREEN;
        if (!hasG && hasP) return BallColor.PURPLE;
        if (!hasG && !hasP) return BallColor.UNKNOWN;
        return (ballProcessor.getGreenArea() >= ballProcessor.getPurpleArea()) ? BallColor.GREEN : BallColor.PURPLE;
    }

    private void decideSeparatorForCurrentDetection() {
        BallColor c = getDetectedColor();
        if (c == BallColor.GREEN) setSeparatorLeftGreen(); else setSeparatorRightPurple();
    }

    private void setSeparatorLeftGreen() { robot.separator.setPosition(SEPARATOR_LEFT_GREEN); }
    private void setSeparatorRightPurple() { robot.separator.setPosition(SEPARATOR_RIGHT_PURPLE); }

    private void startIntake() { robot.intake.setPower(1.0); }
    private void stopIntake()  { robot.intake.setPower(0.0); }

    // REV 2m Distance – labda átmenet detektor
    private boolean updatePassDetector() {
        double dcm = robot.sensorDistance.getDistance(DistanceUnit.CM);
        if (dcm <= 0) return false; // érvénytelen érték
        if (!passInProgress && dcm < PASS_DISTANCE_CM) { passInProgress = true; passElapsed.reset(); }
        if (passInProgress && dcm >= PASS_DISTANCE_CM && passElapsed.milliseconds() > PASS_MIN_MS) {
            passInProgress = false; ballsCollected++; return true; }
        return false;
    }

    private AprilTagDetection getClosestBackdrop() {
        if (aprilTag == null) return null;
        List<AprilTagDetection> dets = aprilTag.getDetections();
        AprilTagDetection best = null; double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection d : dets) {
            if (d != null && d.metadata != null && (d.id == RobotConstants.BLUE_GOAL_TAG_ID || d.id == RobotConstants.RED_GOAL_TAG_ID)) {
                if (d.ftcPose != null && d.ftcPose.range < bestRange) { bestRange = d.ftcPose.range; best = d; }
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

    // --- Telemetry ---
    private void displayInitTelemetry() {
        telemetry.addLine("=== INIT PRECOMPUTE ===");
        telemetry.addData("StartPose", startPose);
        telemetry.addData("ShootPose", shootPose);
        telemetry.addData("Allowed X", String.format("[%.1f, %.1f]", ALLOWED_MIN_X, ALLOWED_MAX_X));
        telemetry.update();
    }

    private void displayRuntimeTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Auto Time (s)", "%.2f", autoTimer.seconds());
        if (ballProcessor != null) {
            telemetry.addData("Green area", ballProcessor.getGreenArea());
            telemetry.addData("Purple area", ballProcessor.getPurpleArea());
        }
        telemetry.addData("Separator", robot.separator.getPosition());
        telemetry.addData("Balls collected", ballsCollected);
        telemetry.addData("Distance (cm)", robot.sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Shoot sequence", new String(sequence));
        telemetry.addData("ShotIndex", shotIndex);
        telemetry.addData("Allowed X", String.format("[%.1f, %.1f]", ALLOWED_MIN_X, ALLOWED_MAX_X));
        telemetry.update();
    }

    // --- Predefined ball rows (by row) – simplified from your original set ---
    private static final Pose[] BALLS_TOP_ROW = new Pose[] {
            new Pose(19.0, 84.0, 0.0), new Pose(24.0, 84.0, 0.0), new Pose(29.0, 84.0, 0.0),
            new Pose(115.0, 84.0, 0.0), new Pose(120.0, 84.0, 0.0), new Pose(125.0, 84.0, 0.0)
    };
    private static final Pose[] BALLS_BOTTOM_ROW = new Pose[] {
            new Pose(19.0, 36.0, 0.0), new Pose(24.0, 36.0, 0.0), new Pose(29.0, 36.0, 0.0),
            new Pose(115.0, 36.0, 0.0), new Pose(120.0, 36.0, 0.0), new Pose(125.0, 36.0, 0.0)
    };

    // --- X clamp helper function ---
    private double clampX(double x) {
        if (x < ALLOWED_MIN_X) return ALLOWED_MIN_X;
        if (x > ALLOWED_MAX_X) return ALLOWED_MAX_X;
        return x;
    }
}
