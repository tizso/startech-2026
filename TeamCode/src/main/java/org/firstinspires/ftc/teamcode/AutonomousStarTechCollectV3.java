package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * AutonomousStarTechCollectV3 (Partial Re‑Shoot, Time‑Budgeted)
 * V3_Fix2: adds missing beginReturnToShoot() and buildPostShootSequence().
 */
@Autonomous(name = "Autonomous StarTech V2", group = "Opmode")
@Disabled
public class AutonomousStarTechCollectV3 extends LinearOpMode {

    // --- Robot & follower ---
    HardwareBox robot = new HardwareBox();
    private Follower follower;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private BallDetectionProcessor ballProc;

    // --- State machine ---
    private enum State {
        SETUP, MOVE_TO_SHOOT, WAIT_FOR_MOVE, FINE_AIM, WAIT_FOR_FINE_AIM,
        CHECK_SPEED, FIRE, NEXT_SHOT,
        COLLECT_INIT, GOTO_BALL, WAIT_GOTO, VISION_HOME, PICK_WAIT, NEXT_BALL,
        SEARCH_INIT, SEARCH_TURN, SEARCH_CHECK,
        RETURN_TO_SHOOT, WAIT_RETURN_TO_SHOOT, POST_FINE_AIM, POST_WAIT_FOR_FINE_AIM,
        POST_SHOOT_CHECK_SPEED, POST_FIRE, POST_NEXT_SHOT,
        PARK, WAIT_FOR_PARK, END
    }
    private State currentState = State.SETUP;

    // Timers
    private final ElapsedTime timer   = new ElapsedTime();
    private final ElapsedTime overall = new ElapsedTime();

    // --- Time budget (tune on field) ---
    private static final double AUTON_TIME_BUDGET_S   = 29.0;
    private static final double SAFETY_MARGIN_S       = 0.50;
    private static final double AVG_TRANSIT_IN_PER_S  = 38.0;
    private static final double EST_FINE_AIM_S        = 0.35;
    private static final double EST_SHOT_CYCLE_S      = (0.25 + 0.12) + 0.20;
    private static final double EST_HOMING_S          = 1.00;

    // --- Landmarks & IDs ---
    private static final Pose BLUE_BACKDROP_POSE = new Pose(16.01, 133.28, Math.toRadians(-126.0));
    private static final Pose RED_BACKDROP_POSE  = new Pose(127.99, 133.28, Math.toRadians(-54.0));
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;

    // --- Geometry & camera offset ---
    private static final double CAMERA_FORWARD_OFFSET = 7.0;
    private static final double YAW_THRESHOLD_DEG     = 60.0;
    private static final double X_OFFSET_IN           = 20.0;
    private static final double Y_OFFSET_IN           = 48.0;

    // --- Shooter / PIDF ---
    private static final int TICKS_PER_REV = 28;
    private static final double MOTOR_FREE_RPM = 6000.0;
    private static final double MOTOR_FREE_RPS = MOTOR_FREE_RPM / 60.0;
    private static final double MAX_TICKS_PER_SEC = MOTOR_FREE_RPS * TICKS_PER_REV;
    private static final double PIDF_P = 0.0020;
    private static final double PIDF_I = 0.00010;
    private static final double PIDF_D = 0.00030;
    private static final double PIDF_F_BASE = 12.0 / MAX_TICKS_PER_SEC;

    // Shooter timing
    private static final double SHOOTER_SPEED_TIMEOUT_SEC = 2.0;
    private static final double GATE_OPEN_TIME  = 0.25;
    private static final double GATE_CLOSE_TIME = 0.12;

    // Ballistics
    private static final double R_WHEEL = 0.048;   // m (96 mm)
    private static final double SLIP    = 1.28;    // tuned
    private static final double Y_TARGET= 1.00;    // m
    private static final double Y_EXIT  = 0.3556;  // m (~14")
    private static final double DELTA_Y = Y_TARGET - Y_EXIT;
    private static final double GEAR_RATIO = 1.5;  // motor/wheel

    // Boost
    private static final double BOOST_FACTOR   = 1.07;
    private static final double BOOST_TIME_SEC = 0.25;
    private boolean boostActive = false; private double boostStartTime = 0.0;

    // Dynamics & sequence
    private double targetTicksPerSec = 900.0;
    private double computedDistanceInch = 0.0;
    private double computedV0 = 0.0;
    private double computedWheelRPM = 0.0;
    private double computedMotorRPM = 0.0;

    private int initialSide = 1; // +1 Blue, -1 Red
    private Pose startPose = null;
    private Pose shootPose = null;
    private int foundID = -1;
    private int backdropId = -1;
    private char[] sequence = {'P','P','P'};
    private int shotIndex = 0;

    // Paths
    private PathChain pathToShoot = null;
    private PathChain pathToPark  = null;
    private PathChain pathReturnToShoot = null;

    private static final double FINE_AIM_EPS_IN   = 0.25;
    private static final double FINE_AIM_TIMEOUT  = 0.40;
    private static final boolean UPDATE_SPEED_ON_FINE_AIM = true;

    // Centerline enforcement
    private static final double CENTER_X = 72.0;
    private boolean isBlueSide() { return initialSide > 0; }
    private double clampToOwnHalf(double x) {
        return isBlueSide() ? Math.min(x, CENTER_X - 0.5) : Math.max(x, CENTER_X + 0.5);
    }

    // Ball positions (own half)
    private static final Pose[] BALLS_BLUE = new Pose[] {
        new Pose(19.0, 84.0, 0.0), new Pose(24.0, 84.0, 0.0), new Pose(29.0, 84.0, 0.0)
    };
    private static final Pose[] BALLS_RED  = new Pose[] {
        new Pose(115.0, 84.0, 0.0), new Pose(120.0, 84.0, 0.0), new Pose(125.0, 84.0, 0.0)
    };
    private int ballIdx = 0;
    private final boolean[] picked = new boolean[3];

    // Separator & quota
    private static final double SEP_LEFT_GREEN   = 0.30;
    private static final double SEP_RIGHT_PURPLE = 0.80;
    private enum BallColor { PURPLE, GREEN, UNKNOWN }
    private static final int MAX_PURPLE = 2;
    private static final int MAX_GREEN  = 1;
    private int purpleCount = 0;
    private int greenCount  = 0;
    private BallColor lastCommandedColor = BallColor.UNKNOWN;

    // Ball counting (REV Distance)
    private int    ballsBaseline       = 0;
    private int    ballsNow            = 0;
    private boolean ballPresencePrev   = false;
    private double lastPresenceTs      = 0.0;
    private double lastBallRegisteredTs= -10.0;
    private static final double PRESENCE_DEBOUNCE_S       = 0.15;
    private static final double MIN_TIME_BETWEEN_BALLS_S  = 0.35;

    // Vision homing / search
    private static final double CAM_TILT_DOWN_POS = 0.20;
    private static final double CAM_TILT_UP_POS   = 0.60;
    private static final double VISION_X_TOL_PX   = 24;
    private static final double VISION_STEP_IN    = 4.0;
    private static final double VISION_TIMEOUT_S  = 2.5;

    private static final double SCAN_MAX_TIME_S     = 3.5;
    private static final double SCAN_STEP_TIMEOUT_S = 0.6;
    private static final double[] SCAN_YAW_DEG      = { 0, +20, -20, +40, -40, +60, -60, +80, -80 };
    private static final double[] SCAN_TILT_POS     = { 0.18, 0.24, 0.30 };
    private int    scanYawIdx       = 0;
    private int    scanTiltIdx      = 0;
    private double scanStartHeading = 0.0;
    private double scanStartTime    = 0.0;

    private static final double TURN_EPS_IN = 0.20;

    // Re‑shoot
    private char[] postSequence = new char[0];
    private int postShotIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init hardware & follower
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        robot.outtakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        double fComp = compensatedF(PIDF_F_BASE);
        robot.outtakeLeft.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, fComp);
        robot.outtakeRight.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, fComp);

        // Vision init
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            ballProc = new BallDetectionProcessor();
            VisionPortal.Builder vpBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(ballProc);
            visionPortal = vpBuilder.build();
            visionPortal.setProcessorEnabled(ballProc, false);
            try { aprilTag.setDecimation(2.0f); } catch (Exception ignored) {}
        } catch (Exception e) { aprilTag = null; visionPortal = null; }

        // INIT: compute poses (no motion)
        while (opModeInInit()) {
            handleSetup();
            if (startPose != null) follower.setStartingPose(startPose);
            if (startPose != null && shootPose != null) {
                double dx = shootPose.getX() - startPose.getX();
                double dy = shootPose.getY() - startPose.getY();
                computedDistanceInch = Math.hypot(dx, dy);
                computeBallisticSpeed(computedDistanceInch);
                if (pathToShoot == null) {
                    Pose shootClamped = new Pose(
                        clampToOwnHalf(shootPose.getX()), shootPose.getY(), shootPose.getHeading());
                    pathToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), shootClamped))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootClamped.getHeading())
                        .build();
                }
            }
            telemetry.addData("INIT Side", (initialSide>0)?"Blue":"Red");
            telemetry.addData("ShootPose", shootPose);
            telemetry.update();
        }

        // START
        waitForStart(); overall.reset(); setShooterVelocityTicks(targetTicksPerSec);
        if (pathToShoot != null) { follower.followPath(pathToShoot); currentState = State.WAIT_FOR_MOVE; }
        else { currentState = State.CHECK_SPEED; }
        timer.reset();

        // MAIN LOOP
        while (opModeIsActive() && !isStopRequested()) {
            follower.update(); updateBoostState();

            switch (currentState) {
                // Initial shooting
                case WAIT_FOR_MOVE:
                    if (!follower.isBusy()) { currentState = State.FINE_AIM; timer.reset(); }
                    break;
                case FINE_AIM:
                    startFineAim(); currentState = State.WAIT_FOR_FINE_AIM; timer.reset();
                    break;
                case WAIT_FOR_FINE_AIM:
                    if (!follower.isBusy() || timer.seconds()>FINE_AIM_TIMEOUT) { currentState = State.CHECK_SPEED; timer.reset(); }
                    break;
                case CHECK_SPEED:
                    if (isShooterReady() || timer.seconds()>SHOOTER_SPEED_TIMEOUT_SEC) { applyBoost(); currentState = State.FIRE; timer.reset(); }
                    break;
                case FIRE:
                    fireBall(sequence[shotIndex]); currentState = State.NEXT_SHOT; timer.reset();
                    break;
                case NEXT_SHOT:
                    if (timer.seconds() > (GATE_OPEN_TIME + GATE_CLOSE_TIME)) {
                        shotIndex++;
                        if (shotIndex < sequence.length) { currentState = State.CHECK_SPEED; }
                        else {
                            // Start collection
                            ballsBaseline = ballsNow; ballIdx = 0;
                            try { if (robot.cameraTilt!=null) robot.cameraTilt.setPosition(CAM_TILT_DOWN_POS);} catch(Exception ignored){}
                            if (visionPortal!=null && ballProc!=null) visionPortal.setProcessorEnabled(ballProc,true);
                            try { if (robot.intake!=null) robot.intake.setPower(1.0);} catch(Exception ignored){}
                            gotoBallIndex(ballIdx); currentState = State.WAIT_GOTO; timer.reset();
                        }
                    }
                    break;

                // Collection
                case WAIT_GOTO:
                    if (!follower.isBusy()) { currentState = State.VISION_HOME; timer.reset(); }
                    else if (shouldBailToReshootNow()) { beginReturnToShoot(); }
                    break;
                case VISION_HOME: {
                    updateBallCounter(overall.seconds());
                    if (shouldBailToReshootNow()) { beginReturnToShoot(); break; }
                    org.opencv.core.Rect r = getBestRectFor(neededColor());
                    if (r == null) { currentState = State.SEARCH_INIT; timer.reset(); break; }
                    BallColor seen = classifyRect(r); setSeparatorFor(seen); lastCommandedColor = seen;
                    double cx = imgCenterX(); double rx = r.x + r.width/2.0; double xErr = rx - cx;
                    if (Math.abs(xErr) > VISION_X_TOL_PX) { nudgeStrafe(Math.signum(xErr) * (VISION_STEP_IN*0.6)); }
                    else { setSeparatorFor(lastCommandedColor); nudgeForward(VISION_STEP_IN); }
                    currentState = State.PICK_WAIT; timer.reset();
                    break; }
                case PICK_WAIT: {
                    updateBallCounter(overall.seconds());
                    boolean gotNew = (ballsNow >= ballsBaseline + purpleCount + greenCount + 1);
                    boolean timeout = (timer.seconds()>2.0);
                    if (gotNew) {
                        if (ballIdx>=0 && ballIdx<3) picked[ballIdx] = true;
                        if (lastCommandedColor==BallColor.GREEN) greenCount++;
                        else if (lastCommandedColor==BallColor.PURPLE) purpleCount++;
                        else { if (greenCount<MAX_GREEN) greenCount++; else purpleCount++; }
                        if (shouldBailToReshootNow()) { beginReturnToShoot(); break; }
                        currentState = State.NEXT_BALL; timer.reset();
                    } else if (timeout) { currentState = State.VISION_HOME; timer.reset(); }
                    break; }
                case NEXT_BALL:
                    if (purpleCount>=MAX_PURPLE && greenCount>=MAX_GREEN) {
                        if (hasTimeForReshoot(purpleCount+greenCount)) { beginReturnToShoot(); }
                        else { goSafePark(); currentState = State.WAIT_FOR_PARK; timer.reset(); }
                    } else {
                        ballIdx++;
                        if (ballIdx<3) { gotoBallIndex(ballIdx); currentState = State.WAIT_GOTO; timer.reset(); }
                        else { currentState = State.SEARCH_INIT; timer.reset(); }
                    }
                    break;

                // Search
                case SEARCH_INIT: {
                    if (shouldBailToReshootNow()) { beginReturnToShoot(); break; }
                    Pose cur = follower.getPose(); if (cur==null) { currentState = State.VISION_HOME; break; }
                    scanStartHeading = cur.getHeading(); scanStartTime = overall.seconds(); scanYawIdx=0; scanTiltIdx=0;
                    try { if (robot.cameraTilt!=null) robot.cameraTilt.setPosition(SCAN_TILT_POS[scanTiltIdx]); } catch(Exception ignored){}
                    double desired = scanStartHeading + Math.toRadians(SCAN_YAW_DEG[scanYawIdx]);
                    rotateToHeading(desired); currentState = State.SEARCH_TURN; timer.reset();
                    break; }
                case SEARCH_TURN:
                    if (!follower.isBusy() || timer.seconds()>SCAN_STEP_TIMEOUT_S) { currentState = State.SEARCH_CHECK; timer.reset(); }
                    else if (shouldBailToReshootNow()) { beginReturnToShoot(); }
                    break;
                case SEARCH_CHECK: {
                    if (shouldBailToReshootNow()) { beginReturnToShoot(); break; }
                    org.opencv.core.Rect r = getBestRectFor(neededColor()); if (r!=null) { currentState=State.VISION_HOME; timer.reset(); break; }
                    if (overall.seconds()-scanStartTime > SCAN_MAX_TIME_S) { currentState=State.NEXT_BALL; timer.reset(); break; }
                    scanYawIdx++; if (scanYawIdx>=SCAN_YAW_DEG.length) {
                        scanYawIdx=0; scanTiltIdx = Math.min(scanTiltIdx+1, SCAN_TILT_POS.length-1);
                        try { if (robot.cameraTilt!=null) robot.cameraTilt.setPosition(SCAN_TILT_POS[scanTiltIdx]); } catch(Exception ignored){}
                        nudgeForward(VISION_STEP_IN*0.8);
                    }
                    double desired = scanStartHeading + Math.toRadians(SCAN_YAW_DEG[scanYawIdx]);
                    rotateToHeading(desired); currentState=State.SEARCH_TURN; timer.reset();
                    break; }

                // Re‑shoot
                case WAIT_RETURN_TO_SHOOT:
                    if (!follower.isBusy()) { currentState = State.POST_FINE_AIM; timer.reset(); }
                    if (isOutOfTimeSoon(0.5)) { goSafePark(); currentState = State.WAIT_FOR_PARK; timer.reset(); }
                    break;
                case POST_FINE_AIM:
                    startFineAim(); currentState = State.POST_WAIT_FOR_FINE_AIM; timer.reset();
                    break;
                case POST_WAIT_FOR_FINE_AIM:
                    if (!follower.isBusy() || timer.seconds()>FINE_AIM_TIMEOUT) { currentState = State.POST_SHOOT_CHECK_SPEED; timer.reset(); }
                    if (isOutOfTimeSoon(EST_SHOT_CYCLE_S * (postSequence.length - postShotIndex))) { goSafePark(); currentState = State.WAIT_FOR_PARK; timer.reset(); }
                    break;
                case POST_SHOOT_CHECK_SPEED:
                    if (isShooterReady() || timer.seconds()>SHOOTER_SPEED_TIMEOUT_SEC) { applyBoost(); currentState = State.POST_FIRE; timer.reset(); }
                    if (isOutOfTimeSoon(EST_SHOT_CYCLE_S * (postSequence.length - postShotIndex))) { goSafePark(); currentState = State.WAIT_FOR_PARK; timer.reset(); }
                    break;
                case POST_FIRE:
                    if (postShotIndex < postSequence.length) fireBall(postSequence[postShotIndex]);
                    currentState = State.POST_NEXT_SHOT; timer.reset();
                    break;
                case POST_NEXT_SHOT:
                    if (timer.seconds() > (GATE_OPEN_TIME + GATE_CLOSE_TIME)) {
                        postShotIndex++;
                        if (postShotIndex < postSequence.length) currentState = State.POST_SHOOT_CHECK_SPEED;
                        else { goSafePark(); currentState = State.WAIT_FOR_PARK; timer.reset(); }
                    }
                    break;

                // Park & End
                case WAIT_FOR_PARK:
                    if (!follower.isBusy() || timer.seconds()>3.0) currentState = State.END;
                    break;
                case END:
                    robot.outtakeLeft.setPower(0); robot.outtakeRight.setPower(0);
                    Pose finalPose = follower.getPose();
                    if (finalPose!=null) { PoseStorage.savePoseToFile(finalPose, initialSide); OpModeData.lastPose = finalPose; OpModeData.initialSide = initialSide; }
                    requestOpModeStop();
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData("t/Remain", String.format("%.2f / %.2f", overall.seconds(), (AUTON_TIME_BUDGET_S - overall.seconds())));
            telemetry.addData("Picked P/G", String.format("%d / %d", purpleCount, greenCount));
            telemetry.addData("BallsNow", ballsNow);
            telemetry.update();
        }

        if (visionPortal!=null) visionPortal.close();
    }

    // --- Early bail / time budget helpers ---
    private double estimateReshootSecondsFor(int balls) {
        Pose cur = follower.getPose(); if (cur==null) return 2.0;
        Pose shootClamped = new Pose(clampToOwnHalf(shootPose.getX()), shootPose.getY(), shootPose.getHeading());
        double dx = shootClamped.getX() - cur.getX(); double dy = shootClamped.getY() - cur.getY();
        double distIn = Math.hypot(dx, dy);
        double tMove = distIn / Math.max(10.0, AVG_TRANSIT_IN_PER_S);
        double tShots = Math.max(0, balls) * EST_SHOT_CYCLE_S;
        return tMove + EST_FINE_AIM_S + tShots;
    }
    private double nearestRemainingBallDistanceIn() {
        Pose cur = follower.getPose(); if (cur==null) return Double.POSITIVE_INFINITY;
        Pose[] balls = isBlueSide()?BALLS_BLUE:BALLS_RED; double best = Double.POSITIVE_INFINITY;
        for (int i=0;i<3;i++) {
            if (picked[i]) continue;
            Pose b = balls[i]; double dx = clampToOwnHalf(b.getX()) - cur.getX(); double dy = b.getY() - cur.getY();
            best = Math.min(best, Math.hypot(dx, dy));
        }
        return best;
    }
    private double estimateCollectOneMoreSeconds() {
        double distIn = nearestRemainingBallDistanceIn();
        if (!Double.isFinite(distIn)) return Double.POSITIVE_INFINITY;
        double tMove = distIn / Math.max(10.0, AVG_TRANSIT_IN_PER_S);
        return tMove + EST_HOMING_S;
    }
    private boolean shouldBailToReshootNow() {
        int collected = purpleCount + greenCount;
        if (collected <= 0) return false;
        double tReshootNow = estimateReshootSecondsFor(collected);
        boolean fitsNow = (overall.seconds() + tReshootNow + SAFETY_MARGIN_S) <= AUTON_TIME_BUDGET_S;
        if (!fitsNow) return false;
        if (collected >= 3) return true;
        double tCollect1 = estimateCollectOneMoreSeconds();
        double tReshootNext = estimateReshootSecondsFor(collected + 1);
        boolean fitsWithOneMore = (overall.seconds() + tCollect1 + tReshootNext + SAFETY_MARGIN_S) <= AUTON_TIME_BUDGET_S;
        return !fitsWithOneMore;
    }
    private boolean hasTimeForReshoot(int balls) {
        double est = estimateReshootSecondsFor(balls);
        return (overall.seconds() + est + SAFETY_MARGIN_S) <= AUTON_TIME_BUDGET_S;
    }

    /** Global time‑budget guard */
    private boolean isOutOfTimeSoon(double upcomingSeconds) {
        return (overall.seconds() + upcomingSeconds + SAFETY_MARGIN_S) > AUTON_TIME_BUDGET_S;
    }

    /** Centerline‑safe park */
    private void goSafePark() {
        Pose cur = follower.getPose(); if (cur == null) cur = new Pose(clampToOwnHalf(CENTER_X - 12), 84, 0);
        double targetX = isBlueSide() ? 18.0 : 144.0 - 18.0;
        double targetY = cur.getY();
        Pose park = new Pose(clampToOwnHalf(targetX), targetY, cur.getHeading());
        pathToPark = follower.pathBuilder().addPath(new BezierLine(cur, park)).build();
        follower.followPath(pathToPark);
    }

    /**
     * MISSING BEFORE: Begin the return‑to‑shoot flow and prepare postSequence
     * from current counts; also tidy vision/intake state.
     */
    private void beginReturnToShoot() {
        // 1) Build post sequence from current collected counts
        postSequence = buildPostShootSequence(purpleCount, greenCount);
        postShotIndex = 0;

        // 2) Compute ballistic target from CURRENT pose to shooting pose
        Pose cur = follower.getPose();
        if (cur == null) cur = new Pose(clampToOwnHalf(CENTER_X - 12), 84, 0);
        Pose shootClamped = new Pose(clampToOwnHalf(shootPose.getX()), shootPose.getY(), shootPose.getHeading());
        double dx = shootClamped.getX() - cur.getX();
        double dy = shootClamped.getY() - cur.getY();
        computedDistanceInch = Math.hypot(dx, dy);
        computeBallisticSpeed(computedDistanceInch);
        setShooterVelocityTicks(targetTicksPerSec);

        // 3) Path and state transition
        pathReturnToShoot = follower.pathBuilder()
            .addPath(new BezierLine(cur, shootClamped))
            .setLinearHeadingInterpolation(cur.getHeading(), shootClamped.getHeading())
            .build();
        follower.followPath(pathReturnToShoot);
        currentState = State.WAIT_RETURN_TO_SHOOT;
        timer.reset();

        // 4) Vision/intake housekeeping
        try { if (robot.cameraTilt!=null) robot.cameraTilt.setPosition(CAM_TILT_UP_POS);} catch(Exception ignored){}
        if (visionPortal!=null && ballProc!=null) visionPortal.setProcessorEnabled(ballProc,false);
        try { if (robot.intake!=null) robot.intake.setPower(0.0);} catch(Exception ignored){}
    }

    /**
     * Build a P/G sequence from current counts, alternating to reduce drop.
     * E.g., 2P+1G -> P,G,P
     */
    private char[] buildPostShootSequence(int purples, int greens) {
        List<Character> out = new ArrayList<>();
        char pc='P', gc='G'; int p=purples, g=greens;
        boolean takePurple = (p>=g);
        while (p>0 || g>0) {
            if (takePurple && p>0) { out.add(pc); p--; }
            else if (!takePurple && g>0) { out.add(gc); g--; }
            takePurple = !takePurple;
            if (p==0 && g>0) { while (g-->0) out.add(gc); }
            if (g==0 && p>0) { while (p-->0) out.add(pc); }
        }
        char[] arr = new char[out.size()];
        for (int i=0;i<out.size();i++) arr[i] = out.get(i);
        return arr;
    }

    // --- AprilTag setup & shooting pose ---
    private void handleSetup() {
        List<AprilTagDetection> dets = (aprilTag!=null)?aprilTag.getDetections():Collections.emptyList();
        AprilTagDetection obelisk=null, backdrop=null; double bestRange=Double.MAX_VALUE;
        for (AprilTagDetection d : dets) {
            if (d==null || d.metadata==null) continue;
            if (d.id>=21 && d.id<=23) obelisk=d;
            if (d.id==BLUE_GOAL_TAG_ID || d.id==RED_GOAL_TAG_ID) {
                double r = (d.ftcPose!=null)?d.ftcPose.range:Double.MAX_VALUE; if (r<bestRange){bestRange=r; backdrop=d;}
            }
        }
        if (obelisk==null || backdrop==null) return;
        initialSide = (obelisk.ftcPose.x<0)?-1:1; foundID=obelisk.id; backdropId=backdrop.id; sequence=getSequenceForID(foundID);
        Pose landmarkPose = (backdrop.id==BLUE_GOAL_TAG_ID)?BLUE_BACKDROP_POSE:RED_BACKDROP_POSE;
        double yawDeg = backdrop.ftcPose.yaw; double yawRad=Math.toRadians(yawDeg);
        double robotHeadingRad = landmarkPose.getHeading()-yawRad;
        double relX=backdrop.ftcPose.x, relY=backdrop.ftcPose.y;
        double worldX = relX*Math.cos(robotHeadingRad) - relY*Math.sin(robotHeadingRad);
        double worldY = relX*Math.sin(robotHeadingRad) + relY*Math.cos(robotHeadingRad);
        double camX = landmarkPose.getX()-worldX; double camY = landmarkPose.getY()-worldY;
        double robotX = camX + CAMERA_FORWARD_OFFSET*Math.sin(robotHeadingRad);
        double robotY = camY - CAMERA_FORWARD_OFFSET*Math.cos(robotHeadingRad);
        startPose = new Pose(robotX, robotY, robotHeadingRad);
        Pose correctLandmark = (initialSide>0)?BLUE_BACKDROP_POSE:RED_BACKDROP_POSE;
        Pose baseLandmark = (Math.abs(yawDeg)>YAW_THRESHOLD_DEG)?correctLandmark:landmarkPose;
        double targetX = baseLandmark.getX() - initialSide*X_OFFSET_IN; double targetY = baseLandmark.getY() - Y_OFFSET_IN;
        shootPose = new Pose(targetX, targetY, baseLandmark.getHeading());
    }

    private char[] getSequenceForID(int id) {
        switch (id) {
            case 21: return new char[]{'G','P','P'};
            case 22: return new char[]{'P','G','P'};
            case 23: return new char[]{'P','P','G'};
            default: return new char[]{'P','P','P'};
        }
    }

    // --- Fine aim ---
    private Pose getBackdropPoseForAim() {
        AprilTagDetection bd = getClosestBackdrop();
        if (bd!=null) return (bd.id==BLUE_GOAL_TAG_ID)?BLUE_BACKDROP_POSE:RED_BACKDROP_POSE;
        return (initialSide>0)?BLUE_BACKDROP_POSE:RED_BACKDROP_POSE;
    }
    private AprilTagDetection getClosestBackdrop() {
        if (aprilTag==null) return null; List<AprilTagDetection> dets = aprilTag.getDetections();
        AprilTagDetection best=null; double bestRange=Double.MAX_VALUE;
        for (AprilTagDetection d: dets) {
            if (d==null || d.metadata==null) continue; if (d.id==BLUE_GOAL_TAG_ID || d.id==RED_GOAL_TAG_ID) {
                double r = (d.ftcPose!=null)?d.ftcPose.range:Double.MAX_VALUE; if (r<bestRange){bestRange=r; best=d;}
            }
        }
        return best;
    }
    private double computePerpendicularHeading() { return getBackdropPoseForAim().getHeading(); }
    private void startFineAim() {
        Pose current = follower.getPose(); if (current==null) return; double desired = computePerpendicularHeading();
        if (UPDATE_SPEED_ON_FINE_AIM) {
            Pose landmark = getBackdropPoseForAim();
            double targetX = landmark.getX() - initialSide*X_OFFSET_IN; double targetY = landmark.getY() - Y_OFFSET_IN;
            double dx = targetX-current.getX(); double dy = targetY-current.getY();
            computedDistanceInch = Math.hypot(dx, dy); computeBallisticSpeed(computedDistanceInch); setShooterVelocityTicks(targetTicksPerSec);
        }
        double eps = FINE_AIM_EPS_IN; Pose tgt = new Pose(
            clampToOwnHalf(current.getX()+eps*Math.cos(desired)), current.getY()+eps*Math.sin(desired), desired);
        PathChain fine = follower.pathBuilder().addPath(new BezierLine(current, tgt))
            .setLinearHeadingInterpolation(current.getHeading(), desired).build();
        follower.followPath(fine);
    }

    // --- Shooter helpers ---
    private void computeBallisticSpeed(double distanceInch) {
        double x = distanceInch*0.0254; double g=9.81; double denom = x-DELTA_Y; if (denom<=0.05) denom=0.05;
        computedV0 = Math.sqrt((g*x*x)/denom); double vWheel = computedV0*SLIP; double rpsWheel = vWheel/(2.0*Math.PI*R_WHEEL);
        computedWheelRPM = rpsWheel*60.0; computedMotorRPM = computedWheelRPM*GEAR_RATIO; double motorRPS = computedMotorRPM/60.0;
        targetTicksPerSec = motorRPS*TICKS_PER_REV;
    }
    private void fireBall(char type) {
        if (type=='G') robot.servoInL.setPower(1.0); else robot.servoInR.setPower(1.0);
        sleep((long)(GATE_OPEN_TIME*1000)); robot.servoInL.setPower(0.0); robot.servoInR.setPower(0.0);
        sleep((long)(GATE_CLOSE_TIME*1000));
    }
    private void applyBoost() { robot.outtakeLeft.setVelocity(targetTicksPerSec*BOOST_FACTOR); robot.outtakeRight.setVelocity(targetTicksPerSec*BOOST_FACTOR); boostActive=true; boostStartTime=overall.seconds(); }
    private void updateBoostState() { if (boostActive && overall.seconds()-boostStartTime>BOOST_TIME_SEC) { setShooterVelocityTicks(targetTicksPerSec); boostActive=false; } }
    private void setShooterVelocityTicks(double ticksPerSec) { robot.outtakeLeft.setVelocity(ticksPerSec); robot.outtakeRight.setVelocity(ticksPerSec); }
    private boolean isShooterReady() { double vL=robot.outtakeLeft.getVelocity(), vR=robot.outtakeRight.getVelocity(); double tol=0.92; return (vL>=targetTicksPerSec*tol)&&(vR>=targetTicksPerSec*tol); }
    private double compensatedF(double baseF) { double volts=getBatteryVoltage(); return baseF*(12.0/Math.max(10.0,volts)); }
    private double getBatteryVoltage() { double v=12.0; try{ for(VoltageSensor vs:hardwareMap.voltageSensor){ if (vs!=null){ v=vs.getVoltage(); break; } } }catch(Exception ignored){} return v; }

    // --- Half‑safe nudges & turns ---
    private void nudgeForward(double inches) {
        Pose cur=follower.getPose(); if (cur==null) return; double hdg=cur.getHeading();
        Pose tgt=new Pose(clampToOwnHalf(cur.getX()+inches*Math.cos(hdg)), cur.getY()+inches*Math.sin(hdg), hdg);
        PathChain chain=follower.pathBuilder().addPath(new BezierLine(cur,tgt)).setLinearHeadingInterpolation(hdg, hdg).build(); follower.followPath(chain);
    }
    private void nudgeStrafe(double inches) {
        Pose cur=follower.getPose(); if (cur==null) return; double hdg=cur.getHeading();
        Pose tgt=new Pose(clampToOwnHalf(cur.getX()+inches*Math.cos(hdg+Math.PI/2.0)), cur.getY()+inches*Math.sin(hdg+Math.PI/2.0), hdg);
        PathChain chain=follower.pathBuilder().addPath(new BezierLine(cur,tgt)).setLinearHeadingInterpolation(hdg, hdg).build(); follower.followPath(chain);
    }
    private void rotateToHeading(double desiredHeading) {
        Pose cur=follower.getPose(); if (cur==null) return;
        Pose tgt=new Pose(clampToOwnHalf(cur.getX()+TURN_EPS_IN*Math.cos(desiredHeading)), cur.getY()+TURN_EPS_IN*Math.sin(desiredHeading), desiredHeading);
        PathChain turn=follower.pathBuilder().addPath(new BezierLine(cur,tgt)).setLinearHeadingInterpolation(cur.getHeading(), desiredHeading).build(); follower.followPath(turn);
    }

    // --- Ball helpers ---
    private Pose[] getOwnSideBalls() { return isBlueSide()?BALLS_BLUE:BALLS_RED; }
    private void gotoBallIndex(int idx) {
        Pose[] balls=getOwnSideBalls(); idx=Math.max(0,Math.min(idx,balls.length-1)); Pose cur=follower.getPose(); if (cur==null) return;
        Pose raw=balls[idx]; Pose tgt=new Pose(clampToOwnHalf(raw.getX()), raw.getY(), cur.getHeading());
        PathChain path=follower.pathBuilder().addPath(new BezierLine(cur,tgt)).setLinearHeadingInterpolation(cur.getHeading(), cur.getHeading()).build(); follower.followPath(path);
    }

    /** Image center fallback (320 for 640x480). */
    private int imgCenterX() { return 320; }

    private org.opencv.core.Rect getBestRectFor(BallColor need) {
        if (ballProc==null) return null; org.opencv.core.Rect pr=ballProc.getBestPurpleContourRect(); org.opencv.core.Rect gr=ballProc.getBestGreenContourRect();
        if (need==BallColor.PURPLE) return pr; if (need==BallColor.GREEN) return gr;
        if (pr==null && gr==null) return null; if (pr!=null && gr==null) return pr; if (pr==null && gr!=null) return gr;
        int cx=imgCenterX(); double dxP=Math.abs((pr.x+pr.width/2.0)-cx); double dxG=Math.abs((gr.x+gr.width/2.0)-cx); return (dxP<=dxG)?pr:gr;
    }

    private BallColor classifyRect(org.opencv.core.Rect r) {
        if (r==null || ballProc==null) return BallColor.UNKNOWN; org.opencv.core.Rect pr=ballProc.getBestPurpleContourRect(); org.opencv.core.Rect gr=ballProc.getBestGreenContourRect();
        if (pr!=null && r==pr) return BallColor.PURPLE; if (gr!=null && r==gr) return BallColor.GREEN;
        double rx=r.x+r.width/2.0; if (pr!=null){double px=pr.x+pr.width/2.0; if (Math.abs(px-rx)<1e-6) return BallColor.PURPLE;}
        if (gr!=null){double gx=gr.x+gr.width/2.0; if (Math.abs(gx-rx)<1e-6) return BallColor.GREEN;}
        double ap=(pr!=null)?pr.width*pr.height:-1; double ag=(gr!=null)?gr.width*gr.height:-1; if (ap<0 && ag<0) return BallColor.UNKNOWN; return (ap>=ag)?BallColor.PURPLE:BallColor.GREEN;
    }

    private BallColor neededColor() {
        if (greenCount<MAX_GREEN) return BallColor.GREEN; if (purpleCount<MAX_PURPLE) return BallColor.PURPLE; return BallColor.UNKNOWN;
    }

    private void setSeparatorFor(BallColor c) { if (robot.separator==null) return; if (c==BallColor.GREEN) robot.separator.setPosition(SEP_LEFT_GREEN); if (c==BallColor.PURPLE) robot.separator.setPosition(SEP_RIGHT_PURPLE); }

    private void updateBallCounter(double nowSeconds) {
        double mm=9999.0; try{ mm=robot.sensorDistance.getDistance(DistanceUnit.MM);}catch(Exception ignored){}
        boolean present = mm>0 && mm<120;
        if (present) {
            if (!ballPresencePrev) { lastPresenceTs=nowSeconds; }
            else {
                boolean stable=(nowSeconds-lastPresenceTs)>=PRESENCE_DEBOUNCE_S; boolean spaced=(nowSeconds-lastBallRegisteredTs)>=MIN_TIME_BETWEEN_BALLS_S;
                if (stable && spaced) { ballsNow++; lastBallRegisteredTs=nowSeconds; lastPresenceTs=nowSeconds+1e9; }
            }
        }
        ballPresencePrev=present;
    }
}
