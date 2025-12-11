package org.firstinspires.ftc.teamcode;

// Autonomous NexGen – Ballistic Boost + Sequence (Tuned + Tuning Guide) + TelemetryPanel
// After INIT: all calculations (pose, motif, ballistic, paths) + camera profiles
// After START: move, shut, park, live telemetry

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;

import java.util.Collections;
import java.util.List;

@Autonomous(name = "Autonomous StarTech V1", group = "Opmode")

public class AutonomousStarTechNew extends LinearOpMode {

    // --- Robot & path follower ---
    HardwareBox robot = new HardwareBox();
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- Telemetry panel ---
    private TelemetryPanel panel;

    // --- State machine ---
    private enum State { SETUP, MOVE_TO_SHOOT, WAIT_FOR_MOVE, FINE_AIM, WAIT_FOR_FINE_AIM, CHECK_SPEED, FIRE, NEXT_SHOT, PARK, WAIT_FOR_PARK, END }
    private State currentState = State.SETUP;
    private final ElapsedTime timer = new ElapsedTime();

    // --- Field landmark poses & IDs ---
    private static final Pose BLUE_BACKDROP_POSE = new Pose(16.01, 133.28, Math.toRadians(-126.0));
    private static final Pose RED_BACKDROP_POSE  = new Pose(127.99, 133.28, Math.toRadians(-54.0));
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;

    // --- Geometry & camera offset ---
    private static final double CAMERA_FORWARD_OFFSET = 7.0;  // camera ~7 in ahead of robot center
    private static final double YAW_THRESHOLD_DEG = 60.0;     // beyond this yaw, use correct landmark for shoot pose
    private static final double X_OFFSET_IN = 20.0;           // tuned lateral offset (inches)
    private static final double Y_OFFSET_IN = 48.0;           // tuned forward offset (inches)

    // --- Shooter / PIDF constants ---
    private static final int TICKS_PER_REV = 28;
    private static final double MOTOR_FREE_RPM = 6000.0;
    private static final double MOTOR_FREE_RPS = MOTOR_FREE_RPM / 60.0;
    private static final double MAX_TICKS_PER_SEC = MOTOR_FREE_RPS * TICKS_PER_REV;
    private static final double PIDF_P = 0.0020;
    private static final double PIDF_I = 0.00010;
    private static final double PIDF_D = 0.00030;
    private static final double PIDF_F_BASE = 12.0 / MAX_TICKS_PER_SEC; // base F scaled by battery voltage

    // --- Shooter timing ---
    private static final double SHOOTER_SPEED_TIMEOUT_SEC = 2.0;
    private static final double GATE_OPEN_TIME  = 0.25;  // seconds gate open
    private static final double GATE_CLOSE_TIME = 0.12;  // seconds gate closed

    // --- Ballistic constants ---
    private static final double R_WHEEL = 0.048; // m (96 mm diameter)
    private static final double SLIP = 1.28;     // tuned slip factor
    private static final double Y_TARGET = 1.00; // m (basket rim height)
    private static final double Y_EXIT   = 0.3556; // m (ball exit height ~14")
    private static final double DELTA_Y  = Y_TARGET - Y_EXIT;
    private static final double GEAR_RATIO = 1.5; // motor rev / wheel rev (tuned)

    // --- Boost parameters ---
    private static final double BOOST_FACTOR = 1.07; // tuned boost multiplier
    private static final double BOOST_TIME_SEC = 0.25; // seconds
    private boolean boostActive = false;
    private double boostStartTime = 0.0;

    // --- Dynamic targets & telemetry fields ---
    private double targetTicksPerSec = 900.0;
    private double computedDistanceInch = 0.0;
    private double computedV0 = 0.0;
    private double computedWheelRPM = 0.0;
    private double computedMotorRPM = 0.0;

    // --- Strategy & sequence ---
    private int initialSide = 1; // +1 Blue, -1 Red
    private Pose startPose = null;
    private Pose shootPose = null;
    private int foundID = -1; // obelisk ID 21..23
    private int backdropId = -1; // obelisk ID 21..23
    private char[] sequence = {'P','P','P'}; // fallback
    private int shotIndex = 0;

    // --- Paths ---
    private PathChain pathToShoot = null;
    private PathChain pathToPark  = null;
    private static final double FINE_AIM_EPS_IN = 0.25;
    private static final double FINE_AIM_TIMEOUT = 0.40;
    private static final boolean UPDATE_SPEED_ON_FINE_AIM = true;
    private double lastDesiredHeading = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware & follower init ---
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // shooter motor
        robot.outtakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        double fComp = compensatedF(PIDF_F_BASE);
        robot.outtakeLeft.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, fComp);
        robot.outtakeRight.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, fComp);

        // --- Vision init (AprilTag) ---
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

            try {
                aprilTag.setDecimation(2.0f);
            } catch (Exception ignored) {
                // A te SDK-dban lehet, hogy nincs setDecimation → marad az alapértelmezett
            }

        } catch (Exception e) {
            aprilTag = null; visionPortal = null;
        }

        // --- Kamera profil (erős fényre) + TelemetryPanel ---
        CameraProfiles.applyBrightLight(visionPortal, aprilTag);
        panel = new TelemetryPanel(telemetry, visionPortal, aprilTag);

        // --- INIT: detect tags, compute poses & sequence, ballistics, paths (no motion) ---
        while (opModeInInit()) {
            // Gyors profilváltás gamepaddal
            if (gamepad1.dpad_up)    CameraProfiles.applyBrightLight(visionPortal, aprilTag);
            if (gamepad1.dpad_left)  CameraProfiles.applyBalanced(visionPortal, aprilTag);
            if (gamepad1.dpad_down)  CameraProfiles.applyIndoor(visionPortal, aprilTag);

            handleSetup(); // meghatározza: initialSide, foundID, sequence, startPose, shootPose

            // follower kezdőpose beállítás (nem mozgat)
            if (startPose != null) {
                follower.setStartingPose(startPose);
            }

            // ha most lett meg mindkét pose, számoljuk ki a ballisztikát és készítsük elő az utakat
            if (startPose != null && shootPose != null) {
                // távolság + ballisztika INIT alatt
                double dx = shootPose.getX() - startPose.getX();
                double dy = shootPose.getY() - startPose.getY();
                computedDistanceInch = Math.sqrt(dx*dx + dy*dy);
                computeBallisticSpeed(computedDistanceInch);

                // Útvonal a lövési pozícióba (csak építés; NEM indítjuk!)
                if (pathToShoot == null) {
                    pathToShoot = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading())
                            .build();
                }
            }

            // --- Saját telemetria ---
            telemetry.addLine("=== INIT PRECOMPUTE ===");
            telemetry.addData("ObeliskID", foundID);
            telemetry.addData("BackdropId", backdropId);
            telemetry.addData("Sequence", new String(sequence));
            telemetry.addData("StartPose", startPose);
            telemetry.addData("ShootPose", shootPose);
            telemetry.addData("Distance (in)", computedDistanceInch);
            telemetry.addData("Ballistic v0 (m/s)", computedV0);
            telemetry.addData("Wheel RPM", computedWheelRPM);
            telemetry.addData("Motor RPM", computedMotorRPM);
            telemetry.addData("Target ticks/s", targetTicksPerSec);
            telemetry.addData("Battery V", getBatteryVoltage());

            // --- Telemetry panel (részletes) ---
            panel.update();
            panel.renderFull();
            telemetry.update();
        }

        // --- Start: innentől lehet mozogni ---
        waitForStart();

        // shooter célfordulat feladása
        setShooterVelocityTicks(targetTicksPerSec);

        // Mozgás a lövőpontra csak most indulhat
        if (pathToShoot != null) {
            follower.followPath(pathToShoot);
            currentState = State.WAIT_FOR_MOVE;
        } else {
            currentState = State.CHECK_SPEED; // fallback: ahol vagyunk, onnan lövünk
        }

        timer.reset();

        // --- Main loop: 3 lövés, majd parkolás ---
        while (opModeIsActive() && !isStopRequested()) {
            // profilváltás futás közben is
            if (gamepad1.dpad_up)    CameraProfiles.applyBrightLight(visionPortal, aprilTag);
            if (gamepad1.dpad_left)  CameraProfiles.applyBalanced(visionPortal, aprilTag);
            if (gamepad1.dpad_down)  CameraProfiles.applyIndoor(visionPortal, aprilTag);

            follower.update();
            updateBoostState();

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
                    if (!follower.isBusy() || timer.seconds() > FINE_AIM_TIMEOUT) {
                        currentState = State.CHECK_SPEED;
                        timer.reset();
                    }
                    break;

                case CHECK_SPEED:
                    // readiness by speed or timeout, then apply boost
                    if (isShooterReady() || timer.seconds() > SHOOTER_SPEED_TIMEOUT_SEC) {
                        applyBoost();
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
                    if (timer.seconds() > (GATE_OPEN_TIME + GATE_CLOSE_TIME)) {
                        shotIndex++;
                        if (shotIndex < sequence.length) {
                            currentState = State.CHECK_SPEED;
                        } else {
                            // park útvonal előkészítése és indítása (most már mozoghatunk)
                            Pose park = new Pose(
                                    follower.getPose().getX() + 25 * initialSide,
                                    follower.getPose().getY(),
                                    follower.getPose().getHeading()
                            );
                            pathToPark = follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), park))
                                    .build();
                            follower.followPath(pathToPark);
                            currentState = State.WAIT_FOR_PARK;
                            timer.reset();
                        }
                    }
                    break;

                case WAIT_FOR_PARK:
                    if (!follower.isBusy() || timer.seconds() > 3.0) {
                        currentState = State.END;
                    }
                    break;

                case END:
                    // stop shooter és pose mentése TeleOp számára
                    robot.outtakeLeft.setPower(0);
                    robot.outtakeRight.setPower(0);

                    Pose finalPose = follower.getPose();
                    if (finalPose != null) {
                        PoseStorage.savePoseToFile(finalPose, initialSide);
                        OpModeData.lastPose = finalPose;
                        OpModeData.initialSide = initialSide;
                    }
                    requestOpModeStop();
                    break;
            }

            // --- Saját telemetria ---
            telemetry.addData("State", currentState);
            telemetry.addData("ObeliskID", foundID);
            telemetry.addData("Sequence", new String(sequence));
            telemetry.addData("CurrentShot", shotIndex < sequence.length ? sequence[shotIndex] : '-');
            telemetry.addData("Distance (in)", computedDistanceInch);
            telemetry.addData("Ballistic v0 (m/s)", computedV0);
            telemetry.addData("Wheel RPM", computedWheelRPM);
            telemetry.addData("Motor RPM", computedMotorRPM);
            telemetry.addData("Target ticks/s", targetTicksPerSec);
            telemetry.addData("ShooterVel L", robot.outtakeLeft.getVelocity());
            telemetry.addData("ShooterVel R", robot.outtakeRight.getVelocity());
            telemetry.addData("Battery V", getBatteryVoltage());

            // --- Telemetry panel (minimal) ---
            panel.update();
            panel.renderMinimal();
            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    /**
     * Detect obelisk (21–23) and closest backdrop (20/24).
     * Decide side, sequence, compute start & shoot poses according to yaw rule.
     * (INIT alatt fut, többször hívódhat)
     */
    private void handleSetup() {
        List<AprilTagDetection> dets = (aprilTag != null) ? aprilTag.getDetections() : Collections.emptyList();

        AprilTagDetection obelisk = null;
        AprilTagDetection backdrop = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection d : dets) {
            if (d == null || d.metadata == null) continue;

            if (d.id >= 21 && d.id <= 23) obelisk = d;

            if (d.id == BLUE_GOAL_TAG_ID || d.id == RED_GOAL_TAG_ID) {
                double r = (d.ftcPose != null) ? d.ftcPose.range : Double.MAX_VALUE;
                if (r < bestRange) { bestRange = r; backdrop = d; }
            }
        }

        if (obelisk == null || backdrop == null) return;

        initialSide = (obelisk.ftcPose.x < 0) ? -1 : 1;
        foundID = obelisk.id;
        backdropId = backdrop.id;
        sequence = getSequenceForID(foundID);

        Pose landmarkPose = (backdrop.id == BLUE_GOAL_TAG_ID) ? BLUE_BACKDROP_POSE : RED_BACKDROP_POSE;
        double yawDeg = backdrop.ftcPose.yaw;
        double yawRad = Math.toRadians(yawDeg);

        // Compute start pose from seen backdrop
        double robotHeadingRad = landmarkPose.getHeading() - yawRad;
        double relX = backdrop.ftcPose.x, relY = backdrop.ftcPose.y;

        double worldX = relX * Math.cos(robotHeadingRad) - relY * Math.sin(robotHeadingRad);
        double worldY = relX * Math.sin(robotHeadingRad) + relY * Math.cos(robotHeadingRad);

        double camX = landmarkPose.getX() - worldX;
        double camY = landmarkPose.getY() - worldY;

        double robotX = camX + CAMERA_FORWARD_OFFSET * Math.sin(robotHeadingRad);
        double robotY = camY - CAMERA_FORWARD_OFFSET * Math.cos(robotHeadingRad);

        startPose = new Pose(robotX, robotY, robotHeadingRad);

        // Compute shooting pose from correct or seen landmark depending on yaw
        Pose correctLandmark = (initialSide > 0) ? BLUE_BACKDROP_POSE : RED_BACKDROP_POSE;
        Pose baseLandmark = (Math.abs(yawDeg) > YAW_THRESHOLD_DEG) ? correctLandmark : landmarkPose;

        double targetX = baseLandmark.getX() - initialSide * X_OFFSET_IN;
        double targetY = baseLandmark.getY() - Y_OFFSET_IN;
        double heading = baseLandmark.getHeading();

        shootPose = new Pose(targetX, targetY, heading);

        // INIT diagnosztika
        telemetry.addData("StartPose", startPose);
        telemetry.addData("ShootPose", shootPose);
        telemetry.addData("YawDeg", yawDeg);
        telemetry.addData("BackdropID", backdrop.id);
    }

    /** Return shot sequence by obelisk ID */
    private char[] getSequenceForID(int id) {
        switch (id) {
            case 21: return new char[]{'G','P','P'};
            case 22: return new char[]{'P','G','P'};
            case 23: return new char[]{'P','P','G'};
            default: return new char[]{'P','P','P'};
        }
    }

    /**
     * Válaszd ki a legközelebbi háttér taget (20/24), ha épp látszik.
     */
    private AprilTagDetection getClosestBackdrop() {
        if (aprilTag == null) return null;
        List<AprilTagDetection> dets = aprilTag.getDetections();
        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection d : dets) {
            if (d == null || d.metadata == null) continue;
            if (d.id == BLUE_GOAL_TAG_ID || d.id == RED_GOAL_TAG_ID) {
                double r = (d.ftcPose != null) ? d.ftcPose.range : Double.MAX_VALUE;
                if (r < bestRange) { bestRange = r; best = d; }
            }
        }
        return best;
    }

    /**
     * A finomcélzáshoz használt landmark (statikus póz): detektált 20/24 alapján,
     * ha nincs detektálás, az oldal (kék/piros) szerinti.
     */
    private Pose getBackdropPoseForAim() {
        AprilTagDetection backdrop = getClosestBackdrop();
        if (backdrop != null) {
            return (backdrop.id == BLUE_GOAL_TAG_ID) ? BLUE_BACKDROP_POSE : RED_BACKDROP_POSE;
        }
        return (initialSide > 0) ? BLUE_BACKDROP_POSE : RED_BACKDROP_POSE;
    }

    /**
     * Merőleges (normál) irány: a landmark headingje.
     */
    private double computePerpendicularHeading() {
        Pose landmark = getBackdropPoseForAim();
        return landmark.getHeading();
    }

    /**
     * Finomcélzás indítása: gyors heading-ráállás merőlegesen a háttér tagre.
     * - opcionálisan újraszámítjuk a sebességet a valós pose → offsetelt lövőpont távolság alapján.
     * - minimális "nudge" Bezier path a Pedro 2.0-hoz, hogy a heading-interpoláció determinisztikusan lefusson.
     */
    private void startFineAim() {
        Pose current = follower.getPose();
        if (current == null) return;

        // Merőleges (normál) irány a háttér tagre
        double desired = computePerpendicularHeading();
        lastDesiredHeading = desired;

        // Sebesség újraszámítása a valós helyről a landmarkhoz mért offsetelt lövőpontra
        if (UPDATE_SPEED_ON_FINE_AIM) {
            Pose landmark = getBackdropPoseForAim();
            double targetX = landmark.getX() - initialSide * X_OFFSET_IN;
            double targetY = landmark.getY() - Y_OFFSET_IN;
            double dx = targetX - current.getX();
            double dy = targetY - current.getY();
            computedDistanceInch = Math.hypot(dx, dy);
            computeBallisticSpeed(computedDistanceInch);
            setShooterVelocityTicks(targetTicksPerSec);
        }

        // Minimális elmozdulás (nudge), hogy a follower garantáltan végrehajtsa a heading-interpolációt
        double eps = FINE_AIM_EPS_IN;
        Pose target = new Pose(
                current.getX() + eps * Math.cos(desired),
                current.getY() + eps * Math.sin(desired),
                desired
        );

        PathChain fine = follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), desired)
                .build();

        follower.followPath(fine);

        /* Alternatíva (ha van direkt turn API a Pedro 2.0 buildetekben):
        follower.turnToHeading(desired);
        */
    }

    /** Compute ballistic target speed and derived telemetry fields (INIT alatt is hívjuk) */
    private void computeBallisticSpeed(double distanceInch) {
        double x = distanceInch * 0.0254; // inches → meters
        double g = 9.81; // m/s^2
        double denom = x - DELTA_Y; // account for height difference
        if (denom <= 0.05) denom = 0.05; // avoid near-zero division

        computedV0 = Math.sqrt((g * x * x) / denom);

        double vWheel = computedV0 * SLIP; // slip compensation
        double revPerSecWheel = vWheel / (2.0 * Math.PI * R_WHEEL);
        computedWheelRPM = revPerSecWheel * 60.0;
        computedMotorRPM = computedWheelRPM * GEAR_RATIO; // gearing to motor RPM
        double motorRevPerSec = computedMotorRPM / 60.0;

        targetTicksPerSec = motorRevPerSec * TICKS_PER_REV; // encoder ticks/s target
    }

    /** Open gate with correct CRServo for given shot type (G or P) */
    private void fireBall(char type) {
        if (type == 'G') robot.servoInL.setPower(1.0); else robot.servoInR.setPower(1.0);
        sleep((long)(GATE_OPEN_TIME * 1000));
        robot.servoInL.setPower(0.0); robot.servoInR.setPower(0.0);
        sleep((long)(GATE_CLOSE_TIME * 1000));
    }

    /** Apply short boost to reduce post-shot drop */
    private void applyBoost() {
        robot.outtakeLeft.setVelocity(targetTicksPerSec * BOOST_FACTOR);
        robot.outtakeRight.setVelocity(targetTicksPerSec * BOOST_FACTOR);
        boostActive = true;
        boostStartTime = timer.seconds();
    }

    /** Restore nominal shooter target after boost interval */
    private void updateBoostState() {
        if (boostActive && timer.seconds() - boostStartTime > BOOST_TIME_SEC) {
            setShooterVelocityTicks(targetTicksPerSec);
            boostActive = false;
        }
    }

    /** Set both shooter motors to target velocity in ticks/s */
    private void setShooterVelocityTicks(double ticksPerSec) {
        robot.outtakeLeft.setVelocity(ticksPerSec);
        robot.outtakeRight.setVelocity(ticksPerSec);
    }

    /** Readiness check: both motors near target or timeout will fire */
    private boolean isShooterReady() {
        double vL = robot.outtakeLeft.getVelocity();
        double vR = robot.outtakeRight.getVelocity();
        double tol = 0.92; // 92% of target
        return (vL >= targetTicksPerSec * tol) && (vR >= targetTicksPerSec * tol);
    }

    /** Battery-compensated F gain */
    private double compensatedF(double baseF) {
        double volts = getBatteryVoltage();
        return baseF * (12.0 / Math.max(10.0, volts));
    }

    /** Read battery voltage */
    private double getBatteryVoltage() {
        double v = 12.0;
        try {
            for (VoltageSensor vs : hardwareMap.voltageSensor) {
                if (vs != null) { v = vs.getVoltage(); break; }
            }
        } catch (Exception ignored) {}
        return v;
    }
}
