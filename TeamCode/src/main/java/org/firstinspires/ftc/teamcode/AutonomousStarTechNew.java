package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.CameraSettingsManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.OpModeData;

import java.util.Collections;
import java.util.List;

@Autonomous(name = "Autonomous StarTech", group = "Opmode")

public class AutonomousStarTechNew extends LinearOpMode {

    // --- Core Robot Components ---
    HardwareBox robot = new HardwareBox();
    private Follower follower;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ShooterManager shooterManager;
    private CameraSettingsManager camMgr;

    // --- State machine ---
    private enum State {SETUP, MOVE_TO_SHOOT, WAIT_FOR_MOVE, FINE_AIM, WAIT_FOR_FINE_AIM, CHECK_SPEED, FIRE, NEXT_SHOT, PARK, WAIT_FOR_PARK, END}

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
    private char[] sequence = {'P', 'P', 'P'};
    private int shotIndex = 0;
    private PathChain pathToShoot = null;
    private PathChain pathToPark = null;

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
                computedDistanceInch = Math.sqrt(dx * dx + dy * dy);
                if (pathToShoot == null) {
                    pathToShoot = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading())
                            .build();
                }
            }
        }

        // --- START ---
        waitForStart();
        shooterManager.setSpeedFromDistance(1690, true);
        //shooterManager.setSpeedFromDistance(computedDistanceInch, true);

        if (stayAndTurnMode) {
            currentState = State.FINE_AIM;
        } else if (pathToShoot != null) {
            follower.followPath(pathToShoot);
            currentState = State.WAIT_FOR_MOVE;
        } else {
            currentState = State.CHECK_SPEED;
        }

        timer.reset();
        sleep((long) (RobotConstants.AUTON_START_DELAY_SEC * 1000));
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
                fireBall(sequence[shotIndex], shotIndex);
                currentState = State.NEXT_SHOT;
                timer.reset();
                break;
            case NEXT_SHOT:
                if (timer.seconds() > (RobotConstants.SHOOTING_SERVO_RUN_TIME_SEC + RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC)) {
                    shotIndex++;
                    robot.servoInR.setPower(0);
                    robot.servoInL.setPower(0);
                    if (shotIndex < sequence.length) {
                        currentState = State.CHECK_SPEED;
                    } else {
                        int rad = initialSide > 0 ? 180 : 0;
                        double yPos = stayAndTurnMode?16:130;
                        Pose park = new Pose(follower.getPose().getX() - (14 * initialSide), yPos, Math.toRadians(rad));
                        pathToPark = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), park))
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), park.getHeading())
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
                shooterManager.stop();
                Pose finalPose = follower.getPose();
                if (finalPose != null) {
                    PoseStorage.savePoseToFile(finalPose, initialSide, backGoalPose);
                    OpModeData.lastPose = finalPose;
                    OpModeData.initialSide = initialSide;
                    OpModeData.backGoalPose = backGoalPose;
                }
                requestOpModeStop();
                break;
        }
    }

    private void initVision() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

            camMgr = new CameraSettingsManager(telemetry);
            camMgr.loadActiveProfile();
            camMgr.setProfile(camMgr.getProfile());
            camMgr.load();
            camMgr.attachAndApply(visionPortal);

        } catch (Exception e) {
            aprilTag = null;
            visionPortal = null;
        }
    }

    private Pose backGoalPose = null;

    private void handleSetup() {
        List<AprilTagDetection> dets = (aprilTag != null) ? aprilTag.getDetections() : Collections.emptyList();
        AprilTagDetection obelisk = null;
        AprilTagDetection backGoal = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection d : dets) {
            if (d == null || d.metadata == null || d.ftcPose == null) continue;
            if (d.id >= 21 && d.id <= 23) obelisk = d;
            if (d.id == RobotConstants.BLUE_GOAL_TAG_ID || d.id == RobotConstants.RED_GOAL_TAG_ID) {
                if (d.ftcPose.range < bestRange) {
                    bestRange = d.ftcPose.range;
                    backGoal = d;
                }
            }
        }

        if (obelisk == null || backGoal == null) return;

        stayAndTurnMode = Math.abs(obelisk.ftcPose.yaw) < RobotConstants.OBELISK_YAW_THRESHOLD_DEG;
        foundID = obelisk.id;
        backdropId = backGoal.id;
        sequence = getSequenceForID(foundID);

        double robotHeadingRad = 0;
        double robotX = 0;
        double robotY = 0;
        double shootHeading = 0;

        backGoalPose = (backGoal.id == RobotConstants.BLUE_GOAL_TAG_ID) ? RobotConstants.BLUE_BACKDROP_POSE : RobotConstants.RED_BACKDROP_POSE;//(16.01,133.28,-126.0)
        robotHeadingRad = Math.abs(Math.toDegrees(backGoalPose.getHeading())) - (backGoal.ftcPose.yaw - backGoal.ftcPose.bearing);
        if (stayAndTurnMode) {

            initialSide = (obelisk.ftcPose.x < 0) ? -1 : 1;
            shootHeading = robotHeadingRad;
            robotX = backGoalPose.getX() - backGoal.ftcPose.x;
            robotY = backGoalPose.getY() - backGoal.ftcPose.y - RobotConstants.CAMERA_FORWARD_OFFSET;

        } else {
            double x = backGoal.ftcPose.x;
            double y = backGoal.ftcPose.y;
            double p = Math.sqrt(y * y + x * x);
            shootHeading = (backGoal.id == RobotConstants.BLUE_GOAL_TAG_ID) ? RobotConstants.ANGEL_SHORT_RED : RobotConstants.ANGEL_SHORT_BLUE;
            initialSide = (obelisk.ftcPose.x > 0) ? -1 : 1;
            robotX = backGoalPose.getX() - ((p - RobotConstants.CAMERA_FORWARD_OFFSET) * initialSide);
            robotY = backGoalPose.getY() - ((x - RobotConstants.CAMERA_FORWARD_OFFSET) * initialSide);
        }

        startPose = new Pose(robotX, robotY, Math.toRadians(robotHeadingRad));

        shootPose = new Pose(
                robotX + initialSide * RobotConstants.X_OFFSET_IN,
                robotY - RobotConstants.Y_OFFSET_IN,
                Math.toRadians(shootHeading));

        telemetry.addLine("=== INIT PRECOMPUTE ===");
        telemetry.addData("Tag Distance (in)", "%.2f", backGoal.ftcPose.range);
        telemetry.addData("BackGoal x Distance", "%.2f", backGoal.ftcPose.x);
        telemetry.addData("BackGoal y Distance", "%.2f", backGoal.ftcPose.y);
        telemetry.addData("BackGoal yaw", "%.2f", backGoal.ftcPose.yaw);
        telemetry.addData("BackGoal bearing", "%.2f", backGoal.ftcPose.bearing);
        telemetry.addData("Obelisk x", "%.2f", obelisk.ftcPose.x);
        telemetry.addData("Obelisk y", "%.2f", obelisk.ftcPose.y);
        telemetry.addData("Obelisk yaw", "%.2f", obelisk.ftcPose.yaw);
        telemetry.addData("Obelisk bearing", "%.1f°", obelisk.ftcPose.bearing);
        telemetry.addData("InitialSide", (initialSide > 0) ? "BLUE(+1)" : "RED(-1)");

        telemetry.addData("Shooter Mode", ShooterManager.USE_ENCODER_FOR_SHOOTER ? "VELOCITY" : "POWER");
        telemetry.addData("Stay and Turn Mode", stayAndTurnMode ? "ACTIVE" : "Inactive");
        telemetry.addData("ObeliskID", foundID);
        telemetry.addData("BackdropId", backdropId);
        telemetry.addData("StartPose", startPose);
        telemetry.addData("ShootPose", shootPose);
        telemetry.addData("Distance (in)", computedDistanceInch);
        telemetry.addData("Camera profile", camMgr.getProfile());

        telemetry.update();

    }

    double bbb = 0;

    private void startFineAim() {
        Pose current = follower.getPose();
        if (current == null) return;

        double desiredHeading;
        AprilTagDetection backdrop = getClosestBackGoal();
        if (backdrop != null && backdrop.ftcPose != null && stayAndTurnMode) {
            double rad = initialSide > 0 ? RobotConstants.ANGEL_LONG_BLUE : RobotConstants.ANGEL_LONG_RED;
            desiredHeading = current.getHeading() + (Math.toRadians(rad) * initialSide);
        } else {
            desiredHeading = shootPose.getHeading(); // Fallback
        }

        if (RobotConstants.UPDATE_SPEED_ON_FINE_AIM && backdrop != null && backdrop.ftcPose != null) {
            computedDistanceInch = backdrop.ftcPose.range;
            double vel = 0;
            if(stayAndTurnMode){
                vel = initialSide > 0 ? RobotConstants.VEL_LONG_BLUE : RobotConstants.VEL_LONG_RED;
            } else {
                vel = initialSide > 0 ? RobotConstants.VEL_SHORT_BLUE : RobotConstants.VEL_SHORT_RED;
            }
            shooterManager.setSpeedFromDistance(vel, true);
        }

        // Add a small "nudge" to the target pose to ensure the follower executes the turn.
        // A pure zero-distance path can sometimes be ignored by the path follower.
        double eps = 1.0; // A small nudge in inches
        Pose target;
        if (stayAndTurnMode) {
            target = new Pose(
                    current.getX() + eps,
                    current.getY() + eps,
                    desiredHeading);
        } else {
            target = new Pose(
                    shootPose.getX(),
                    shootPose.getY() + eps,
                    shootPose.getHeading());
        }
        PathChain fine = follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), desiredHeading)
                .build();
        follower.followPath(fine);
    }

    private void shootPurple() {
        robot.intake.setPower(1);
        sleep((long) (RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC * 1000));
        robot.intake.setPower(0);
        sleep((long) (RobotConstants.SHOOTING_SERVO_STOP_TIME_SEC * 1000));
        robot.servoInR.setPower(1.0);
    }


    private void fireBall(char type, int idx) {
        robot.servoInR.setDirection(DcMotorSimple.Direction.FORWARD);

        if (foundID == 21) {
            if (idx == 0 && type == 'G') {
                robot.servoInL.setPower(1.0);
            } else if (idx == 1 && type == 'P') {
                robot.servoInR.setPower(1.0);
            } else if (idx == 2 && type == 'P') {
                shootPurple();
            }
        } else if (foundID == 22) {
            if (idx == 0 && type == 'P') {
                robot.servoInR.setPower(1.0);
            } else if (idx == 1 && type == 'G') {
                robot.servoInL.setPower(1.0);
            } else if (idx == 2 && type == 'P') {
                shootPurple();
            }
        } else if (foundID == 23) {
            if (idx == 0 && type == 'P') {
                robot.servoInR.setPower(1.0);
            } else if (idx == 1 && type == 'P') {
                shootPurple();
            } else if (idx == 2 && type == 'G') {
                robot.servoInL.setPower(1.0);
            }
        }
    }

    private AprilTagDetection getClosestBackGoal() {
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
            case 21:
                return new char[]{'G', 'P', 'P'};
            case 22:
                return new char[]{'P', 'G', 'P'};
            default:
                return new char[]{'P', 'P', 'G'};
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
        //panel.update();
        //panel.renderFull();
        telemetry.update();
    }

    private void displayRuntimeTelemetry() {
        Pose current = follower.getPose();
        telemetry.addData("State", currentState);
        telemetry.addData("Shooter", shooterManager.getTelemetryData());
        telemetry.addData("BackdropId", backdropId);
        telemetry.addData("currentX", current.getX());
        telemetry.addData("currentY", current.getY());
        telemetry.addData("current Heading", current.getHeading());
        telemetry.addData("Distance (in)", computedDistanceInch);
        telemetry.addData("Range (in)", bbb);
        //panel.update();
        // panel.renderMinimal();
        telemetry.update();
    }
}
