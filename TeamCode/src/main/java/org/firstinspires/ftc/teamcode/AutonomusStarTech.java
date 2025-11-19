package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

import java.util.List;
import java.util.Locale;

@Autonomous(name = "Autonomus StarTech", group = "Opmode")
@Configurable
@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class AutonomusStarTech extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    HardwareBox robot = new HardwareBox();
    private Follower follower;

    private enum State {
        SETUP, 
        SEARCH_INITIAL_TAG, 
        ADJUST_TO_INITIAL_TAG, 
        TURN_TO_GOAL_TAG, 
        ADJUST_TO_GOAL_TAG, 
        SHOOT, 
        PARK, 
        END
    }
    private State currentState = State.SETUP;

    // --- Constants ---
    private static final double AUTON_START_DELAY_SEC = 5.0;
    private static final double AUTON_TIMEOUT_SEC = 30.0;

    private static final Pose INITIAL_TAG_POSE = new Pose(72, 144, Math.toRadians(-90));
    private static final double HEADING_DECISION_RANGE = 72.0; 
    private static final double LATERAL_OFFSET_INITIAL = 24.0; 
    private static final double DISTANCE_FROM_APRILTAG = 40.0; 
    private static final double GOAL_TAG_DISTANCE = 35.0; 
    private static final double PARK_DISTANCE = 20.0; 

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
    private AprilTagDetection detectedTag = null;

    // --- Logic Flow Variables ---
    private int foundID = -1;
    private int initialSide = 0;
    private boolean isSpinningUp = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        initAprilTag();

        while (opModeInInit()) {
            if (handleSetup()) { 
                break; 
            }
        }

        telemetry.addData("Status", "Setup complete. Press START.");
        telemetry.update();
        
        waitForStart();
        
        if (currentState == State.SETUP) {
            telemetry.addData("ERROR", "Could not set starting pose. Shutting down.");
            telemetry.update();
            sleep(5000);
            return;
        }

        sleep((long) (AUTON_START_DELAY_SEC * 1000));
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            if (runtime.seconds() > AUTON_TIMEOUT_SEC) currentState = State.END;

            follower.update();
            updateDetectedTagForState();

            switch (currentState) {
                case SEARCH_INITIAL_TAG: handleSearchInitialTag(); break;
                case ADJUST_TO_INITIAL_TAG: handleAdjustToInitialTag(); break;
                case TURN_TO_GOAL_TAG: handleTurnToGoalTag(); break;
                case ADJUST_TO_GOAL_TAG: handleAdjustToGoalTag(); break;
                case SHOOT: handleShoot(); break;
                case PARK: handlePark(); break;
                case END: handleEnd(); break;
                default: currentState = State.END; break;
            }

            telemetry.addData("Time", "%.1f / %.1f", runtime.seconds(), AUTON_TIMEOUT_SEC);
            telemetry.addData("State", currentState.toString());
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }
        handleEnd();
    }

    private boolean handleSetup(){
        telemetry.addData("Status", "SETUP: Waiting for AprilTag...");
        AprilTagDetection initialTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
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
            currentState = State.SEARCH_INITIAL_TAG;
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
            currentState = State.ADJUST_TO_INITIAL_TAG;
        } else {
             follower.setTeleOpDrive(0,0,0.2, true);
        }
    }
    
    private void handleAdjustToInitialTag() {
        if (detectedTag == null) { currentState = State.SEARCH_INITIAL_TAG; return; }
        double targetX = initialSide * LATERAL_OFFSET_INITIAL;
        double errorX = detectedTag.ftcPose.x - targetX;
        double errorY = detectedTag.ftcPose.y - DISTANCE_FROM_APRILTAG;
        if (Math.abs(errorX) < POSITIONING_TOLERANCE && Math.abs(errorY) < POSITIONING_TOLERANCE) {
            follower.setTeleOpDrive(0, 0, 0, true);
            currentState = State.TURN_TO_GOAL_TAG;
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
            currentState = State.ADJUST_TO_GOAL_TAG;
        }
    }

    private void handleAdjustToGoalTag() {
        if (detectedTag == null) { 
            if(isSpinningUp) { robot.setOuttake(0.0); isSpinningUp = false; }
            currentState = State.TURN_TO_GOAL_TAG; 
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
            currentState = State.SHOOT;
        } else {
            double forwardPower = -FORWARD_GAIN * errorRange;
            double turnPower = -TURN_GAIN * errorBearing;
            follower.setTeleOpDrive(clip(forwardPower), 0, clip(turnPower), true);
        }
    }

    private void handleShoot() {
        shootArtifacts();
        currentState = State.PARK;
    }

    private void handlePark() {
        if (detectedTag == null) {
            currentState = State.END;
            return;
        }
        double targetX = initialSide * PARK_DISTANCE;
        double errorX = detectedTag.ftcPose.x - targetX;
        double errorY = detectedTag.ftcPose.y - GOAL_TAG_DISTANCE;
        double errorBearing = detectedTag.ftcPose.bearing;
        if (Math.abs(errorX) < POSITIONING_TOLERANCE) {
            follower.setTeleOpDrive(0, 0, 0, true);
            currentState = State.END;
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
    
    private void updateDetectedTagForState() { 
        detectedTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty()) return;
        int[] targetIds;
        switch (currentState) {
            case SETUP:
            case SEARCH_INITIAL_TAG:
            case ADJUST_TO_INITIAL_TAG:
                targetIds = new int[]{21, 22, 23};
                break;
            case TURN_TO_GOAL_TAG:
            case ADJUST_TO_GOAL_TAG:
            case PARK:
                targetIds = new int[]{(initialSide < 0) ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID};
                break;
            default:
                targetIds = new int[]{};
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

    private void initAprilTag() { 
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
     }
     
    private double clip(double value) { 
        return Math.max(-MAX_DRIVE_POWER, Math.min(value, MAX_DRIVE_POWER)); 
    }
}
