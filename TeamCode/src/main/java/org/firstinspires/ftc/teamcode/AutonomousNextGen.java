package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This is the new, clean autonomous program for the 2026 season.
 * Step 5: Added a final parking maneuver after shooting.
 */
@Autonomous(name = "Autonomous NextGen", group = "Opmode")
public class AutonomousNextGen extends LinearOpMode {

    // Core components
    HardwareBox robot = new HardwareBox();
    private Follower follower;

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private enum State {
        SETUP,
        START_MOVE,
        WAIT_FOR_MOVE,
        SHOOT,
        PARK,             // New state for parking maneuver
        WAIT_FOR_PARK,    // New state to wait for parking to finish
        END
    }
    private State currentState = State.SETUP;

    // Logic Flow Variables
    private int initialSide = 0; // 1 for Blue/Left, -1 for Red/Right
    private int foundID = -1;

    @Override
    public void runOpMode() {
        // Initialize hardware, pathing, and vision
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        initVision();

        telemetry.addData("Status", "Initialized. Waiting for side detection...");
        telemetry.update();

        // Loop while the OpMode is in the Init phase
        while (opModeInInit()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection relevantTag = null;

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && (detection.id == 21 || detection.id == 22 || detection.id == 23)) {
                    relevantTag = detection;
                    break; 
                }
            }

            if (relevantTag != null) {
                foundID = relevantTag.id; // Store the ID for shooting logic
                if (relevantTag.ftcPose.x < 0) {
                    initialSide = -1; // Red Side
                    telemetry.addData("Detected Side", "Red Side (Robot is RIGHT of tag)");
                } else {
                    initialSide = 1; // Blue Side
                    telemetry.addData("Detected Side", "Blue Side (Robot is LEFT of tag)");
                }
                telemetry.addData("Status", "Side detected. Ready to start!");
            } else {
                telemetry.addData("Detected Side", "Unknown (No relevant tag visible)");
            }
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;
        
        currentState = State.START_MOVE; // Set initial state after start

        // Main loop after Start
        while (opModeIsActive()) {
            follower.update(); // Continuously update the follower

            switch (currentState) {
                case START_MOVE:
                    double turnAngleDeg = 45.0 * initialSide;
                    Pose startPose = new Pose(0, 0, 0);
                    Pose shooutingPose = new Pose(0, 72, 0);
                    PathChain moveAndTurnPath = follower.pathBuilder()
                            .addPath(new BezierLine(startPose, shooutingPose))
                            .setLinearHeadingInterpolation(0, Math.toRadians(turnAngleDeg))
                            .build();
                    follower.followPath(moveAndTurnPath);
                    telemetry.addData("Status", "Executing path: Move 72 inches, Turn " + turnAngleDeg + " deg");
                    currentState = State.WAIT_FOR_MOVE;
                    break;

                case WAIT_FOR_MOVE:
                    if (!follower.isBusy()) {
                        telemetry.addData("Status", "Path finished! Proceeding to shoot.");
                        currentState = State.SHOOT;
                    }
                    break;

                case SHOOT:
                    performShooting();
                    telemetry.addData("Status", "Shooting complete. Proceeding to park.");
                    currentState = State.PARK;
                    break;

                case PARK:
                    Pose currentPose = follower.getPose();
                    double additionalTurnRad = Math.toRadians(45.0 * initialSide);
                    double targetHeadingRad = currentPose.getHeading() + additionalTurnRad;

                    double strafeDistance = -30.0 * initialSide;

                    double sideVecX = -Math.sin(currentPose.getHeading());
                    double sideVecY = Math.cos(currentPose.getHeading());
                    double dispX = strafeDistance * sideVecX;
                    double dispY = strafeDistance * sideVecY;

                    Pose targetPose = new Pose(currentPose.getX() + dispX, currentPose.getY() + dispY, targetHeadingRad);
                    
                    PathChain parkPath = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, targetPose))
                            .build();
                    
                    follower.followPath(parkPath);
                    telemetry.addData("Status", "Executing parking maneuver.");
                    currentState = State.WAIT_FOR_PARK; // CRITICAL FIX: Transition to the wait state
                    break;
                
                case WAIT_FOR_PARK:
                    if (!follower.isBusy()) {
                        telemetry.addData("Status", "Parking finished!");
                        currentState = State.END;
                    }
                    break;

                case END:
                    telemetry.addData("Status", "OpMode finished.");
                    requestOpModeStop();
                    break;
            }

            telemetry.update();
        }

        visionPortal.close();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void performShooting() { 
        robot.setOuttake(1.0); 
        sleep(500);

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
    }
}
