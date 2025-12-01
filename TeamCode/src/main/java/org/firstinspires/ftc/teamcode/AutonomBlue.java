package org.firstinspires.ftc.teamcode;


/*
Hello :D my names mikey and i'm the head of software on team 21721. I was looking at the april tag sample code on the PP (pedro pathing) website and it kinda confused me or just wasn't
what I needed to do, so I decided to make my own! Before you worry about the code itself u need to know a bit about April tags. April tags are basically just QR codes; in the sense
that when you scan them they give u a numerical value. the april tag values for this season are as the following-

Blue Goal: 20
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
Red Goal: 24

So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
*/

// FTC SDK

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Autonomus Blue", group = "Opmode")
@Configurable // Panels
@Disabled

@SuppressWarnings("FieldCanBeLocal")
// Stop Android Studio from bugging about variables being predefined
public class AutonomBlue extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();
    HardwareBox robot = new HardwareBox();

    // Initialize poses
    private final int DISTANCE_FROM_APRILTAG = 50;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePoseLong = new Pose(0, 72, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose scorePoseShort = new Pose(24, 20, Math.toRadians(115)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose parkingPose = new Pose(0, 40, Math.toRadians(90)); // Parking Pose of our robot.
    private Pose targetPose = new Pose();

    // Initialize variables for paths
    private PathChain parkingPos;
    private PathChain scorePos;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathState; // Current state machine value

    private int foundID; // Current state machine value, dictates which one to run


    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    public void shootArtifacts() {
        // Put your shooting logic/functions here
        robot.setOuttake(0.75);
        sleep(2000L);

        if(foundID == 21) {
            robot.shutGreenArtifact();
            robot.shutPurpleArtifact();
            robot.shutPurpleArtifact();
        } else if(foundID == 22){
            robot.shutPurpleArtifact();
            robot.shutGreenArtifact();
            robot.shutPurpleArtifact();
        } else if(foundID == 23){
            robot.shutPurpleArtifact();
            robot.shutPurpleArtifact();
            robot.shutGreenArtifact();
        }
        robot.setOuttake(0.0);
        // Put your shooting logic/functions here
    }


    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        robot.init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        initAprilTag();

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }


        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        while (!opModeIsActive()) {
            telemetry.update();
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                telemetry.addData("Appril", "Tag ID %d", detection.id);
                if (detection.metadata != null) {
                    foundID = detection.id;
                    desiredTag = detection;
                    buildPaths();
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
                telemetry.addData("Distanta de la AT: ", desiredTag.ftcPose.range);
                telemetry.addData("xPos", desiredTag.ftcPose.x);
                telemetry.addData("angel", desiredTag.ftcPose.yaw);
            }
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathState(0);

        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose
            // Update the state machine based on the current AprilTag ID
            updateSateMachine();
            // Log to Panels and driver station (custom log function)
            log("AprilTag ID", foundID);
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            log("Ditanta de la appril", desiredTag.ftcPose.range);
            telemetry.update(); // Update the driver station after logging
        }
    }

    public void buildPaths() {
        double range = desiredTag.ftcPose.range;
        targetPose = range < DISTANCE_FROM_APRILTAG ? scorePoseShort : scorePoseLong;

        scorePos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        parkingPos = follower.pathBuilder()
                .addPath(new BezierLine(targetPose, parkingPose))
                .setLinearHeadingInterpolation(targetPose.getHeading(), parkingPose.getHeading())
                .build();
    }

    //below is the state machine or each pattern

    public void updateSateMachine() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePos);
                setpathState(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {
                    shootArtifacts();
                    setpathState(2);
                }
                break;
            case 2:
                follower.followPath(parkingPos);
                setpathState(-1); //set it to -1 so it stops the state machine execution
                break;
        }
    }

    // Setter methods for pathState variables placed at the class level
    void setpathState(int newPathState) {
        this.pathState = newPathState;
    }

    /**
     * start the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        // if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
        //     exposureControl.setMode(ExposureControl.Mode.Manual);
        //     sleep(50);
        // }
        // exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        // sleep(20);
        // GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        // gainControl.setGain(gain);
        // sleep(20);
    }
}
