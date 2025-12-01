package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareBox;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This is the main TeleOp program for the 2026 season.
 * It uses a field-centric drive system based on the pose loaded from the Autonomous phase.
 * All robot actions are controlled manually by the driver.
 * Now updated for dual-motor, velocity-controlled shooter.
 */
@Configurable
@TeleOp(name = "TeleOp StarTech - 2026", group="00-TeleOp")

public class TeleOpStarTech extends OpMode {

    HardwareBox robot;
    private Follower follower;
    private boolean slowMode = false;
    private double SLOW_DOWN_FACTOR = 1.0;

    // Booleans to toggle mechanisms
    private boolean intake = false;
    private boolean outtake = false;
    private boolean reverse = false;
    private boolean sep = false;

    // Gamepad state variables
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- Shooter Constants ---
    // Default RPM if no tag is visible
    private static final double SHOOTER_RPM = 4800; 
    private static final double TICKS_PER_REV = 28; 
    private static final double SHOOTER_VELOCITY_TICKS_PER_SEC = (SHOOTER_RPM / 60.0) * TICKS_PER_REV;

    // --- Ballistic Calculation Constants ---
    private static final double BASKET_HEIGHT_M = 1.0; // heigh of the target basket in meter.
    private static final double ROBOT_SHOOT_HEIGHT_M = 0.381; // 15 inches, height of the shooter from the ground
    private static final double SHOOT_ANGLE_DEG = 45; // angle of the shooting can be lower
    private static final double G = 9.81; // gravity
    private static final double WHEEL_DIAMETER_M = 0.096; // shooting wheel diameter
    private static final double SLIP_FACTOR = 1.3; // slip factor

    private static final double MIN_RPM = 3600; // min rpm if the robot is closer to the target basket
    private static final double MAX_RPM = 5000; // max rpm if the robot is the highest distance from the target basket
    
    // --- Automation Variables ---
    private int autoStartingSide = 0; // -1 for right (Red), 1 for left (Blue), 0 for unknown

    /**
     * Initializes the robot and follower.
     * Implements the hybrid pose loading logic to ensure a seamless transition from Autonomous.
     */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        initVision(); // Initialize vision

        // --- Hybrid Pose Loading Logic ---
        Pose startingPose;
        String loadSource;
        PoseStorage.StoredPose storedPoseFromFile = PoseStorage.loadPoseFromFile();
        boolean isFilePoseDefault = storedPoseFromFile.pose.getX() == 0 && storedPoseFromFile.pose.getY() == 0 && storedPoseFromFile.pose.getHeading() == 0;
        
        if (isFilePoseDefault && OpModeData.lastPose != null) {
            startingPose = OpModeData.lastPose;
            autoStartingSide = OpModeData.initialSide; // Load side from static backup
            loadSource = "Static Backup";
        } else {
            startingPose = storedPoseFromFile.pose;
            autoStartingSide = storedPoseFromFile.initialSide; // Load side from file
            loadSource = "File";
        }

        follower.setStartingPose(startingPose);
        follower.update();

        robot = new HardwareBox();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized (18338)");
        telemetry.addData("Pose Load Source", loadSource);
        telemetry.addData("Loaded Pose", "X: %.2f, Y: %.2f, H: %.2f", startingPose.getX(), startingPose.getY(), startingPose.getHeading());
        telemetry.addData("Loaded Auto Side", (autoStartingSide < 0 ? "Right/Red" : (autoStartingSide > 0 ? "Left/Blue" : "Unknown")));
        telemetry.update();
    }

    /**
     * Starts the teleop driving mode for the follower.
     */
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * Main loop of the OpMode. Handles driver input.
     */
    @Override
    public void loop() {
        // Update follower and gamepads
        follower.update();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // The follower handles switching between path following and teleop drive internally.
        // We can simply call all handlers every loop.
        handleManualDrive();
        handleGamepadControls();
        handleShooter();
        
        updateTelemetry();
    }
    
    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Handles manual, driver-controlled robot movement.
     */
    private void handleManualDrive() {
        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1.0;
        // This command is ignored by the follower when a path is active.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                true // Robot Centric Driving
        );
    }

    /**
     * Handles all gamepad button presses for mechanisms and mode changes.
     */
    private void handleGamepadControls() {
        // --- Automation Trigger ---
        if (currentGamepad1.start && !previousGamepad1.start) {
            // Use placeholder coordinates for now. These should be tuned.
            Pose targetPose = (autoStartingSide > 0) ? 
                                new Pose(24, 72, Math.toRadians(90)) : // Blue side target
                                new Pose(120, 72, Math.toRadians(90));   // Red side target

            PathChain pathToTarget = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), targetPose))
                    .build();
            
            // This command gives control to the follower until the path is done.
            follower.followPath(pathToTarget);
        }

        // --- Manual Toggles ---
        if (currentGamepad1.a && !previousGamepad1.a) slowMode = !slowMode;
        if (currentGamepad1.b && !previousGamepad1.b) intake = !intake;
        if (currentGamepad1.x && !previousGamepad1.x) outtake = !outtake;
        if (currentGamepad1.y && !previousGamepad1.y) reverse = !reverse;
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) sep = !sep;

        // Servos
        robot.servoInR.setPower(currentGamepad1.right_bumper ? 1.0 : 0.0);
        robot.servoInL.setPower(currentGamepad1.left_bumper ? 1.0 : 0.0);
        
        // --- Motors ---
        robot.intake.setPower(intake ? 0.9 : 0.0);
        robot.separator.setPosition(sep ? 0.8 : 0.3);

        // Motor/Servo Direction
        if (reverse) {
            robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInL.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInR.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.servoInL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.servoInR.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Dynamically calculates and sets shooter wheel velocity based on AprilTag distance.
     */
    private void handleShooter() {
        if (!outtake) {
            robot.outtakeLeft.setPower(0);
            robot.outtakeRight.setPower(0);
            return;
        }

        double targetRpm = SHOOTER_RPM; // Default RPM
        int goalTagId = (autoStartingSide > 0) ? 20 : 24; // Blue or Red goal tag
        AprilTagDetection goalTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == goalTagId) {
                goalTag = detection;
                break; 
            }
        }

        if (goalTag != null) {
            double distance_in = goalTag.ftcPose.range;
            double distance_m = distance_in * 0.0254;
            telemetry.addData("Tag Distance", "%.2f in", distance_in);

            double theta_rad = Math.toRadians(SHOOT_ANGLE_DEG);
            double y = BASKET_HEIGHT_M - ROBOT_SHOOT_HEIGHT_M;
            
            double cos_theta = Math.cos(theta_rad);
            double tan_theta = Math.tan(theta_rad);

            double denominator = 2 * cos_theta * cos_theta * (distance_m * tan_theta - y);

            if (denominator > 0) {
                double v_squared = (G * distance_m * distance_m) / denominator;
                double v_ball = Math.sqrt(v_squared);
                
                double v_wheel = v_ball * SLIP_FACTOR;
                double omega_wheel = v_wheel / (WHEEL_DIAMETER_M / 2.0);
                double calculatedRpm = omega_wheel * 60.0 / (2.0 * Math.PI);
                
                // Clamp the calculated RPM to the min/max values
                targetRpm = Math.max(MIN_RPM, Math.min(calculatedRpm, MAX_RPM));
            }
        }
        
        telemetry.addData("Calculated Target RPM", "%.0f", targetRpm);
        double targetVelocity = (targetRpm / 60.0) * TICKS_PER_REV;
        robot.outtakeLeft.setVelocity(targetVelocity);
        robot.outtakeRight.setVelocity(targetVelocity);
    }

    /**
     * Updates the driver station telemetry with key information.
     */
    private void updateTelemetry() {
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Shooter Actual Velocity L", robot.outtakeLeft.getVelocity());
        telemetry.addData("Shooter Actual Velocity R", robot.outtakeRight.getVelocity());
        telemetry.update();
    }
}
