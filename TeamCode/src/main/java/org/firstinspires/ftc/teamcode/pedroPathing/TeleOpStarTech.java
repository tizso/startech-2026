package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareBox;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp StarTech - 2026", group="00-TeleOp")

public class TeleOpStarTech extends OpMode {

    HardwareBox robot;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double SLOW_DOWN_FACTOR = 1.0;

    private boolean intake = false;
    private boolean outtake = false;
    private boolean servoL = false;
    private boolean servoR = false;

    // AprilTag detection
    private static final int DESIRED_TAG_ID = 23; // Change this to the ID of the AprilTag you want to detect
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    Gamepad cg1 = new Gamepad();
    Gamepad cg2 = new Gamepad();

    Gamepad pg1 = new Gamepad();
    Gamepad pg2 = new Gamepad();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new HardwareBox();
        robot.init(hardwareMap);

        initAprilTag();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        telemetry.addData("Initializare", "18338");
        telemetry.update();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();


        pg1.copy(cg1);
        pg2.copy(cg2);

        cg1.copy(gamepad1);
        cg2.copy(gamepad2);

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                    -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                    -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (cg1.a && !pg1.a) {
            slowMode = !slowMode;
            /*follower.followPath(pathChain.get());
            automatedDrive = true;*/
        } else if (cg1.b && !pg1.b) {
            intake = !intake;

        } else if (cg1.x && !pg1.x) {
            outtake = !outtake;
        } else if (cg1.y && !pg1.y) {

        } else if(cg1.right_bumper && !pg1.right_bumper){
            servoR = !servoR;
        }

        if(intake){
            robot.intake.setPower(0.9);
        } else {
            robot.intake.setPower(0);
        }

        // AprilTag logic
        desiredTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                desiredTag = detection;
                break; // Found the tag we want, so stop searching
            }
        }

        if (desiredTag != null) {
            // An AprilTag is visible, so we override the outtake power.
            double range = desiredTag.ftcPose.range;
            telemetry.addData("AprilTag Range", range);

            // Scale power between 0.2 and 1.0 based on range.
            // farther = more power, closer = less power
            // For example, under 5 inches is minimum power, over 20 inches is maximum power.
            double minRange = 5.0; // inches
            double maxRange = 20.0; // inches
            double minPower = 0.7;
            double maxPower = 1.0;

            double power;
            if (range <= minRange) {
                power = minPower;
            } else if (range >= maxRange) {
                power = maxPower;
            } else {
                // Linear interpolation
                power = minPower + (range - minRange) * (maxPower - minPower) / (maxRange - minRange);
            }
            robot.outtake.setPower(power);
            telemetry.addData("Auto Outtake Power", power);
        } else {
            // No AprilTag is visible, so revert to manual control.
            if(outtake){
                robot.outtake.setPower(0.7);
            } else {
                robot.outtake.setPower(0);
            }
        }


        if(servoR){
            robot.servoInR.setPower(1.0);
        } else {
            robot.servoInR.setPower(0.0);
        }

        //Stop automated following if the follower is done
        /*if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }*/

        SLOW_DOWN_FACTOR = slowMode ? 0.3 : 1;

        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);

        telemetry.update();
    }

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
}
