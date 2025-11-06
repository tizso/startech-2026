package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareBox;

import java.util.function.Supplier;

@TeleOp(name = "TeleOp StarTech dd - 2026", group="00-TeleOp")
@Disabled
public class TeleOpStarTechOld extends LinearOpMode {


    HardwareBox robot = new HardwareBox();

    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;

    private double SLOW_DOWN_FACTOR = 0.9;
    private boolean slow = false;

    private boolean intake = false;
    private boolean outtake = false;
    private boolean servoL = false;
    private boolean servoR = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        follower.startTeleopDrive();

        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();



        sleep(200);

        waitForStart();

        while (opModeIsActive()){
            telemetry.update();
            follower.update();
            telemetry.addData("Running StarTech TeleOp Mode adopted for Team:","18338");

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                    -gamepad1.left_stick_x * SLOW_DOWN_FACTOR,
                    -gamepad1.right_stick_x * SLOW_DOWN_FACTOR,
                    true // Robot Centric
            );

            if (currentGamepad1.a && !previousGamepad1.a) {
                slow = !slow;
            /*follower.followPath(pathChain.get());
            automatedDrive = true;*/
            } else if (currentGamepad1.b && !previousGamepad1.b) {
                intake = !intake;

            } else if (currentGamepad1.x && !previousGamepad1.x) {
                outtake = !outtake;
            } else if (currentGamepad1.y && !previousGamepad1.y) {

            } else if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                servoR = !servoR;
            }

            if(intake){
                robot.intake.setPower(0.9);
            } else {
                robot.intake.setPower(0);
            }

            if(outtake){
                robot.outtake.setPower(0.9);
            } else {
                robot.outtake.setPower(0);
            }

            if(servoR){
                robot.servoInR.setPower(1.0);
            } else {
                robot.servoInR.setPower(0.0);
            }
            telemetry.addData("intake: ", intake);
            telemetry.addData("outtake: ", outtake);
            telemetry.addData("servoR: ",servoR);


            SLOW_DOWN_FACTOR = slow?0.3:0.9;


        }

    }
}
