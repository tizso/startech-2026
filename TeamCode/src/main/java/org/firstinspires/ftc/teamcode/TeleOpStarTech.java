package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TeleOpStarTech extends LinearOpMode {

    public static Follower follower;

    HardwareBox robot = new HardwareBox();

    private double SLOW_DOWN_FACTOR = 0.9;
    private boolean slow = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        robot.init(hardwareMap);

        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();

        sleep(200);

        while (opModeIsActive()){

            telemetry.addData("Running StarTech TeleOp Mode adopted for Team:","18338");

            follower.setTeleOpDrive(-gamepad1.left_stick_y*SLOW_DOWN_FACTOR, -gamepad1.left_stick_x*SLOW_DOWN_FACTOR, -gamepad1.right_stick_x*SLOW_DOWN_FACTOR, true);
            follower.update();

            if(currentGamepad1.a && !previousGamepad1.a){
                slow = !slow;
            }

            SLOW_DOWN_FACTOR = slow?0.3:0.9;

            if(robot.sensorDistance.getDistance(DistanceUnit.CM)<20){
                SLOW_DOWN_FACTOR = 0.1;
            } else {
                SLOW_DOWN_FACTOR = 0.9;
            }

        }

    }
}
