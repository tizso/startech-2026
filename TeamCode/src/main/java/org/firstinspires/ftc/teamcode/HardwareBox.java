package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareBox extends LinearOpMode{

    double SLIDER_DIFF = 0.9435;
    //public NormalizedColorSensor color;

    public DistanceSensor sensorDistance;

    public DcMotor intake;
    public DcMotor outtake = null;

    public CRServo servoInR = null;
    public CRServo servoInL = null;
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBox(){

    }
    public void runOpMode(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        intake = hwMap.get(DcMotor.class, "intake");
        outtake = hwMap.get(DcMotor.class, "outtake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);


        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setPower(0);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Define and initialize ALL installed servos.

        servoInR = hwMap.get(CRServo.class, "servoInR");
        //servoInL = hwMap.get(CRServo.class, "servoInL");

        // Initialize the distance sensor
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_distance");
    }


    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


    /*public void setIntake(double value){
        intake.setPower(value);
    }*/

    /*public void setOuttake(double value){
        outtake.setPower(value);
    }*/


    /*public void setServoInL(int value){
        servoInL.setPower(value);
    }*/

    /*public void setServoInR(int value){
        servoInR.setPower(value);
    }*/

}
