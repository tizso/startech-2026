package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    /**
     * .mass(5) greutatea robotului in kg (poate sa fie un double)
     * */

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-49.51)
            .lateralZeroPowerAcceleration(-72.56) //81.16 87.21
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.11,0,0.007,0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.4, 0, 0.01, 0.025))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000008, 0.5, 0.00038))
            .centripetalScaling(0.0005);


    /**
     * .maxPower() -> trebuie sa fie un numar intre 0 si 1, preferabil 1
     * ...MotorName -> denumirea motoarelor
     * ...MotorDirection() -> directia de rotare a motoarelor
     * */
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")   //portul 0 de pe Control Hub
            .leftRearMotorName("leftBack")     //portul 1 de pe Control Hub
            .rightFrontMotorName("rightFront") //portul 2 de pe Control Hub
            .rightRearMotorName("rightBack")   //portul 3 de pe Control Hub
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(57.34)
            .yVelocity(40.16)
            .useBrakeModeInTeleOp(true);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001982867518)
            .strafeTicksToInches(.001963588179)
            .turnTicksToInches(.001956129858)
            .leftPodY(6.1)        //Distanta encoderului din partea stanga fata de mijlocul robotului in inch
            .rightPodY(-6.1)      //Distanta encoderului din partea dreapta fata de mijlocul robotului in -inch
            .strafePodX(-7.1)   //Distanta encoderului perpenducular fata de mijlocul robotului in -inch
            .leftEncoder_HardwareMapName("leftFront")    //portul 0 de pe Control Hub
            .rightEncoder_HardwareMapName("rightBack")   //portul 3 de pe Control Hub
            .strafeEncoder_HardwareMapName("rightFront") //portul 2 de pe Control Hub
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
