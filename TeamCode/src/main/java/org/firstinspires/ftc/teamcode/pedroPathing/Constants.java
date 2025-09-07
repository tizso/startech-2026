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
            .mass(5)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0));

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
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1)        //Distanta encoderului din partea stanga fata de mijlocul robotului in inch
            .rightPodY(-1)      //Distanta encoderului din partea dreapta fata de mijlocul robotului in -inch
            .strafePodX(-2.5)   //Distanta encoderului perpenducular fata de mijlocul robotului in -inch
            .leftEncoder_HardwareMapName("leftFront")    //portul 0 de pe Control Hub
            .rightEncoder_HardwareMapName("rightBack")   //portul 3 de pe Control Hub
            .strafeEncoder_HardwareMapName("rightFront") //portul 2 de pe Control Hub
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
