package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * A central place to store all robot-specific constants for easy tuning and management.
 */
public class RobotConstants {

    // --- Field Landmark Poses & AprilTag IDs ---
    public static final Pose BLUE_BACKDROP_POSE = new Pose(16.01, 133.28, Math.toRadians(126.0));
    public static final Pose RED_BACKDROP_POSE  = new Pose(127.99, 133.28, Math.toRadians(54.0));
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID  = 24;

    // --- Robot Geometry ---
    public static final double CAMERA_FORWARD_OFFSET = 7.0; // inches

    public static final double AUTON_START_DELAY_SEC = 0;

    // --- Autonomous Tuning ---
    public static final double YAW_THRESHOLD_DEG = 60.0; // To detect wrong-side tag for shoot pose calculation
    public static final double OBELISK_YAW_THRESHOLD_DEG = 30.0; // To trigger stay-and-turn mode
    public static final double X_OFFSET_IN = 32.0; // Lateral offset from backdrop for shooting
    public static final double Y_OFFSET_IN = 32.0; // Forward offset from backdrop for shooting

    // --- Fine Aiming Constants ---
    public static final double SHOOTING_BACK_TIME = 6.0; // Small nudge distance for heading correction path
    public static final double FINE_AIM_TIMEOUT_SEC = 5.0; // Max seconds to wait for fine-aim turn
    public static final boolean UPDATE_SPEED_ON_FINE_AIM = true;

    // --- General Mechanism Timing ---
    public static final double SHOOTER_READY_TIMEOUT_SEC = 7.0;
    public static final double SHOOTING_SERVO_RUN_TIME_SEC = 1.25;
    public static final double SHOOTING_SERVO_STOP_TIME_SEC = 0.75;


    public static final double ANGEL_LONG_BLUE = 25;
    public static final double ANGEL_LONG_RED = 24;
    public static final double VEL_LONG_BLUE = 2015;
    public static final double VEL_LONG_RED = 2060;


    public static final double ANGEL_SHORT_BLUE = 145;
    public static final double ANGEL_SHORT_RED = 38;
    public static final double VEL_SHORT_BLUE = 1685;
    public static final double VEL_SHORT_RED = 1650;

}
