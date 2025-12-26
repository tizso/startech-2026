package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * A central place to store all robot-specific constants for easy tuning and management.
 */
public class RobotConstants {

    // --- Field Landmark Poses & AprilTag IDs ---
    public static final Pose BLUE_BACKDROP_POSE = new Pose(16.01, 133.28, Math.toRadians(-126.0));
    public static final Pose RED_BACKDROP_POSE  = new Pose(127.99, 133.28, Math.toRadians(54.0));
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID  = 24;

    // --- Robot Geometry ---
    public static final double CAMERA_FORWARD_OFFSET = 7.0; // inches

    // --- Autonomous Tuning ---
    public static final double YAW_THRESHOLD_DEG = 60.0; // To detect wrong-side tag for shoot pose calculation
    public static final double OBELISK_YAW_THRESHOLD_DEG = 10.0; // To trigger stay-and-turn mode
    public static final double X_OFFSET_IN = 30.0; // Lateral offset from backdrop for shooting
    public static final double Y_OFFSET_IN = 48.0; // Forward offset from backdrop for shooting

    // --- Fine Aiming Constants ---
    public static final double FINE_AIM_EPS_IN = 0.25; // Small nudge distance for heading correction path
    public static final double FINE_AIM_TIMEOUT_SEC = 0.40; // Max seconds to wait for fine-aim turn
    public static final boolean UPDATE_SPEED_ON_FINE_AIM = true;

    // --- General Mechanism Timing ---
    public static final double SHOOTER_READY_TIMEOUT_SEC = 2.0;
    public static final double SHOOTING_SERVO_RUN_TIME_SEC = 0.25;
    public static final double SHOOTING_SERVO_STOP_TIME_SEC = 0.12;

}
