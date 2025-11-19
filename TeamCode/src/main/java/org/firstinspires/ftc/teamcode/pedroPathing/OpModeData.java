package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

/**
 * A static class that stores persistent data between OpModes at the memory level.
 * It primarily serves as a "backup" solution if file-based saving/loading fails.
 * The data stored here only lives until the app is restarted.
 */
public class OpModeData {

    /**
     * The last pose saved by an OpMode (typically the autonomous program).
     */
    public static Pose lastPose = null;

    /**
     * The starting side determined in the autonomous program (-1: left, 1: right).
     */
    public static int initialSide = 0; // 0 -> unknown
}
