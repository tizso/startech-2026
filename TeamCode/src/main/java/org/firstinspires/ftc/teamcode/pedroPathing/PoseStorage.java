package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.util.Locale;

/**
 * The PoseStorage class is responsible for saving and loading the robot's position
 * and autonomous side information.
 * It stores the data in a file named `last_pose.txt` on the Robot Controller's internal storage.
 */
public class PoseStorage {

    private static final String FILENAME = "last_pose.txt";
    private static final int POSE_DATA_COUNT = 7; // x, y, heading, side, bx, by, bh



    /**
     * Inner class to store the pose and side information together.
     */
    public static class StoredPose {
        public final Pose pose;
        public final int initialSide; // -1 for left, 1 for right, 0 for unknown
        public final Pose backGoalPose;


        public StoredPose(Pose pose, int initialSide, Pose backGoalPose) {
            this.pose = pose;
            this.initialSide = initialSide;
            this.backGoalPose = backGoalPose;

        }
    }

    /**
     * Saves the given pose and side information to the file.
     * @param pose The pose to be saved.
     * @param side The side to be saved (-1: left, 1: right).
     * @param bPose The back goal pose to be saved.
     */
    public static void savePoseToFile(Pose pose, int side, Pose bPose) {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        // Format: x,y,heading,side
        String dataString = String.format(Locale.US, "%.2f,%.2f,%.2f,%d,.2f,%.2f,%.2f", pose.getX(), pose.getY(), pose.getHeading(), side, bPose.getX(), bPose.getY(), bPose.getHeading());
        // The ReadWriteFile.writeFile method handles errors internally, so a try-catch block is not needed.
        ReadWriteFile.writeFile(file, dataString);
    }

    /**
     * Loads the pose and side information from the file.
     * If the file does not exist or an error occurs, it returns a default (0,0,0) pose and an
     * unknown (0) side.
     * @return The loaded StoredPose object, or a default one.
     */
    public static StoredPose loadPoseFromFile() {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        try {
            String dataString = ReadWriteFile.readFile(file);
            String[] parts = dataString.split(",");
            if (parts.length == POSE_DATA_COUNT) {
                double x = Double.parseDouble(parts[0]);
                double y = Double.parseDouble(parts[1]);
                double heading = Double.parseDouble(parts[2]);
                int side = Integer.parseInt(parts[3]);
                double bx = Double.parseDouble(parts[4]);
                double by = Double.parseDouble(parts[5]);
                double bh = Double.parseDouble(parts[6]);
                return new StoredPose(new Pose(x, y, heading), side, new Pose(bx, by, bh));
            }
        } catch (NumberFormatException e) {
            // In case of an error (e.g., corrupted file), return a default
        }
        // Default values if loading fails (e.g., non-existent or empty file)
        return new StoredPose(new Pose(), 0, new Pose());
    }
}
