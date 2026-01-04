
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;

/**
 * This OpMode provides a tool for generating lens intrinsic camera parameters.
 * It is based on the FTC SDK sample ConceptAprilTagLensIntrinsics, but with
 * robust data handling to prevent crashes.
 *
 * HOW TO USE:
 * 1. Print the calibration paper from the FTC docs.
 *    PRINT IT AT 100% SCALE. DO NOT SCALE TO FIT.
 * 2. Attach the paper to a flat, rigid surface.
 * 3. Run this OpMode.
 * 4. While the OpMode is running, point the camera at the calibration paper.
 *    - The telemetry will show "Tags Visible: Yes".
 *    - Get varied views: different angles, distances, and positions in the camera frame.
 * 5. Press the 'A' button on the gamepad to capture a frame. Capture at least 10-15 good frames.
 * 6. Once you have enough frames, press the 'B' button to run the calibration.
 * 7. The OpMode will display the lens intrinsics (fx, fy, cx, cy).
 * 8. Press 'B' again to exit and copy these values into your code.
 */
@TeleOp(name = "Concept: Camera Lens Calibration", group = "Concept")
@Disabled
public class ConceptAprilTagLensIntrinsics extends LinearOpMode {

    // A4: 210 mm × 297 mm
    final static double paperWidthMeters  = 0.210;   // 210 mm
    final static double paperHeightMeters = 0.297;   // 297 mm

    final static double tagSizeMeters = 0.028;

    final static int numTagsX = 6;
    final static int numTagsY = 8;

    final static int resWidth  = 640;
    final static int resHeight = 480;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    // --- OpenCV Calibration Data ---
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    // These lists will hold the points for each captured frame.
    ArrayList<Mat> imagePoints; // List of 2D points (in pixels) from the camera image
    ArrayList<Mat> objectPointsPerFrame; // List of 3D points (in meters) from the known paper layout
    MatOfPoint3f allObjectPoints;

    // --- State Flag ---
    boolean calibrationComplete = false;

    @Override
    public void runOpMode() {
        initAprilTag();
        initOpenCV();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready to capture frames for calibration.");
        telemetry.addLine("Point camera at paper. Press (A) to capture.");
        telemetry.addLine("Press (B) to Calibrate after >10 frames.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            // Update telemetry in the loop.
            if (!calibrationComplete) {

                int tagCount = (detections != null) ? detections.size() : 0;

                telemetry.addLine("=== Calibration Status ===");
                telemetry.addData("Tags Detected", tagCount);
                if (tagCount > 0) {
                    StringBuilder ids = new StringBuilder();
                    for (AprilTagDetection d : detections) {
                        ids.append(d.id).append(" ");
                    }
                    telemetry.addData("Tag IDs", ids.toString());
                    telemetry.addData("Tip", tagCount < 4 ? "Move camera to see more tags!" : "Good coverage");
                } else {
                    telemetry.addData("Tip", "No tags detected! Adjust camera.");
                }

                telemetry.addData("Frames Captured", imagePoints.size());
                telemetry.addLine("Press (A) to capture frame, (B) to calibrate after >=10 frames.");
                telemetry.update();

            }

            // 'A' button to capture a frame
            if (gamepad1.a && !calibrationComplete && detections != null && detections.size() >= 4) {
                addImagePoints(detections);
                telemetry.addData("Last Capture", "Success!");
                telemetry.update();
                sleep(500); // Debounce button press
            }
            else if (gamepad1.a && !calibrationComplete && detections != null && detections.size() < 4) {
                telemetry.addData("Warning", "Need >=4 tags visible to capture!");
            }


            // 'B' button to calibrate or exit
            if (gamepad1.b) {
                if (calibrationComplete) {
                    // Exit if already done
                    break; // Exit the while loop
                } else if (imagePoints.size() >= 10) {
                    // Run calibration
                    telemetry.clear();
                    telemetry.addLine("Calibrating... Please wait.");
                    telemetry.update();

                    calibrateCamera();
                    calibrationComplete = true;

                    // Show results
                    telemetry.clear();
                    telemetry.addLine("Calibration Complete!");
                    telemetry.addLine(String.format("fx: %.4f", cameraMatrix.get(0, 0)[0]));
                    telemetry.addLine(String.format("fy: %.4f", cameraMatrix.get(1, 1)[0]));
                    telemetry.addLine(String.format("cx: %.4f", cameraMatrix.get(0, 2)[0]));
                    telemetry.addLine(String.format("cy: %.4f", cameraMatrix.get(1, 2)[0]));
                    telemetry.addLine("\nPress (B) to exit.");
                } else {
                    // Not enough data
                    telemetry.addData("Warning", "Need at least 10 frames to calibrate!");
                    sleep(500);
                }
            }

            telemetry.update();
            sleep(20);
        }
    }

    /** Initializes the AprilTag processor and VisionPortal. */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(resWidth, resHeight))
                .addProcessor(aprilTag)
                .build();
    }

    /** Sets up the OpenCV matrices needed for calibration. */
    private void initOpenCV() {
        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        distCoeffs = new MatOfDouble();
        imagePoints = new ArrayList<>();
        objectPointsPerFrame = new ArrayList<>(); // Correctly holds 3D points for each frame
        imageSize = new Size(resWidth, resHeight);
        allObjectPoints = getObjectPoints();
    }

    /** Creates the master list of 3D points representing the corners of all AprilTags on the paper. */
    private MatOfPoint3f getObjectPoints() {
        ArrayList<Point3> points = new ArrayList<>();

        double margin = 0.005; // 5 mm
        double usableW = paperWidthMeters  - 2.0 * margin;
        double usableH = paperHeightMeters - 2.0 * margin;

        double cellW = tagSizeMeters;
        double cellH = tagSizeMeters;

        double gridW = numTagsX * cellW;
        double gridH = numTagsY * cellH;

        double startX = -paperWidthMeters / 2.0  + margin + (usableW - gridW) / 2.0;  // left (-), right (+)
        double startY =  paperHeightMeters / 2.0 - margin - (usableH - gridH) / 2.0;  // up (+), down (-)

        double half = tagSizeMeters / 2.0;

        for (int row = 0; row < numTagsY; row++) {
            for (int col = 0; col < numTagsX; col++) {

                double cx = startX + col * cellW + cellW / 2.0;
                double cy = startY - row * cellH - cellH / 2.0;

                points.add(new Point3(cx - half, cy + half, 0)); // Top-left
                points.add(new Point3(cx + half, cy + half, 0)); // Top-right
                points.add(new Point3(cx + half, cy - half, 0)); // Bottom-right
                points.add(new Point3(cx - half, cy - half, 0)); // Bottom-left
            }
        }

        MatOfPoint3f result = new MatOfPoint3f();
        result.fromList(points);
        return result;
    }

    /** Processes detections from a frame and adds the 2D and 3D points to our datasets. */
    private void addImagePoints(List<AprilTagDetection> detections) {
        MatOfPoint2f imagePointsThisFrame = new MatOfPoint2f();
        MatOfPoint3f objectPointsThisFrame = new MatOfPoint3f();

        List<Point3> all3d = allObjectPoints.toList();

        for (AprilTagDetection d : detections) {
            int id = d.id;

            if (id < 0 || id >= numTagsX * numTagsY) {
                continue;
            }

            for (int i = 0; i < 4; i++) {
                Point p = new Point(d.corners[i].x, d.corners[i].y);
                imagePointsThisFrame.push_back(new MatOfPoint2f(p));
            }

            for (int i = 0; i < 4; i++) {
                Point3 P = all3d.get(id * 4 + i);
                objectPointsThisFrame.push_back(new MatOfPoint3f(P));
            }
        }

        if (imagePointsThisFrame.total() > 0) {
            imagePoints.add(imagePointsThisFrame);
            objectPointsPerFrame.add(objectPointsThisFrame); // Store the corresponding 3D points
        }
    }

    /** Runs the OpenCV camera calibration function using the collected datasets. */
    private void calibrateCamera() {
        ArrayList<Mat> rvecs = new ArrayList<>();
        ArrayList<Mat> tvecs = new ArrayList<>();

        Calib3d.calibrateCamera(
                objectPointsPerFrame,
                imagePoints,
                imageSize,
                cameraMatrix,
                distCoeffs,
                rvecs,
                tvecs
        );
    }
}
