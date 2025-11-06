package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class BallDetectionProcessor implements VisionProcessor {

    private final Scalar lowerPurple = new Scalar(120, 50, 50); // Lower bound for purple color in HSV
    private final Scalar upperPurple = new Scalar(160, 255, 255); // Upper bound for purple color in HSV
    private final Scalar lowerGreen = new Scalar(40, 50, 50);   // Lower bound for green color in HSV
    private final Scalar upperGreen = new Scalar(80, 255, 255);  // Upper bound for green color in HSV

    private final List<MatOfPoint> purpleContours = new ArrayList<>();
    private final List<MatOfPoint> greenContours = new ArrayList<>();
    private final Mat hsvMat = new Mat();
    private final Mat purpleMask = new Mat();
    private final Mat greenMask = new Mat();
    private final Mat hierarchy = new Mat();

    private Rect bestPurpleContourRect = null;
    private Rect bestGreenContourRect = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Find purple contours
        Core.inRange(hsvMat, lowerPurple, upperPurple, purpleMask);
        purpleContours.clear();
        Imgproc.findContours(purpleMask, purpleContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        bestPurpleContourRect = findBestContour(purpleContours);

        // Find green contours
        Core.inRange(hsvMat, lowerGreen, upperGreen, greenMask);
        greenContours.clear();
        Imgproc.findContours(greenMask, greenContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        bestGreenContourRect = findBestContour(greenContours);

        return frame; // Return the original frame
    }

    private Rect findBestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
            }
        }
        return bestRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5 * scaleCanvasDensity);

        if (bestPurpleContourRect != null) {
            paint.setColor(Color.MAGENTA);
            canvas.drawRect(scaleRect(bestPurpleContourRect, scaleBmpPxToCanvasPx), paint);
        }

        if (bestGreenContourRect != null) {
            paint.setColor(Color.GREEN);
            canvas.drawRect(scaleRect(bestGreenContourRect, scaleBmpPxToCanvasPx), paint);
        }
    }

    public Rect getBestPurpleContourRect() {
        return bestPurpleContourRect;
    }

    public Rect getBestGreenContourRect() {
        return bestGreenContourRect;
    }

    private android.graphics.Rect scaleRect(Rect rect, float scale) {
        return new android.graphics.Rect((int) (rect.x * scale), (int) (rect.y * scale), (int) ((rect.x + rect.width) * scale), (int) ((rect.y + rect.height) * scale));
    }
}
