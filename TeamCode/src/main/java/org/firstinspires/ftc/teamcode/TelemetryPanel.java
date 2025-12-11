package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * TelemetryPanel – egységes kamera + AprilTag diagnosztikai panel FTC-hez.
 *
 * Cél: egy helyen lásd a kritikus adatokat:
 *  - Tag detektálási állapot (ID, távolság, yaw, bearing, seen ago)
 *  - Kamera beállítások (exposure, gain, white balance, focus)
 *  - AprilTag processor decimation
 *  - VisionPortal FPS
 *
 * Használat (INIT és futás közben):
 *   TelemetryPanel panel = new TelemetryPanel(telemetry, visionPortal, aprilTagProcessor);
 *   // INIT loop
 *   panel.update();
 *   panel.renderFull();
 *   telemetry.update();
 *   // START után a fő ciklusban
 *   panel.update();
 *   panel.renderMinimal(); // vagy renderFull();
 *
 * A panel belső időzítést használ (ElapsedTime) a "seen ago" megjelenítéshez.
 */
public class TelemetryPanel {

    private final Telemetry telemetry;
    private final VisionPortal vp;
    private final AprilTagProcessor atp;
    private final ElapsedTime clock = new ElapsedTime();

    private double lastSeenTimeSec = Double.NaN;
    private AprilTagDetection bestDet = null;
    private int numDetections = 0;
    private double fps = Double.NaN;

    public TelemetryPanel(Telemetry telemetry, VisionPortal vp, AprilTagProcessor atp) {
        this.telemetry = telemetry;
        this.vp = vp;
        this.atp = atp;
        clock.reset();
    }

    /** Frissíti a legutolsó detektálást, számlálót és FPS-t. */
    public void update() {
        List<AprilTagDetection> dets;
        try {
            dets = (atp != null) ? atp.getDetections() : Collections.emptyList();
        } catch (Exception e) {
            dets = Collections.emptyList();
        }

        numDetections = (dets != null) ? dets.size() : 0;
        if (numDetections > 0) {
            bestDet = dets.stream()
                    .filter(d -> d != null && d.ftcPose != null)
                    .min(Comparator.comparingDouble(d -> d.ftcPose.range))
                    .orElse(dets.get(0));
            lastSeenTimeSec = clock.seconds();
        }

        // FPS, ha elérhető
        fps = tryGetFps();
    }

    /** Teljes panel – INIT alatt hasznos. */
    public void renderFull() {
        telemetry.addLine("=== CAMERA & TAG DIAG ===");
        renderTagBlock();
        renderCameraBlock();
        //renderProcessorBlock();
        renderStreamBlock();
    }

    /** Minimal panel – futás közben hasznos. */
    public void renderMinimal() {
        telemetry.addLine("=== CAM/TAG ===");
        renderTagSummary();
        renderStreamBlock();
    }

    // ---- Blocks ----

    private void renderTagBlock() {
        if (bestDet != null && bestDet.ftcPose != null) {
            telemetry.addData("Tag(s)", numDetections);
            telemetry.addData("BestID", bestDet.id);
            telemetry.addData("Range(m)", fmt(bestDet.ftcPose.range));
            telemetry.addData("Yaw(deg)", fmt(bestDet.ftcPose.yaw));
            telemetry.addData("Bear(deg)", fmt(bestDet.ftcPose.bearing));
            telemetry.addData("Seen ago(s)", fmt(seenAgo()));
        } else {
            telemetry.addData("Tag(s)", numDetections);
            telemetry.addData("BestID", "-");
            telemetry.addData("Range(m)", "-");
            telemetry.addData("Yaw/Bear", "-");
            telemetry.addData("Seen ago(s)", fmt(seenAgo()));
        }
    }

    private void renderTagSummary() {
        if (bestDet != null && bestDet.ftcPose != null) {
            telemetry.addData("Tag", String.format("%d  R=%.2f  Y=%.1f  t=%.1fs", bestDet.id,
                    safe(bestDet.ftcPose.range), safe(bestDet.ftcPose.yaw), seenAgo()));
        } else {
            telemetry.addData("Tag", String.format("none  t=%.1fs", seenAgo()));
        }
    }

    private void renderCameraBlock() {
        // Exposure
        ExposureControl exp = getCtrl(ExposureControl.class);
        if (exp != null) {
            try {
                ExposureControl.Mode mode = exp.getMode();
                long us = exp.getExposure(TimeUnit.MICROSECONDS);
                telemetry.addData("Exposure", mode + ", " + us + "us");
            } catch (Exception e) {
                telemetry.addData("Exposure", "(n/a)");
            }
        } else {
            telemetry.addData("Exposure", "(no ctrl)");
        }

        // Gain
        GainControl gain = getCtrl(GainControl.class);
        if (gain != null) {
            try {
                telemetry.addData("Gain", gain.getGain());
            } catch (Exception e) {
                telemetry.addData("Gain", "(n/a)");
            }
        } else {
            telemetry.addData("Gain", "(no ctrl)");
        }

        // White balance
        WhiteBalanceControl wb = getCtrl(WhiteBalanceControl.class);
        if (wb != null) {
            try {
                telemetry.addData("WB", wb.getMode() + ", " + wb.getWhiteBalanceTemperature() + "K");
            } catch (Exception e) {
                telemetry.addData("WB", "(n/a)");
            }
        } else {
            telemetry.addData("WB", "(no ctrl)");
        }

        // Focus
        FocusControl focus = getCtrl(FocusControl.class);
        if (focus != null) {
            try {
                telemetry.addData("Focus", focus.getMode());
            } catch (Exception e) {
                telemetry.addData("Focus", "(n/a)");
            }
        } else {
            telemetry.addData("Focus", "(no ctrl)");
        }
    }

    /*private void renderProcessorBlock() {
        float dec = tryGetDecimation();
        if (!Float.isNaN(dec)) {
            telemetry.addData("AT decimation", dec);
        } else {
            telemetry.addData("AT decimation", "(n/a)");
        }
    }*/

    private void renderStreamBlock() {
        if (!Double.isNaN(fps)) {
            telemetry.addData("FPS", fmt(fps));
        }
    }

    // ---- Helpers ----

    private double seenAgo() {
        if (Double.isNaN(lastSeenTimeSec)) return Double.POSITIVE_INFINITY;
        return Math.max(0.0, clock.seconds() - lastSeenTimeSec);
    }

    /*private float tryGetDecimation() {
        try {
            return atp.getDecimation();
        } catch (Exception e) {
            return Float.NaN;
        }
    }*/

    private double tryGetFps() {
        try {
            return (vp != null) ? vp.getFps() : Double.NaN;
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    private <T extends CameraControl> T getCtrl(Class<T> cls) {
        try {
            return (vp != null) ? vp.getCameraControl(cls) : null;
        } catch (Exception e) {
            return null;
        }
    }

    private static String fmt(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "-";
        return String.format(java.util.Locale.US, "%.2f", v);
    }

    private static double safe(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return Double.NaN;
        return v;
    }
}
