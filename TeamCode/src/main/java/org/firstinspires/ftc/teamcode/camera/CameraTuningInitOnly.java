
package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/**
 * Camera Tuning (Init-only, Profiles + Advisor)
 * --------------------------------------------
 * - Works only during INIT loop (FTC legal). No changes after START.
 * - Profile management: R3/L3 cycle profiles, START marks active, B saves, BACK x2 deletes.
 * - Advisor: suggests which parameter to change (exposure/gain) and in which direction.
 */
@TeleOp(name="Camera Tuning (Init-only, Profiles + Advisor)", group="Vision")
public class CameraTuningInitOnly extends LinearOpMode {

    private VisionPortal vp;
    private AprilTagProcessor tag;
    private CameraSettingsManager camMgr;
    private Gamepad last = new Gamepad();

    private List<String> profiles;
    private int profIndex = 0;
    private long deleteArmTs = 0; // double-press confirmation window for deletion

    // --- Advisor state ---
    private static final int WINDOW = 12;            // ~0.6s if loop ~50ms
    private static final double BEARING_STD_HIGH = 3.0; // deg; motion blur suspicion
    private static final int MISS_FRAMES_HIGH = 16;     // how many frames since last detection
    private static final double RANGE_NEAR_IN = 24.0;
    private static final double RANGE_FAR_IN  = 120.0;

    private Deque<Double> bearingBuf = new ArrayDeque<>();
    private Deque<Double> rangeBuf   = new ArrayDeque<>();
    private int framesSinceSeen = 0;

    private enum AdviceType {
        NONE,
        EXPOSURE_UP_SMALL, EXPOSURE_DOWN_SMALL,
        EXPOSURE_UP_LARGE, EXPOSURE_DOWN_LARGE,
        GAIN_UP_SMALL,     GAIN_DOWN_SMALL,
        GAIN_UP_LARGE,     GAIN_DOWN_LARGE
    }

    private static class Advice {
        AdviceType type = AdviceType.NONE;
        String message = "Stable detection. Minor fine-tuning is optional.";
        int expDeltaMs = 0;
        int gainDelta  = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Vision init
        tag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();
        vp  = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tag)
                .build();

        // Manager + seed default profiles
        camMgr = new CameraSettingsManager(telemetry);
        camMgr.seedDefaultProfilesIfMissing();
        camMgr.loadActiveProfile();
        profiles = camMgr.listProfiles();

        if (!profiles.contains(camMgr.getProfile())) {
            camMgr.setProfile(profiles.get(0));
        }
        profIndex = profiles.indexOf(camMgr.getProfile());
        camMgr.load();
        camMgr.attachAndApply(vp);

        telemetry.addLine("Camera Tuning (Init-only, Profiles + Advisor)");
        telemetry.addLine("R3/L3: switch profile  |  START: mark active  |  B: SAVE  |  BACK x2: DELETE");
        telemetry.addLine("A: MANUAL/AUTO  |  X: DEFAULT  |  Y: LOAD");
        telemetry.addLine("DPad Up/Down: exposure ±1ms  |  Left/Right: gain ±1  |  LB/RB: exposure+gain ±5");
        telemetry.addLine("Advisor suggests which parameter to change for better Tag visibility/stability.");
        telemetry.update();

        // INIT-only loop
        while (opModeInInit() && !isStopRequested()) {
            handleButtons();

            // Advisor + telemetry
            AprilTagDetection best = getClosestByRange();
            boolean visible = (best != null && best.ftcPose != null);
            Double rangeIn = null, bearingDeg = null;

            if (visible) {
                rangeIn    = best.ftcPose.range;
                bearingDeg = best.ftcPose.bearing;
                framesSinceSeen = 0;

                push(rangeBuf, rangeIn, WINDOW);
                push(bearingBuf, bearingDeg, WINDOW);
            } else {
                framesSinceSeen++;
            }

            Advice advice = makeAdvice(visible, rangeIn, bearingStd(bearingBuf), framesSinceSeen);
            addTelemetry(visible, rangeIn, bearingDeg, advice);
            telemetry.update();

            // Optional: apply large-step advice quickly if driver presses bumpers:
            quickApplyLargeAdvice(advice);

            sleep(50);
        }

        if (vp != null) vp.close();
    }

    private void handleButtons() {
        // Profile switching: R3/L3
        if (gamepad1.right_stick_button && !last.right_stick_button) { // next
            profIndex = (profIndex + 1) % profiles.size();
            camMgr.setProfile(profiles.get(profIndex));
            camMgr.load(); camMgr.apply();
            clearAdvisor();
        }
        if (gamepad1.left_stick_button && !last.left_stick_button) { // prev
            profIndex = (profIndex - 1 + profiles.size()) % profiles.size();
            camMgr.setProfile(profiles.get(profIndex));
            camMgr.load(); camMgr.apply();
            clearAdvisor();
        }

        // Mark active / Save
        if (gamepad1.start && !last.start) camMgr.saveActiveProfile();
        if (gamepad1.b && !last.b) { camMgr.save(); profiles = camMgr.listProfiles(); }

        // Modify settings
        if (gamepad1.a && !last.a) { camMgr.toggleMode(); clearAdvisor(); }
        if (gamepad1.x && !last.x) { camMgr.resetDefaults(8, 15, true); clearAdvisor(); }
        if (gamepad1.y && !last.y) { camMgr.load(); camMgr.apply(); clearAdvisor(); }

        if (gamepad1.dpad_up    && !last.dpad_up)    { camMgr.bumpExposure(+1); clearAdvisor(); }
        if (gamepad1.dpad_down  && !last.dpad_down)  { camMgr.bumpExposure(-1); clearAdvisor(); }
        if (gamepad1.dpad_right && !last.dpad_right) { camMgr.bumpGain(+1);     clearAdvisor(); }
        if (gamepad1.dpad_left  && !last.dpad_left)  { camMgr.bumpGain(-1);     clearAdvisor(); }

        if (gamepad1.right_bumper && !last.right_bumper) { camMgr.bumpExposure(+5); camMgr.bumpGain(+5); clearAdvisor(); }
        if (gamepad1.left_bumper  && !last.left_bumper)  { camMgr.bumpExposure(-5); camMgr.bumpGain(-5); clearAdvisor(); }

        // Delete profile: BACK twice within 2s
        if (gamepad1.back && !last.back) {
            long now = System.currentTimeMillis();
            if (now - deleteArmTs < 2000) {
                String toDel = profiles.get(profIndex);
                if (camMgr.deleteProfile(toDel)) {
                    telemetry.addData("Deleted profile", toDel);
                    profiles = camMgr.listProfiles();
                    profIndex = 0;
                    camMgr.setProfile(profiles.get(profIndex));
                    camMgr.load(); camMgr.apply();
                    clearAdvisor();
                } else {
                    telemetry.addData("Delete failed", toDel);
                }
                deleteArmTs = 0;
            } else {
                deleteArmTs = now;
                telemetry.addLine("Press BACK again within 2s to DELETE current profile!");
            }
        }

        last.copy(gamepad1);
    }

    // --- Advisor logic ---
    private Advice makeAdvice(boolean visible, Double rangeInch, double bearingStdDeg, int missFrames) {
        Advice adv = new Advice();

        // No tag visible for a while: brighten image (prefer exposure first in MANUAL)
        if (!visible && missFrames >= MISS_FRAMES_HIGH) {
            if (camMgr.isManual()) {
                adv.type = AdviceType.EXPOSURE_UP_LARGE;
                adv.expDeltaMs = +5;
                adv.message = "Tag not visible: increase exposure (+5 ms). If still missing, increase gain.";
            } else {
                adv.type = AdviceType.GAIN_UP_LARGE;
                adv.gainDelta = +5;
                adv.message = "AUTO mode and no tag visible: increase gain (+5).";
            }
            return adv;
        }

        if (!visible) {
            adv.type = AdviceType.EXPOSURE_UP_SMALL;
            adv.expDeltaMs = +1;
            adv.message = "Tag not visible: try +1 ms exposure, then +1 gain if needed.";
            return adv;
        }

        // Tag visible: check stability (bearing variance)
        if (bearingStdDeg > BEARING_STD_HIGH && camMgr.isManual()) {
            adv.type = AdviceType.EXPOSURE_DOWN_LARGE;
            adv.expDeltaMs = -3;
            adv.message = String.format("Unstable aim (bearing σ≈%.1f°): decrease exposure (−3 ms) to reduce motion blur.", bearingStdDeg);
            return adv;
        }

        // Very close: risk of overexposure
        if (rangeInch != null && rangeInch < RANGE_NEAR_IN && camMgr.isManual()) {
            adv.type = AdviceType.EXPOSURE_DOWN_SMALL;
            adv.expDeltaMs = -1;
            adv.message = "Close tag (<24\"): decrease exposure (−1 ms) to avoid washout.";
            return adv;
        }

        // Very far: lift gain first
        if (rangeInch != null && rangeInch > RANGE_FAR_IN) {
            adv.type = AdviceType.GAIN_UP_SMALL;
            adv.gainDelta = +1;
            adv.message = "Far tag (>120\"): increase gain (+1). Consider a small exposure bump if still weak.";
            return adv;
        }

        adv.type = AdviceType.NONE;
        adv.message = "Stable detection. Minor fine-tuning is optional (±1 ms / ±1 gain).";
        return adv;
    }

    private void addTelemetry(boolean visible, Double rangeIn, Double bearingDeg, Advice adv) {
        telemetry.addData("Profile", camMgr.getProfile());
        telemetry.addData("AprilTag visible", visible ? "Yes" : "No");
        if (visible) {
            telemetry.addData("Range (in)", String.format("%.1f", rangeIn));
            telemetry.addData("Bearing (deg)", String.format("%.1f", bearingDeg));
            telemetry.addData("Bearing σ (deg)", String.format("%.1f", bearingStd(bearingBuf)));
        } else {
            telemetry.addData("Frames since last detection", framesSinceSeen);
        }

        telemetry.addData("Mode", camMgr.isManual() ? "MANUAL" : "AUTO");
        telemetry.addData("Exposure (ms)", camMgr.getExposureMs());
        telemetry.addData("Gain", camMgr.getGain());

        telemetry.addLine("--- Advisor ---");
        telemetry.addData("Suggestion", adv.message);
        switch (adv.type) {
            case EXPOSURE_UP_LARGE:
                telemetry.addData("Quick key", "RB (+5 ms) or DPad Up (+1 ms)"); break;
            case EXPOSURE_DOWN_LARGE:
                telemetry.addData("Quick key", "LB (−5 ms) or DPad Down (−1 ms)"); break;
            case EXPOSURE_UP_SMALL:
                telemetry.addData("Quick key", "DPad Up (+1 ms)"); break;
            case EXPOSURE_DOWN_SMALL:
                telemetry.addData("Quick key", "DPad Down (−1 ms)"); break;
            case GAIN_UP_LARGE:
                telemetry.addData("Quick key", "RB (+5 gain) or DPad Right (+1)"); break;
            case GAIN_DOWN_LARGE:
                telemetry.addData("Quick key", "LB (−5 gain) or DPad Left (−1)"); break;
            case GAIN_UP_SMALL:
                telemetry.addData("Quick key", "DPad Right (+1 gain)"); break;
            case GAIN_DOWN_SMALL:
                telemetry.addData("Quick key", "DPad Left (−1 gain)"); break;
            default:
                telemetry.addData("Quick key", "Fine: DPad (±1), Coarse: LB/RB (±5)");
        }
    }

    private void quickApplyLargeAdvice(Advice adv) {
        if (adv.type == AdviceType.EXPOSURE_UP_LARGE && gamepad1.right_bumper && !last.right_bumper) {
            camMgr.bumpExposure(+5);
        }
        if (adv.type == AdviceType.EXPOSURE_DOWN_LARGE && gamepad1.left_bumper && !last.left_bumper) {
            camMgr.bumpExposure(-5);
        }
        if (adv.type == AdviceType.GAIN_UP_LARGE && gamepad1.right_bumper && !last.right_bumper) {
            camMgr.bumpGain(+5);
        }
        if (adv.type == AdviceType.GAIN_DOWN_LARGE && gamepad1.left_bumper && !last.left_bumper) {
            camMgr.bumpGain(-5);
        }
    }

    // --- Helpers ---
    private AprilTagDetection getClosestByRange() {
        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection d : tag.getDetections()) {
            if (d != null && d.ftcPose != null) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
            }
        }
        return best;
    }

    private static void push(Deque<Double> buf, Double v, int win) {
        if (v == null) return;
        buf.addLast(v);
        while (buf.size() > win) buf.removeFirst();
    }

    private static double bearingStd(Deque<Double> buf) {
        if (buf.isEmpty()) return 0.0;
        double mean = 0.0;
        for (double x : buf) mean += x;
        mean /= buf.size();
        double var = 0.0;
        for (double x : buf) var += (x - mean) * (x - mean);
        var /= buf.size();
        return Math.sqrt(var);
    }

    private void clearAdvisor() {
        bearingBuf.clear();
        rangeBuf.clear();
        framesSinceSeen = 0;
    }
}
