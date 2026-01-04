
package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * CameraSettingsManager
 * ---------------------
 * - Handles exposure/gain settings with named profiles.
 * - Persists settings under: /sdcard/FIRST/camera_settings_<profile>.txt
 * - Active profile name under: /sdcard/FIRST/camera_settings_active.txt
 *
 * Seed profiles are tailored for Logitech C270 and FTC DECODE lighting patterns.
 */
public class CameraSettingsManager {
    // --- Profile name and files ---
    private String profile = "default";

    private File getProfileFile(String prof) {
        return AppUtil.getInstance().getSettingsFile("camera_settings_" + prof + ".txt");
    }

    private File getActiveProfileFile() {
        return AppUtil.getInstance().getSettingsFile("camera_settings_active.txt");
    }

    // --- Current values (good starting points for C270 in DECODE) ---
    private boolean manualExposure = true; // true=MANUAL, false=AUTO
    private long exposureMs = 8;           // 5–10 ms is a typical DECODE starting range
    private int gainValue = 15;            // moderate gain

    // --- Limits (camera dependent; queried from VisionPortal if available) ---
    private long minExpMs = 1, maxExpMs = 50;
    private int minGain = 1, maxGain = 255;

    // --- Controls ---
    private ExposureControl exposureCtl;
    private GainControl gainCtl;

    private final Telemetry telemetry;

    public CameraSettingsManager(Telemetry telemetry) { this.telemetry = telemetry; }

    // ======================== Profile management ========================
    /** Creates default (seed) profiles for C270 if missing. */
    public void seedDefaultProfilesIfMissing() {
        // MANUAL mode – C270 typical DECODE starting values (ms/gain)
        ensureProfile("default", true, 8, 15);
        ensureProfile("match_indoor_gym", true, 6, 18);
        ensureProfile("match_arena_bright", true, 5, 12);
        ensureProfile("practice_lab", true, 10, 15);
        ensureProfile("warehouse_sunlight", true, 4, 12);
        ensureProfile("low_light_training", true, 12, 25);

        // If no active profile yet, pick a sensible default:
        File act = getActiveProfileFile();
        if (!act.exists()) {
            setProfile("match_indoor_gym");
            saveActiveProfile();
        }
    }

    /** Creates a profile file if it does not exist. */
    private void ensureProfile(String prof, boolean manual, long expMs, int gain) {
        File f = getProfileFile(prof);
        if (f.exists()) return;
        String s = "mode=" + (manual ? "MANUAL" : "AUTO") + "\n" +
                "exposureMs=" + expMs + "\n" +
                "gain=" + gain + "\n";
        ReadWriteFile.writeFile(f, s);
    }

    public void setProfile(String prof) { if (prof != null && !prof.isEmpty()) profile = prof; }
    public String getProfile() { return profile; }

    /** Persists the active profile name. */
    public void saveActiveProfile() {
        ReadWriteFile.writeFile(getActiveProfileFile(), profile + "\n");
    }

    /** Loads the active profile name; defaults to "default" if not present. */
    public void loadActiveProfile() {
        File f = getActiveProfileFile();
        if (!f.exists()) { profile = "default"; return; }
        String s = ReadWriteFile.readFile(f).trim();
        profile = (s.isEmpty() ? "default" : s);
    }

    /** Lists available profile files under /sdcard/FIRST. */
    public List<String> listProfiles() {
        List<String> out = new ArrayList<>();
        File dir = getActiveProfileFile().getParentFile();
        File[] files = dir.listFiles((d, name) ->
                name.startsWith("camera_settings_") && name.endsWith(".txt") && !name.contains("active"));
        if (files != null) {
            for (File f : files) {
                String nm = f.getName().replace("camera_settings_", "").replace(".txt", "");
                out.add(nm);
            }
        }
        if (out.isEmpty()) out.add("default");
        return out;
    }

    /** Deletes the profile file (settings only, not the active-name file). */
    public boolean deleteProfile(String prof) {
        File f = getProfileFile(prof);
        return f.exists() && f.delete();
    }

    // ======================== Load / Save settings ========================
    /** Loads settings from the current profile file. */
    public void load() {
        File file = getProfileFile(profile);
        if (!file.exists()) return;
        String s = ReadWriteFile.readFile(file);
        for (String line : s.split("\\r?\\n")) {
            String[] kv = line.split("=");
            if (kv.length != 2) continue;
            String k = kv[0].trim(), v = kv[1].trim();
            try {
                switch (k) {
                    case "mode": manualExposure = "MANUAL".equalsIgnoreCase(v); break;
                    case "exposureMs": exposureMs = Long.parseLong(v); break;
                    case "gain": gainValue = Integer.parseInt(v); break;
                }
            } catch (Exception ignore) {}
        }
    }

    /** Saves settings to the current profile file. */
    public void save() {
        File file = getProfileFile(profile);
        String s = "mode=" + (manualExposure ? "MANUAL" : "AUTO") + "\n" +
                "exposureMs=" + exposureMs + "\n" +
                "gain=" + gainValue + "\n";
        ReadWriteFile.writeFile(file, s);
    }

    // ======================== VisionPortal hookup ========================
    /**
     * Grabs camera controls, reads limits, then applies current values.
     * IMPORTANT: Only safe when the portal is STREAMING.
     */
    public void attachAndApply(VisionPortal vp) {
        if (vp == null || vp.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // Not streaming yet: skip attaching controls to avoid IllegalStateException
            if (telemetry != null) {
                telemetry.addLine("Camera not STREAMING yet; skipping control attach.");
            }
            return;
        }

        exposureCtl = vp.getCameraControl(ExposureControl.class);
        gainCtl     = vp.getCameraControl(GainControl.class);

        if (exposureCtl != null) {
            try {
                minExpMs = exposureCtl.getMinExposure(TimeUnit.MILLISECONDS);
                maxExpMs = exposureCtl.getMaxExposure(TimeUnit.MILLISECONDS);
            } catch (Throwable ignore) {}
        }
        if (gainCtl != null) {
            try {
                minGain = gainCtl.getMinGain();
                maxGain = gainCtl.getMaxGain();
            } catch (Throwable ignore) {}
        }
        apply(); // Will be guarded with try/catch internally
    }

    /**
     * Applies MANUAL/AUTO mode, clamps exposure/gain within camera limits.
     * Robust against IllegalStateException if streaming stops.
     */
    public void apply() {
        // If we have no controls, nothing to do.
        if (exposureCtl == null && gainCtl == null) return;

        // Exposure
        if (exposureCtl != null) {
            try {
                exposureCtl.setMode(manualExposure ? ExposureControl.Mode.Manual
                        : ExposureControl.Mode.Auto);
                if (manualExposure) {
                    long clamped = clamp(exposureMs, minExpMs, maxExpMs);
                    exposureCtl.setExposure(clamped, TimeUnit.MILLISECONDS);
                    exposureMs = clamped;
                }
            } catch (IllegalStateException ise) {
                if (telemetry != null) telemetry.addData("Exposure apply skipped", ise.getMessage());
            } catch (Throwable t) {
                if (telemetry != null) telemetry.addData("Exposure apply error", t.getMessage());
            }
        }

        // Gain
        if (gainCtl != null) {
            try {
                int clamped = (int) clamp(gainValue, minGain, maxGain);
                gainCtl.setGain(clamped);
                gainValue = clamped;
            } catch (IllegalStateException ise) {
                if (telemetry != null) telemetry.addData("Gain apply skipped", ise.getMessage());
            } catch (Throwable t) {
                if (telemetry != null) telemetry.addData("Gain apply error", t.getMessage());
            }
        }
    }

    // ======================== Gamepad helpers ========================
    public void toggleMode() { manualExposure = !manualExposure; apply(); }
    public void bumpExposure(int deltaMs) { exposureMs = clamp(exposureMs + deltaMs, minExpMs, maxExpMs); apply(); }
    public void bumpGain(int delta)      { gainValue = (int) clamp(gainValue + delta, minGain, maxGain); apply(); }
    public void resetDefaults(long defExpMs, int defGain, boolean defManual) {
        manualExposure = defManual; exposureMs = defExpMs; gainValue = defGain; apply();
    }

    // ======================== Telemetry (English) ========================
    public void addTelemetry(boolean tagVisible, Double tagRangeInch) {
        if (telemetry == null) return;
        telemetry.addLine("=== Camera Settings ===");
        telemetry.addData("Profile", profile);
        telemetry.addData("Mode", manualExposure ? "MANUAL" : "AUTO");
        telemetry.addData("Exposure (ms)", "%d (min:%d max:%d)", exposureMs, minExpMs, maxExpMs);
        telemetry.addData("Gain", "%d (min:%d max:%d)", gainValue, minGain, maxGain);
        telemetry.addData("AprilTag visible", tagVisible ? "Yes" : "No");
        String tip = "";
        if (!tagVisible) {
            tip = "Increase exposure (+1..+5 ms) or gain (+1..+5). Check camera angle.";
        } else if (tagRangeInch != null) {
            if (tagRangeInch < 24 && manualExposure && exposureMs > (minExpMs + 3)) {
                tip = "Very close tag: decrease exposure (−1..−3 ms) to avoid overexposure.";
            } else if (tagRangeInch > 120 && gainValue < (maxGain - 5)) {
                tip = "Distant tag: increase gain (+1..+5), consider a small exposure bump.";
            } else {
                tip = "If loss during motion, decrease exposure to reduce motion blur.";
            }
        }
        telemetry.addData("Tip", tip);
    }

    // --- Getters ---
    public boolean isManual()   { return manualExposure; }
    public long getExposureMs() { return exposureMs; }
    public int getGain()        { return gainValue; }

    // --- Helpers ---
    private static long clamp(long v, long lo, long hi) { return Math.max(lo, Math.min(hi, v)); }
}
