package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;

import java.util.concurrent.TimeUnit;

/**
 * Utility osztály FTC kameraprofilokhoz (Logitech C270-hez is biztonságos).
 * Cél: erős fényben stabil AprilTag detektálás (kontraszt megőrzése),
 * beltérben pedig zajmentesebb kép.
 *
 * Használat (INIT alatt javasolt):
 *   CameraProfiles.applyBrightLight(visionPortal, aprilTagProcessor);
 *
 * Ha a kamera nem támogat manuális expozíciót/gain-t, a metódusok
 * visszatérnek false-szal, de megpróbálnak biztonságos beállításokat.
 */
public final class CameraProfiles {

    private CameraProfiles() { }

    /**
     * Erős fény profil: rövid expozíció, alacsony gain, fix WB, fix fókusz (ha támogatott),
     * AprilTag decimation 2.0 (kiegyensúlyozott sebesség/kontraszt).
     * @return igaz, ha a manuális expozíció beállítása sikerült.
     */
    public static boolean applyBrightLight(VisionPortal vp, AprilTagProcessor atp) {
        boolean manualExposure = trySetExposureUs(vp, ExposureControl.Mode.Manual, 500); // 0.5 ms
        trySetGain(vp, 0); // min gain
        trySetWhiteBalanceTemp(vp, 5600); // kültéri napsütés ~5600K
        trySetFocusFixed(vp);
        setAprilTagDecimation(atp, 2.0f);
        return manualExposure;
    }

    /**
     * Beltéri profil: közepes expozíció, mérsékelt gain, melegebb WB, kisebb decimation (több részlet).
     */
    public static boolean applyIndoor(VisionPortal vp, AprilTagProcessor atp) {
        boolean manualExposure = trySetExposureUs(vp, ExposureControl.Mode.Manual, 3000); // 3 ms
        trySetGain(vp, 6);
        trySetWhiteBalanceTemp(vp, 4200); // melegebb műfény
        trySetFocusFixed(vp);
        setAprilTagDecimation(atp, 1.5f);
        return manualExposure;
    }

    /**
     * Kiegyensúlyozott profil: ha nem tudod előre a fényviszonyokat.
     */
    public static boolean applyBalanced(VisionPortal vp, AprilTagProcessor atp) {
        boolean manualExposure = trySetExposureUs(vp, ExposureControl.Mode.Manual, 1500); // 1.5 ms
        trySetGain(vp, 3);
        trySetWhiteBalanceTemp(vp, 5000);
        trySetFocusFixed(vp);
        setAprilTagDecimation(atp, 2.0f);
        return manualExposure;
    }

    // ------ Helpers --------------------------------------------------------

    /**
     * Expozíció beállítása mikro-szekundumban (ha támogatott). Ha a manuális mód
     * nem támogatott, megpróbál Auto módot bekapcsolva tartani és visszatér false-szal.
     */
    public static boolean trySetExposureUs(VisionPortal vp, ExposureControl.Mode mode, long exposureUs) {
        if (vp == null) return false;
        ExposureControl exp = vp.getCameraControl(ExposureControl.class);
        if (exp == null) return false;
        try {
            if (exp.isModeSupported(mode)) {
                exp.setMode(mode);
                exp.setExposure(exposureUs, TimeUnit.MICROSECONDS);
                return true;
            } else {
                // Ha nem támogatott a manuális, hagyjuk Auto-n.
                if (exp.isModeSupported(ExposureControl.Mode.Auto)) {
                    exp.setMode(ExposureControl.Mode.Auto);
                }
                return false;
            }
        } catch (Exception ignored) {
            return false;
        }
    }

    /** Gain beállítása (ha támogatott). */
    public static boolean trySetGain(VisionPortal vp, int gainValue) {
        if (vp == null) return false;
        GainControl gain = vp.getCameraControl(GainControl.class);
        if (gain == null) return false;
        try {
            gain.setGain(gainValue);
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }

    /** Fix fehéregyensúly Kelvinben (ha támogatott). */
    public static boolean trySetWhiteBalanceTemp(VisionPortal vp, int kelvin) {
        if (vp == null) return false;
        WhiteBalanceControl wb = vp.getCameraControl(WhiteBalanceControl.class);
        if (wb == null) return false;
        try {
            wb.setMode(WhiteBalanceControl.Mode.MANUAL);
            wb.setWhiteBalanceTemperature(kelvin);
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }

    /** Fókusz fixre állítása (ha a kamera támogatja). */
    public static boolean trySetFocusFixed(VisionPortal vp) {
        if (vp == null) return false;
        FocusControl focus = vp.getCameraControl(FocusControl.class);
        if (focus == null) return false;
        try {
            if (focus.isModeSupported(FocusControl.Mode.Fixed)) {
                focus.setMode(FocusControl.Mode.Fixed);
                return true;
            } else if (focus.isModeSupported(FocusControl.Mode.Auto)) {
                focus.setMode(FocusControl.Mode.Auto);
                return true;
            }
            return false;
        } catch (Exception ignored) {
            return false;
        }
    }

    /** AprilTag decimation futásidőben (ha a processor támogatja). */
    public static boolean setAprilTagDecimation(AprilTagProcessor atp, float decimation) {
        if (atp == null) return false;
        try {
            // Az FTC AprilTagProcessor támogatja a decimation beállítását.
            atp.setDecimation(decimation);
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }
}
