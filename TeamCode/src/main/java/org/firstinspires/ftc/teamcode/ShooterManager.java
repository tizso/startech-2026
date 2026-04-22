package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages all shooter logic, supporting both encoder-based (velocity) and
 * power-based (no-encoder) control modes. This centralizes configuration
 * and makes the OpModes cleaner.
 */
public class ShooterManager {

    // --- Main Switch: Motor Control Mode ---
    // true = use `setVelocity()` (requires encoder).
    // false = use `setPower()` (no encoder needed).
    public static final boolean USE_ENCODER_FOR_SHOOTER = true;

    // --- Hardware ---
    private final DcMotorEx outtakeLeft;
    private final DcMotorEx outtakeRight;
    private final VoltageSensor batteryVoltageSensor;

    // --- State and Timing ---
    private final ElapsedTime boostTimer = new ElapsedTime();
    private boolean boostActive = false;
    private double lastCalculatedPower = 0.0;
    private double lastCalculatedVelocity = 0.0;

    // --- Distance Constants for Scaling ---
    /** Maximum distance (in inches) to shoot from. The upper limit for scaling. */
    private static final double MAX_SHOOTING_DISTANCE_IN = 123.0;
    /** Minimum distance (in inches) to shoot from. The lower limit for scaling. */
    private static final double MIN_SHOOTING_DISTANCE_IN = 40.0;

    // --- Constants for Velocity-Based Control ---
    /** Minimum velocity (in ticks/sec) for the minimum distance. */
    private static final double MIN_VELOCITY_TICKS_PER_SEC = 1500; // Value to be calibrated
    /** Maximum velocity (in ticks/sec) for the maximum distance. */
    private static final double MAX_VELOCITY_TICKS_PER_SEC = 2150; // Value to be calibrated
    /** Default velocity if no tag is visible. */
    private static final double DEFAULT_VELOCITY_TICKS_PER_SEC = 2000;

    // --- Constants for Power-Based Control (No Encoder) ---
    /** The battery voltage for which the `setPower` values were tuned. The base value for compensation. */
    private static final double NOMINAL_VOLTAGE = 12.5;
    /** Motor power (0.0-1.0) for the maximum distance. */
    private static final double MAX_SHOOTER_POWER = 0.8;
    /** Motor power (0.0-1.0) for the minimum distance. */
    private static final double MIN_SHOOTER_POWER = 0.45;
    /** Default motor power if no AprilTag is visible (distance is unknown). */
    private static final double DEFAULT_SHOOTER_POWER = 0.7;

    // --- General Constants ---
    /** Multiplier to temporarily increase speed/power before shooting. */
    private static final double BOOST_FACTOR = 1.01;
    /** Duration of the active "boost" state, in seconds. */
    private static final double BOOST_TIME_SEC = 0.25;

    /**
     * Constructor for the shooter manager.
     * @param leftMotor The left shooter motor.
     * @param rightMotor The right shooter motor.
     * @param voltageSensor The robot's voltage sensor.
     */
    public ShooterManager(DcMotorEx leftMotor, DcMotorEx rightMotor, VoltageSensor voltageSensor) {
        this.outtakeLeft = leftMotor;
        this.outtakeRight = rightMotor;
        this.batteryVoltageSensor = voltageSensor;
    }

    /**
     * Sets the shooter speed based on the distance to the target, using the selected control mode.
     * @param value The distance to the target, in inches.
     */
    public void setSpeedFromDistance(double value, boolean isAutonomous) {
        if (USE_ENCODER_FOR_SHOOTER) {
            // --- Velocity Control Logic ---
            double targetTicksPerSec = isAutonomous?value:computeVelocity(value);
            this.lastCalculatedVelocity = targetTicksPerSec;
            outtakeLeft.setVelocity(targetTicksPerSec);
            outtakeRight.setVelocity(targetTicksPerSec);
        } else {
            // --- Power Control Logic ---
            double targetPower = isAutonomous?value:computeVelocity(value);
            this.lastCalculatedPower = targetPower;
            outtakeLeft.setPower(targetPower);
            outtakeRight.setPower(targetPower);
        }
    }

    /**
     * Stops the shooter motors.
     */
    public void stop() {
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }

    /**
     * Applies a temporary boost to the shooter motors.
     */
    public void applyBoost() {
        if (USE_ENCODER_FOR_SHOOTER) {
            outtakeLeft.setVelocity(lastCalculatedVelocity * BOOST_FACTOR);
            outtakeRight.setVelocity(lastCalculatedVelocity * BOOST_FACTOR);
        } else {
            double boostedPower = Math.min(lastCalculatedPower * BOOST_FACTOR, 1.0);
            outtakeLeft.setPower(boostedPower);
            outtakeRight.setPower(boostedPower);
        }
        if(outtakeLeft.getVelocity() >= lastCalculatedVelocity) {
            boostActive = true;
            boostTimer.reset();
        }
    }

    /**
     * Checks if the boost duration has expired and restores normal speed.
     * @param currentDistanceInch The current distance to the target, needed to restore speed.
     */
    public void updateBoostState(double currentDistanceInch) {
        if (boostActive && boostTimer.seconds() > BOOST_TIME_SEC) {
            setSpeedFromDistance(currentDistanceInch, false); // Restore normal speed
            boostActive = false;
        }
    }

    /**
     * Checks if the shooter is at its target speed.
     * In power-based mode, this always returns false, relying on a timeout.
     * @return True if the shooter is ready, false otherwise.
     */
    public boolean isReady() {
        if (USE_ENCODER_FOR_SHOOTER) {
            double vL = outtakeLeft.getVelocity();
            double vR = outtakeRight.getVelocity();
            double tol = 0.95; // 92% of target
            return (vL >= lastCalculatedVelocity * tol) && (vR >= lastCalculatedVelocity * tol);
        }
        return false; // For power-based mode, rely on timeout in the OpMode
    }

    /**
     * Returns a string with relevant telemetry data for the current mode.
     * @return A formatted string for telemetry.
     */
    public String getTelemetryData() {
        if (USE_ENCODER_FOR_SHOOTER) {
            return String.format("VELOCITY | Tgt: %.1f | Left: %.1f | Right: %.1f ", lastCalculatedVelocity, outtakeLeft.getVelocity(), outtakeRight.getVelocity());
        } else {
            return String.format("POWER | Tgt: %.2f | V: %.2fV", lastCalculatedPower, getBatteryVoltage());
        }
    }


    public double getTargetVelocityTicksPerSec() {
        return lastCalculatedVelocity;
    }

    public double getAverageVelocityTicksPerSec() {
        double vL = outtakeLeft.getVelocity();  // ticks/sec
        double vR = outtakeRight.getVelocity(); // ticks/sec
        return (vL + vR) / 2.0;
    }


    // --- Private Helper Methods ---

    private double computeVelocity(double distanceInch) {
        if (distanceInch <= 0) { // Safety for invalid distance
            return DEFAULT_VELOCITY_TICKS_PER_SEC;
        }
        // Clamp the distance to our effective shooting range
        double clampedDistance = Math.max(MIN_SHOOTING_DISTANCE_IN, Math.min(distanceInch, MAX_SHOOTING_DISTANCE_IN));

        // Linearly interpolate the velocity based on distance
        double distanceRatio = (clampedDistance - MIN_SHOOTING_DISTANCE_IN) / (MAX_SHOOTING_DISTANCE_IN - MIN_SHOOTING_DISTANCE_IN);
        double targetVelocity = MIN_VELOCITY_TICKS_PER_SEC + distanceRatio * (MAX_VELOCITY_TICKS_PER_SEC - MIN_VELOCITY_TICKS_PER_SEC);
        if(targetVelocity <= 1400){
            targetVelocity = DEFAULT_VELOCITY_TICKS_PER_SEC;
        }
        return targetVelocity;
    }

    private double computePower(double distanceInch) {
        if (distanceInch <= 0) { // Safety for invalid distance
            return DEFAULT_SHOOTER_POWER;
        }
        double clampedDistance = Math.max(MIN_SHOOTING_DISTANCE_IN, Math.min(distanceInch, MAX_SHOOTING_DISTANCE_IN));
        double distanceRatio = (clampedDistance - MIN_SHOOTING_DISTANCE_IN) / (MAX_SHOOTING_DISTANCE_IN - MIN_SHOOTING_DISTANCE_IN);
        double targetPower = MIN_SHOOTER_POWER + distanceRatio * (MAX_SHOOTER_POWER - MIN_SHOOTER_POWER);

        double currentVoltage = getBatteryVoltage();
        double compensatedPower = targetPower * (NOMINAL_VOLTAGE / currentVoltage);
        return Math.min(compensatedPower, 1.0);
    }
    
    private double getBatteryVoltage() {
        double voltage = batteryVoltageSensor.getVoltage();
        if (voltage <= 9) { // Safety check for invalid reading
            return NOMINAL_VOLTAGE; 
        }
        return voltage;
    }
}
