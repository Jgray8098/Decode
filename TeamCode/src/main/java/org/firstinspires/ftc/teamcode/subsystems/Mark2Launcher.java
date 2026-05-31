package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.LAUNCHER_MOTOR_ONE;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.LAUNCHER_MOTOR_TWO;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.GATE_SERVO_LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.GATE_SERVO_RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.AIM_SERVO_LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.AIM_SERVO_RIGHT;


/**
 * Launcher hardware subsystem.
 *
 * <p>This class owns motors, servos, and RPM measurement only. Manual and
 * automated game logic live in launcher controller classes.</p>
 */
public class Mark2Launcher {

    // Hardware-map names live in Mark2HardwareMapNames — imported as static above.

    // -------------------------------------------------------------------------
    // Servo positions  (tune!)
    // -------------------------------------------------------------------------
    /** Hood servo resting position when launcher is idle. */
    public static final double HOOD_SERVO_RESET_POSITION  = 0.0;
    /** Gate servos "push" position — feeds ball into launch mechanism. */
    public static final double FEEDER_SERVO_FEED_POSITION = 0.25;
    /** Gate servos retracted / resting position — default when not firing. */
    public static final double FEEDER_SERVO_IDLE_POSITION = 0.50;

    /** Minimum aim servo position reached by full-left stick input. */
    public static final double AIM_MIN_POS = 0.0;
    /** Maximum aim servo position reached by full-right stick input. */
    public static final double AIM_MAX_POS = 1.0;
    /** Stick deadzone applied to manual aim control. */
    public static final double AIM_STICK_DEADZONE = 0.08;
    /** Maximum manual aim travel speed in servo-position units per second. */
    public static final double AIM_MAX_RATE_PER_SEC = 0.35;
    /** Curves manual aim stick input so small stick motions are less sensitive. */
    public static final double AIM_STICK_CURVE = 2.0;

    /** Encoder ticks per motor revolution. Update if the launcher motor model changes. */
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    /** Flywheel revolutions per motor revolution. Mark2 launcher is currently 1:1. */
    private static final double FLYWHEEL_PER_MOTOR_REV = 1.0;

    /** Approximate free speed used by the custom feedforward model. Tune on robot. */
    private static final double FLYWHEEL_MAX_RPM = 6000.0;

    /** Nominal battery voltage used for voltage compensation. */
    private static final double NOMINAL_VOLTAGE = 12.0;

    // Default custom flywheel velocity controller gains.
    private static final double DEFAULT_FLYWHEEL_KS = 0.2;
    private static final double DEFAULT_FLYWHEEL_KV =
            (1.0 - DEFAULT_FLYWHEEL_KS) / FLYWHEEL_MAX_RPM;
    private static final double DEFAULT_FLYWHEEL_KP = 0.001;
    private static final double DEFAULT_FLYWHEEL_KI = 0.0;
    private static final double DEFAULT_FLYWHEEL_KD = 0.0;

    private static final double FLYWHEEL_INTEGRAL_MIN = -500.0;
    private static final double FLYWHEEL_INTEGRAL_MAX = 500.0;

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx launcherMotorOne;
    private DcMotorEx launcherMotorTwo;
    private VoltageSensor battery;
    private Servo hoodPositionServo;
    private Servo gateServoLeft;
    private Servo gateServoRight;
    /** Aim (turret) servos — pan the launcher left/right. */
    private Servo aimServoLeft;
    private Servo aimServoRight;

    /** Tracks the last commanded aim position for telemetry. */
    private double aimServoPos = 0.5;

    /** True when all servos were successfully initialised. False = motors only. */
    private final boolean hasServos;

    // -------------------------------------------------------------------------
    // RPM measurement state
    // -------------------------------------------------------------------------
    private double measuredRpm  = 0.0;
    private double measuredRpmOne = 0.0;
    private double measuredRpmTwo = 0.0;
    private double targetRpm = 0.0;
    private double targetTicksPerSec = 0.0;
    private double flywheelPowerOne = 0.0;
    private double flywheelPowerTwo = 0.0;
    private double flywheelKs = DEFAULT_FLYWHEEL_KS;
    private double flywheelKv = DEFAULT_FLYWHEEL_KV;
    private double flywheelKp = DEFAULT_FLYWHEEL_KP;
    private double flywheelKi = DEFAULT_FLYWHEEL_KI;
    private double flywheelKd = DEFAULT_FLYWHEEL_KD;
    private double integralOne = 0.0;
    private double integralTwo = 0.0;
    private double previousErrorOne = 0.0;
    private double previousErrorTwo = 0.0;

    // Fallback tick-based velocity
    private int lastPosOne = 0;
    private int lastPosTwo = 0;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Full constructor — requires all motors AND servos to be present in
     * the hardware configuration.  Use this once all hardware is installed.
     */
    public Mark2Launcher(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /**
     * Partial constructor for staged hardware bring-up.
     *
     * @param hardwareMap the OpMode hardware map
     * @param hasServos   {@code true}  = full init (motors + servos)<br>
     *                    {@code false} = motors only; servo fields are left null
     *                    and every servo call in this class is skipped safely.
     */
    public Mark2Launcher(HardwareMap hardwareMap, boolean hasServos) {
        this.hasServos = hasServos;

        launcherMotorOne = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_ONE);
        launcherMotorTwo = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_TWO);

        if (hardwareMap.voltageSensor != null && hardwareMap.voltageSensor.iterator().hasNext()) {
            battery = hardwareMap.voltageSensor.iterator().next();
        }

        if (hasServos) {
            hoodPositionServo = hardwareMap.servo.get(HOOD_SERVO);
            gateServoLeft     = hardwareMap.servo.get(GATE_SERVO_LEFT);
            gateServoRight    = hardwareMap.servo.get(GATE_SERVO_RIGHT);
            aimServoLeft      = hardwareMap.servo.get(AIM_SERVO_LEFT);
            aimServoRight     = hardwareMap.servo.get(AIM_SERVO_RIGHT);

            // Servos are intentionally NOT commanded here.
            // No servo position is assumed safe until the driver confirms safety
            // in Mark2InitialTesting.init_loop().
        }

        launcherMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastPosOne = launcherMotorOne.getCurrentPosition();
        lastPosTwo = launcherMotorTwo.getCurrentPosition();


    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /** Command both launcher motors to hold a target flywheel RPM. */
    public void setFlywheelTargetRpm(double rpm) {
        double newTargetRpm = Math.max(0.0, rpm);
        if (Math.abs(newTargetRpm - targetRpm) > 1e-6) {
            resetFlywheelController();
        }

        targetRpm = newTargetRpm;
        targetTicksPerSec = flywheelRpmToMotorTicksPerSec(targetRpm);

        if (targetRpm <= 0.0) {
            launcherMotorOne.setPower(0.0);
            launcherMotorTwo.setPower(0.0);
            flywheelPowerOne = 0.0;
            flywheelPowerTwo = 0.0;
        }
    }

    /** Stop launcher motors only. */
    public void stopFlywheels() {
        targetRpm = 0.0;
        targetTicksPerSec = 0.0;
        resetFlywheelController();
        launcherMotorOne.setPower(0.0);
        launcherMotorTwo.setPower(0.0);
        flywheelPowerOne = 0.0;
        flywheelPowerTwo = 0.0;
        measuredRpm = 0.0;
        measuredRpmOne = 0.0;
        measuredRpmTwo = 0.0;
    }

    /** Stop launcher motors and return launcher servos to their idle positions. */
    public void stop() {
        stopFlywheels();
        if (hasServos) {
            setHoodPosition(HOOD_SERVO_RESET_POSITION);
            resetFeeder();
        }
    }

    /** Retract the feeder gate servos to their idle position. */
    public void resetFeeder() {
        setFeederPosition(FEEDER_SERVO_IDLE_POSITION);
    }

    public void setHoodPosition(double position) {
        if (!hasServos) return;
        hoodPositionServo.setPosition(clamp(position, 0.0, 1.0));
    }

    public void setFeederPosition(double position) {
        if (!hasServos) return;
        double clippedPosition = clamp(position, 0.0, 1.0);
        gateServoLeft.setPosition(clippedPosition);
        gateServoRight.setPosition(clippedPosition);
    }

    /**
     * Pan the launcher aim servos to the specified position.
     * Both servos move to the same commanded value; if the physical linkage
     * requires one to be mirrored, reverse that servo in the Robot Controller
     * configuration rather than here.
     *
     * @param position Servo position [0.0 - 1.0]. Values outside the shared
     *                 aim range are clamped.
     */
    public void setAimPosition(double position) {
        if (!hasServos) return;
        aimServoPos = clamp(position, AIM_MIN_POS, AIM_MAX_POS);
        aimServoLeft.setPosition(aimServoPos);
        aimServoRight.setPosition(aimServoPos);
    }

    /**
     * Rate-control manual aim from stick input using a nominal 50 Hz loop.
     */
    public void setAimFromStick(double stickX) {
        setAimFromStick(stickX, 0.02);
    }

    /**
     * Rate-control manual aim from stick input.
     * Inputs inside {@link #AIM_STICK_DEADZONE} leave the current aim position unchanged.
     */
    public void setAimFromStick(double stickX, double dtSec) {
        if (!hasServos) return;

        double stickAbs = Math.abs(stickX);
        if (stickAbs <= AIM_STICK_DEADZONE) return;

        double usableStick = (stickAbs - AIM_STICK_DEADZONE) / (1.0 - AIM_STICK_DEADZONE);
        double curvedStick = Math.pow(clamp(usableStick, 0.0, 1.0), AIM_STICK_CURVE);
        double direction = Math.signum(stickX);
        double aimStep = direction * curvedStick * AIM_MAX_RATE_PER_SEC * Math.max(dtSec, 0.0);

        setAimPosition(aimServoPos + aimStep);
    }

    /** Last commanded aim servo position. Returns 0.5 if servos not installed. */
    public double getAimPosition() { return aimServoPos; }

    /** Refresh measured launcher RPM and run the custom flywheel PIDF controller. */
    public void updateMeasuredRpm(double dtSec) {
        double tpsOne = estimateTps(launcherMotorOne, true,  dtSec);
        double tpsTwo = estimateTps(launcherMotorTwo, false, dtSec);
        measuredRpmOne = tpsToRpm(tpsOne);
        measuredRpmTwo = tpsToRpm(tpsTwo);
        measuredRpm = (measuredRpmOne + measuredRpmTwo) / 2.0;

        updateFlywheelController(dtSec);
    }

    public double getMeasuredRpm()  { return measuredRpm; }
    public double getMeasuredRpmOne() { return measuredRpmOne; }
    public double getMeasuredRpmTwo() { return measuredRpmTwo; }
    public double getTargetRpm()    { return targetRpm; }
    public double getTargetTicksPerSec() { return targetTicksPerSec; }
    public double getFlywheelPowerOne() { return flywheelPowerOne; }
    public double getFlywheelPowerTwo() { return flywheelPowerTwo; }
    /** Last position sent to hood servo. Returns NaN if servos not installed. */
    public double getHoodPosition()   { return hasServos ? hoodPositionServo.getPosition() : Double.NaN; }
    /** Last position sent to the feeder gate servos. Returns NaN if servos not installed. */
    public double getFeederPosition() { return hasServos ? gateServoLeft.getPosition()  : Double.NaN; }
    /** Returns {@code true} if servos are present and wired. */
    public boolean hasServos()        { return hasServos; }

    public void setFlywheelPidCoefficients(double kP, double kI, double kD) {
        flywheelKp = kP;
        flywheelKi = kI;
        flywheelKd = kD;
        resetFlywheelController();
    }

    public void setFlywheelFeedforwardCoefficients(double kS, double kV) {
        flywheelKs = kS;
        flywheelKv = kV;
        resetFlywheelController();
    }

    private double estimateTps(DcMotorEx motor, boolean isMotorOne, double dtSec) {
        double v = motor.getVelocity();
        if (Math.abs(v) > 1e-3) return Math.abs(v);

        int pos    = motor.getCurrentPosition();
        int last   = isMotorOne ? lastPosOne : lastPosTwo;
        int dTicks = pos - last;
        if (isMotorOne) lastPosOne = pos; else lastPosTwo = pos;
        return Math.abs(dTicks / Math.max(dtSec, 1e-4));
    }

    private static double tpsToRpm(double tps) {
        return (tps * 60.0 / MOTOR_TICKS_PER_REV) * FLYWHEEL_PER_MOTOR_REV;
    }

    private static double flywheelRpmToMotorTicksPerSec(double flywheelRpm) {
        double motorRpm = flywheelRpm / FLYWHEEL_PER_MOTOR_REV;
        return motorRpm * MOTOR_TICKS_PER_REV / 60.0;
    }

    private void updateFlywheelController(double dtSec) {
        if (targetRpm <= 0.0) return;

        flywheelPowerOne = calculateFlywheelPower(
                targetRpm, measuredRpmOne, true, dtSec);
        flywheelPowerTwo = calculateFlywheelPower(
                targetRpm, measuredRpmTwo, false, dtSec);

        launcherMotorOne.setPower(flywheelPowerOne);
        launcherMotorTwo.setPower(flywheelPowerTwo);
    }

    private double calculateFlywheelPower(
            double targetRpm, double currentRpm, boolean isMotorOne, double dtSec) {
        double dt = Math.max(dtSec, 1e-4);
        double error = targetRpm - currentRpm;

        if (isMotorOne) {
            integralOne = clamp(integralOne + error * dt,
                    FLYWHEEL_INTEGRAL_MIN, FLYWHEEL_INTEGRAL_MAX);
            double derivative = (error - previousErrorOne) / dt;
            previousErrorOne = error;
            return calculateFlywheelOutput(targetRpm, error, integralOne, derivative);
        } else {
            integralTwo = clamp(integralTwo + error * dt,
                    FLYWHEEL_INTEGRAL_MIN, FLYWHEEL_INTEGRAL_MAX);
            double derivative = (error - previousErrorTwo) / dt;
            previousErrorTwo = error;
            return calculateFlywheelOutput(targetRpm, error, integralTwo, derivative);
        }
    }

    private double calculateFlywheelOutput(
            double targetRpm, double error, double integral, double derivative) {
        double voltageCompensation = NOMINAL_VOLTAGE / getBatteryVoltage();
        double feedforward = (flywheelKs + flywheelKv * targetRpm) * voltageCompensation;
        double feedback = flywheelKp * error
                + flywheelKi * integral
                + flywheelKd * derivative;
        return clamp(feedforward + feedback, 0.0, 1.0);
    }

    private void resetFlywheelController() {
        integralOne = 0.0;
        integralTwo = 0.0;
        previousErrorOne = 0.0;
        previousErrorTwo = 0.0;
    }

    private double getBatteryVoltage() {
        if (battery == null) return NOMINAL_VOLTAGE;

        double voltage = battery.getVoltage();
        if (voltage < 7.0 || voltage > 20.0) return NOMINAL_VOLTAGE;
        return voltage;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
