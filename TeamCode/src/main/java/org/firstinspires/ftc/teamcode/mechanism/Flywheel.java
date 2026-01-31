package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Flywheel mechanism using ONLY a custom PIDF velocity controller on BOTH motors.
 *
 * UPDATED:
 *  - Hood servo will NOT move during INIT.
 *  - Hood servo only moves after you enable it (call enableHoodControl(true) from OpMode.start()).
 *  - NEW: Hood START position. On Play pressed, hood goes to hoodStartPos, and OFF state holds hoodStartPos.
 *  - Added CLOSE_AUTO state @ 1275 RPM for Autonomous.
 *  - Battery voltage compensation applied to feedforward (kF term).
 */
public class Flywheel {

    public enum State { OFF, CLOSE, LONG, CLOSE_AUTO }

    // --- Hardware names ---
    private final String rightName;
    private final String leftName;
    private final String hoodName;

    // --- Hardware ---
    private DcMotorEx right;
    private DcMotorEx left;
    private Servo hood;

    // Battery voltage
    private VoltageSensor battery;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // --- Hood positions (tune these) ---
    // Start position: where hood sits when TeleOp starts and when flywheel is OFF
    private double hoodStartPos     = 0.00;
    private double hoodClosePos     = 0.15;
    private double hoodLongPos      = 0.40;
    private double lastHoodCmd      = -1.0;

    // Prevent hood movement until OpMode.start()
    private boolean hoodEnabled = false;

    // --- Encoder / kinematics constants ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    // Use exact tooth ratio (motor pulley / flywheel pulley):
    private static final double FLYWHEEL_PER_MOTOR  = 53.0 / 75.0; // 0.706666...
    private static final double MOTOR_MAX_RPM       = 6000.0;
    private static final double MOTOR_MAX_TPS       = (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV) / 60.0;

    // ---- Target flywheel speeds (RPM) ----
    private static final double CLOSE_FLYWHEEL_RPM      = 1450.0;
    private static final double LONG_FLYWHEEL_RPM       = 1800.0;
    private static final double CLOSE_AUTO_FLYWHEEL_RPM = 1375.0;

    // --- Custom PIDF (on ticks/sec) ---
    private double kP = 0.003;
    private double kI = 0.0000;
    private double kD = 0.000;
    private double kF = 1.25 / MOTOR_MAX_TPS;

    private double iRight = 0, iLeft = 0;
    private double prevErrRight = 0, prevErrLeft = 0;

    // Anti-windup
    private static final double I_MIN = -0.5, I_MAX = 0.5;

    // Output clamp
    private static final double PWR_MIN = 0.0, PWR_MAX = 1.0;

    // Optional trims if one side is weaker/stronger
    private double rightFFScale = 1.00;
    private double leftFFScale  = 1.00;

    private State state = State.OFF;

    // Cached telemetry
    private double targetFlywheelRpm = 0.0;
    private double targetTps = 0.0;
    private double measRpmRight = 0.0, measRpmLeft = 0.0;

    // Velocity estimate (use SDK getVelocity if available; fallback to ticks/dt)
    private int lastPosRight = 0, lastPosLeft = 0;

    // ---------- Constructors ----------
    public Flywheel(String rightMotorName, String leftMotorName) {
        this(rightMotorName, leftMotorName, null);
    }

    public Flywheel(String rightMotorName, String leftMotorName, String hoodServoName) {
        this.rightName = rightMotorName;
        this.leftName  = leftMotorName;
        this.hoodName  = hoodServoName;
    }

    public void init(HardwareMap hw) {
        right = hw.get(DcMotorEx.class, rightName);
        left  = hw.get(DcMotorEx.class, leftName);

        // Grab a voltage sensor (first one available)
        if (hw.voltageSensor != null && hw.voltageSensor.iterator().hasNext()) {
            battery = hw.voltageSensor.iterator().next();
        }

        // Hood servo (optional) — DO NOT command position during INIT
        if (hoodName != null && !hoodName.isEmpty()) {
            hood = hw.get(Servo.class, hoodName);
            // hood.setDirection(Servo.Direction.REVERSE);
        }

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastPosRight = right.getCurrentPosition();
        lastPosLeft  = left.getCurrentPosition();

        stop(); // hood will NOT move here because hoodEnabled=false during INIT
    }

    /**
     * Call from OpMode.start() to allow hood motion after Play is pressed.
     * When enabled, hood immediately goes to hoodStartPos.
     */
    public void enableHoodControl(boolean enabled) {
        hoodEnabled = enabled;
        if (hoodEnabled) {
            setHoodPosition(hoodStartPos);
        }
    }

    // --- State API ---
    public void setState(State s) { state = s; }
    public State getState() { return state; }

    public void toggleClose() { state = (state == State.CLOSE) ? State.OFF : State.CLOSE; }
    public void toggleLong()  { state = (state == State.LONG)  ? State.OFF : State.LONG;  }

    // Convenience for Autonomous
    public void setCloseAuto() { state = State.CLOSE_AUTO; }

    public void stop() {
        state = State.OFF;
        targetFlywheelRpm = 0.0;
        targetTps = 0.0;
        iRight = iLeft = 0;
        prevErrRight = prevErrLeft = 0;

        if (right != null) right.setPower(0.0);
        if (left  != null) left.setPower(0.0);

        // Hood should NOT move during INIT, so only command it if enabled.
        // In OFF, hood should sit at START.
        if (hoodEnabled) setHoodPosition(hoodStartPos);
    }

    // --- Hood config helpers ---
    public void setHoodPositions(double closePos, double longPos) {
        hoodClosePos = clamp(closePos, 0.0, 1.0);
        hoodLongPos  = clamp(longPos, 0.0, 1.0);
    }

    public void setHoodStartPos(double startPos) {
        hoodStartPos = clamp(startPos, 0.0, 1.0);
    }

    public double getHoodPosition() {
        return (hood != null) ? hood.getPosition() : Double.NaN;
    }

    // Optional: helpful for telemetry
    public double getBatteryVoltage() {
        return getBatteryVoltageInternal();
    }

    // --- Tuning helpers ---
    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
    }

    public void setFFScales(double rightScale, double leftScale) {
        this.rightFFScale = rightScale;
        this.leftFFScale  = leftScale;
    }

    // --- Main loop ---
    public void update(double dtSec) {
        // 1) Decide target from state
        switch (state) {
            case CLOSE:      targetFlywheelRpm = CLOSE_FLYWHEEL_RPM;      break;
            case LONG:       targetFlywheelRpm = LONG_FLYWHEEL_RPM;       break;
            case CLOSE_AUTO: targetFlywheelRpm = CLOSE_AUTO_FLYWHEEL_RPM; break;
            default:         targetFlywheelRpm = 0.0;                     break;
        }

        // 1b) Hood follows state ONLY after enabled (after Play)
        if (hoodEnabled) {
            switch (state) {
                case LONG:
                    setHoodPosition(hoodLongPos);
                    break;

                case CLOSE:
                case CLOSE_AUTO:
                    setHoodPosition(hoodClosePos);
                    break;

                case OFF:
                default:
                    setHoodPosition(hoodStartPos);
                    break;
            }
        }

        targetTps = rpmToTps(targetFlywheelRpm);

        // 2) Measure velocities (tps)
        double rightTps = estimateTps(right, true, dtSec);
        double leftTps  = estimateTps(left,  false, dtSec);

        // 3) PIDF on each side
        double pwrRight, pwrLeft;
        if (state == State.OFF || targetTps <= 1e-6) {
            pwrRight = 0.0; pwrLeft = 0.0;
            iRight = iLeft = 0;
            prevErrRight = prevErrLeft = 0;
        } else {
            // Voltage compensation factor (higher when battery is low)
            double vBatt = getBatteryVoltageInternal();
            double vComp = NOMINAL_VOLTAGE / vBatt;

            // Right
            double errR = targetTps - rightTps;
            iRight = clamp(iRight + errR * dtSec, I_MIN, I_MAX);
            double dRight = (errR - prevErrRight) / Math.max(dtSec, 1e-4);
            double ffRight = (kF * targetTps * rightFFScale) * vComp;
            pwrRight = clamp(ffRight + kP * errR + kI * iRight + kD * dRight, PWR_MIN, PWR_MAX);
            prevErrRight = errR;

            // Left
            double errL = targetTps - leftTps;
            iLeft = clamp(iLeft + errL * dtSec, I_MIN, I_MAX);
            double dLeft = (errL - prevErrLeft) / Math.max(dtSec, 1e-4);
            double ffLeft = (kF * targetTps * leftFFScale) * vComp;
            pwrLeft = clamp(ffLeft + kP * errL + kI * iLeft + kD * dLeft, PWR_MIN, PWR_MAX);
            prevErrLeft = errL;
        }

        // 4) Apply power
        right.setPower(pwrRight);
        left.setPower(pwrLeft);

        // 5) Cache telemetry in RPM
        measRpmRight = tpsToRpm(rightTps);
        measRpmLeft  = tpsToRpm(leftTps);
    }

    // --- Telemetry getters ---
    public double getTargetRpm()        { return targetFlywheelRpm; }
    public double getMeasuredRightRpm() { return measRpmRight; }
    public double getMeasuredLeftRpm()  { return measRpmLeft;  }

    // --- Internals ---
    private double estimateTps(DcMotorEx m, boolean isRight, double dt) {
        double v = m.getVelocity(); // ticks/sec if supported
        if (Math.abs(v) > 1e-3) return v;

        int pos = m.getCurrentPosition();
        int last = isRight ? lastPosRight : lastPosLeft;
        int dTicks = pos - last;
        if (isRight) lastPosRight = pos; else lastPosLeft = pos;
        return dTicks / Math.max(dt, 1e-4);
    }

    private double getBatteryVoltageInternal() {
        if (battery == null) return NOMINAL_VOLTAGE;
        double v = battery.getVoltage();
        // guard against weird readings
        if (v < 7.0 || v > 20.0) return NOMINAL_VOLTAGE;
        return v;
    }

    private void setHoodPosition(double pos) {
        if (hood == null) return;
        pos = clamp(pos, 0.0, 1.0);
        if (Math.abs(pos - lastHoodCmd) > 1e-4) {
            hood.setPosition(pos);
            lastHoodCmd = pos;
        }
    }

    private static double rpmToTps(double rpm) {
        double motorRpm = rpm / FLYWHEEL_PER_MOTOR;
        return (motorRpm * MOTOR_TICKS_PER_REV) / 60.0;
    }

    private static double tpsToRpm(double tps) {
        double motorRpm = (tps * 60.0) / MOTOR_TICKS_PER_REV;
        return motorRpm * FLYWHEEL_PER_MOTOR;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}



