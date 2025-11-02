package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Flywheel mechanism using ONLY a custom PIDF velocity controller on BOTH motors.
 * - Both motors: RUN_WITHOUT_ENCODER (no built-in FTC velocity control used)
 * - Control variable: motor ticks/second (tps) toward a target derived from a desired flywheel RPM
 * - Feedforward: kF * targetTps
 *
 * Call update(dtSeconds) each loop with your measured loop delta time.
 */
public class Flywheel {

    public enum State { OFF, CLOSE, LONG }

    // --- Hardware names ---
    private final String rightName;
    private final String leftName;

    // --- Hardware ---
    private DcMotorEx right;
    private DcMotorEx left;

    // --- Encoder / kinematics constants ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double FLYWHEEL_PER_MOTOR  = 1.0; // gear ratio (flywheelRPM / motorRPM)
    private static final double MOTOR_MAX_RPM       = 6000.0;
    private static final double MOTOR_MAX_TPS       = (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV) / 60.0; // ~2800 tps

    // ---- Target flywheel speeds (RPM) ----
    private static final double CLOSE_FLYWHEEL_RPM = 2300.0;
    private static final double LONG_FLYWHEEL_RPM  = 3000.0;

    // --- Custom PIDF (on ticks/sec) ---
    // Good starting points; tune on-field.
    private double kP = 0.005;          // power per (tps error)
    private double kI = 0.0000;
    private double kD = 0.000;
    private double kF = 1.0 / MOTOR_MAX_TPS; // feedforward to map target tps → ≈power

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

    public Flywheel(String rightMotorName, String leftMotorName) {
        this.rightName = rightMotorName;
        this.leftName  = leftMotorName;
    }

    public void init(HardwareMap hw) {
        right = hw.get(DcMotorEx.class, rightName);
        left  = hw.get(DcMotorEx.class, leftName);

        // Directions preserved from your previous setup
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // We rely ONLY on our own controller:
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastPosRight = right.getCurrentPosition();
        lastPosLeft  = left.getCurrentPosition();

        stop();
    }

    // --- State API ---
    public void setState(State s) { state = s; }
    public State getState() { return state; }
    public void toggleClose() { state = (state == State.CLOSE) ? State.OFF : State.CLOSE; }
    public void toggleLong()  { state = (state == State.LONG)  ? State.OFF : State.LONG;  }

    public void stop() {
        state = State.OFF;
        targetFlywheelRpm = 0.0;
        targetTps = 0.0;
        iRight = iLeft = 0;
        prevErrRight = prevErrLeft = 0;
        if (right != null) right.setPower(0.0);
        if (left  != null) left.setPower(0.0);
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
    /** Call once per loop with dt in seconds. */
    public void update(double dtSec) {
        // 1) Decide target from state
        switch (state) {
            case CLOSE: targetFlywheelRpm = CLOSE_FLYWHEEL_RPM; break;
            case LONG:  targetFlywheelRpm = LONG_FLYWHEEL_RPM;  break;
            default:    targetFlywheelRpm = 0.0;                break;
        }
        targetTps = rpmToTps(targetFlywheelRpm);

        // 2) Measure velocities (tps)
        double rightTps = estimateTps(right, true, dtSec);
        double leftTps  = estimateTps(left,  false, dtSec);

        // 3) PIDF on each side
        double pwrRight = 0.0, pwrLeft = 0.0;
        if (state == State.OFF || targetTps <= 1e-6) {
            pwrRight = 0.0; pwrLeft = 0.0;
            iRight = iLeft = 0;
            prevErrRight = prevErrLeft = 0;
        } else {
            // Right
            double errR = targetTps - rightTps;
            iRight = clamp(iRight + errR * dtSec, I_MIN, I_MAX);
            double dRight = (errR - prevErrRight) / Math.max(dtSec, 1e-4);
            double ffRight = kF * targetTps * rightFFScale;
            pwrRight = clamp(ffRight + kP * errR + kI * iRight + kD * dRight, PWR_MIN, PWR_MAX);
            prevErrRight = errR;

            // Left
            double errL = targetTps - leftTps;
            iLeft = clamp(iLeft + errL * dtSec, I_MIN, I_MAX);
            double dLeft = (errL - prevErrLeft) / Math.max(dtSec, 1e-4);
            double ffLeft = kF * targetTps * leftFFScale;
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
        // Prefer SDK’s velocity if available; many firmware versions provide it even in RUN_WITHOUT_ENCODER.
        double v = m.getVelocity(); // ticks/sec if supported
        if (Math.abs(v) > 1e-3) return v;

        // Fallback: differentiate encoder position
        int pos = m.getCurrentPosition();
        int last = isRight ? lastPosRight : lastPosLeft;
        int dTicks = pos - last;
        if (isRight) lastPosRight = pos; else lastPosLeft = pos;
        return dTicks / Math.max(dt, 1e-4);
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
