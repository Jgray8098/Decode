package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.Range;
import java.util.function.DoubleSupplier;

/**
 * Vision-agnostic heading lock tuned for consistency (reduces approach-direction bias).
 *
 * Key upgrades:
 *  - desiredTxDeg setpoint (tx -> desiredTxDeg)
 *  - "Reset only on lock engage" (prevents re-entry bias)
 *  - Vision grace window (briefly keep controlling if vision blips)
 *  - Static friction feedforward kS (pushes through stiction consistently)
 *  - Optional conditional minTurn (only when error is "large")
 *  - Integrator enabled + anti-windup, integrates only outside deadband
 *
 * Sign conventions:
 *  - Limelight tx: +right
 *  - err = (tx - desiredTxDeg)
 *  - Positive err -> tag is too far right -> robot should turn right (omega positive or negative depends on your drive wiring,
 *    but your existing config already works, so keep kP sign as-is).
 */
public class HeadingLockController {

    public static class Config {
        // Base PID
        public double kP = 0.03;
        public double kI = 0.01;     // NEW: small I helps remove approach-direction steady bias
        public double kD = 0.002;

        // Deadband on error (deg)
        public double deadbandDeg = 0.6;

        // Static friction assist (applied outside deadband)
        public double kS = 0.05;     // NEW: 0.03–0.07 typical for mecanum

        // Optional min-turn clamp (only applied when error is "large")
        public double minTurn = 0.07;
        public double minTurnEngageDeg = 2.0; // NEW: only force minTurn when |err| > this

        // Output limits
        public double maxOmega = 0.9;

        // Vision validity + dropout handling
        public long visionStaleMs = 150;
        public long visionGraceMs = 120; // NEW: keep controlling briefly if vision drops

        // If IMU is provided, briefly hold heading when vision drops
        public long imuHoldMs = 250;

        // Integrator clamp
        public double iMaxOutput = 0.25; // max contribution of I term to omega (prevents windup)
    }

    public interface Vision {
        boolean hasTarget();
        double  getTxDeg();      // +right
        int     getTid();
        long    lastUpdateMs();
    }

    private final Vision vision;
    private final DoubleSupplier imuYawDeg; // may be null
    private final Config cfg;

    // Integrator / derivative state (on error)
    private double iSum = 0;
    private double lastErr = 0;

    // Heading hold state (optional IMU)
    private double holdHeadingDeg = 0;
    private long   lastVisionMs = 0;

    // Desired tag / setpoint
    private int desiredTid = -1; // -1 = any
    private double desiredTxDeg = 0.0;

    // Lock engagement edge detect
    private boolean wasWantLock = false;

    // Vision grace
    private double lastTxDeg = 0.0;
    private long   lastTxMs = 0;

    public HeadingLockController(Vision vision, DoubleSupplier imuYawDegOrNull, Config cfg) {
        this.vision = vision;
        this.imuYawDeg = imuYawDegOrNull;
        this.cfg = (cfg != null) ? cfg : new Config();
    }

    public void setDesiredTid(int tid) { this.desiredTid = tid; }

    /** Control drives tx -> desiredTxDeg (0 = center). */
    public void setDesiredTxDeg(double desiredTxDeg) {
        if (Math.abs(desiredTxDeg - this.desiredTxDeg) > 1e-6) {
            reset(); // avoid integral kick on setpoint change
        }
        this.desiredTxDeg = desiredTxDeg;
    }

    public double getDesiredTxDeg() { return desiredTxDeg; }

    public void reset() {
        iSum = 0;
        lastErr = 0;
    }

    /** Call each loop. Returns omega for your drive. */
    public double update(double dtSec, double driverOmega, boolean lockRequestedHold, boolean useHoldBehavior) {
        boolean wantLock = lockRequestedHold;
        long now = System.currentTimeMillis();

        // Reset ONLY when lock is first requested (prevents approach-direction hysteresis)
        if (wantLock && !wasWantLock) {
            reset();
        }
        wasWantLock = wantLock;

        // If not locking, pass driver omega
        if (!wantLock) {
            reset(); // clear for next lock engage
            return driverOmega;
        }

        // Determine if vision is usable right now
        boolean visionValidNow = isVisionValid(now) && isDesiredTid(vision.getTid());

        // Use vision if valid
        if (visionValidNow) {
            double tx = vision.getTxDeg();
            lastTxDeg = tx;
            lastTxMs  = now;

            lastVisionMs = now;
            if (imuYawDeg != null) holdHeadingDeg = imuYawDeg.getAsDouble();

            return computeOmegaFromTx(tx, dtSec);
        }

        // Vision not valid: try grace window (use last known tx for a short period)
        boolean graceOk = (now - lastTxMs) <= cfg.visionGraceMs;
        if (graceOk) {
            return computeOmegaFromTx(lastTxDeg, dtSec);
        }

        // If IMU exists and we recently had vision, hold heading briefly
        if (imuYawDeg != null && (now - lastVisionMs) < cfg.imuHoldMs) {
            double err = normDelta(holdHeadingDeg - imuYawDeg.getAsDouble());
            double omega = 0.015 * err;
            return Range.clip(omega, -0.4, 0.4);
        }

        // Otherwise: freeze rotation while lock is held (prevents re-entry randomness)
        return 0.0;
    }

    private double computeOmegaFromTx(double txDeg, double dtSec) {
        // Error is tx minus setpoint
        double err = txDeg - desiredTxDeg;

        // Apply deadband for control (but keep raw err for minTurn engage decisions if you want)
        double e = (Math.abs(err) < cfg.deadbandDeg) ? 0.0 : err;

        // Integrate only when outside deadband
        if (cfg.kI != 0.0 && e != 0.0) {
            iSum += e * dtSec;

            // Clamp integrator so I-term contribution can't exceed iMaxOutput
            double iCap = cfg.iMaxOutput / Math.max(Math.abs(cfg.kI), 1e-9);
            iSum = Range.clip(iSum, -iCap, iCap);
        } else if (e == 0.0) {
            // Optional: bleed off integrator slowly near target (prevents lingering bias)
            iSum *= 0.85;
            if (Math.abs(iSum) < 1e-4) iSum = 0.0;
        }

        // Derivative on error (works well with grace + kS; keep kD small)
        double deriv = (e - lastErr) / Math.max(dtSec, 1e-3);
        lastErr = e;

        double omega = cfg.kP * e + cfg.kI * iSum + cfg.kD * deriv;

        // Static friction assist outside deadband
        if (e != 0.0 && cfg.kS != 0.0) {
            omega += Math.copySign(cfg.kS, e);
        }

        // Optional min turn clamp only when error is meaningfully large
        if (Math.abs(err) > cfg.minTurnEngageDeg && Math.abs(omega) < cfg.minTurn) {
            omega = Math.copySign(cfg.minTurn, err);
        }

        return Range.clip(omega, -cfg.maxOmega, cfg.maxOmega);
    }

    private boolean isVisionValid(long now) {
        return vision.hasTarget() && (now - vision.lastUpdateMs()) <= cfg.visionStaleMs;
    }

    private boolean isDesiredTid(int tid) { return desiredTid < 0 || tid == desiredTid; }

    private static double normDelta(double deg) {
        double d = (deg + 540.0) % 360.0 - 180.0;
        return (d < -180) ? d + 360 : d;
    }
}



