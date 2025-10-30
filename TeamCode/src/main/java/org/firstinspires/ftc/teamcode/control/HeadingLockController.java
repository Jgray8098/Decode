package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.Range;
import java.util.function.DoubleSupplier;

/** Vision-agnostic heading lock: tx(deg) -> omega command. */
public class HeadingLockController {

    public static class Config {
        public double kP = 0.03;
        public double kI = 0.0;
        public double kD = 0.002;
        public double deadbandDeg = 1.0;
        public double minTurn = 0.07;
        public double maxOmega = 0.9;
        public long   visionStaleMs = 150;
        public long   imuHoldMs = 250; // brief hold when vision drops
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

    private double iSum = 0, lastErr = 0;
    private double holdHeadingDeg = 0;
    private long   lastVisionMs = 0;
    private int    desiredTid = -1; // -1 = any

    public HeadingLockController(Vision vision, DoubleSupplier imuYawDegOrNull, Config cfg) {
        this.vision = vision;
        this.imuYawDeg = imuYawDegOrNull;
        this.cfg = (cfg != null) ? cfg : new Config();
    }

    public void setDesiredTid(int tid) { this.desiredTid = tid; }
    public void reset() { iSum = 0; lastErr = 0; }

    /** Call each loop. Returns omega for your drive. */
    public double update(double dtSec, double driverOmega, boolean lockRequestedHold, boolean useHoldBehavior) {
        boolean wantLock = lockRequestedHold;
        long now = System.currentTimeMillis();
        boolean visionValid = isVisionValid(now) && isDesiredTid(vision.getTid());

        if (wantLock && visionValid) {
            double tx = vision.getTxDeg();
            double omega = pidStep(tx, dtSec);
            if (Math.abs(tx) > cfg.deadbandDeg && Math.abs(omega) < cfg.minTurn) {
                omega = Math.copySign(cfg.minTurn, tx);
            }
            lastVisionMs = now;
            if (imuYawDeg != null) holdHeadingDeg = imuYawDeg.getAsDouble();
            return Range.clip(omega, -cfg.maxOmega, cfg.maxOmega);
        }

        if (wantLock && imuYawDeg != null && (now - lastVisionMs) < cfg.imuHoldMs) {
            double err = normDelta(holdHeadingDeg - imuYawDeg.getAsDouble());
            double omega = 0.015 * err;
            return Range.clip(omega, -0.4, 0.4);
        }

        reset();
        return driverOmega;
    }

    private boolean isVisionValid(long now) {
        return vision.hasTarget() && (now - vision.lastUpdateMs()) <= cfg.visionStaleMs;
    }

    private boolean isDesiredTid(int tid) { return desiredTid < 0 || tid == desiredTid; }

    private double pidStep(double errorDeg, double dtSec) {
        double e = Math.abs(errorDeg) < cfg.deadbandDeg ? 0.0 : errorDeg;
        if (cfg.kI != 0) {
            iSum += e * dtSec;
            double iMax = 0.3;
            double iCap = iMax / Math.max(Math.abs(cfg.kI), 1e-9);
            iSum = Range.clip(iSum, -iCap, iCap);
        }
        double deriv = (e - lastErr) / Math.max(dtSec, 1e-3);
        lastErr = e;
        return cfg.kP * e + cfg.kI * iSum + cfg.kD * deriv;
    }

    private static double normDelta(double deg) {
        double d = (deg + 540.0) % 360.0 - 180.0;
        return (d < -180) ? d + 360 : d;
    }
}
