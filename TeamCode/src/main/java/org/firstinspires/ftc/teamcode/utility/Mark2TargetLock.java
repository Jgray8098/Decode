package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

/**
 * Computes turret aim commands that keep the launcher pointed at a fixed field target.
 *
 * <p>All tuning constants live here as {@code public static final} fields.
 * Edit them to match your field setup before a match.</p>
 *
 * <p>All geometry is based on field pose from Pinpoint odometry.</p>
 */
public class Mark2TargetLock {

    // -------------------------------------------------------------------------
    // Tuning constants — edit these before each match as needed.
    // -------------------------------------------------------------------------
    /** Field X coordinate of the red alliance goal (inches). */
    public static final double RED_GOAL_X_IN   = 72.0;
    /** Field Y coordinate of the red alliance goal (inches). */
    public static final double RED_GOAL_Y_IN   = -36.0;

    /** Field X coordinate of the blue alliance goal (inches). */
    public static final double BLUE_GOAL_X_IN  = 72.0;
    /** Field Y coordinate of the blue alliance goal (inches). */
    public static final double BLUE_GOAL_Y_IN  = 36.0;

    /** Leftward mechanical turret limit, degrees relative to robot forward. */
    public static final double TURRET_MIN_DEG  = -120.0;
    /** Rightward mechanical turret limit, degrees relative to robot forward. */
    public static final double TURRET_MAX_DEG  =  120.0;

    /** Aim servo position corresponding to the leftmost turret angle. */
    public static final double AIM_SERVO_MIN   = Mark2Launcher.AIM_MIN_POS;
    /** Aim servo position corresponding to the rightmost turret angle. */
    public static final double AIM_SERVO_MAX   = Mark2Launcher.AIM_MAX_POS;

    /** Set true if a positive turret angle should map to a lower servo position. */
    public static final boolean INVERT_SERVO_DIRECTION = false;

    /** Default alliance used by the target-lock system. Change before a match. */
    public static final Alliance DEFAULT_ALLIANCE = Alliance.RED;

    // -------------------------------------------------------------------------

    public enum Alliance { RED, BLUE }

    public static class FieldPoint {
        public final double xInches;
        public final double yInches;

        public FieldPoint(double xInches, double yInches) {
            this.xInches = xInches;
            this.yInches = yInches;
        }
    }

    public static class Config {
        public FieldPoint redGoal  = new FieldPoint(RED_GOAL_X_IN,  RED_GOAL_Y_IN);
        public FieldPoint blueGoal = new FieldPoint(BLUE_GOAL_X_IN, BLUE_GOAL_Y_IN);
        public double  turretMinDeg          = TURRET_MIN_DEG;
        public double  turretMaxDeg          = TURRET_MAX_DEG;
        public double  aimServoMin           = AIM_SERVO_MIN;
        public double  aimServoMax           = AIM_SERVO_MAX;
        public boolean invertServoDirection  = INVERT_SERVO_DIRECTION;
    }

    private final Config config;

    public Mark2TargetLock() {
        this(new Config());
    }

    public Mark2TargetLock(Config config) {
        this.config = (config != null) ? config : new Config();
    }

    public Config getConfig() { return config; }

    public double computeDesiredTurretDeg(Pose2D robotPose, Alliance alliance) {
        FieldPoint goal = getGoal(alliance);

        double robotX          = robotPose.getX(DistanceUnit.INCH);
        double robotY          = robotPose.getY(DistanceUnit.INCH);
        double robotHeadingRad = robotPose.getHeading(AngleUnit.RADIANS);

        double fieldHeadingRad   = Math.atan2(goal.yInches - robotY, goal.xInches - robotX);
        double relativeTurretRad = normalizeRad(fieldHeadingRad - robotHeadingRad);

        return clamp(Math.toDegrees(relativeTurretRad), config.turretMinDeg, config.turretMaxDeg);
    }

    public double computeAimServoPosition(Pose2D robotPose, Alliance alliance) {
        return turretDegToServoPosition(computeDesiredTurretDeg(robotPose, alliance));
    }

    public double lockToGoal(Mark2Launcher launcher, Pose2D robotPose, Alliance alliance) {
        double pos = computeAimServoPosition(robotPose, alliance);
        launcher.setAimPosition(pos);
        return pos;
    }

    public double turretDegToServoPosition(double turretDeg) {
        double clamped = clamp(turretDeg, config.turretMinDeg, config.turretMaxDeg);
        double norm = (clamped - config.turretMinDeg)
                / Math.max(config.turretMaxDeg - config.turretMinDeg, 1e-9);
        if (config.invertServoDirection) norm = 1.0 - norm;
        return config.aimServoMin + norm * (config.aimServoMax - config.aimServoMin);
    }

    public FieldPoint getGoal(Alliance alliance) {
        return alliance == Alliance.BLUE ? config.blueGoal : config.redGoal;
    }

    private static double normalizeRad(double r) {
        while (r >  Math.PI) r -= 2.0 * Math.PI;
        while (r <= -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
