package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;
import org.firstinspires.ftc.teamcode.utility.InterpolatingTreeMap;
import org.firstinspires.ftc.teamcode.utility.LaunchSetpoint;
import org.firstinspires.ftc.teamcode.utility.Mark2TargetLock;

/**
 * TeleOp-only launcher setpoint controller.
 *
 * <p>When enabled, this continuously converts the Pinpoint robot pose into a
 * distance from the selected alliance goal, then commands flywheel RPM and hood
 * position from the distance setpoint table.</p>
 */
public class Mark2AutoLauncherController {

    /**
     * Fraction of target RPM that counts as ready for the driver/operator to feed.
     * 0.90 = ready once the launcher is at least 90% of target RPM.
     */
    private static final double RPM_READY_FRACTION = 0.90;

    /** Field X coordinate of the blue alliance goal (inches). */
    public static final double BLUE_GOAL_X_IN = 11.57;
    /** Field Y coordinate of the blue alliance goal (inches). */
    public static final double BLUE_GOAL_Y_IN = 134.8;

    /** Field X coordinate of the red alliance goal (inches). */
    public static final double RED_GOAL_X_IN = 130.5;
    /** Field Y coordinate of the red alliance goal (inches). */
    public static final double RED_GOAL_Y_IN = 134.8;

    private final Mark2Launcher launcher;
    private final InterpolatingTreeMap setpointMap = new InterpolatingTreeMap();

    private double targetRpm = 0.0;
    private double targetHoodPosition = Mark2Launcher.HOOD_SERVO_RESET_POSITION;
    private double lastDistanceInches = Double.NaN;

    public Mark2AutoLauncherController(Mark2Launcher launcher) {
        this.launcher = launcher;

        // Distance (inches) -> LaunchSetpoint (rpm, hoodPosition). Tune on robot.
        setpointMap.put(24.0,  new LaunchSetpoint(2000.0, 0.20));
        setpointMap.put(36.0,  new LaunchSetpoint(2300.0, 0.50));
        setpointMap.put(48.0,  new LaunchSetpoint(2500.0, 0.70));
        setpointMap.put(72.0,  new LaunchSetpoint(2800.0, 0.80));
        setpointMap.put(96.0,  new LaunchSetpoint(2950.0, 0.80));
        setpointMap.put(120.0, new LaunchSetpoint(3000.0, 0.80));
        setpointMap.put(144.0, new LaunchSetpoint(3200.0, 0.80));
    }

    public LaunchSetpoint updateForPose(
            Pose2D robotPose,
            Mark2TargetLock.Alliance alliance,
            double dtSec
    ) {
        LaunchSetpoint setpoint = applySetpointForPose(robotPose, alliance);
        launcher.updateMeasuredRpm(dtSec);
        return setpoint;
    }

    public LaunchSetpoint applySetpointForPose(Pose2D robotPose, Mark2TargetLock.Alliance alliance) {
        return applySetpointForDistance(computeDistanceToGoalInches(robotPose, alliance));
    }

    public LaunchSetpoint applySetpointForDistance(double distanceFromTargetInches) {
        LaunchSetpoint setpoint = setpointMap.get(distanceFromTargetInches);
        targetRpm = setpoint.rpm;
        targetHoodPosition = Mark2Launcher.clipHoodPosition(setpoint.hoodPosition);
        lastDistanceInches = distanceFromTargetInches;

        launcher.setFlywheelTargetRpm(targetRpm);
        launcher.setHoodPosition(targetHoodPosition);
        return setpoint;
    }

    public double computeDistanceToGoalInches(Pose2D robotPose, Mark2TargetLock.Alliance alliance) {
        double dx = getGoalXInches(alliance) - robotPose.getX(DistanceUnit.INCH);
        double dy = getGoalYInches(alliance) - robotPose.getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }

    public void stop() {
        targetRpm = 0.0;
        targetHoodPosition = Mark2Launcher.HOOD_SERVO_RESET_POSITION;
        lastDistanceInches = Double.NaN;
        launcher.stopFlywheels();
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    public double getLastDistanceInches() {
        return lastDistanceInches;
    }

    public double getMeasuredRpm() {
        return launcher.getMeasuredRpm();
    }

    public boolean isAtSpeed() {
        return targetRpm > 0.0 && launcher.getMeasuredRpm() >= targetRpm * RPM_READY_FRACTION;
    }

    private static double getGoalXInches(Mark2TargetLock.Alliance alliance) {
        return alliance == Mark2TargetLock.Alliance.BLUE ? BLUE_GOAL_X_IN : RED_GOAL_X_IN;
    }

    private static double getGoalYInches(Mark2TargetLock.Alliance alliance) {
        return alliance == Mark2TargetLock.Alliance.BLUE ? BLUE_GOAL_Y_IN : RED_GOAL_Y_IN;
    }
}
