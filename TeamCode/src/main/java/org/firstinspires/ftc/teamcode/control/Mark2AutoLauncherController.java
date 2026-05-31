package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;
import org.firstinspires.ftc.teamcode.utility.InterpolatingTreeMap;
import org.firstinspires.ftc.teamcode.utility.LaunchSetpoint;

/**
 * Automated launcher controller for match TeleOp and autonomous shooting.
 *
 * <p>This class owns the shot sequence. {@link Mark2Launcher} owns only the
 * motors, servos, and sensor reads.</p>
 */
public class Mark2AutoLauncherController {

    public enum LauncherState {
        /** Motors off, servos retracted. */
        IDLE,
        /** Motors running; waiting for measured RPM to reach the ready threshold. */
        SPINNING_UP,
        /** RPM threshold met; shared launch-feed sequence is running. */
        FEEDING,
        /** Shot complete. Call stop() or resetFeeder() to return to IDLE. */
        DONE
    }

    /**
     * Fraction of target RPM that counts as ready to feed.
     * 0.90 = fire once the launcher is at least 90% of target RPM.
     */
    private static final double RPM_READY_FRACTION = 0.90;

    private final Mark2Launcher launcher;
    private final Mark2LaunchSequence launchSequence;
    private final InterpolatingTreeMap setpointMap = new InterpolatingTreeMap();

    private LauncherState state = LauncherState.IDLE;
    private double targetRpm = 0.0;

    public Mark2AutoLauncherController(Mark2Launcher launcher, Mark2Intake intake) {
        this.launcher = launcher;
        this.launchSequence = new Mark2LaunchSequence(launcher, intake);

        // Distance (inches) -> LaunchSetpoint (rpm, hoodPosition). Tune on robot.
        setpointMap.put(24.0,  new LaunchSetpoint(2000.0, 0.10));
        setpointMap.put(48.0,  new LaunchSetpoint(2500.0, 0.20));
        setpointMap.put(72.0,  new LaunchSetpoint(3000.0, 0.30));
        setpointMap.put(96.0,  new LaunchSetpoint(3800.0, 0.42));
        setpointMap.put(120.0, new LaunchSetpoint(4500.0, 0.55));
        setpointMap.put(144.0, new LaunchSetpoint(5400.0, 0.70));
    }

    public void shoot(double distanceFromTargetInches) {
        if (state != LauncherState.IDLE && state != LauncherState.DONE) return;

        LaunchSetpoint setpoint = setpointMap.get(distanceFromTargetInches);
        targetRpm = setpoint.rpm;

        launcher.setFlywheelTargetRpm(targetRpm);
        launcher.setHoodPosition(setpoint.hoodPosition);
        launcher.resetFeeder();

        state = LauncherState.SPINNING_UP;
    }

    public void update(double dtSec) {
        switch (state) {
            case SPINNING_UP:
                launcher.updateMeasuredRpm(dtSec);
                if (launcher.getMeasuredRpm() >= targetRpm * RPM_READY_FRACTION) {
                    if (launchSequence.startIfFlywheelRunning(targetRpm > 0.0)) {
                        state = LauncherState.FEEDING;
                    }
                }
                break;

            case FEEDING:
                launcher.updateMeasuredRpm(dtSec);
                launchSequence.update(dtSec);
                if (!launchSequence.isActive()) {
                    state = LauncherState.DONE;
                }
                break;

            case DONE:
            case IDLE:
            default:
                break;
        }
    }

    public void stop() {
        launchSequence.cancel();
        launcher.stop();
        state = LauncherState.IDLE;
        targetRpm = 0.0;
    }

    /**
     * Retract feeder and return the sequence to IDLE without stopping flywheels.
     * This preserves the old TeleOp behavior where another held-B shot can start
     * immediately after DONE.
     */
    public void resetFeeder() {
        launchSequence.resetFeeder();
        state = LauncherState.IDLE;
    }

    public LauncherState getState() {
        return state;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getMeasuredRpm() {
        return launcher.getMeasuredRpm();
    }

    public boolean isAtSpeed() {
        return targetRpm > 0.0 && launcher.getMeasuredRpm() >= targetRpm * RPM_READY_FRACTION;
    }

    public String getLaunchSequenceStateName() {
        return launchSequence.getState().name();
    }
}
