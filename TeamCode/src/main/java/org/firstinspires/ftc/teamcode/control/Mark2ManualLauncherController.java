package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

/**
 * Manual launcher controller for driver-controlled TeleOp tuning.
 *
 * <p>This maps gamepad controls directly to launcher hardware commands. It is
 * intentionally separate from the automated shot controller.</p>
 */
public class Mark2ManualLauncherController {

    private enum ShotZone {
        CLOSE("CLOSE", 2500.0, 0.70),
        FAR("FAR", 3200.0, 0.70);

        final String label;
        final double rpm;
        final double hoodPosition;

        ShotZone(String label, double rpm, double hoodPosition) {
            this.label = label;
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    /** Safe centered position used during initial bring-up. */
    public static final double SERVO_CENTER_POS = 0.50;

    private final Mark2Launcher launcher;
    private final Mark2Intake intake;
    private final Mark2LaunchSequence launchSequence;

    private double hoodServoPos = SERVO_CENTER_POS;
    private double feederServoPos = SERVO_CENTER_POS;
    private double targetRpmCommand = 0.0;
    private ShotZone selectedZone = ShotZone.CLOSE;
    private boolean flywheelRunning = false;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevLaunchFeed = false;
    private boolean prevY = false;

    public Mark2ManualLauncherController(Mark2Launcher launcher) {
        this(launcher, null);
    }

    public Mark2ManualLauncherController(Mark2Launcher launcher, Mark2Intake intake) {
        this.launcher = launcher;
        this.intake = intake;
        this.launchSequence = new Mark2LaunchSequence(launcher, intake);
    }

    /** Move all launcher servos to safe center positions for inspection. */
    public void centerServos() {
        hoodServoPos = SERVO_CENTER_POS;
        feederServoPos = SERVO_CENTER_POS;

        launcher.setHoodPosition(hoodServoPos);
        launcher.setFeederPosition(feederServoPos);
        launcher.setAimPosition(SERVO_CENTER_POS);
    }

    public void update(Gamepad operator, double dtSec) {
        update(operator, dtSec, true);
    }

    public void update(Gamepad operator, double dtSec, boolean manualAimEnabled) {
        update(operator, dtSec, true, false, manualAimEnabled, operator.b);
    }

    public void update(
            Gamepad operator,
            double dtSec,
            boolean manualAimEnabled,
            boolean launchFeedPressed
    ) {
        update(operator, dtSec, true, false, manualAimEnabled, launchFeedPressed);
    }

    public void updateWithExternalLauncherSetpoint(
            Gamepad operator,
            double dtSec,
            boolean externalLauncherReady
    ) {
        updateWithExternalLauncherSetpoint(operator, dtSec, externalLauncherReady, true);
    }

    public void updateWithExternalLauncherSetpoint(
            Gamepad operator,
            double dtSec,
            boolean externalLauncherReady,
            boolean manualAimEnabled
    ) {
        update(operator, dtSec, false, externalLauncherReady, manualAimEnabled, operator.b);
    }

    public void updateWithExternalLauncherSetpoint(
            Gamepad operator,
            double dtSec,
            boolean externalLauncherReady,
            boolean manualAimEnabled,
            boolean launchFeedPressed
    ) {
        update(operator, dtSec, false, externalLauncherReady, manualAimEnabled, launchFeedPressed);
    }

    private void update(
            Gamepad operator,
            double dtSec,
            boolean manageLauncherSetpoint,
            boolean externalLauncherReady,
            boolean manualAimEnabled,
            boolean launchFeedPressed
    ) {
        boolean dpadUp = operator.dpad_up;
        boolean dpadDown = operator.dpad_down;
        boolean y = operator.y;

        if (manageLauncherSetpoint && dpadUp && !prevDpadUp) {
            toggleZone(ShotZone.CLOSE);
        }

        if (manageLauncherSetpoint && dpadDown && !prevDpadDown) {
            toggleZone(ShotZone.FAR);
        }

        if (intake != null && launchFeedPressed && !prevLaunchFeed) {
            if (launchSequence.startIfFlywheelRunning(flywheelRunning || externalLauncherReady)) {
                feederServoPos = Mark2Launcher.FEEDER_SERVO_FEED_POSITION;
            }
        }

        if (intake != null && y && !prevY) {
            launchSequence.resetFeeder();
            feederServoPos = Mark2Launcher.FEEDER_SERVO_IDLE_POSITION;
        }

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevLaunchFeed = launchFeedPressed;
        prevY = y;

        if (manageLauncherSetpoint) {
            launcher.setFlywheelTargetRpm(targetRpmCommand);
            launcher.updateMeasuredRpm(dtSec);
        }

        if (manualAimEnabled) {
            launcher.setAimFromStick(operator.left_stick_x, dtSec);
        }

        launchSequence.update(dtSec);

        if (intake != null && !launchSequence.isActive()) {
            if (operator.y) {
                intake.PickUpDifferential();
            } else if (operator.x) {
                intake.ReverseArm();
            } else {
                intake.HoldPosition();
            }
        }
    }

    public void stop() {
        targetRpmCommand = 0.0;
        flywheelRunning = false;
        if (launchSequence.isActive()) {
            launchSequence.cancel();
            feederServoPos = Mark2Launcher.FEEDER_SERVO_IDLE_POSITION;
        }
        launcher.stopFlywheels();
    }

    public double getTargetRpmCommand() {
        return targetRpmCommand;
    }

    public String getSelectedZoneName() {
        return selectedZone.label;
    }

    public double getSelectedZoneRpm() {
        return selectedZone.rpm;
    }

    public double getSelectedZoneHoodPosition() {
        return selectedZone.hoodPosition;
    }

    public boolean isFlywheelRunning() {
        return flywheelRunning;
    }

    public double getHoodServoPos() {
        return hoodServoPos;
    }

    public double getFeederServoPos() {
        return feederServoPos;
    }

    public String getLaunchSequenceStateName() {
        return launchSequence.getState().name();
    }

    public boolean isLaunchSequenceActive() {
        return launchSequence.isActive();
    }

    public boolean isLaunchSequenceRunningIntake() {
        return launchSequence.isRunningIntake();
    }

    private void toggleZone(ShotZone zone) {
        if (flywheelRunning && selectedZone == zone) {
            stop();
            return;
        }

        selectedZone = zone;
        hoodServoPos = zone.hoodPosition;
        targetRpmCommand = zone.rpm;
        flywheelRunning = true;
        launcher.setHoodPosition(hoodServoPos);
    }
}
