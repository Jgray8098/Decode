package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

/**
 * Shared launch-feed sequence used by manual and automated launcher controllers.
 */
public class Mark2LaunchSequence {

    public enum State {
        IDLE,
        WAITING_FOR_INTAKE,
        RUNNING_INTAKE
    }

    public static final double INTAKE_START_DELAY_S = 0.30;
    public static final double INTAKE_RUN_TIME_S = 2.00;

    private final Mark2Launcher launcher;
    private final Mark2Intake intake;

    private State state = State.IDLE;
    private double delayElapsedS = 0.0;
    private double intakeElapsedS = 0.0;

    public Mark2LaunchSequence(Mark2Launcher launcher, Mark2Intake intake) {
        this.launcher = launcher;
        this.intake = intake;
    }

    public boolean startIfFlywheelRunning(boolean flywheelRunning) {
        if (!flywheelRunning || intake == null || state != State.IDLE) {
            return false;
        }

        launcher.setFeederPosition(Mark2Launcher.FEEDER_SERVO_FEED_POSITION);
        intake.HoldPosition();

        delayElapsedS = 0.0;
        intakeElapsedS = 0.0;
        state = State.WAITING_FOR_INTAKE;
        return true;
    }

    public void update(double dtSec) {
        if (intake == null) {
            return;
        }

        switch (state) {
            case WAITING_FOR_INTAKE:
                delayElapsedS += dtSec;
                if (delayElapsedS >= INTAKE_START_DELAY_S) {
                    intake.PickUp();
                    intakeElapsedS = delayElapsedS - INTAKE_START_DELAY_S;
                    state = State.RUNNING_INTAKE;
                    finishIfIntakeTimeElapsed();
                }
                break;

            case RUNNING_INTAKE:
                intakeElapsedS += dtSec;
                finishIfIntakeTimeElapsed();
                break;

            case IDLE:
            default:
                break;
        }
    }

    public void resetFeeder() {
        launcher.resetFeeder();
    }

    public void cancel() {
        if (intake != null) {
            intake.HoldPosition();
        }
        launcher.resetFeeder();
        delayElapsedS = 0.0;
        intakeElapsedS = 0.0;
        state = State.IDLE;
    }

    public State getState() {
        return state;
    }

    public boolean isActive() {
        return state != State.IDLE;
    }

    public boolean isRunningIntake() {
        return state == State.RUNNING_INTAKE;
    }

    private void finishIfIntakeTimeElapsed() {
        if (intakeElapsedS >= INTAKE_RUN_TIME_S) {
            intake.HoldPosition();
            delayElapsedS = 0.0;
            intakeElapsedS = 0.0;
            state = State.IDLE;
        }
    }
}
