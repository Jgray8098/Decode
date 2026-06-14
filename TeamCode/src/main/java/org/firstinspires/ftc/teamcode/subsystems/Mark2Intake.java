package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.*;

public class Mark2Intake {
    private DcMotor intakeMotorOne;
    private DcMotor intakeMotorTwo;
    private DigitalChannel beamBreakSensor;

    private Servo intakeServoLeft;
    private Servo intakeServoRight;

    // Hardware-map names live in Mark2HardwareMapNames — imported as static above.
    private static final double INTAKE_POWER               = .99;
    private static final double INTAKE_HOLD_ARTIFACT_POWER = .15;
    /**
     * Motor two power as a fraction of {@link #INTAKE_POWER} when running in
     * differential (TeleOp) mode.
     */
    private static final double INTAKE_MOTOR_TWO_FRACTION  = 1.0 / 2.5;
    private static final boolean BEAM_BREAK_DETECTED_STATE = true;
    private static final double BEAM_BREAK_SEAT_DELAY_S = 0.50;

    /** Arm raised — default position when idle, stopped, or reversing. */
    private static final double INTAKE_SERVO_STOWED_POSITION  = 0.34;
    /** Arm lowered to sweep position — used only while intaking forward. */
    private static final double INTAKE_SERVO_INTAKE_POSITION  = 0.58;

    private double intakeServoPosition = INTAKE_SERVO_STOWED_POSITION;
    private boolean beamBreakBallLatched = false;
    private double beamBreakSeatDelayElapsedS = 0.0;

    public Mark2Intake(HardwareMap hardwareMap){
        this(hardwareMap, true);
    }

    public Mark2Intake(HardwareMap hardwareMap, boolean commandServoOnInit){
        intakeMotorOne  = hardwareMap.dcMotor.get(INTAKE_MOTOR_ONE);
        intakeMotorTwo  = hardwareMap.dcMotor.get(INTAKE_MOTOR_TWO);
        beamBreakSensor = hardwareMap.get(DigitalChannel.class, BEAM_BREAK_SENSOR);

        intakeServoLeft  = hardwareMap.servo.get(INTAKE_SERVO_LEFT);
        intakeServoRight = hardwareMap.servo.get(INTAKE_SERVO_RIGHT);

        intakeMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        beamBreakSensor.setMode(DigitalChannel.Mode.INPUT);

        if (commandServoOnInit) {
            setServoPosition(INTAKE_SERVO_STOWED_POSITION);   // arm up at start
        }
    }

    public void PickUp() {
        intakeMotorOne.setPower(INTAKE_POWER);
        intakeMotorTwo.setPower(INTAKE_POWER);
        setServoPosition(INTAKE_SERVO_INTAKE_POSITION);   // arm down to sweep
    }

    /**
     * TeleOp intake mode — motor one at full {@link #INTAKE_POWER},
     * motor two at {@link #INTAKE_MOTOR_TWO_FRACTION} of that.
     * Arm moves down to sweep position.
     */
    public void PickUpDifferential() {
        PickUpDifferential(0.0);
    }

    public void PickUpDifferential(double dtSec) {
        updateBeamBreakLatch(dtSec);

        intakeMotorOne.setPower(INTAKE_POWER);
        intakeMotorTwo.setPower(shouldRunIntakeMotorTwo() ? INTAKE_POWER * INTAKE_MOTOR_TWO_FRACTION : 0.0);
        setServoPosition(INTAKE_SERVO_INTAKE_POSITION);   // arm down to sweep
    }

    public void ContinueBeamBreakSeatDelay(double dtSec) {
        updateBeamBreakSeatDelay(dtSec);

        if (!isBeamBreakSeatDelayActive()) {
            HoldPosition();
            return;
        }

        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(INTAKE_POWER * INTAKE_MOTOR_TWO_FRACTION);
        setServoPosition(INTAKE_SERVO_INTAKE_POSITION);
    }

    /** Stop motors and raise arm to stowed position. */
    public void HoldPosition() {
        finishBeamBreakSeatDelay();
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    public void Hold() {
        finishBeamBreakSeatDelay();
        intakeMotorOne.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        intakeMotorTwo.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    public void Stop() {
        finishBeamBreakSeatDelay();
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    /** Reverse motors with arm raised — for unjamming or ejecting. */
    public void Reverse() {
        finishBeamBreakSeatDelay();
        intakeMotorOne.setPower(-INTAKE_POWER);
        intakeMotorTwo.setPower(-INTAKE_POWER);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    /**
     * Reverse intake with arm raised (same as {@link #Reverse()} — kept for
     * backward compatibility with TeleOp button mapping).
     */
    public void ReverseArm() {
        finishBeamBreakSeatDelay();
        intakeMotorOne.setPower(-INTAKE_POWER);
        intakeMotorTwo.setPower(-INTAKE_POWER);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    /**
     * Directly command the intake servo to a specific position.
     * Use this from a testing OpMode to find the correct values for
     * {@code INTAKE_SERVO_INTAKE_POSITION}, {@code INTAKE_SERVO_HOLD_POSITION},
     * and {@code INTAKE_SERVO_STOWED_POSITION}.
     *
     * @param position Servo position  [0.0 – 1.0].
     */
    public void setServoPosition(double position) {
        intakeServoPosition = clamp(position, 0.0, 1.0);
        intakeServoLeft.setPosition(intakeServoPosition);
        intakeServoRight.setPosition(1.0 - intakeServoPosition);
    }

    /** Returns the last commanded intake servo position (for telemetry). */
    public double getServoPosition() {
        return intakeServoPosition;
    }

    public boolean isBeamBreakDetected() {
        return beamBreakSensor.getState() == BEAM_BREAK_DETECTED_STATE;
    }

    public boolean isBeamBreakBallLatched() {
        return beamBreakBallLatched;
    }

    public boolean isBeamBreakSeatDelayActive() {
        return beamBreakBallLatched && beamBreakSeatDelayElapsedS < BEAM_BREAK_SEAT_DELAY_S;
    }

    public void resetBeamBreakBallLatch() {
        beamBreakBallLatched = false;
        beamBreakSeatDelayElapsedS = 0.0;
    }

    private void updateBeamBreakLatch(double dtSec) {
        if (!beamBreakBallLatched && isBeamBreakDetected()) {
            beamBreakBallLatched = true;
            beamBreakSeatDelayElapsedS = 0.0;
            return;
        }

        updateBeamBreakSeatDelay(dtSec);
    }

    private void updateBeamBreakSeatDelay(double dtSec) {
        if (!isBeamBreakSeatDelayActive()) {
            return;
        }

        beamBreakSeatDelayElapsedS += Math.max(0.0, dtSec);
        if (beamBreakSeatDelayElapsedS > BEAM_BREAK_SEAT_DELAY_S) {
            beamBreakSeatDelayElapsedS = BEAM_BREAK_SEAT_DELAY_S;
        }
    }

    private boolean shouldRunIntakeMotorTwo() {
        return !beamBreakBallLatched || isBeamBreakSeatDelayActive();
    }

    private void finishBeamBreakSeatDelay() {
        if (beamBreakBallLatched) {
            beamBreakSeatDelayElapsedS = BEAM_BREAK_SEAT_DELAY_S;
        } else {
            beamBreakSeatDelayElapsedS = 0.0;
        }
    }

    private static double clamp(double value, double min, double max)
    {return Math.max(min, Math.min(max, value));
    }
}
