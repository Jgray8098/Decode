package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.*;

public class Mark2Intake {
    private DcMotor intakeMotorOne;
    private DcMotor intakeMotorTwo;

    private Servo intakeServoLeft;
    private Servo intakeServoRight;

    // Hardware-map names live in Mark2HardwareMapNames — imported as static above.
    private static final double INTAKE_POWER               = .99;
    private static final double INTAKE_HOLD_ARTIFACT_POWER = .15;
    /**
     * Motor two power as a fraction of {@link #INTAKE_POWER} when running in
     * differential (TeleOp) mode.
     */
    private static final double INTAKE_MOTOR_TWO_FRACTION  = 1.0 / 3.0;

    /** Arm raised — default position when idle, stopped, or reversing. */
    private static final double INTAKE_SERVO_STOWED_POSITION  = 0.34;
    /** Arm lowered to sweep position — used only while intaking forward. */
    private static final double INTAKE_SERVO_INTAKE_POSITION  = 0.58;

    private double intakeServoPosition = INTAKE_SERVO_STOWED_POSITION;

    public Mark2Intake(HardwareMap hardwareMap){
        intakeMotorOne  = hardwareMap.dcMotor.get(INTAKE_MOTOR_ONE);
        intakeMotorTwo  = hardwareMap.dcMotor.get(INTAKE_MOTOR_TWO);

        intakeServoLeft  = hardwareMap.servo.get(INTAKE_SERVO_LEFT);
        intakeServoRight = hardwareMap.servo.get(INTAKE_SERVO_RIGHT);

        intakeMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        setServoPosition(INTAKE_SERVO_STOWED_POSITION);   // arm up at start
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
        intakeMotorOne.setPower(INTAKE_POWER);
        intakeMotorTwo.setPower(INTAKE_POWER * INTAKE_MOTOR_TWO_FRACTION);
        setServoPosition(INTAKE_SERVO_INTAKE_POSITION);   // arm down to sweep
    }

    /** Stop motors and raise arm to stowed position. */
    public void HoldPosition() {
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    public void Hold() {
        intakeMotorOne.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        intakeMotorTwo.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    public void Stop() {
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    /** Reverse motors with arm raised — for unjamming or ejecting. */
    public void Reverse() {
        intakeMotorOne.setPower(-INTAKE_POWER);
        intakeMotorTwo.setPower(-INTAKE_POWER);
        setServoPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    /**
     * Reverse intake with arm raised (same as {@link #Reverse()} — kept for
     * backward compatibility with TeleOp button mapping).
     */
    public void ReverseArm() {
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

    private static double clamp(double value, double min, double max)
    {return Math.max(min, Math.min(max, value));
    }
}
