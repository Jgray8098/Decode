package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.INTAKE_MOTOR_ONE;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.INTAKE_MOTOR_TWO;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.INTAKE_SERVO;

public class Mark2Intake {
    private DcMotor intakeMotorOne;
    private DcMotor intakeMotorTwo;
    private Servo intakeServo;
    // Hardware-map names live in Mark2HardwareMapNames — imported as static above.
    private static final double INTAKE_POWER                 = .75;
    private static final double INTAKE_HOLD_ARTIFACT_POWER   = .15;
    private static final double INTAKE_SERVO_INTAKE_POSITION = 0.0;
    private static final double INTAKE_SERVO_STOWED_POSITION = 0.0;
    private static final double INTAKE_SERVO_HOLD_POSITION   = 0.0;

    public Mark2Intake(HardwareMap hardwareMap){
        intakeMotorOne  = hardwareMap.dcMotor.get(INTAKE_MOTOR_ONE);
        intakeMotorTwo = hardwareMap.dcMotor.get(INTAKE_MOTOR_TWO);
        intakeServo = hardwareMap.servo.get(INTAKE_SERVO);

        intakeMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void PickUp(){
        intakeMotorOne.setPower(INTAKE_POWER);
        intakeMotorTwo.setPower(INTAKE_POWER);
        intakeServo.setPosition(INTAKE_SERVO_INTAKE_POSITION);
    }

    public void Hold(){
        intakeMotorOne.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        intakeMotorTwo.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        intakeServo.setPosition(INTAKE_SERVO_HOLD_POSITION);
    }

    public void Stop(){
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
        intakeServo.setPosition(INTAKE_SERVO_STOWED_POSITION);
    }

    /** Run intake motors in reverse — useful for unjamming or testing motor direction. */
    public void Reverse() {
        intakeMotorOne.setPower(-INTAKE_POWER);
        intakeMotorTwo.setPower(-INTAKE_POWER);
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
        intakeServo.setPosition(position);
    }

    /** Returns the last commanded intake servo position (for telemetry). */
    public double getServoPosition() {
        return intakeServo.getPosition();
    }
}
