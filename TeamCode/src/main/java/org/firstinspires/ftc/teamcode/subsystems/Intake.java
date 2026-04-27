package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor intakeMotorOne;
    private DcMotor intakeMotorTwo;
    private Servo intakeServo;
    private static final double INTAKE_POWER = .75;
    private static final double INTAKE_HOLD_ARTIFACT_POWER = .15;
    private static final double INTAKE_SERVO_GATE_CLOSED_POSITION = 0.0;
    private static final double INTAKE_SERVO_GATE_OPEN_POSITION = 0.0;
    private static final String INTAKE_MOTOR_ONE_NAME = "IntakeMotorOne";
    private static final String INTAKE_MOTOR_TWO_NAME = "IntakeMotorTwo";
    private static final String INTAKE_SERVO_NAME = "IntakeServo";

    public Intake(HardwareMap hardwareMap){
        intakeMotorOne  = hardwareMap.dcMotor.get(INTAKE_MOTOR_ONE_NAME);
        intakeMotorTwo = hardwareMap.dcMotor.get(INTAKE_MOTOR_TWO_NAME);
        intakeServo = hardwareMap.servo.get(INTAKE_SERVO_NAME);

        intakeMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void PickUp(){
        intakeMotorOne.setPower(INTAKE_POWER);
        intakeMotorTwo.setPower(INTAKE_POWER);
        intakeServo.setPosition(INTAKE_SERVO_GATE_CLOSED_POSITION);
    }

    public void Hold(){
        intakeMotorOne.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        intakeMotorTwo.setPower(INTAKE_HOLD_ARTIFACT_POWER);
        intakeServo.setPosition(INTAKE_SERVO_GATE_OPEN_POSITION);
    }

    public void Stop(){
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
        intakeServo.setPosition(INTAKE_SERVO_GATE_OPEN_POSITION);
    }
}
