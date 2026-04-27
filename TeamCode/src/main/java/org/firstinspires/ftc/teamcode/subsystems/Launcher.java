package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private DcMotor launcherMotorOne;
    private DcMotor launcherMotorTwo;
    private Servo launcherServoOne;
    private Servo launcherServoTwo;
    private Servo launcherServoThree;

    private static double LAUNCHER_SERVO_SHOOT_POSITION = 0.0;
    private static final String LAUNCHER_MOTOR_ONE_NAME = "LauncherMotorOne";
    private static final String LAUNCHER_MOTOR_TWO_NAME = "LauncherMotorTwo";
    private static final String LAUNCHER_SERVO_ONE_NAME = "LauncherServoOne";
    private static final String LAUNCHER_SERVO_TWO_NAME = "LauncherServoTwo";
    private static final String LAUNCHER_SERVO_THREE_NAME = "LauncherServoThree";

    public Launcher(HardwareMap hardwareMap){
        launcherMotorOne  = hardwareMap.dcMotor.get(LAUNCHER_MOTOR_ONE_NAME);
        launcherMotorTwo = hardwareMap.dcMotor.get(LAUNCHER_MOTOR_TWO_NAME);
        launcherServoOne = hardwareMap.servo.get(LAUNCHER_SERVO_ONE_NAME);
        launcherServoTwo = hardwareMap.servo.get(LAUNCHER_SERVO_TWO_NAME);
        launcherServoThree = hardwareMap.servo.get(LAUNCHER_SERVO_THREE_NAME);

        launcherMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherServoTwo.setDirection(Servo.Direction.REVERSE);
    }

    public void Shoot(){
        launcherMotorOne.setPower(1);
        launcherMotorTwo.setPower(1);
        launcherServoOne.setPosition(LAUNCHER_SERVO_SHOOT_POSITION);
        launcherServoOne.setPosition(LAUNCHER_SERVO_SHOOT_POSITION);
    }
}
