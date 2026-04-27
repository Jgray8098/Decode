package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.internal.webserver.websockets.CommandNotImplementedException;

import kotlin.NotImplementedError;

public class Drivetrain {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    //FOR THE ODOMETRY PODS, reference SensorOctoQuad.java and/or SensorGoBildaPinpoint.java
    private GoBildaPinpointDriver odometryPods;

    private static final String FRONT_LEFT_MOTOR_NAME = "LeftFrontMotor";
    private static final String FRONT_RIGHT_MOTOR_NAME = "RightFrontMotor";
    private static final String REAR_LEFT_MOTOR_NAME = "LeftBackMotor";
    private static final String REAR_RIGHT_MOTOR_NAME = "RightBackMotor";
    private static final String IMU_NAME = "imu";

    // Tunable parameters
    private double deadzone = 0.06;      // Small stick motion ignored
    private double expoTranslate = 0.5;  // Softens forward/strafe response
    private double expoRotate = 0.5;     // Softens turn response

    public Drivetrain(HardwareMap hardwareMap, I2cDeviceSynchSimple odometryPodsDeviceClient){
        frontLeftMotor  = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor   = hardwareMap.dcMotor.get(REAR_LEFT_MOTOR_NAME);
        backRightMotor  = hardwareMap.dcMotor.get(REAR_RIGHT_MOTOR_NAME);
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        odometryPods = new GoBildaPinpointDriver(odometryPodsDeviceClient, false);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

//    public void driveTeleop(double forward, double right, double rotate) {
    public void driveTeleop(Gamepad driverController) {
        // Apply deadzone and expo shaping
        double forward = applyExpo(applyDeadzone(driverController.left_stick_y, deadzone), expoTranslate);
        double right   = applyExpo(applyDeadzone(driverController.left_stick_x, deadzone), expoTranslate);
        double rotate  = applyExpo(applyDeadzone(driverController.right_stick_x, deadzone), expoRotate);

        // Mecanum power calculation
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // Normalize so no value exceeds |1|
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max; fr /= max; bl /= max; br /= max;

        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);
    }

    public void driveToPosition(double targetXPosition, double targetYPosition, double targetRotation) {
        // This method would use the odometry pods to determine the robot's current position and orientation,
        // calculate the error between the current position and the target position, and then drive the motors
        // in a way that minimizes that error. This could be implemented using a PID controller or a similar control algorithm.
        throw new NotImplementedError("driveToPosition is not implemented yet");
    }

    public void aim(){
        // This method would use the IMU to determine the robot's current orientation,
        // calculate the error between the current orientation and the target orientation,
        // and then drive the motors in a way that minimizes that error.
        // This could be implemented using a PID controller or a similar control algorithm.
        throw new NotImplementedError("aim is not implemented yet");
    }

    private double applyDeadzone(double x, double dz) {
        if (Math.abs(x) < dz) return 0.0;
        double sign = Math.signum(x);
        double mag = (Math.abs(x) - dz) / (1.0 - dz);
        return sign * mag;
    }

    // Expo curve: (1 - e)*x + e*x³
    private double applyExpo(double x, double expo) {
        return (1.0 - expo) * x + expo * Math.pow(x, 3);
    }
}
