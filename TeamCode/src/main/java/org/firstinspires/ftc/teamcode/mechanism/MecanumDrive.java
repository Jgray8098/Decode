package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Tunable parameters
    private double deadzone = 0.06;      // Small stick motion ignored
    private double expoTranslate = 0.5;  // Softens forward/strafe response
    private double expoRotate = 0.5;     // Softens turn response

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor  = hardwareMap.dcMotor.get("LeftFrontMotor");
        frontRightMotor = hardwareMap.dcMotor.get("RightFrontMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("LeftBackMotor");
        backRightMotor  = hardwareMap.dcMotor.get("RightBackMotor");

        // Adjust direction to match your build
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double forward, double right, double rotate) {
        // Apply deadzone and expo shaping
        forward = applyExpo(applyDeadzone(forward, deadzone), expoTranslate);
        right   = applyExpo(applyDeadzone(right, deadzone), expoTranslate);
        rotate  = applyExpo(applyDeadzone(rotate, deadzone), expoRotate);

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

    // --- Helpers ---

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