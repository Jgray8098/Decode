package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.REAR_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.REAR_RIGHT_MOTOR;

public class Mark2Drivetrain {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // IMU removed — not currently installed on robot.-+6
    // Restore: add `import com.qualcomm.robotcore.hardware.IMU;`,
    // re-add `import static ...Mark2HardwareMapNames.IMU_SENSOR;`,
    // add `private IMU imu;`, and `imu = hardwareMap.get(IMU.class, IMU_SENSOR);`
    // in both constructors.

    //FOR THE ODOMETRY PODS, reference SensorOctoQuad.java and/or SensorGoBildaPinpoint.java
    private GoBildaPinpointDriver odometryPods;

    /** True when a Pinpoint client was provided at construction time. */
    private final boolean hasPinpoint;

    // -------------------------------------------------------------------------
    // Teleop tuning
    // -------------------------------------------------------------------------
    private double deadzone = 0.06;      // Small stick motion ignored
    private double expoTranslate = 0.5;  // Softens forward/strafe response
    private double expoRotate = 0.5;     // Softens turn response

    /**
     * When true, forward and strafe inputs are negated so a driver on the
     * opposite side of the field experiences the same "push forward = away
     * from me" behaviour.  Toggle with {@link #toggleAllianceFlip()}.
     */
    private boolean allianceFlipped = false;

    // -------------------------------------------------------------------------
    // driveToPosition / aim — PID gains  (tune with Constants.java as reference)
    // -------------------------------------------------------------------------
    /** Translational PID — proportional gain (inches error → drive power). */
    private static final double XY_KP = 0.045;
    private static final double XY_KI = 0.0;
    private static final double XY_KD = 0.001;

    /** Heading PID — proportional gain (radians error → rotation power). */
    private static final double ROT_KP = 0.65;
    private static final double ROT_KI = 0.0;
    private static final double ROT_KD = 0.002;

    /** Robot is considered "at position" when XY error is within this many inches. */
    private static final double XY_TOLERANCE_INCHES   = 1.0;
    /** Robot is considered "aimed" when heading error is within this many degrees. */
    private static final double ROT_TOLERANCE_DEGREES = 2.0;

    /** Maximum fraction of full power applied for translation corrections. */
    private static final double MAX_DRIVE_POWER = 0.7;
    /** Maximum fraction of full power applied for rotation corrections. */
    private static final double MAX_ROT_POWER   = 0.5;

    // -------------------------------------------------------------------------
    // PID state — driveToPosition
    // -------------------------------------------------------------------------
    private double dtp_xIntegral = 0, dtp_xLastError = 0;
    private double dtp_yIntegral = 0, dtp_yLastError = 0;
    private double dtp_rIntegral = 0, dtp_rLastError = 0;
    private double dtp_lastTargetX = Double.NaN, dtp_lastTargetY = Double.NaN, dtp_lastTargetRot = Double.NaN;

    public Mark2Drivetrain(HardwareMap hardwareMap, I2cDeviceSynchSimple odometryPodsDeviceClient){
        frontLeftMotor  = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        backLeftMotor   = hardwareMap.dcMotor.get(REAR_LEFT_MOTOR);
        backRightMotor  = hardwareMap.dcMotor.get(REAR_RIGHT_MOTOR);
        odometryPods = new GoBildaPinpointDriver(odometryPodsDeviceClient, false);
        hasPinpoint = true;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * No-Pinpoint constructor for bench / initial testing without odometry hardware.
     * {@link #driveToPosition} and {@link #aim} are no-ops (return false immediately).
     * {@link #getPose} returns a zeroed pose.
     */
    public Mark2Drivetrain(HardwareMap hardwareMap){
        frontLeftMotor  = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        backLeftMotor   = hardwareMap.dcMotor.get(REAR_LEFT_MOTOR);
        backRightMotor  = hardwareMap.dcMotor.get(REAR_RIGHT_MOTOR);
        odometryPods = null;
        hasPinpoint = false;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

//    public void driveTeleop(double forward, double right, double rotate) {
    public void driveTeleop(Gamepad driverController) {
        // Apply deadzone and expo shaping
        double flip    = allianceFlipped ? -1.0 : 1.0;
        double forward = flip * applyExpo(applyDeadzone(driverController.left_stick_y, deadzone), expoTranslate);
        double right   = flip * applyExpo(applyDeadzone(driverController.left_stick_x, deadzone), expoTranslate);
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

    /**
     * Drive the robot to a specific field-coordinate position and heading.
     * Uses field-centric PID: X/Y errors are resolved in field space then
     * rotated into robot-centric forward/strafe commands each loop.
     *
     * <p><b>Must be called every OpMode loop iteration</b> until it returns
     * {@code true}.  Passing a different target automatically resets the
     * PID integrators.</p>
     *
     * @param targetXPosition  Desired field X position in inches.
     * @param targetYPosition  Desired field Y position in inches.
     * @param targetRotation   Desired final heading in degrees (field frame, CCW positive).
     * @param dtSec            Seconds elapsed since the last loop call.
     *                         Compute with {@code (System.nanoTime() - lastNs) / 1e9}.
     * @return {@code true} once the robot is within tolerance of the target pose.
     */
    public boolean driveToPosition(double targetXPosition, double targetYPosition,
                                   double targetRotation, double dtSec) {
        if (!hasPinpoint) return false;   // odometry unavailable; caller should not block on this

        // Reset integrators whenever a new target is commanded
        if (targetXPosition != dtp_lastTargetX
                || targetYPosition  != dtp_lastTargetY
                || targetRotation   != dtp_lastTargetRot) {
            dtp_xIntegral = 0; dtp_xLastError = 0;
            dtp_yIntegral = 0; dtp_yLastError = 0;
            dtp_rIntegral = 0; dtp_rLastError = 0;
            dtp_lastTargetX   = targetXPosition;
            dtp_lastTargetY   = targetYPosition;
            dtp_lastTargetRot = targetRotation;
        }

        odometryPods.update();
        Pose2D pose = odometryPods.getPosition();

        double currentX   = pose.getX(DistanceUnit.INCH);
        double currentY   = pose.getY(DistanceUnit.INCH);
        double currentH   = pose.getHeading(AngleUnit.RADIANS);
        double targetHRad = Math.toRadians(targetRotation);

        double xError   = targetXPosition - currentX;
        double yError   = targetYPosition - currentY;
        double rotError = normalizeAngleRad(targetHRad - currentH);

        boolean atXY  = Math.hypot(xError, yError) < XY_TOLERANCE_INCHES;
        boolean atRot = Math.abs(Math.toDegrees(rotError)) < ROT_TOLERANCE_DEGREES;

        if (atXY && atRot) {
            stopMotors();
            return true;
        }

        double dt = Math.max(dtSec, 1e-4);

        // --- Translational PID (field frame) ---
        dtp_xIntegral += xError * dt;
        double xOut = XY_KP * xError
                    + XY_KI * dtp_xIntegral
                    + XY_KD * (xError - dtp_xLastError) / dt;
        dtp_xLastError = xError;

        dtp_yIntegral += yError * dt;
        double yOut = XY_KP * yError
                    + XY_KI * dtp_yIntegral
                    + XY_KD * (yError - dtp_yLastError) / dt;
        dtp_yLastError = yError;

        // --- Heading PID ---
        dtp_rIntegral += rotError * dt;
        double rotOut = ROT_KP * rotError
                      + ROT_KI * dtp_rIntegral
                      + ROT_KD * (rotError - dtp_rLastError) / dt;
        dtp_rLastError = rotError;

        // --- Rotate field-frame translation into robot-centric forward / strafe ---
        //   robotFwd    =  dx·cos(H) + dy·sin(H)
        //   robotStrafe = -dx·sin(H) + dy·cos(H)
        double cosH = Math.cos(currentH);
        double sinH = Math.sin(currentH);
        double fwd    = clamp( xOut * cosH + yOut * sinH, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        double strafe = clamp(-xOut * sinH + yOut * cosH, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        double rot    = clamp(rotOut,                     -MAX_ROT_POWER,   MAX_ROT_POWER);

        // --- Mecanum kinematics ---
        double fl = fwd + strafe + rot;
        double fr = fwd - strafe - rot;
        double bl = fwd - strafe + rot;
        double br = fwd + strafe - rot;

        double maxPow = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                               Math.max(Math.abs(bl), Math.abs(br))));
        frontLeftMotor.setPower(fl / maxPow);
        frontRightMotor.setPower(fr / maxPow);
        backLeftMotor.setPower(bl / maxPow);
        backRightMotor.setPower(br / maxPow);

        return false;
    }

    /**
     * Rotate the robot in place until it faces a given field coordinate.
     * The robot is assumed to be stationary; only the rotation motors are driven.
     *
     * <p><b>Must be called every OpMode loop iteration</b> until it returns {@code true}.</p>
     *
     * @param targetXPosition  X position of the point to aim at (inches, field frame).
     * @param targetYPosition  Y position of the point to aim at (inches, field frame).
     * @return {@code true} once the robot is within {@value #ROT_TOLERANCE_DEGREES}° of the target direction.
     */
    public boolean aim(double targetXPosition, double targetYPosition) {
        if (!hasPinpoint) return false;   // odometry unavailable

        odometryPods.update();
        Pose2D pose = odometryPods.getPosition();

        double currentX       = pose.getX(DistanceUnit.INCH);
        double currentY       = pose.getY(DistanceUnit.INCH);
        double currentHeading = pose.getHeading(AngleUnit.RADIANS);

        double angleToTarget  = Math.atan2(targetYPosition - currentY, targetXPosition - currentX);
        double rotationError  = normalizeAngleRad(angleToTarget - currentHeading);

        if (Math.abs(Math.toDegrees(rotationError)) < ROT_TOLERANCE_DEGREES) {
            stopMotors();
            return true;
        }

        double rotPower = clamp(ROT_KP * rotationError, -MAX_ROT_POWER, MAX_ROT_POWER);

        frontLeftMotor.setPower(  rotPower);
        backLeftMotor.setPower(   rotPower);
        frontRightMotor.setPower(-rotPower);
        backRightMotor.setPower( -rotPower);

        return false;
    }

    /** Maximum power fraction applied during safe / test driving. */
    private static final double SAFE_DRIVE_POWER_CAP  = 0.99;
    /** Power cap applied when the left trigger is held — snail mode for precise manoeuvring. */
    public  static final double SNAIL_MODE_POWER_CAP  = 0.60;

    /**
     * Drive the robot with full expo/deadzone shaping but capped at
     * {@value #SAFE_DRIVE_POWER_CAP} total power on any wheel.
     * Use this during initial testing to reduce the risk of impact.
     *
     * <p>Holding the <b>left trigger</b> (any value &gt; 0) engages snail mode,
     * further capping power to {@value #SNAIL_MODE_POWER_CAP} for finer control.</p>
     *
     * <p>Control mapping (testing layout):
     * right stick = translate (Y = forward, X = strafe), left stick X = rotate,
     * left trigger = snail mode.</p>
     */
    public void driveSafe(Gamepad gamepad) {
        double flip    = allianceFlipped ? -1.0 : 1.0;
        double forward = flip * applyExpo(applyDeadzone(-gamepad.right_stick_y, deadzone), expoTranslate);
        double right   = flip * applyExpo(applyDeadzone( gamepad.right_stick_x, deadzone), expoTranslate);
        double rotate  = applyExpo(applyDeadzone( gamepad.left_stick_x,  deadzone), expoRotate);

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // Normalize to ≤ 1, then apply the appropriate power cap
        double cap = (gamepad.left_trigger > 0) ? SNAIL_MODE_POWER_CAP : SAFE_DRIVE_POWER_CAP;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        frontLeftMotor.setPower(fl  / max * cap);
        frontRightMotor.setPower(fr / max * cap);
        backLeftMotor.setPower(bl   / max * cap);
        backRightMotor.setPower(br  / max * cap);
    }

    /**
     * Toggle the alliance-flip state.  When flipped, forward and strafe inputs
     * are negated so a driver seated on the opposite side of the field experiences
     * the same "push forward = away from me" behaviour.  Rotation is unaffected.
     */
    public void toggleAllianceFlip() {
        allianceFlipped = !allianceFlipped;
    }

    /** @return {@code true} if the drive controls are currently flipped for the opposite alliance. */
    public boolean isAllianceFlipped() {
        return allianceFlipped;
    }

    /**
     * Refresh the odometry estimate and return the current field pose.
     * Convenience method for reading position in a testing OpMode without
     * needing direct access to the odometry driver.
     */
    public Pose2D getPose() {
        if (!hasPinpoint) {
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        }
        odometryPods.update();
        return odometryPods.getPosition();
    }

    /**
     * Override the odometry driver's current position estimate.
     * Call this once after construction (before the match starts) so that
     * the robot's known starting position is reflected in all subsequent
     * {@link #driveToPosition} calculations.
     *
     * @param xInches         Field X coordinate of the starting position (inches).
     * @param yInches         Field Y coordinate of the starting position (inches).
     * @param headingDegrees  Starting heading in degrees (CCW positive, field frame).
     */
    public void setStartingPose(double xInches, double yInches, double headingDegrees) {
        if (!hasPinpoint) return;   // silently ignored when running without odometry
        odometryPods.setPosition(
                new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
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

    /** Wrap an angle in radians to the range (-π, π]. */
    private static double normalizeAngleRad(double radians) {
        while (radians >  Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Cut all drive motor power immediately. */
    private void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
