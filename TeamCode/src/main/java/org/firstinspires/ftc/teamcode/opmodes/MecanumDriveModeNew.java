package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;

@TeleOp()
public class MecanumDriveModeNew extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    // ---- Flywheel motors ----
    // Right motor has the encoder; left motor follows open-loop.
    private DcMotorEx flywheelRight; // encoder connected
    private DcMotor   flywheelLeft;  // no encoder

    // --- Encoder / kinematics constants (goBILDA 6000 rpm, 1:1) ---
    // 28 CPR * 4 (quadrature edges) = 112 ticks per motor rev on REV Hub.
    private static final double MOTOR_TICKS_PER_REV = 28;
    // 1:1 belt/gear: flywheel revs == motor revs
    private static final double FLYWHEEL_PER_MOTOR = 1.0;

    // Max theoretical motor speed (no load)
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double MOTOR_MAX_TICKS_PER_SEC = (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV) / 60.0; // ~11200

    // ---- Targets (flywheel RPM == motor RPM at 1:1). Tune on your bot. ----
    private static final double CLOSE_FLYWHEEL_RPM = 2300;
    private static final double LONG_FLYWHEEL_RPM  = 3000;

    // ---- Velocity PIDF for the RIGHT motor (starting point; tune!) ----
    private static final PIDFCoefficients VEL_PIDF = new PIDFCoefficients(
            15,   // P  (increase for quicker spin-up; reduce if oscillation)
            0.000,   // I  (usually 0 for flywheels)
            2,   // D  (add a touch if overshoot)
            15   // F  (raise until steady-state ≈ target without oscillation)
    );

    // Open-loop scale for the left follower motor (helps match torque losses)
    private static final double LEFT_POWER_SCALE = 1.05; // try 1.00–1.10 if left lags

    private enum FlywheelState { OFF, CLOSE, LONG }
    private FlywheelState state = FlywheelState.OFF;

    // Button edge-detection
    private boolean prevLB = false, prevRB = false, prevA = false;

    // --- Helpers ---
    private static double flywheelRpmToMotorTicksPerSec(double flywheelRpm) {
        // With 1:1, flywheel RPM = motor RPM
        double motorRpm = flywheelRpm / FLYWHEEL_PER_MOTOR;
        return (motorRpm * MOTOR_TICKS_PER_REV) / 60.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void init() {
        drive.init(hardwareMap);

        // Map motors (change names to match your RC config)
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight"); // encoder here
        flywheelLeft  = hardwareMap.get(DcMotor.class,   "flywheelLeft");  // no encoder

        // Float on zero power so it coasts down smoothly
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // If mounted mirrored, reverse exactly ONE so both spin the wheel forward
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Right: velocity control with encoder
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            flywheelRight.setVelocityPIDFCoefficients(VEL_PIDF.p, VEL_PIDF.i, VEL_PIDF.d, VEL_PIDF.f);
        } catch (Exception ignored) {}

        // Left: open-loop power follower (no encoder)
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // ---- Mecanum Drive ----
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotate  =  gamepad1.left_stick_x;
        drive.drive(forward, right, rotate);

        // ---- Toggle controls: LB=CLOSE, RB=LONG, A=OFF ----
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean a  = gamepad1.a;

        if (lb && !prevLB) state = (state == FlywheelState.CLOSE) ? FlywheelState.OFF : FlywheelState.CLOSE;
        if (rb && !prevRB) state = (state == FlywheelState.LONG)  ? FlywheelState.OFF : FlywheelState.LONG;
        if (a  && !prevA ) state = FlywheelState.OFF;

        prevLB = lb; prevRB = rb; prevA = a;

        // ---- Apply commands ----
        double targetFlywheelRpm =
                (state == FlywheelState.CLOSE) ? CLOSE_FLYWHEEL_RPM :
                        (state == FlywheelState.LONG)  ? LONG_FLYWHEEL_RPM  : 0.0;

        if (state == FlywheelState.OFF) {
            // Right: stop; Left follows 0 power
            flywheelRight.setPower(0.0);
            flywheelLeft.setPower(0.0);
        } else {
            // Right motor (master): closed-loop velocity on encoder
            double targetTps = flywheelRpmToMotorTicksPerSec(targetFlywheelRpm);
            flywheelRight.setVelocity(targetTps);

            // Left motor (follower): open-loop power proportional to target speed
            // Power ≈ fraction of max ticks/sec, optionally scaled for matching losses.
            double leftPower = (targetTps / MOTOR_MAX_TICKS_PER_SEC) * LEFT_POWER_SCALE;
            flywheelLeft.setPower(clamp(leftPower, 0.0, 1.0));
        }

        // ---- Telemetry (from encoder/right side) ----
        double measuredTps   = flywheelRight.getVelocity();                      // ticks/sec (right motor)
        double measuredMotorRpm = (measuredTps * 60.0) / MOTOR_TICKS_PER_REV;    // motor rpm
        double measuredFlywheelRpm = measuredMotorRpm * FLYWHEEL_PER_MOTOR;      // same at 1:1

        telemetry.addData("Flywheel State", state);
        telemetry.addData("Target Flywheel RPM", "%.0f", targetFlywheelRpm);
        telemetry.addData("Right Measured RPM", "%.0f", measuredFlywheelRpm);
        telemetry.addData("Left Power (cmd)", "%.2f", flywheelLeft.getPower());
        telemetry.update();
    }
}
