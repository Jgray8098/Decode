package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;

@TeleOp(name="MecanumDriveModeNew")
public class MecanumDriveModeNew extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    // ---- Flywheel motors ----
    private DcMotorEx flywheelRight; // encoder connected
    private DcMotor   flywheelLeft;  // no encoder

    // ---- Intake ----
    private DcMotor intakeMotor;
    private boolean intakeOn = false;
    private boolean lastButtonState = false; // B toggle

    // ---------- Indexer (forward-only, step-by-slot) ----------
    private DcMotorEx indexer;
    private static final int TICKS_PER_REV = 1425; // verify for your indexer motor/ratio
    private static final int SLOTS = 3;
    private static final double INDEXER_POWER = 0.5;

    private int ticksPerSlot = TICKS_PER_REV / SLOTS;
    private int targetPosition = 0;
    private boolean indexerMoving = false;
    private boolean prevY = false; // Y button edge for indexer

    // --- Encoder / kinematics constants (goBILDA 6000 rpm, 1:1) ---
    private static final double MOTOR_TICKS_PER_REV = 28; // REV Hub ticks per motor rev
    private static final double FLYWHEEL_PER_MOTOR = 1.0;

    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double MOTOR_MAX_TICKS_PER_SEC = (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV) / 60.0;

    // ---- Flywheel targets (RPM at wheel; 1:1) ----
    private static final double CLOSE_FLYWHEEL_RPM = 2300;
    private static final double LONG_FLYWHEEL_RPM  = 3000;

    // ---- Velocity PIDF for the RIGHT flywheel motor ----
    private static final PIDFCoefficients VEL_PIDF = new PIDFCoefficients(
            21,    // P
            0.000, // I
            2,     // D
            15     // F
    );

    // Open-loop scale for the left follower motor
    private static final double LEFT_POWER_SCALE = 1.1;

    private enum FlywheelState { OFF, CLOSE, LONG }
    private FlywheelState state = FlywheelState.OFF;

    // Button edge-detection (flywheel)
    private boolean prevLB = false, prevRB = false, prevA = false;

    // --- Helpers ---
    private static double flywheelRpmToMotorTicksPerSec(double flywheelRpm) {
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
        intakeMotor   = hardwareMap.get(DcMotor.class,   "intakeMotor");
        indexer       = hardwareMap.get(DcMotorEx.class, "Indexer");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Flywheels: float on zero power for smooth coast
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // If mounted mirrored, reverse exactly ONE so both spin forward
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Right (master) uses velocity control
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            flywheelRight.setVelocityPIDFCoefficients(VEL_PIDF.p, VEL_PIDF.i, VEL_PIDF.d, VEL_PIDF.f);
        } catch (Exception ignored) {}

        // Left (follower) open-loop
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ----- Indexer init -----
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // stay here until we set a target
        ticksPerSlot = TICKS_PER_REV / SLOTS;
        targetPosition = 0;
        indexerMoving = false;

        telemetry.addLine("Indexer: manually place at HOME, then start.");
        telemetry.addLine("Press Y to advance one slot forward (non-blocking).");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------- MECANUM DRIVE ----------
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotate  =  gamepad1.left_stick_x;
        drive.drive(forward, right, rotate);

        // ---------- FLYWHEEL TOGGLES ----------
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean a  = gamepad1.a; // A = OFF toggle

        if (lb && !prevLB) state = (state == FlywheelState.CLOSE) ? FlywheelState.OFF : FlywheelState.CLOSE;
        if (rb && !prevRB) state = (state == FlywheelState.LONG)  ? FlywheelState.OFF : FlywheelState.LONG;
        if (a  && !prevA ) state = FlywheelState.OFF;

        prevLB = lb; prevRB = rb; prevA = a;

        double targetFlywheelRpm =
                (state == FlywheelState.CLOSE) ? CLOSE_FLYWHEEL_RPM :
                        (state == FlywheelState.LONG)  ? LONG_FLYWHEEL_RPM  : 0.0;

        if (state == FlywheelState.OFF) {
            flywheelRight.setPower(0.0);
            flywheelLeft.setPower(0.0);
        } else {
            double targetTps = flywheelRpmToMotorTicksPerSec(targetFlywheelRpm);
            flywheelRight.setVelocity(targetTps);

            double leftPower = (targetTps / MOTOR_MAX_TICKS_PER_SEC) * LEFT_POWER_SCALE;
            flywheelLeft.setPower(clamp(leftPower, 0.0, 1.0));
        }

        // ---------- INTAKE TOGGLE (gamepad1.b) ----------
        boolean currentButtonState = gamepad1.b;
        if (currentButtonState && !lastButtonState) {
            intakeOn = !intakeOn;
        }
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
        lastButtonState = currentButtonState;

        // ---------- INDEXER: one-slot advance on Y (non-blocking) ----------
        boolean y = gamepad1.y;

        // Rising edge: issue a new move if we're not already moving
        if (y && !prevY && !indexerMoving) {
            targetPosition += ticksPerSlot;                 // always forward
            indexer.setTargetPosition(targetPosition);      // target FIRST
            indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexer.setPower(INDEXER_POWER);
            indexerMoving = true;
        }

        // Consider a small tolerance in case isBusy() is noisy
        int posErr = targetPosition - indexer.getCurrentPosition();
        boolean atTarget = Math.abs(posErr) <= 10 || !indexer.isBusy();

        if (indexerMoving && atTarget) {
            indexer.setPower(0.0);
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // safe idle mode
            indexerMoving = false;
        }

        prevY = y; // edge tracking for indexer button

        // ---------- TELEMETRY ----------
        telemetry.addData("Indexer curr", indexer.getCurrentPosition());
        telemetry.addData("Indexer next", targetPosition + ticksPerSlot);
        telemetry.addData("Indexer moving", indexerMoving);

        double measuredTps = flywheelRight.getVelocity();
        double measuredMotorRpm = (measuredTps * 60.0) / MOTOR_TICKS_PER_REV;
        double measuredFlywheelRpm = measuredMotorRpm * FLYWHEEL_PER_MOTOR;

        telemetry.addData("Flywheel State", state);
        telemetry.addData("Target Flywheel RPM", "%.0f", targetFlywheelRpm);
        telemetry.addData("Right Measured RPM", "%.0f", measuredFlywheelRpm);
        telemetry.addData("Left Power (cmd)", "%.2f", flywheelLeft.getPower());
        telemetry.update();
    }
}