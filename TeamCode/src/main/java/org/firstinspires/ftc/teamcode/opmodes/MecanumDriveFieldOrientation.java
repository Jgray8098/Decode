package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDriveField;

//@TeleOp(name="MecanumDriveFieldOrientation")
public class MecanumDriveFieldOrientation extends OpMode {
    MecanumDriveField drive = new MecanumDriveField();
    double forward, strafe, rotate;

    // ---- Flywheel motors ----
    private DcMotorEx flywheelRight; // encoder connected
    private DcMotor flywheelLeft;  // no encoder

    // ---- Intake (gamepad2: Y forward while held, A reverse while held) ----
    private DcMotor intakeMotor;

    // ---------- Indexer (forward-only, step-by-slot) ----------
    private DcMotorEx indexer;
    private static final int TICKS_PER_REV = 1425; // verify for your indexer motor/ratio
    private static final int SLOTS = 3;
    private static final double INDEXER_POWER = 0.7;

    private int ticksPerSlot = TICKS_PER_REV / SLOTS;
    private int targetPosition = 0;
    private boolean indexerMoving = false;
    private boolean prevLB2 = false; // gamepad2.left_bumper edge

    // ---------- Cam servo (gamepad2.right_bumper) ----------
    private Servo camServo;
    private static final double CAM_INIT_POSITION = 0.65;
    private static final double CAM_OPEN_POSITION = 0.0;
    private boolean camOpen = false;
    private boolean prevRB2 = false;

    // --- Encoder / kinematics constants (goBILDA 6000 rpm, 1:1) ---
    private static final double MOTOR_TICKS_PER_REV = 28; // REV Hub ticks per motor rev
    private static final double FLYWHEEL_PER_MOTOR = 1.0;

    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double MOTOR_MAX_TICKS_PER_SEC = (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV) / 60.0;

    // ---- Flywheel targets (RPM at wheel; 1:1) ----
    private static final double CLOSE_FLYWHEEL_RPM = 2300;
    private static final double LONG_FLYWHEEL_RPM = 3000;

    // ---- Velocity PIDF for the RIGHT flywheel motor ----
    private static final PIDFCoefficients VEL_PIDF = new PIDFCoefficients(
            21,    // P
            0.000, // I
            2,     // D
            15     // F
    );

    // Open-loop scale for the left follower motor
    private static final double LEFT_POWER_SCALE = 1.1;

    private enum FlywheelState {OFF, CLOSE, LONG}

    private FlywheelState state = FlywheelState.OFF;

    // Button edge-detection (flywheel on gamepad2 dpad)
    private boolean prevUp2 = false, prevDown2 = false;

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

        // Map devices
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight"); // encoder here
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");  // no encoder
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        indexer = hardwareMap.get(DcMotorEx.class, "Indexer");       // check name in RC config
        camServo = hardwareMap.get(Servo.class, "camServo");

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
        } catch (Exception ignored) {
        }

        // Left (follower) open-loop
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ----- Indexer init -----
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // stay here until we set a target
        ticksPerSlot = TICKS_PER_REV / SLOTS;
        targetPosition = 0;
        indexerMoving = false;

        // Servo init
        camServo.setPosition(CAM_INIT_POSITION);

        telemetry.addLine("Indexer: GP2 LB = advance one slot.");
        telemetry.addLine("Cam:     GP2 RB = toggle cam.");
        telemetry.addLine("Intake:  GP2 Y = FORWARD while held, GP2 A = REVERSE while held.");
        telemetry.addLine("Flywheel (GP2): Dpad Up = CLOSE (toggle), Dpad Down = LONG (toggle).");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------- MECANUM DRIVE (gamepad1 sticks) ----------
        forward = -gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);

        // ---------- FLYWHEEL CONTROL (gamepad2 dpad, toggle logic) ----------
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;

        // Dpad Up: if already CLOSE -> OFF, else switch to CLOSE
        if (up2 && !prevUp2) {
            state = (state == FlywheelState.CLOSE) ? FlywheelState.OFF : FlywheelState.CLOSE;
        }
        // Dpad Down: if already LONG -> OFF, else switch to LONG
        if (down2 && !prevDown2) {
            state = (state == FlywheelState.LONG) ? FlywheelState.OFF : FlywheelState.LONG;
        }
        prevUp2 = up2;
        prevDown2 = down2;

        double targetFlywheelRpm =
                (state == FlywheelState.CLOSE) ? CLOSE_FLYWHEEL_RPM :
                        (state == FlywheelState.LONG) ? LONG_FLYWHEEL_RPM : 0.0;

        if (state == FlywheelState.OFF) {
            flywheelRight.setPower(0.0);
            flywheelLeft.setPower(0.0);
        } else {
            double targetTps = flywheelRpmToMotorTicksPerSec(targetFlywheelRpm);
            flywheelRight.setVelocity(targetTps);

            double leftPower = (targetTps / MOTOR_MAX_TICKS_PER_SEC) * LEFT_POWER_SCALE;
            flywheelLeft.setPower(clamp(leftPower, 0.0, 1.0));
        }

        // ---------- INTAKE (gamepad2: hold-to-run) ----------
        // Y while held = forward; A while held = reverse; Y has priority if both are held
        double intakePower = 0.0;
        if (gamepad2.y) {
            intakePower = 1.0;
        } else if (gamepad2.a) {
            intakePower = -1.0;
        }
        intakeMotor.setPower(intakePower);

        // ---------- INDEXER: one-slot advance on GP2 LEFT BUMPER ----------
        boolean lb2 = gamepad2.left_bumper;

        // Rising edge: issue a new move if we're not already moving
        if (lb2 && !prevLB2 && !indexerMoving) {
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

        prevLB2 = lb2; // edge tracking for indexer button (GP2 LB)

        // ---------- CAM SERVO TOGGLE on GP2 RIGHT BUMPER ----------
        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2) {
            camOpen = !camOpen;
            camServo.setPosition(camOpen ? CAM_OPEN_POSITION : CAM_INIT_POSITION);
        }
        prevRB2 = rb2;

        // ---------- TELEMETRY ----------
        telemetry.addData("Flywheel State", state);
        telemetry.addData("Flywheel Target RPM", "%.0f", targetFlywheelRpm);
        double measuredTps = flywheelRight.getVelocity();
        double measuredMotorRpm = (measuredTps * 60.0) / MOTOR_TICKS_PER_REV;
        double measuredFlywheelRpm = measuredMotorRpm * FLYWHEEL_PER_MOTOR;
        telemetry.addData("Flywheel Right Measured RPM", "%.0f", measuredFlywheelRpm);

        telemetry.addData("Intake power", "%.1f", intakePower);

        telemetry.addData("Indexer curr", indexer.getCurrentPosition());
        telemetry.addData("Indexer next", targetPosition + ticksPerSlot);
        telemetry.addData("Indexer moving", indexerMoving);

        telemetry.addData("Left Power (cmd)", "%.2f", flywheelLeft.getPower());
        telemetry.update();
    }
}