package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;

@TeleOp(name="MecanumDriveModeNew")
public class MecanumDriveModeNew extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    // ====== HEADING LOCK: Tag IDs & Pipelines ======
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    private static final int PIPE_BLUE = 0;  // <-- set these to YOUR actual LL pipeline slots
    private static final int PIPE_RED  = 1;  // <-- e.g., blue pipeline in 0, red pipeline in 1

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    private int  selectedTid = BLUE_GOAL_TID;
    private int  selectedPipe = PIPE_BLUE;
    private boolean prevRB1 = false; // G1 right bumper edge
    private long lastNs;

    // ---- Flywheel motors ----
    private DcMotorEx flywheelRight;
    private DcMotor   flywheelLeft;

    // ---- Intake ----
    private DcMotor intakeMotor;

    // ---------- Indexer ----------
    private DcMotorEx indexer;
    private static final int TICKS_PER_REV = 1425;
    private static final int SLOTS = 3;
    private static final double INDEXER_POWER = 0.7;

    private int ticksPerSlot = TICKS_PER_REV / SLOTS;
    private int targetPosition = 0;
    private boolean indexerMoving = false;
    private boolean prevLB2 = false;

    // ---------- Cam servo ----------
    private Servo camServo;
    private static final double CAM_INIT_POSITION = 0.65;
    private static final double CAM_OPEN_POSITION = 0.0;
    private boolean camOpen = false;
    private boolean prevRB2 = false;

    // --- Encoder / kinematics constants ---
    private static final double MOTOR_TICKS_PER_REV = 28;
    private static final double FLYWHEEL_PER_MOTOR = 1.0;
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double MOTOR_MAX_TICKS_PER_SEC = (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV) / 60.0;

    // ---- Flywheel targets ----
    private static final double CLOSE_FLYWHEEL_RPM = 2300;
    private static final double LONG_FLYWHEEL_RPM  = 3000;

    // ---- Velocity PIDF for RIGHT flywheel ----
    private static final PIDFCoefficients VEL_PIDF = new PIDFCoefficients(21, 0.000, 2, 15);

    private enum FlywheelState { OFF, CLOSE, LONG }
    private FlywheelState state = FlywheelState.OFF;

    // Button edges (GP2 D-pad)
    private boolean prevUp2 = false, prevDown2 = false;

    private static double flywheelRpmToMotorTicksPerSec(double flywheelRpm) {
        double motorRpm = flywheelRpm / FLYWHEEL_PER_MOTOR;
        return (motorRpm * MOTOR_TICKS_PER_REV) / 60.0;
    }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    @Override
    public void init() {
        drive.init(hardwareMap);

        // ====== Limelight + controller ======
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // DS name must match
        llVision = new LimelightVisionFtc(limelight);

        // Default to BLUE (pipeline & TID)
        selectedTid  = BLUE_GOAL_TID;
        selectedPipe = PIPE_BLUE;
        llVision.setPipeline(selectedPipe);
        llVision.setPreferredTid(selectedTid);
        llVision.start(100);

        HeadingLockController.Config cfg = new HeadingLockController.Config();
        lockCtrl = new HeadingLockController(llVision, /* no IMU for robot-centric */ null, cfg);
        lockCtrl.setDesiredTid(selectedTid);

        lastNs = System.nanoTime();

        // ====== Your existing hardware ======
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft  = hardwareMap.get(DcMotor.class,   "flywheelLeft");
        intakeMotor   = hardwareMap.get(DcMotor.class,   "intakeMotor");
        indexer       = hardwareMap.get(DcMotorEx.class, "Indexer");
        camServo      = hardwareMap.get(Servo.class, "camServo");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try { flywheelRight.setVelocityPIDFCoefficients(VEL_PIDF.p, VEL_PIDF.i, VEL_PIDF.d, VEL_PIDF.f); } catch (Exception ignored) {}

        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerSlot = TICKS_PER_REV / SLOTS;
        targetPosition = 0;
        indexerMoving = false;

        camServo.setPosition(CAM_INIT_POSITION);

        telemetry.addLine("[G1] LB=Hold Align, RB=Toggle Goal & Pipeline (BLUE↔RED)");
        telemetry.addLine("Indexer: GP2 LB = advance one slot.");
        telemetry.addLine("Cam:     GP2 RB = toggle cam.");
        telemetry.addLine("Intake:  GP2 Y = FORWARD while held, GP2 A = REVERSE while held.");
        telemetry.addLine("Flywheel (GP2): Dpad Up = CLOSE (toggle), Dpad Down = LONG (toggle).");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ====== Vision update ======
        llVision.poll();

        // G1 Right Bumper toggles BLUE<->RED AND switches pipeline accordingly
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            if (selectedTid == BLUE_GOAL_TID) {
                selectedTid  = RED_GOAL_TID;
                selectedPipe = PIPE_RED;
            } else {
                selectedTid  = BLUE_GOAL_TID;
                selectedPipe = PIPE_BLUE;
            }
            lockCtrl.setDesiredTid(selectedTid);
            llVision.setPreferredTid(selectedTid);
            llVision.setPipeline(selectedPipe); // <-- switch camera pipeline too
        }
        prevRB1 = rb1;

        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9; lastNs = now;

        // ---------- MECANUM DRIVE (robot-centric) ----------
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotateDriver =  gamepad1.left_stick_x;

        boolean lockHold = gamepad1.left_bumper; // hold to align
        double omega = lockCtrl.update(dt, rotateDriver, lockHold, /*hold*/ true);

        drive.drive(forward, right, omega);

        // ---------- FLYWHEEL ----------
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;

        if (up2 && !prevUp2)    state = (state == FlywheelState.CLOSE) ? FlywheelState.OFF : FlywheelState.CLOSE;
        if (down2 && !prevDown2) state = (state == FlywheelState.LONG)  ? FlywheelState.OFF : FlywheelState.LONG;
        prevUp2 = up2; prevDown2 = down2;

        double targetFlywheelRpm = (state == FlywheelState.CLOSE) ? CLOSE_FLYWHEEL_RPM :
                (state == FlywheelState.LONG)  ? LONG_FLYWHEEL_RPM  : 0.0;

        if (state == FlywheelState.OFF) {
            flywheelRight.setPower(0.0);
            flywheelLeft.setPower(0.0);
        } else {
            double targetTps = flywheelRpmToMotorTicksPerSec(targetFlywheelRpm);
            flywheelRight.setVelocity(targetTps);
            double leftPower = (targetTps / MOTOR_MAX_TICKS_PER_SEC) * 1.1;
            flywheelLeft.setPower(clamp(leftPower, 0.0, 1.0));
        }

        // ---------- INTAKE ----------
        double intakePower = 0.0;
        if (gamepad2.y)      intakePower = 1.0;
        else if (gamepad2.a) intakePower = -1.0;
        intakeMotor.setPower(intakePower);

        // ---------- INDEXER ----------
        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2 && !indexerMoving) {
            targetPosition += ticksPerSlot;
            indexer.setTargetPosition(targetPosition);
            indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexer.setPower(INDEXER_POWER);
            indexerMoving = true;
        }
        int posErr = targetPosition - indexer.getCurrentPosition();
        boolean atTarget = Math.abs(posErr) <= 10 || !indexer.isBusy();
        if (indexerMoving && atTarget) {
            indexer.setPower(0.0);
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            indexerMoving = false;
        }
        prevLB2 = lb2;

        // ---------- CAM SERVO ----------
        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2) {
            camOpen = !camOpen;
            camServo.setPosition(camOpen ? CAM_OPEN_POSITION : CAM_INIT_POSITION);
        }
        prevRB2 = rb2;

        // ---------- TELEMETRY ----------
        telemetry.addData("Goal", (selectedTid == RED_GOAL_TID) ? "RED(24)" : "BLUE(20)");
        telemetry.addData("Pipeline", selectedPipe);
        telemetry.addData("HoldAlign(G1 LB)", lockHold);
        telemetry.addData("HasTarget", llVision.hasTarget());
        telemetry.addData("tid", llVision.getTid());
        telemetry.addData("tx", "%.2f", llVision.getTxDeg());
        telemetry.addData("omega", "%.2f", omega);

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