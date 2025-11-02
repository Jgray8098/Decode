package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;
import org.firstinspires.ftc.teamcode.mechanism.Flywheel;

@TeleOp(name="MecanumDriveModeNew")
public class MecanumDriveModeNew extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    // ====== HEADING LOCK: Tag IDs & Pipelines ======
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    private static final int PIPE_BLUE = 0;  // <-- set these to YOUR actual LL pipeline slots
    private static final int PIPE_RED  = 1;

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    private int  selectedTid = BLUE_GOAL_TID;
    private int  selectedPipe = PIPE_BLUE;
    private boolean prevRB1 = false; // G1 right bumper edge
    private long lastNs;

    // ---- Intake ----
    private DcMotor intakeMotor;

    // ---------- Indexer ----------
    private DcMotorEx indexer;
    private static final int TICKS_PER_REV = 1425;
    private static final int SLOTS = 3;
    private static final double INDEXER_POWER = 0.9;

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

    // ---------- Flywheel Mechanism ----------
    private Flywheel flywheel;
    private boolean prevUp2 = false, prevDown2 = false;

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

        // ====== Other hardware ======
        intakeMotor   = hardwareMap.get(DcMotor.class,   "intakeMotor");
        indexer       = hardwareMap.get(DcMotorEx.class, "Indexer");
        camServo      = hardwareMap.get(Servo.class,     "camServo");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerSlot = TICKS_PER_REV / SLOTS;
        targetPosition = 0;
        indexerMoving = false;

        camServo.setPosition(CAM_INIT_POSITION);

        // ====== Flywheel mechanism ======
        // Uses the same hardware names you had: "flywheelRight" (Ex) and "flywheelLeft" (DcMotor)
        flywheel = new Flywheel("flywheelRight", "flywheelLeft");
        flywheel.init(hardwareMap);

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

        // ---------- FLYWHEEL (via mechanism) ----------
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;

        if (up2 && !prevUp2)     flywheel.toggleClose();
        if (down2 && !prevDown2) flywheel.toggleLong();
        prevUp2 = up2; prevDown2 = down2;

// Custom PIDF update with dt
        flywheel.update(dt);

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

        telemetry.addData("Flywheel State", flywheel.getState());
        telemetry.addData("Flywheel Target RPM", "%.0f", flywheel.getTargetRpm());
        telemetry.addData("Flywheel Right Measured RPM", "%.0f", flywheel.getMeasuredRightRpm());
        telemetry.addData("Flywheel Left Measured RPM", "%.0f",  flywheel.getMeasuredLeftRpm());

        telemetry.addData("Intake power", "%.1f", intakePower);
        telemetry.addData("Indexer curr", indexer.getCurrentPosition());
        telemetry.addData("Indexer next", targetPosition + ticksPerSlot);
        telemetry.addData("Indexer moving", indexerMoving);
        telemetry.update();
    }
}