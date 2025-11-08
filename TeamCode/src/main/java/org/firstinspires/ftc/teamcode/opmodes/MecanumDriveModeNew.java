package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;
import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;

@TeleOp(name = "MecanumDriveModeNew")
public class MecanumDriveModeNew extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;
    private static final int PIPE_BLUE = 0;
    private static final int PIPE_RED  = 1;

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    private int selectedTid = BLUE_GOAL_TID;
    private int selectedPipe = PIPE_BLUE;
    private boolean prevRB1 = false;
    private long lastNs;

    private DcMotor intakeMotor;

    private Indexer indexer;
    private boolean prevLB2 = false; // manual advance
    private boolean prevRB2 = false; // manual cam
    private boolean prevB2  = false; // auto-launch trigger

    private Flywheel flywheel;
    private boolean prevUp2 = false, prevDown2 = false;

    @Override
    public void init() {
        drive.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);

        selectedTid = BLUE_GOAL_TID;
        selectedPipe = PIPE_BLUE;
        llVision.setPipeline(selectedPipe);
        llVision.setPreferredTid(selectedTid);
        llVision.start(100);

        HeadingLockController.Config cfg = new HeadingLockController.Config();
        lockCtrl = new HeadingLockController(llVision, null, cfg);
        lockCtrl.setDesiredTid(selectedTid);

        lastNs = System.nanoTime();

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        indexer = new Indexer("Indexer", "camServo");
        indexer.init(hardwareMap);

        flywheel = new Flywheel("flywheelRight", "flywheelLeft");
        flywheel.init(hardwareMap);

        telemetry.addLine("[G1] LB=Hold Align, RB=Toggle Goal & Pipeline (BLUE↔RED)");
        telemetry.addLine("Indexer: GP2 LB = advance one slot.");
        telemetry.addLine("Cam:     GP2 RB = toggle cam.");
        telemetry.addLine("Auto:    GP2 B  = auto launch 3 balls.");
        telemetry.addLine("Intake:  GP2 Y = FORWARD while held, GP2 A = REVERSE while held.");
        telemetry.addLine("Flywheel (GP2): Dpad Up = CLOSE (toggle), Dpad Down = LONG (toggle).");
    }

    @Override
    public void loop() {
        llVision.poll();

        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            if (selectedTid == BLUE_GOAL_TID) {
                selectedTid = RED_GOAL_TID;
                selectedPipe = PIPE_RED;
            } else {
                selectedTid = BLUE_GOAL_TID;
                selectedPipe = PIPE_BLUE;
            }
            lockCtrl.setDesiredTid(selectedTid);
            llVision.setPreferredTid(selectedTid);
            llVision.setPipeline(selectedPipe);
        }
        prevRB1 = rb1;

        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        double forward = -gamepad1.right_stick_y;
        double right = gamepad1.right_stick_x;
        double rotateDriver = gamepad1.left_stick_x;

        boolean lockHold = gamepad1.left_bumper;
        double omega = lockCtrl.update(dt, rotateDriver, lockHold, true);

        drive.drive(forward, right, omega);

        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) flywheel.toggleClose();
        if (down2 && !prevDown2) flywheel.toggleLong();
        prevUp2 = up2;
        prevDown2 = down2;
        flywheel.update(dt);

        double intakePower = 0.0;
        if (gamepad2.y) intakePower = 1.0;
        else if (gamepad2.a) intakePower = -1.0;
        intakeMotor.setPower(intakePower);

        // ===== AUTO-LAUNCH (GP2 B) =====
        boolean b2 = gamepad2.b;
        if (b2 && !prevB2) {
            indexer.startAutoLaunchAllThree();
        }
        prevB2 = b2;

        // ===== Manual indexer/cam (disabled during auto) =====
        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2 && !indexer.isAutoRunning() && !indexer.isMoving()) {
            indexer.advanceOneSlot();
        }
        prevLB2 = lb2;

// ===== Manual cam (GP2 RB) =====
        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2 && !indexer.isAutoRunning()) {
            indexer.setCamOpen(!indexer.isCamOpen());
        }
        prevRB2 = rb2;

        // Always update indexer (handles motion + auto sequence)
        indexer.update(dt);

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
        telemetry.addData("Flywheel Left Measured RPM", "%.0f", flywheel.getMeasuredLeftRpm());

        telemetry.addData("Intake power", "%.1f", intakePower);
        telemetry.addData("Indexer moving", indexer.isMoving());
        telemetry.addData("Cam open", indexer.isCamOpen());
        telemetry.addData("Auto launching", indexer.isAutoRunning());
        telemetry.update();
    }
}