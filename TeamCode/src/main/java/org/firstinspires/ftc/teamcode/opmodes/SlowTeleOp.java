package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDriveSlow;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;

@TeleOp(name = "SlowTeleOp")
public class SlowTeleOp extends OpMode {
    MecanumDriveSlow drive = new MecanumDriveSlow();

    // Tag IDs
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    // Limit how high pipeline slot can go (adjust to your Limelight config)
    private static final int MAX_PIPE_SLOT = 9;

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    // Alliance/pipe selection
    private int selectedTid;
    private int selectedPipe;
    private int bluePipe = 0;  // editable at runtime/init
    private int redPipe  = 1;  // editable at runtime/init

    // Edge tracking
    private boolean prevRB1 = false;     // G1: toggle BLUE/RED
    private boolean prevDL1 = false;     // G1: dpad left (pipe -1)
    private boolean prevDR1 = false;     // G1: dpad right (pipe +1)

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

        // Default to BLUE goal
        selectedTid  = BLUE_GOAL_TID;
        selectedPipe = bluePipe;
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
        indexer.setHomeCamOnInit(false);   // do NOT move the servo during TeleOp INIT
        indexer.init(hardwareMap);

        flywheel = new Flywheel("flywheelRight", "flywheelLeft");
        flywheel.init(hardwareMap);

        telemetry.addLine("[G1] LB=Hold Align, RB=Toggle Goal (BLUE↔RED)");
        telemetry.addLine("[G1] DPad L/R = change pipeline for SELECTED goal (wrap 0..9)");
        telemetry.addLine("Indexer: GP2 LB = advance one slot.");
        telemetry.addLine("Cam:     GP2 RB = toggle cam.");
        telemetry.addLine("Auto:    GP2 B  = auto launch 3 balls.");
        telemetry.addLine("Intake:  GP2 Y = FORWARD while held, GP2 A = REVERSE while held.");
        telemetry.addLine("Flywheel (GP2): Dpad Up = CLOSE (toggle), Dpad Down = LONG (toggle).");
    }

    /** Allow pipeline slot edits during TeleOp INIT (before start). */
    @Override
    public void init_loop() {
        llVision.poll();
        handleAlliancePipelineControls();   // G1 RB/DPad works during INIT too
        pushVisionConfig();                 // apply any changes
        showTelemetryBasics(false);
        telemetry.update();
    }

    @Override
    public void start() {
        // TeleOp has officially started — safe to home the cam once
        indexer.homeCam();
    }

    @Override
    public void loop() {
        llVision.poll();

        // ===== Alliance toggle + pipeline slot adjust (G1) =====
        handleAlliancePipelineControls();
        pushVisionConfig(); // apply any changes to Limelight

        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        // ---------- MECANUM DRIVE (robot-centric) ----------
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotateDriver =  gamepad1.left_stick_x;

        boolean lockHold = gamepad1.left_bumper; // hold to align
        double omega = lockCtrl.update(dt, rotateDriver, lockHold, true);

        drive.drive(forward, right, omega);

        // ---------- FLYWHEEL ----------
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) flywheel.toggleClose();
        if (down2 && !prevDown2) flywheel.toggleLong();
        prevUp2 = up2;
        prevDown2 = down2;
        flywheel.update(dt);

        // ---------- INTAKE ----------
        double intakePower = 0.0;
        if (gamepad2.y)      intakePower = 1.0;
        else if (gamepad2.a) intakePower = -1.0;
        intakeMotor.setPower(intakePower);

        // ---------- INDEXER ----------
        boolean b2 = gamepad2.b;
        if (b2 && !prevB2) indexer.startAutoLaunchAllThree();
        prevB2 = b2;

        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2 && !indexer.isAutoRunning() && !indexer.isMoving()) {
            indexer.advanceOneSlot();
        }
        prevLB2 = lb2;

        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2 && !indexer.isAutoRunning()) {
            indexer.setCamOpen(!indexer.isCamOpen());
        }
        prevRB2 = rb2;

        // keep indexer state machine updated
        indexer.update(dt);

        // ---------- TELEMETRY ----------
        showTelemetryBasics(true);
        telemetry.addData("Intake power", "%.1f", intakePower);
        telemetry.addData("Indexer moving", indexer.isMoving());
        telemetry.addData("Cam open", indexer.isCamOpen());
        telemetry.addData("Auto launching", indexer.isAutoRunning());
        telemetry.update();
    }

    // ================= Helpers =================

    /** Handle G1 inputs that change alliance or pipeline slot (works in init_loop and loop). */
    private void handleAlliancePipelineControls() {
        // Toggle BLUE <-> RED goal
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            if (selectedTid == BLUE_GOAL_TID) {
                selectedTid = RED_GOAL_TID;
            } else {
                selectedTid = BLUE_GOAL_TID;
            }
            // select that alliance's pipe
            selectedPipe = (selectedTid == BLUE_GOAL_TID) ? bluePipe : redPipe;
            lockCtrl.setDesiredTid(selectedTid);
            llVision.setPreferredTid(selectedTid);
        }
        prevRB1 = rb1;

        // Adjust pipeline slot for the CURRENTLY SELECTED alliance
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        int delta = 0;
        if (dl && !prevDL1) delta = -1;
        if (dr && !prevDR1) delta = +1;

        if (delta != 0) {
            if (selectedTid == BLUE_GOAL_TID) {
                bluePipe = wrapPipe(bluePipe + delta);
                selectedPipe = bluePipe;
            } else {
                redPipe = wrapPipe(redPipe + delta);
                selectedPipe = redPipe;
            }
        }

        prevDL1 = dl;
        prevDR1 = dr;
    }

    /** Push current alliance+pipeline settings into Limelight. */
    private void pushVisionConfig() {
        llVision.setPipeline(selectedPipe);
        // preferred TID already set when toggling; keep it updated for clarity
        llVision.setPreferredTid(selectedTid);
    }

    /** Wrap pipeline slot into [0..MAX_PIPE_SLOT]. */
    private int wrapPipe(int v) {
        if (v < 0) return MAX_PIPE_SLOT;
        if (v > MAX_PIPE_SLOT) return 0;
        return v;
    }

    /** Common telemetry for both init_loop and loop. */
    private void showTelemetryBasics(boolean includeFlywheel) {
        telemetry.addData("Alliance/Goal", (selectedTid == RED_GOAL_TID) ? "RED(24)" : "BLUE(20)");
        telemetry.addData("Active Pipeline", selectedPipe);
        telemetry.addData("Blue Pipe Slot", bluePipe);
        telemetry.addData("Red  Pipe Slot", redPipe);
        telemetry.addData("HasTarget", llVision.hasTarget());
        telemetry.addData("tid", llVision.getTid());
        telemetry.addData("tx", "%.2f", llVision.getTxDeg());

        if (includeFlywheel) {
            telemetry.addData("Flywheel State", flywheel.getState());
            telemetry.addData("Flywheel Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("Flywheel Right Measured RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("Flywheel Left Measured RPM", "%.0f", flywheel.getMeasuredLeftRpm());
        }
    }
}
