package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;

//@TeleOp(name = "MecanumDriveModeNew")
public class MecanumDriveModeNew extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    // Tag IDs
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    // Limit how high pipeline slot can go (adjust to your Limelight config)
    private static final int MAX_PIPE_SLOT = 9;

    // ===== Heading lock tx setpoints =====
    // Close shots aim dead-center (tx -> 0)
    private static final double TX_SETPOINT_CLOSE = 0.0;
    // Long shots:
    //  - BLUE: aim 3° LEFT => tag appears 3° RIGHT => tx setpoint = +3
    //  - RED : aim 3° RIGHT => tag appears 3° LEFT => tx setpoint = -3
    private static final double TX_SETPOINT_BLUE_LONG = +2;
    private static final double TX_SETPOINT_RED_LONG  = -2;

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

    // Indexer
    private Indexer indexer;
    private boolean prevLB2 = false; // manual advance
    private boolean prevRB2 = false; // manual cam
    private boolean prevB2  = false; // auto-launch trigger
    private boolean jogMode = false;
    private boolean prevPS2 = false;

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
        lockCtrl.setDesiredTxDeg(TX_SETPOINT_CLOSE); // start centered

        lastNs = System.nanoTime();

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Indexer for TeleOp
        indexer = new Indexer(
                "Indexer",
                "camServo",
                1425,   // ticksPerRev
                3,      // slots
                0.7,    // TeleOp power
                0.65,   // cam closed
                0.0     // cam open
        );
        indexer.setHomeCamOnInit(false);   // do NOT move the servo during TeleOp INIT
        indexer.init(hardwareMap);
        indexer.setStepDelay(0.15);

        // --------- Flywheel + Hood Servo ---------
        // NOTE: OpMode config only. Actual hood motion is owned by Flywheel class.
        flywheel = new Flywheel("flywheelRight", "flywheelLeft", "hoodServo");
        flywheel.init(hardwareMap);
        flywheel.setHoodStartPos(0.00);         // <-- tune this
        flywheel.setHoodPositions(0.15, 0.40);  // close, long (tune these)

        telemetry.addLine("[G1] LB=Hold Align, RB=Toggle Goal (BLUE↔RED)");
        telemetry.addLine("[G1] DPad L/R = change pipeline for SELECTED goal (wrap 0..9)");
        telemetry.addLine("Indexer: GP2 LB = advance one slot (requires flywheel running).");
        telemetry.addLine("Cam:     GP2 RB = toggle cam.");
        telemetry.addLine("Auto:    GP2 B  = auto launch 3 balls (requires flywheel running).");
        telemetry.addLine("Intake:  GP2 Y = FORWARD while held, GP2 A = REVERSE while held.");
        telemetry.addLine("Flywheel (GP2): Dpad Up = CLOSE (toggle), Dpad Down = LONG (toggle).");
        telemetry.addLine("Heading lock: CLOSE aims tx=0. LONG aims tx=+3(BLUE) / -3(RED).");
        telemetry.addLine("Hood: owned by Flywheel (Start on Play, then Close/Long based on state).");
    }

    /** Allow pipeline slot edits during TeleOp INIT (before start). */
    @Override
    public void init_loop() {
        llVision.poll();
        handleAlliancePipelineControls();
        pushVisionConfig();

        // Keep tx setpoint consistent even in init_loop (optional but nice)
        updateHeadingTxSetpoint();

        showTelemetryBasics(false);
        telemetry.addData("Hood pos", "%.2f", flywheel.getHoodPosition());
        telemetry.addData("Tx Setpoint", "%.1f", lockCtrl.getDesiredTxDeg());
        telemetry.update();
    }

    @Override
    public void start() {
        // TeleOp has officially started — safe to home the cam once
        indexer.homeCam();
        indexer.syncToNextPocketForward(true);  // forward-only snap to pocket center

        // Allow hood motion AFTER Play. Flywheel will move hood to hoodStartPos immediately.
        flywheel.enableHoodControl(true);
    }

    @Override
    public void loop() {
        llVision.poll();

        // ===== Alliance toggle + pipeline slot adjust (G1) =====
        handleAlliancePipelineControls();
        pushVisionConfig();

        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        // ---------- FLYWHEEL (state toggles) ----------
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) flywheel.toggleClose();
        if (down2 && !prevDown2) flywheel.toggleLong();
        prevUp2 = up2;
        prevDown2 = down2;

        // Set heading lock tx target based on alliance + flywheel state
        updateHeadingTxSetpoint();

        // ---------- MECANUM DRIVE (robot-centric) ----------
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotateDriver =  gamepad1.left_stick_x;

        boolean lockHold = gamepad1.left_bumper; // hold to align
        double omega = lockCtrl.update(dt, rotateDriver, lockHold, true);

        drive.drive(forward, right, omega);

        // Update flywheel (and hood) every loop
        flywheel.update(dt);

        // Is launcher "running" (has a nonzero target)?
        boolean flywheelActive = flywheel.getTargetRpm() > 0.0;

        // ---------- INTAKE ----------
        double intakePower = 0.0;
        if (gamepad2.y)      intakePower = 1.0;
        else if (gamepad2.a) intakePower = -1.0;
        intakeMotor.setPower(intakePower);

        // ---------- INDEXER ----------
        boolean ps2 = gamepad2.ps;
        if (ps2 && !prevPS2) {
            jogMode = !jogMode;
            if (jogMode) indexer.enterJogMode();
            else         indexer.exitJogMode(true); // reset encoder on exit
        }
        prevPS2 = ps2;

        if (jogMode) {
            indexer.jog(gamepad2.left_stick_x);
            telemetry.addLine("INDEXER JOG MODE: stick X to jog, PS to exit+reset");
            telemetry.update();
            return; // block normal indexer actions while jogging
        }

        boolean b2 = gamepad2.b;
        // Auto 3-ball launch ONLY if flywheel is running and indexer is free
        if (b2 && !prevB2
                && flywheelActive
                && !indexer.isAutoRunning()
                && !indexer.isMoving()) {
            indexer.startAutoLaunchAllThreeContinuous();
        }
        prevB2 = b2;

        boolean lb2 = gamepad2.left_bumper;
        // Manual advance ONLY if flywheel is running
        if (lb2 && !prevLB2
                && flywheelActive
                && !indexer.isAutoRunning()
                && !indexer.isMoving()) {
            indexer.advanceOneSlot();
        }
        prevLB2 = lb2;

        boolean rb2 = gamepad2.right_bumper;
        // Cam can still be toggled anytime (doesn't advance slots by itself)
        if (rb2 && !prevRB2 && !indexer.isAutoRunning()) {
            indexer.setCamOpen(!indexer.isCamOpen());
        }
        prevRB2 = rb2;

        // Keep indexer state machine updated
        indexer.update(dt);

        // ---------- TELEMETRY ----------
        showTelemetryBasics(true);
        telemetry.addData("Tx Setpoint", "%.1f", lockCtrl.getDesiredTxDeg());
        telemetry.addData("Hood pos", "%.2f", flywheel.getHoodPosition());
        telemetry.addData("Intake power", "%.1f", intakePower);
        telemetry.addData("Indexer moving", indexer.isMoving());
        telemetry.addData("Cam open", indexer.isCamOpen());
        telemetry.addData("Auto launching", indexer.isAutoRunning());
        telemetry.addData("Flywheel Target RPM", "%.0f", flywheel.getTargetRpm());
        telemetry.addData("Indexer allowed to advance", flywheelActive);
        telemetry.update();
    }

    // ================= Helpers =================

    /** Set heading lock target tx based on alliance + flywheel state. */
    private void updateHeadingTxSetpoint() {
        boolean isBlue = (selectedTid == BLUE_GOAL_TID);
        boolean isLong = (flywheel.getState() == Flywheel.State.LONG);

        double txSetpoint = TX_SETPOINT_CLOSE;
        if (isLong) {
            txSetpoint = isBlue ? TX_SETPOINT_BLUE_LONG : TX_SETPOINT_RED_LONG;
        }
        lockCtrl.setDesiredTxDeg(txSetpoint);
    }

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















