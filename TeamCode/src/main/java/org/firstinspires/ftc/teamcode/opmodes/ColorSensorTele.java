package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

// NEW: artifact tracking + motif storage
import org.firstinspires.ftc.teamcode.control.ArtifactTracker;
import org.firstinspires.ftc.teamcode.control.MotifStorage;

@TeleOp(name = "ColorSensorTele")
public class ColorSensorTele extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();

    // Tag IDs
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    private static final int MAX_PIPE_SLOT = 9;

    // ===== Heading lock tx setpoints =====
    private static final double TX_SETPOINT_CLOSE = 0.0;
    private static final double TX_SETPOINT_BLUE_LONG = +2;
    private static final double TX_SETPOINT_RED_LONG  = -2;

    // =========================
    // ===== AUTO-NAV TARGETS ===
    // =========================
    // Pedro coords (inches, heading radians)
    private static final Pose FAR_LAUNCH_RED_POSE  = new Pose(75, 39, Math.toRadians(68));
    private static final Pose FAR_LAUNCH_BLUE_POSE = new Pose(73, 39, Math.toRadians(106));

    // Cancel auto-nav if driver moves sticks beyond this
    private static final double CANCEL_STICK_THRESH = 0.12;

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    // Alliance/pipe selection
    private int selectedTid;
    private int selectedPipe;
    private int bluePipe = 0;
    private int redPipe  = 1;

    // Edge tracking
    private boolean prevRB1 = false;
    private boolean prevDL1 = false;
    private boolean prevDR1 = false;

    private long lastNs;

    private DcMotor intakeMotor;

    // Indexer
    private Indexer indexer;
    private boolean prevLB2 = false;
    private boolean prevRB2 = false; // AUTO-SORT
    private boolean prevB2  = false;
    private boolean jogMode = false;
    private boolean prevPS2 = false;
    private boolean prevX2  = false; // CAM TOGGLE (currently commented)

    private Flywheel flywheel;
    private boolean prevUp2 = false, prevDown2 = false;

    // =========================
    // ===== PEDRO AUTO-NAV =====
    // =========================
    private Follower follower;
    private boolean autoNavActive = false;
    private boolean prevY1 = false;
    private boolean prevB1 = false;
    @SuppressWarnings("unused")
    private PathChain autoNavPath = null;

    // Keep a guaranteed-non-null pose we can fall back to
    private Pose safePose = new Pose(0, 0, 0);

    // =========================
    // ===== ARTIFACT TRACKING ==
    // =========================
    private RevColorSensorV3 colorSensorFrontR, colorSensorBackR, colorSensorFrontL, colorSensorBackL;
    private ArtifactTracker artifactTracker;

    // When we run auto-launch (3 shots), we reset tracker to EMPTY after it finishes
    private boolean resetAfterLaunchPending = false;
    private boolean prevIndexerAutoRunning = false;

    // When we advance one slot, rotate tracker AFTER the move completes
    private boolean advancePending = false;

    // NEW: true if the pending advance was started via startPreAdvanceOneSlot()
    private boolean advanceWasPre = false;

    // Auto-sort state machine (RB2)
    private boolean sortActive = false;
    private int sortStepsRemaining = 0;

    @Override
    public void init() {
        drive.init(hardwareMap);

        // ---------- Pedro follower ----------
        follower = Constants.createFollower(hardwareMap);

        Pose startPose = (PoseStorage.lastPose != null) ? PoseStorage.lastPose : new Pose(0, 0, 0);
        safePose = startPose;

        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();

        // ---------- Limelight ----------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);

        selectedTid  = BLUE_GOAL_TID;
        selectedPipe = bluePipe;
        llVision.setPipeline(selectedPipe);
        llVision.setPreferredTid(selectedTid);
        llVision.start(100);

        HeadingLockController.Config cfg = new HeadingLockController.Config();
        lockCtrl = new HeadingLockController(llVision, null, cfg);
        lockCtrl.setDesiredTid(selectedTid);
        lockCtrl.setDesiredTxDeg(TX_SETPOINT_CLOSE);

        lastNs = System.nanoTime();

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // ---------- Indexer ----------
        indexer = new Indexer(
                "Indexer",
                "camServo",
                1425,
                3,
                0.7,
                0.65,
                0.0
        );
        indexer.setHomeCamOnInit(false);
        indexer.init(hardwareMap);
        indexer.setStepDelay(0.15);

        // ---------- Flywheel ----------
        flywheel = new Flywheel("flywheelRight", "flywheelLeft", "hoodServo");
        flywheel.init(hardwareMap);
        flywheel.setHoodStartPos(0.00);
        flywheel.setHoodPositions(0.15, 0.40);

        // ---------- Color Sensors + Artifact Tracker ----------
        colorSensorFrontR = hardwareMap.get(RevColorSensorV3.class, "colorSensorFrontR");
        colorSensorBackR  = hardwareMap.get(RevColorSensorV3.class, "colorSensorBackR");
        colorSensorFrontL = hardwareMap.get(RevColorSensorV3.class, "colorSensorFrontL");
        colorSensorBackL  = hardwareMap.get(RevColorSensorV3.class, "colorSensorBackL");

        artifactTracker = new ArtifactTracker(
                colorSensorFrontR, colorSensorBackR,
                colorSensorFrontL, colorSensorBackL,
                16,      // gain
                0.06f,   // MIN_BRIGHTNESS_SUM
                0.08f,   // BALL_PRESENT_SUM
                -0.20f,  // PURPLE_SCORE_MAX
                -0.05f   // GREEN_SCORE_MIN
        );

        telemetry.addLine("[G1] LB=Hold Align, RB=Toggle Goal (BLUE↔RED)");
        telemetry.addLine("[G1] DPad L/R = change pipeline for SELECTED goal (wrap 0..9)");
        telemetry.addLine("[G1] Y = AUTO-NAV to FAR LAUNCH pose (cancel by moving sticks)");
        telemetry.addLine("[G1] B = Cancel AUTO-NAV");
        telemetry.addLine("[G2] B = Auto launch 3 (after it completes, tracker assumes indexer EMPTY)");
        telemetry.addLine("[G2] LB = Advance one slot (tracker rotates AFTER move completes)");
        telemetry.addLine("[G2] RB = AUTO SORT (uses PreAdvance to reduce jam-ups when sorting)");
        telemetry.addData("MotifTid (from Auto)", MotifStorage.motifTid);
    }

    @Override
    public void init_loop() {
        llVision.poll();

        autoNavActive = false;

        follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();

        Pose pNow = follower.getPose();
        if (pNow != null) safePose = pNow;

        handleAlliancePipelineControls();
        pushVisionConfig();
        updateHeadingTxSetpoint();

        showTelemetryBasics(false);

        Pose p = follower.getPose();
        if (p == null) telemetry.addData("PedroPose", "NULL (waiting for follower pose)");
        else telemetry.addData("PedroPose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));

        telemetry.addData("MotifTid", MotifStorage.motifTid);
        telemetry.addData("TrackingEnabled", artifactTracker.isTrackingEnabled());
        telemetry.update();
    }

    @Override
    public void start() {
        indexer.homeCam();
        indexer.syncToNextPocketForward(true);
        flywheel.enableHoodControl(true);

        drive.drive(0, 0, 0);

        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, true);

        // We do NOT assume empty at TeleOp start. We start tracking only after a launch completes.
        resetAfterLaunchPending = false;
        prevIndexerAutoRunning = indexer.isAutoRunning();

        advancePending = false;
        advanceWasPre = false;

        sortActive = false;
        sortStepsRemaining = 0;
    }

    @Override
    public void loop() {
        llVision.poll();

        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        // ===== Alliance toggle + pipeline slot adjust (G1) =====
        handleAlliancePipelineControls();
        pushVisionConfig();

        // ---------- Flywheel toggles ----------
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) flywheel.toggleClose();
        if (down2 && !prevDown2) flywheel.toggleLong();
        prevUp2 = up2;
        prevDown2 = down2;

        updateHeadingTxSetpoint();
        flywheel.update(dt);

        boolean flywheelActive = flywheel.getTargetRpm() > 0.0;

        // ---------- AUTO-NAV TRIGGER / CANCEL ----------
        boolean y1 = gamepad1.y;
        if (y1 && !prevY1 && !autoNavActive) startAutoNavToFarLaunch();
        prevY1 = y1;

        boolean b1 = gamepad1.b;
        if (autoNavActive && b1 && !prevB1) cancelAutoNav();
        prevB1 = b1;

        if (autoNavActive && driverIsCommandingSticks()) cancelAutoNav();

        // ---------- PEDRO UPDATE ----------
        if (!autoNavActive) follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();

        Pose pNow = follower.getPose();
        if (pNow != null) safePose = pNow;

        // ---------- DRIVE CONTROL ----------
        if (autoNavActive) {
            if (!follower.isBusy()) cancelAutoNav();
        } else {
            double forward = -gamepad1.right_stick_y;
            double right   =  gamepad1.right_stick_x;
            double rotateDriver =  gamepad1.left_stick_x;

            boolean lockHold = gamepad1.left_bumper;
            double omega = lockCtrl.update(dt, rotateDriver, lockHold, true);

            drive.drive(forward, right, omega);
        }

        // ---------- INTAKE ----------
        double intakePower = 0.0;
        if (gamepad2.y)      intakePower = 1.0;
        else if (gamepad2.a) intakePower = -1.0;
        intakeMotor.setPower(intakePower);

        // ---------- INDEXER: JOG MODE ----------
        boolean ps2 = gamepad2.ps;
        if (ps2 && !prevPS2) {
            jogMode = !jogMode;
            if (jogMode) indexer.enterJogMode();
            else         indexer.exitJogMode(true);
        }
        prevPS2 = ps2;

        if (jogMode) {
            indexer.jog(gamepad2.left_stick_x);
            telemetry.addLine("INDEXER JOG MODE: stick X to jog, PS to exit+reset");
            telemetry.update();
            return;
        }

        // ---------- AUTO-LAUNCH (B2) ----------
        boolean b2 = gamepad2.b;
        if (b2 && !prevB2 && flywheelActive && !indexer.isAutoRunning() && !indexer.isMoving() && !sortActive) {
            indexer.startAutoLaunchAllThreeContinuous();
            resetAfterLaunchPending = true;
        }
        prevB2 = b2;

        // ---------- MANUAL ADVANCE ONE SLOT (LB2) ----------
        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2 && flywheelActive && !indexer.isAutoRunning() && !indexer.isMoving()
                && !sortActive && !advancePending && !indexer.isPreAdvancing()) {
            indexer.advanceOneSlot();
            advancePending = true;
            advanceWasPre = false;
        }
        prevLB2 = lb2;

        // ---------- AUTO SORT (RB2) ----------
        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2 && !indexer.isAutoRunning() && !indexer.isMoving()
                && !sortActive && !advancePending && !indexer.isPreAdvancing()) {

            // Only if tracking is enabled and exactly 2 PURPLE + 1 GREEN
            if (artifactTracker.isTrackingEnabled()
                    && artifactTracker.count(ArtifactTracker.Color.PURPLE) == 2
                    && artifactTracker.count(ArtifactTracker.Color.GREEN) == 1) {

                int motifTid = MotifStorage.motifTid; // 21=GPP, 22=PGP, 23=PPG
                int steps = computeSortStepsForMotif(motifTid);
                if (steps > 0) {
                    sortActive = true;
                    sortStepsRemaining = steps;
                }
            }
        }
        prevRB2 = rb2;

        // ---------- INDEXER UPDATE ----------
        indexer.update(dt);

        // ---------- Handle "advance completed" rotation (manual OR sort) ----------
        boolean advanceDone = advancePending
                && !indexer.isMoving()
                && (!advanceWasPre || !indexer.isPreAdvancing());

        if (advanceDone) {
            artifactTracker.onAdvanceForward();
            advancePending = false;
            advanceWasPre = false;
        }

        // ---------- Detect launch completion -> reset tracking to EMPTY ----------
        boolean autoRunning = indexer.isAutoRunning();
        if (prevIndexerAutoRunning && !autoRunning && resetAfterLaunchPending) {
            artifactTracker.startTrackingEmpty();
            resetAfterLaunchPending = false;

            sortActive = false;
            sortStepsRemaining = 0;

            advancePending = false;
            advanceWasPre = false;
        }
        prevIndexerAutoRunning = autoRunning;

        // ---------- AUTO SORT state machine (advance steps 1 at a time) ----------
        if (sortActive) {
            if (!indexer.isMoving()
                    && !indexer.isAutoRunning()
                    && !advancePending
                    && !indexer.isPreAdvancing()
                    && sortStepsRemaining > 0) {

                // use gentle pre-advance during sorting
                indexer.startPreAdvanceOneSlot();
                advancePending = true;
                advanceWasPre = true;
                sortStepsRemaining--;
            }

            if (sortStepsRemaining == 0
                    && !indexer.isMoving()
                    && !advancePending
                    && !indexer.isPreAdvancing()) {
                sortActive = false;
            }
        }

        // ---------- ARTIFACT TRACKER UPDATE ----------
        artifactTracker.update();

        // ---------- TELEMETRY ----------
        showTelemetryBasics(true);
        telemetry.addData("Tx Setpoint", "%.1f", lockCtrl.getDesiredTxDeg());

        Pose p = follower.getPose();
        if (p == null) {
            telemetry.addData("PedroPose", "NULL (safePose %.1f, %.1f, %.1f°)",
                    safePose.getX(), safePose.getY(), Math.toDegrees(safePose.getHeading()));
        } else {
            telemetry.addData("PedroPose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }

        telemetry.addData("AutoNav", autoNavActive);
        if (autoNavActive) telemetry.addData("Follower busy", follower.isBusy());

        telemetry.addData("MotifTid", MotifStorage.motifTid);
        telemetry.addData("TrackingEnabled", artifactTracker.isTrackingEnabled());
        telemetry.addData("Slots (1,2,3)", "%s %s %s",
                artifactTracker.getSlot(1), artifactTracker.getSlot(2), artifactTracker.getSlot(3));
        telemetry.addData("Counts (G,P,E,U)", "%d %d %d %d",
                artifactTracker.count(ArtifactTracker.Color.GREEN),
                artifactTracker.count(ArtifactTracker.Color.PURPLE),
                artifactTracker.count(ArtifactTracker.Color.EMPTY),
                artifactTracker.count(ArtifactTracker.Color.UNKNOWN));
        telemetry.addData("SortActive", sortActive);
        telemetry.addData("SortStepsRemaining", sortStepsRemaining);
        telemetry.addData("IndexerPreAdvancing", indexer.isPreAdvancing());

        telemetry.update();
    }

    // ================= AUTO-NAV =================

    private void startAutoNavToFarLaunch() {
        Pose target = (selectedTid == RED_GOAL_TID) ? FAR_LAUNCH_RED_POSE : FAR_LAUNCH_BLUE_POSE;

        drive.drive(0, 0, 0);

        follower.startTeleopDrive();

        Pose cur = follower.getPose();
        if (cur == null) cur = safePose;

        autoNavPath = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), target.getHeading())
                .build();

        follower.followPath(autoNavPath, true);
        autoNavActive = true;
    }

    private void cancelAutoNav() {
        autoNavActive = false;
        autoNavPath = null;

        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, true);
        drive.drive(0, 0, 0);
    }

    private boolean driverIsCommandingSticks() {
        double f = Math.abs(-gamepad1.right_stick_y);
        double s = Math.abs( gamepad1.right_stick_x);
        double r = Math.abs( gamepad1.left_stick_x);
        return (f > CANCEL_STICK_THRESH) || (s > CANCEL_STICK_THRESH) || (r > CANCEL_STICK_THRESH);
    }

    // ================= SORT HELPERS =================

    private ArtifactTracker.Color[] desiredForMotif(int tid) {
        // Desired LAUNCH ORDER (assumed): [slot3, slot1, slot2]
        // 21=GPP, 22=PGP, 23=PPG

        if (tid == 21) return new ArtifactTracker.Color[]{
                ArtifactTracker.Color.GREEN, ArtifactTracker.Color.PURPLE, ArtifactTracker.Color.PURPLE
        };

        // ✅ FIX: swap 22 and 23 desired patterns (they were reversed)
        if (tid == 22) return new ArtifactTracker.Color[]{
                ArtifactTracker.Color.PURPLE, ArtifactTracker.Color.PURPLE, ArtifactTracker.Color.GREEN
        };

        if (tid == 23) return new ArtifactTracker.Color[]{
                ArtifactTracker.Color.PURPLE, ArtifactTracker.Color.GREEN, ArtifactTracker.Color.PURPLE
        };

        return null;
    }

    private ArtifactTracker.Color[] launchOrderAfterSteps(int steps) {
        ArtifactTracker.Color s1 = artifactTracker.getSlot(1);
        ArtifactTracker.Color s2 = artifactTracker.getSlot(2);
        ArtifactTracker.Color s3 = artifactTracker.getSlot(3);

        for (int i = 0; i < steps; i++) {
            ArtifactTracker.Color old1 = s1, old2 = s2, old3 = s3;
            s1 = old3;
            s2 = old1;
            s3 = old2;
        }

        // Launch order assumed: slot3 then slot1 then slot2
        return new ArtifactTracker.Color[]{ s3, s1, s2 };
    }

    private int computeSortStepsForMotif(int motifTid) {
        ArtifactTracker.Color[] desired = desiredForMotif(motifTid);
        if (desired == null) return -1;

        for (int k = 0; k < 3; k++) {
            ArtifactTracker.Color[] order = launchOrderAfterSteps(k);
            boolean exact =
                    order[0] == desired[0] &&
                            order[1] == desired[1] &&
                            order[2] == desired[2];
            if (exact) return k;
        }
        return -1;
    }

    // ================= Helpers =================

    private void updateHeadingTxSetpoint() {
        boolean isBlue = (selectedTid == BLUE_GOAL_TID);
        boolean isLong = (flywheel.getState() == Flywheel.State.LONG);

        double txSetpoint = TX_SETPOINT_CLOSE;
        if (isLong) txSetpoint = isBlue ? TX_SETPOINT_BLUE_LONG : TX_SETPOINT_RED_LONG;

        lockCtrl.setDesiredTxDeg(txSetpoint);
    }

    private void handleAlliancePipelineControls() {
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            selectedTid = (selectedTid == BLUE_GOAL_TID) ? RED_GOAL_TID : BLUE_GOAL_TID;
            selectedPipe = (selectedTid == BLUE_GOAL_TID) ? bluePipe : redPipe;

            lockCtrl.setDesiredTid(selectedTid);
            llVision.setPreferredTid(selectedTid);

            if (autoNavActive) cancelAutoNav();
        }
        prevRB1 = rb1;

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

    private void pushVisionConfig() {
        llVision.setPipeline(selectedPipe);
        llVision.setPreferredTid(selectedTid);
    }

    private int wrapPipe(int v) {
        if (v < 0) return MAX_PIPE_SLOT;
        if (v > MAX_PIPE_SLOT) return 0;
        return v;
    }

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
