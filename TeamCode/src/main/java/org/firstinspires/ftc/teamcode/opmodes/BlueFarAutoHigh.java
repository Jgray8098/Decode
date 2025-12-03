package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueFarAutoHigh", group = "Comp")
public class BlueFarAutoHigh extends LinearOpMode {

    // ===== Limelight pipelines & TIDs =====
    private static final int PIPE_OBELISK_1 = 2; // Tag 21 => GPP
    private static final int PIPE_OBELISK_2 = 3; // Tag 22 => PGP
    private static final int PIPE_OBELISK_3 = 4; // Tag 23 => PPG

    private static final int TID_GPP = 21;
    private static final int TID_PGP = 22;
    private static final int TID_PPG = 23;

    // ===== Flywheel (FAR/LONG) gates =====
    private static final double SPINUP_MIN_WAIT_S = 0.60;
    private static final double SPINUP_TIMEOUT_S  = 2.00;
    private static final double RPM_TOL           = 60.0;

    // ===== Drive power profiles =====
    private static final double MAX_POWER_NORMAL = 1.0;
    private static final double MAX_POWER_INTAKE = 0.45;  // tweak (0.4–0.6)

    // ===== Intake settle timing =====
    private static final double INTAKE_SETTLE_S = 0.65;

    // ===== Field Poses (inches, radians) – BLUE FAR (mirrored from your RedFarAutoHigh) =====
    // Mirror across X = 72 on a 144" field:
    //   x' = 144 - x
    //   heading' = 180° - heading

    // Start pose (mirrored)
    private static final Pose START_POSE = new Pose(
            61.58, 9.65, Math.toRadians(90.0));

    // Launch preloads pose (mirrored)
    private static final Pose LAUNCH_PRELOADS_POSE = new Pose(
            59.07, 20.06, Math.toRadians(118.0));

    // Row 1 intake line (mirrored)
    private static final Pose ALIGN_INTAKE1_POSE = new Pose(
            48.50, 34.517, Math.toRadians(180.0));
    private static final Pose INTAKE_P11_POSE = new Pose(
            38.204, 34.517, Math.toRadians(180.0));
    private static final Pose INTAKE_P12_POSE = new Pose(
            32.389, 34.517, Math.toRadians(180.0));
    private static final Pose INTAKE_G11_POSE = new Pose(
            27.793, 34.517, Math.toRadians(180.0));

    // Launch pose for first row (mirrored)
    private static final Pose LAUNCH_FIRST_ROW_POSE = new Pose(
            53.07, 21.06, Math.toRadians(106.0));   // tune on-field if needed

    // Row 2 intake line (mirrored)
    private static final Pose ALIGN_INTAKE2_POSE = new Pose(
            50.50, 58.465, Math.toRadians(180.0));
    private static final Pose INTAKE_P21_POSE = new Pose(
            36.985, 58.465, Math.toRadians(180.0));
    private static final Pose INTAKE_G21_POSE = new Pose(
            32.170, 58.465, Math.toRadians(180.0));
    private static final Pose INTAKE_P22_POSE = new Pose(
            27.137, 58.465, Math.toRadians(180.0));

    // Launch pose for second row (mirrored)
    private static final Pose LAUNCH_SECOND_ROW_POSE = new Pose(
            54.07, 21.06, Math.toRadians(105.0));  // tune separately if needed

    // Final park (mirrored)
    private static final Pose PARK_POSE = new Pose(
            38.079, 14.444, Math.toRadians(90.0));

    // ===== Devices =====
    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private Indexer indexer;
    private Flywheel flywheel;
    private DcMotor intakeMotor;

    // ===== Pedro =====
    private Follower follower;
    private Paths paths;

    // Tag detection
    private int activePipeline = PIPE_OBELISK_1;
    private int detectedTid = -1;
    private long lastPipeSwapNs = 0L;

    // Pre-advance sets (three separate launch events)
    private int preAdvanceTotalPreloads = 0;
    private int preAdvanceRemainingPreloads = 0;

    private int preAdvanceTotalRow1 = 0;
    private int preAdvanceRemainingRow1 = 0;

    private int preAdvanceTotalRow2 = 0;
    private int preAdvanceRemainingRow2 = 0;

    // Intake control
    private boolean intakeActive = false;

    // Intake settle timing
    private double intakeSettleTimerS = 0.0;
    private boolean settleAdvanceIssued = false;

    // Spin-up timing
    private double spinupElapsedS = 0.0;

    // Which tag pattern we ended up using
    private int tidToUse = -1;

    // Launch trigger guards
    private boolean launchPreloadsStarted = false;
    private boolean launchRow1Started    = false;
    private boolean launchRow2Started    = false;

    private enum Phase {
        DRIVE_LAUNCH_PRELOADS,
        ARRIVED_SPINUP_PRELOADS,
        FIRE_THREE_PRELOADS,

        DRIVE_ALIGN_INTAKE1,
        DRIVE_INTAKE_PURPLE11,
        WAIT_INDEXER_AFTER_P11,
        DRIVE_INTAKE_PURPLE12,
        WAIT_INDEXER_AFTER_P12,
        DRIVE_INTAKE_GREEN11,
        WAIT_SETTLE_AFTER_G11,

        DRIVE_LAUNCH_FIRST_ROW,
        ARRIVED_SPINUP_FIRST_ROW,
        FIRE_THREE_FIRST_ROW,

        DRIVE_ALIGN_INTAKE2,
        DRIVE_INTAKE_PURPLE21,
        WAIT_INDEXER_AFTER_P21,
        DRIVE_INTAKE_GREEN21,
        WAIT_SETTLE_AFTER_G21,
        DRIVE_INTAKE_PURPLE22,
        WAIT_SETTLE_AFTER_P22,

        DRIVE_LAUNCH_SECOND_ROW,
        ARRIVED_SPINUP_SECOND_ROW,
        FIRE_THREE_SECOND_ROW,

        DRIVE_PARK,
        DONE
    }
    private Phase phase = Phase.DRIVE_LAUNCH_PRELOADS;

    @Override
    public void runOpMode() {
        // ---- Hardware ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);
        llVision.setPipeline(activePipeline);
        llVision.start(100);

        indexer = new Indexer("Indexer", "camServo");
        indexer.init(hardwareMap);
        indexer.hardZero();   // Auto owns the encoder zero

        flywheel = new Flywheel("flywheelRight", "flywheelLeft");
        flywheel.init(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0.0);

        // ---- Pedro ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);

        setDrivePowerNormal();

        telemetry.addLine("BlueFarAutoHigh: Init – detecting AprilTag while idle.");
        telemetry.addData("Pose Start", poseStr(START_POSE));
        telemetry.addData("Pose Launch Preloads", poseStr(LAUNCH_PRELOADS_POSE));
        telemetry.addData("Pose Align Intake 1", poseStr(ALIGN_INTAKE1_POSE));
        telemetry.addData("Pose P11", poseStr(INTAKE_P11_POSE));
        telemetry.addData("Pose P12", poseStr(INTAKE_P12_POSE));
        telemetry.addData("Pose G11", poseStr(INTAKE_G11_POSE));
        telemetry.addData("Pose Launch Row1", poseStr(LAUNCH_FIRST_ROW_POSE));
        telemetry.addData("Pose Align Intake 2", poseStr(ALIGN_INTAKE2_POSE));
        telemetry.addData("Pose P21", poseStr(INTAKE_P21_POSE));
        telemetry.addData("Pose G21", poseStr(INTAKE_G21_POSE));
        telemetry.addData("Pose P22", poseStr(INTAKE_P22_POSE));
        telemetry.addData("Pose Launch Row2", poseStr(LAUNCH_SECOND_ROW_POSE));
        telemetry.addData("Pose Park", poseStr(PARK_POSE));
        telemetry.update();

        // ===== INIT loop: detect tag while robot is stationary =====
        lastPipeSwapNs = System.nanoTime();
        while (!isStarted() && !isStopRequested()) {
            llVision.poll();

            long now = System.nanoTime();
            if ((now - lastPipeSwapNs) / 1e6 > 50) {
                activePipeline = (activePipeline == PIPE_OBELISK_1)
                        ? PIPE_OBELISK_2
                        : (activePipeline == PIPE_OBELISK_2 ? PIPE_OBELISK_3 : PIPE_OBELISK_1);
                llVision.setPipeline(activePipeline);
                lastPipeSwapNs = now;
            }

            if (llVision.hasTarget()) {
                int tid = llVision.getTid();
                if (tid == TID_GPP || tid == TID_PGP || tid == TID_PPG) {
                    detectedTid = tid;
                    tidToUse = tid;
                }
            }

            telemetry.addLine("BlueFarAutoHigh: INIT – scanning AprilTag");
            telemetry.addData("ActivePipe", activePipeline);
            telemetry.addData("HasTarget", llVision.hasTarget());
            telemetry.addData("TID", llVision.getTid());
            telemetry.addData("Detected TID", detectedTid);
            telemetry.update();
            sleep(30);
        }

        // ===== RUN =====
        if (isStopRequested()) return;

        long lastNs = System.nanoTime();

        if (tidToUse != TID_GPP && tidToUse != TID_PGP && tidToUse != TID_PPG) {
            tidToUse = TID_PPG; // safe default
        }

        preAdvanceTotalPreloads     = computePreAdvancePreloads(tidToUse);
        preAdvanceRemainingPreloads = preAdvanceTotalPreloads;

        // FAR program -> LONG speed
        flywheel.setState(Flywheel.State.LONG);

        setDrivePowerNormal();
        follower.followPath(paths.LaunchPreloads, true);
        phase = Phase.DRIVE_LAUNCH_PRELOADS;

        while (opModeIsActive() && phase != Phase.DONE) {
            double dt = (System.nanoTime() - lastNs) / 1e9;
            lastNs = System.nanoTime();

            llVision.poll();
            flywheel.update(dt);
            indexer.update(dt);
            follower.update();

            switch (phase) {
                // --- Launch preloads ---
                case DRIVE_LAUNCH_PRELOADS: {
                    if (preAdvanceRemainingPreloads > 0
                            && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemainingPreloads--;
                    }

                    if (!follower.isBusy()
                            && preAdvanceRemainingPreloads == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_PRELOADS;
                    }
                    break;
                }

                case ARRIVED_SPINUP_PRELOADS: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_PRELOADS;
                    }
                    break;
                }

                case FIRE_THREE_PRELOADS: {
                    if (!launchPreloadsStarted) {
                        indexer.startAutoLaunchAllThree();
                        launchPreloadsStarted = true;
                    }
                    if (!indexer.isAutoRunning()) {
                        setDrivePowerNormal();
                        follower.followPath(paths.AlignIntake1, true);
                        phase = Phase.DRIVE_ALIGN_INTAKE1;
                    }
                    break;
                }

                // --- Intake Row 1 ---
                case DRIVE_ALIGN_INTAKE1: {
                    if (!follower.isBusy()) {
                        setDrivePowerIntake();
                        intakeStart();
                        follower.followPath(paths.IntakePurple11, true);
                        phase = Phase.DRIVE_INTAKE_PURPLE11;
                    }
                    break;
                }

                case DRIVE_INTAKE_PURPLE11: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        settleAdvanceIssued = false;
                        phase = Phase.WAIT_INDEXER_AFTER_P11;
                    }
                    break;
                }

                case WAIT_INDEXER_AFTER_P11: {
                    intakeSettleTimerS += dt;

                    if (!settleAdvanceIssued
                            && intakeSettleTimerS >= INTAKE_SETTLE_S
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        indexer.advanceOneSlot();
                        settleAdvanceIssued = true;
                    }

                    if (settleAdvanceIssued
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        follower.followPath(paths.IntakePurple12, true);
                        phase = Phase.DRIVE_INTAKE_PURPLE12;
                    }
                    break;
                }

                case DRIVE_INTAKE_PURPLE12: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        settleAdvanceIssued = false;
                        phase = Phase.WAIT_INDEXER_AFTER_P12;
                    }
                    break;
                }

                case WAIT_INDEXER_AFTER_P12: {
                    intakeSettleTimerS += dt;

                    if (!settleAdvanceIssued
                            && intakeSettleTimerS >= INTAKE_SETTLE_S
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        indexer.advanceOneSlot();
                        settleAdvanceIssued = true;
                    }

                    if (settleAdvanceIssued
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        follower.followPath(paths.IntakeGreen11, true);
                        phase = Phase.DRIVE_INTAKE_GREEN11;
                    }
                    break;
                }

                case DRIVE_INTAKE_GREEN11: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        phase = Phase.WAIT_SETTLE_AFTER_G11;
                    }
                    break;
                }

                case WAIT_SETTLE_AFTER_G11: {
                    intakeSettleTimerS += dt;

                    if (intakeSettleTimerS >= INTAKE_SETTLE_S) {
                        intakeStop();

                        preAdvanceTotalRow1     = computePreAdvanceRow1(tidToUse);
                        preAdvanceRemainingRow1 = preAdvanceTotalRow1;

                        setDrivePowerNormal();
                        follower.followPath(paths.LaunchFirstRow, true);
                        phase = Phase.DRIVE_LAUNCH_FIRST_ROW;
                    }
                    break;
                }

                // --- Launch Row 1 ---
                case DRIVE_LAUNCH_FIRST_ROW: {
                    if (preAdvanceRemainingRow1 > 0
                            && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemainingRow1--;
                    }

                    if (!follower.isBusy()
                            && preAdvanceRemainingRow1 == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_FIRST_ROW;
                    }
                    break;
                }

                case ARRIVED_SPINUP_FIRST_ROW: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_FIRST_ROW;
                    }
                    break;
                }

                case FIRE_THREE_FIRST_ROW: {
                    if (!launchRow1Started) {
                        indexer.startAutoLaunchAllThree();
                        launchRow1Started = true;
                    }
                    if (!indexer.isAutoRunning()) {
                        setDrivePowerNormal();
                        follower.followPath(paths.AlignIntake2, true);
                        phase = Phase.DRIVE_ALIGN_INTAKE2;
                    }
                    break;
                }

                // --- Intake Row 2 ---
                case DRIVE_ALIGN_INTAKE2: {
                    if (!follower.isBusy()) {
                        setDrivePowerIntake();
                        intakeStart();
                        follower.followPath(paths.IntakePurple21, true);
                        phase = Phase.DRIVE_INTAKE_PURPLE21;
                    }
                    break;
                }

                case DRIVE_INTAKE_PURPLE21: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        settleAdvanceIssued = false;
                        phase = Phase.WAIT_INDEXER_AFTER_P21;
                    }
                    break;
                }

                case WAIT_INDEXER_AFTER_P21: {
                    intakeSettleTimerS += dt;

                    if (!settleAdvanceIssued
                            && intakeSettleTimerS >= INTAKE_SETTLE_S
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        indexer.advanceOneSlot();
                        settleAdvanceIssued = true;
                    }

                    if (settleAdvanceIssued
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        follower.followPath(paths.IntakeGreen21, true);
                        phase = Phase.DRIVE_INTAKE_GREEN21;
                    }
                    break;
                }

                case DRIVE_INTAKE_GREEN21: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        settleAdvanceIssued = false;
                        phase = Phase.WAIT_SETTLE_AFTER_G21;
                    }
                    break;
                }

                case WAIT_SETTLE_AFTER_G21: {
                    intakeSettleTimerS += dt;

                    if (!settleAdvanceIssued
                            && intakeSettleTimerS >= INTAKE_SETTLE_S
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        indexer.advanceOneSlot();
                        settleAdvanceIssued = true;
                    }

                    if (settleAdvanceIssued
                            && !indexer.isAutoRunning()
                            && !indexer.isMoving()) {
                        follower.followPath(paths.IntakePurple22, true);
                        phase = Phase.DRIVE_INTAKE_PURPLE22;
                    }
                    break;
                }

                case DRIVE_INTAKE_PURPLE22: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        phase = Phase.WAIT_SETTLE_AFTER_P22;
                    }
                    break;
                }

                case WAIT_SETTLE_AFTER_P22: {
                    intakeSettleTimerS += dt;

                    if (intakeSettleTimerS >= INTAKE_SETTLE_S) {
                        intakeStop();

                        preAdvanceTotalRow2     = computePreAdvanceRow2(tidToUse);
                        preAdvanceRemainingRow2 = preAdvanceTotalRow2;

                        setDrivePowerNormal();
                        follower.followPath(paths.LaunchSecondRow, true);
                        phase = Phase.DRIVE_LAUNCH_SECOND_ROW;
                    }
                    break;
                }

                // --- Launch Row 2 ---
                case DRIVE_LAUNCH_SECOND_ROW: {
                    if (preAdvanceRemainingRow2 > 0
                            && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemainingRow2--;
                    }

                    if (!follower.isBusy()
                            && preAdvanceRemainingRow2 == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_SECOND_ROW;
                    }
                    break;
                }

                case ARRIVED_SPINUP_SECOND_ROW: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_SECOND_ROW;
                    }
                    break;
                }

                case FIRE_THREE_SECOND_ROW: {
                    if (!launchRow2Started) {
                        indexer.startAutoLaunchAllThree();
                        launchRow2Started = true;
                    }
                    if (!indexer.isAutoRunning()) {
                        setDrivePowerNormal();
                        follower.followPath(paths.Park, true);
                        phase = Phase.DRIVE_PARK;
                    }
                    break;
                }

                case DRIVE_PARK: {
                    if (!follower.isBusy()) {
                        phase = Phase.DONE;
                    }
                    break;
                }

                case DONE:
                default:
                    break;
            }

            telemetry.addData("Phase", phase);
            telemetry.addData("ActivePipe", activePipeline);
            telemetry.addData("HasTarget", llVision.hasTarget());
            telemetry.addData("TID", llVision.getTid());
            telemetry.addData("Detected TID", detectedTid);
            telemetry.addData("Using TID", tidToUse);
            telemetry.addData("Preloads pre-adv rem", preAdvanceRemainingPreloads);
            telemetry.addData("Row1 pre-adv rem", preAdvanceRemainingRow1);
            telemetry.addData("Row2 pre-adv rem", preAdvanceRemainingRow2);
            telemetry.addData("FW Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("FW Right RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("FW Left RPM", "%.0f", flywheel.getMeasuredLeftRpm());
            telemetry.addData("Spinup Elapsed (s)", "%.2f", spinupElapsedS);
            telemetry.addData("Indexer Auto", indexer.isAutoRunning());
            telemetry.addData("Indexer Moving", indexer.isMoving());
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Intake Settle (s)", "%.2f", intakeSettleTimerS);
            telemetry.addData("Pose", poseStr(follower.getPose()));
            telemetry.update();
        }

        // Safety
        intakeStop();
        indexer.setCamOpen(false);
        flywheel.stop();
    }

    // ===== Drive power helpers =====
    private void setDrivePowerNormal() { follower.setMaxPower(MAX_POWER_NORMAL); }
    private void setDrivePowerIntake() { follower.setMaxPower(MAX_POWER_INTAKE); }

    // ===== Helpers =====
    private boolean flywheelReady() {
        double target = flywheel.getTargetRpm();
        double errR = Math.abs(flywheel.getMeasuredRightRpm() - target);
        double errL = Math.abs(flywheel.getMeasuredLeftRpm() - target);
        return target > 0 && errR < RPM_TOL && errL < RPM_TOL && (spinupElapsedS >= SPINUP_MIN_WAIT_S);
    }

    private void intakeStart() {
        intakeActive = true;
        intakeMotor.setPower(1.0);
    }

    private void intakeStop() {
        intakeActive = false;
        intakeMotor.setPower(0.0);
    }

    // Preloads mapping: PPG → 0, PGP → 1, GPP → 2
    private int computePreAdvancePreloads(int tid) {
        if (tid == TID_PPG) return 0;
        if (tid == TID_PGP) return 1;
        if (tid == TID_GPP) return 2;
        return 0;
    }

    // Keep EXACTLY the same mapping you’re using for your RedFarAutoHigh
    private int computePreAdvanceRow1(int tid) {
        if (tid == TID_PPG) return 1;
        if (tid == TID_PGP) return 2;
        if (tid == TID_GPP) return 0;
        return 0;
    }

    private int computePreAdvanceRow2(int tid) {
        if (tid == TID_PPG) return 2;
        if (tid == TID_PGP) return 0;
        if (tid == TID_GPP) return 1;
        return 0;
    }

    private static String poseStr(Pose p) {
        return String.format("(%.2f, %.2f, %.1f°)",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    // ===== Paths wired to mirrored poses =====
    public static class Paths {
        public PathChain LaunchPreloads;
        public PathChain AlignIntake1;
        public PathChain IntakePurple11;
        public PathChain IntakePurple12;
        public PathChain IntakeGreen11;
        public PathChain LaunchFirstRow;
        public PathChain AlignIntake2;
        public PathChain IntakePurple21;
        public PathChain IntakeGreen21;
        public PathChain IntakePurple22;
        public PathChain LaunchSecondRow;
        public PathChain Park;

        public Paths(Follower follower) {
            LaunchPreloads = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, LAUNCH_PRELOADS_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), LAUNCH_PRELOADS_POSE.getHeading())
                    .build();

            AlignIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_PRELOADS_POSE, ALIGN_INTAKE1_POSE))
                    .setLinearHeadingInterpolation(LAUNCH_PRELOADS_POSE.getHeading(), ALIGN_INTAKE1_POSE.getHeading())
                    .build();

            IntakePurple11 = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE1_POSE, INTAKE_P11_POSE))
                    .setLinearHeadingInterpolation(ALIGN_INTAKE1_POSE.getHeading(), INTAKE_P11_POSE.getHeading())
                    .build();

            IntakePurple12 = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_P11_POSE, INTAKE_P12_POSE))
                    .setLinearHeadingInterpolation(INTAKE_P11_POSE.getHeading(), INTAKE_P12_POSE.getHeading())
                    .build();

            IntakeGreen11 = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_P12_POSE, INTAKE_G11_POSE))
                    .setLinearHeadingInterpolation(INTAKE_P12_POSE.getHeading(), INTAKE_G11_POSE.getHeading())
                    .build();

            LaunchFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_G11_POSE, LAUNCH_FIRST_ROW_POSE))
                    .setLinearHeadingInterpolation(INTAKE_G11_POSE.getHeading(), LAUNCH_FIRST_ROW_POSE.getHeading())
                    .build();

            AlignIntake2 = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_FIRST_ROW_POSE, ALIGN_INTAKE2_POSE))
                    .setLinearHeadingInterpolation(LAUNCH_FIRST_ROW_POSE.getHeading(), ALIGN_INTAKE2_POSE.getHeading())
                    .build();

            IntakePurple21 = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE2_POSE, INTAKE_P21_POSE))
                    .setLinearHeadingInterpolation(ALIGN_INTAKE2_POSE.getHeading(), INTAKE_P21_POSE.getHeading())
                    .build();

            IntakeGreen21 = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_P21_POSE, INTAKE_G21_POSE))
                    .setLinearHeadingInterpolation(INTAKE_P21_POSE.getHeading(), INTAKE_G21_POSE.getHeading())
                    .build();

            IntakePurple22 = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_G21_POSE, INTAKE_P22_POSE))
                    .setLinearHeadingInterpolation(INTAKE_G21_POSE.getHeading(), INTAKE_P22_POSE.getHeading())
                    .build();

            LaunchSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_P22_POSE, LAUNCH_SECOND_ROW_POSE))
                    .setLinearHeadingInterpolation(INTAKE_P22_POSE.getHeading(), LAUNCH_SECOND_ROW_POSE.getHeading())
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_SECOND_ROW_POSE, PARK_POSE))
                    .setLinearHeadingInterpolation(LAUNCH_SECOND_ROW_POSE.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}
