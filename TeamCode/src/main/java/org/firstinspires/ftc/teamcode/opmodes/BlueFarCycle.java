package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.control.MotifStorage;

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

//@Autonomous(name = "Blue Far CYCLE", group = "Comp")
public class BlueFarCycle extends LinearOpMode {

    // =========================================================
    // ===================== PATHING INFO ======================
    // =========================================================

    // Poses (inches, radians)
    private static final Pose START_POSE           = new Pose(62.006,  9.021, Math.toRadians(90));
    private static final Pose LAUNCH_PRELOADS_POSE = new Pose(59.152, 22.663, Math.toRadians(117));

    // Intake 1 (main @ y=34) --------------------------------------------
    private static final Pose ALIGN_INTAKE1_POSE    = new Pose(50.000, 34.0, Math.toRadians(180));
    private static final Pose INTAKE_G11_POSE       = new Pose(22.793, 35.0, Math.toRadians(180)); // your adjusted G11
    private static final Pose LAUNCH_FIRST_ROW_POSE = new Pose(52.000, 21.000, Math.toRadians(112));

    // NEW: Intake 1 (second pass @ y=10) --------------------------------
    private static final Pose ALIGN_INTAKE1_LOW_POSE  = new Pose(50.000, 12.0, Math.toRadians(180));
    private static final Pose INTAKE_G11_LOW_POSE     = new Pose(22.793, 10.0, Math.toRadians(180));

    // Intake 2 sequence (main) ------------------------------------------
    private static final Pose ALIGN_INTAKE2_POSE     = new Pose(55.000, 15.000, Math.toRadians(180));
    private static final Pose INTAKE_GP_POSE         = new Pose(17.454, 15.000, Math.toRadians(180));
    private static final Pose ALIGN_INTAKE2_POSE_2   = new Pose(18.951, 16.227, Math.toRadians(180));
    private static final Pose ALIGN_INTAKE2_POSE_3   = new Pose(18.969, 12.018, Math.toRadians(180));
    private static final Pose INTAKE_PP_POSE         = new Pose(13.675, 10.982, Math.toRadians(180));
    private static final Pose LAUNCH_SECOND_ROW_POSE = new Pose(54.000, 21.000, Math.toRadians(112));

    private static final Pose PARK_POSE              = new Pose(38.810, 15.074, Math.toRadians(90));

    // Heading interpolation endpoints (degrees)
    private static final double H_START_TO_PRELOADS_START_DEG = 90;
    private static final double H_START_TO_PRELOADS_END_DEG   = 117;

    private static final double H_PRELOADS_TO_ALIGN2_START_DEG = 117;
    private static final double H_PRELOADS_TO_ALIGN2_END_DEG   = 180;

    private static final double H_SECOND_RETURN_START_DEG      = 180;
    private static final double H_SECOND_RETURN_END_DEG        = 109;

    private static final double H_ROW2_TO_ALIGN1_START_DEG     = 109;
    private static final double H_ROW2_TO_ALIGN1_END_DEG       = 180;

    private static final double H_ROW1_RETURN_START_DEG        = 180;
    private static final double H_ROW1_RETURN_END_DEG          = 111;

    private static final double H_PARK_START_DEG               = 111;
    private static final double H_PARK_END_DEG                 = 90;

    // NEW: return from Row1 launch -> AlignIntake1Low (y=12) for second intake1-like cycle
    private static final double H_ROW1_TO_ALIGN1LOW_START_DEG   = 111;
    private static final double H_ROW1_TO_ALIGN1LOW_END_DEG     = 180;

    // =========================================================
    // =================== LIMELIGHT / TAGS ====================
    // =========================================================
    private static final int PIPE_OBELISK_1 = 2; // Tag 21 => GPP
    private static final int PIPE_OBELISK_2 = 3; // Tag 22 => PGP
    private static final int PIPE_OBELISK_3 = 4; // Tag 23 => PPG

    private static final int TID_GPP = 21;
    private static final int TID_PGP = 22;
    private static final int TID_PPG = 23;

    private static final long PIPE_SWAP_PERIOD_MS = 50;

    // =========================================================
    // =================== FLYWHEEL / SHOOT ====================
    // =========================================================
    private static final double SPINUP_MIN_WAIT_S = 0.60;
    private static final double SPINUP_TIMEOUT_S  = 2.00;
    private static final double RPM_TOL           = 60.0;

    private static final double HOOD_CLOSE_POS = 0.15;
    private static final double HOOD_LONG_POS  = 0.40;

    // =========================================================
    // ==================== DRIVE POWER ========================
    // =========================================================
    private static final double MAX_POWER_NORMAL  = 1.0;
    private static final double MAX_POWER_INTAKE2 = 0.75;

    private static final double INTAKE_SETTLE_S = 0.65;

    // =========================================================
    // ==================== DEVICES / STATE ====================
    // =========================================================
    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private Indexer indexer;
    private Flywheel flywheel;
    private DcMotor intakeMotor;

    private Follower follower;
    private Paths paths;

    // Tag detection
    private int activePipeline = PIPE_OBELISK_1;
    private int detectedTid = -1;
    private long lastPipeSwapNs = 0L;

    // Pre-advance sets
    private int preAdvanceRemainingPreloads = 0;
    private int preAdvanceRemainingRow1 = 0;
    private int preAdvanceRemainingRow2 = 0;

    // Intake control
    private boolean intakeActive = false;

    // settle / advance
    private double intakeSettleTimerS = 0.0;
    private int intakeAdvanceRemaining = 0;

    // Spin-up timing
    private double spinupElapsedS = 0.0;

    // Tag choice
    private int tidToUse = TID_PPG;
    private boolean usedFallbackPPG = true;

    // Launch trigger guards
    private boolean launchPreloadsStarted = false;
    private boolean launchRow2Started     = false;
    private boolean launchRow1Started     = false;
    private boolean launchRow1LowStarted  = false;

    private enum Phase {
        DRIVE_LAUNCH_PRELOADS,
        ARRIVED_SPINUP_PRELOADS,
        FIRE_THREE_PRELOADS,

        // ===== Main Intake 2 + Launch Row2 =====
        DRIVE_ALIGN_INTAKE2,
        DRIVE_INTAKE_GP,
        DRIVE_ALIGN2POSE2_AND_INDEX_AFTER_GP,
        DRIVE_ALIGN_INTAKE2_3,
        DRIVE_INTAKE_PP,
        DRIVE_LAUNCH_ROW2_AND_INDEX_AFTER_PP,
        ARRIVED_SPINUP_SECOND_ROW,
        FIRE_THREE_SECOND_ROW,

        // ===== Intake 1 + Launch Row1 (main) =====
        DRIVE_ALIGN_INTAKE1,
        DRIVE_INTAKE1_SWEEP_TO_G11,
        DRIVE_LAUNCH_ROW1_AND_PREADV_AFTER_G11,
        ARRIVED_SPINUP_FIRST_ROW,
        FIRE_THREE_FIRST_ROW,

        // ===== NEW: Intake 1-like @ y=12 + Launch Row1 again =====
        DRIVE_ALIGN_INTAKE1_LOW,
        DRIVE_INTAKE1_LOW_SWEEP_TO_G11,
        DRIVE_LAUNCH_ROW1_LOW_AND_PREADV_AFTER_G11,
        ARRIVED_SPINUP_FIRST_ROW_LOW,
        FIRE_THREE_FIRST_ROW_LOW,

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
        indexer.hardZero();

        flywheel = new Flywheel("flywheelRight", "flywheelLeft", "hoodServo");
        flywheel.init(hardwareMap);
        flywheel.setHoodPositions(HOOD_CLOSE_POS, HOOD_LONG_POS);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0.0);

        // ---- Pedro ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);

        setDrivePowerNormal();

        telemetry.addLine("Blue Far CYCLE: INIT – scanning AprilTag until START (last seen wins)");
        telemetry.addData("No Reverse", "Intake never reverses");
        telemetry.addData("Parallelization", "Keep driving while indexer advances");
        telemetry.addData("NEW", "After Row1 launch, do another Intake1-style cycle at y=12 then launch again");
        telemetry.update();

        // ===== INIT loop: scan until START =====
        lastPipeSwapNs = System.nanoTime();
        while (!isStarted() && !isStopRequested()) {
            llVision.poll();

            long nowNs = System.nanoTime();
            if ((nowNs - lastPipeSwapNs) / 1e6 > PIPE_SWAP_PERIOD_MS) {
                activePipeline = (activePipeline == PIPE_OBELISK_1)
                        ? PIPE_OBELISK_2
                        : (activePipeline == PIPE_OBELISK_2 ? PIPE_OBELISK_3 : PIPE_OBELISK_1);
                llVision.setPipeline(activePipeline);
                lastPipeSwapNs = nowNs;
            }

            if (llVision.hasTarget()) {
                int tid = llVision.getTid();
                if (tid == TID_GPP || tid == TID_PGP || tid == TID_PPG) {
                    detectedTid = tid;
                    tidToUse = tid;
                    usedFallbackPPG = false;
                    MotifStorage.motifTid = tidToUse;
                }
            }

            telemetry.addLine("INIT – scanning AprilTag until START (last seen wins)");
            telemetry.addData("ActivePipe", activePipeline);
            telemetry.addData("HasTarget", llVision.hasTarget());
            telemetry.addData("Cam TID", llVision.getTid());
            telemetry.addData("Last Seen Valid TID", detectedTid);
            telemetry.addData("TID To Use (frozen at START)", tidToUse);
            telemetry.addData("Fallback PPG?", (detectedTid == -1));
            telemetry.update();

            sleep(20);
        }

        // ===== RUN =====
        if (isStopRequested()) return;

        flywheel.enableHoodControl(true);

        long lastNs = System.nanoTime();

        if (detectedTid == -1) {
            tidToUse = TID_PPG;
            usedFallbackPPG = true;
        } else {
            usedFallbackPPG = false;
        }
        MotifStorage.motifTid = tidToUse;

        preAdvanceRemainingPreloads = computePreAdvancePreloads(tidToUse);

        // FAR -> LONG speed
        flywheel.setState(Flywheel.State.LONG);

        // start
        setDrivePowerNormal();
        follower.followPath(paths.LaunchPreloadsPose, true);
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
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()) {
                        indexer.startPreAdvanceOneSlot();
                        preAdvanceRemainingPreloads--;
                    }

                    if (!follower.isBusy()
                            && preAdvanceRemainingPreloads == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()) {
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
                        setDrivePowerIntake2();
                        follower.followPath(paths.AlignIntake2Pose, true);
                        phase = Phase.DRIVE_ALIGN_INTAKE2;
                    }
                    break;
                }

                // =========================================================
                // ===== Main Intake 2 + Launch Row2 ========================
                // =========================================================
                case DRIVE_ALIGN_INTAKE2: {
                    if (!follower.isBusy()) {
                        setDrivePowerIntake2();
                        intakeStart();
                        follower.followPath(paths.IntakeGPPose, true);
                        phase = Phase.DRIVE_INTAKE_GP;
                    }
                    break;
                }

                case DRIVE_INTAKE_GP: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        intakeAdvanceRemaining = 2;

                        setDrivePowerIntake2();
                        follower.followPath(paths.AlignIntake2Pose2, true);

                        phase = Phase.DRIVE_ALIGN2POSE2_AND_INDEX_AFTER_GP;
                    }
                    break;
                }

                case DRIVE_ALIGN2POSE2_AND_INDEX_AFTER_GP: {
                    intakeSettleTimerS += dt;

                    if (intakeSettleTimerS >= INTAKE_SETTLE_S) {
                        if (intakeAdvanceRemaining > 0
                                && !indexer.isAutoRunning()
                                && !indexer.isMoving()
                                && !indexer.isPreAdvancing()
                                && !indexer.isIntakeAdvancing()) {
                            indexer.startIntakeAdvanceOneSlot();
                            intakeAdvanceRemaining--;
                        }
                    }

                    if (!follower.isBusy()) {
                        setDrivePowerIntake2();
                        follower.followPath(paths.AlignIntake2Pose3, true);
                        phase = Phase.DRIVE_ALIGN_INTAKE2_3;
                    }
                    break;
                }

                case DRIVE_ALIGN_INTAKE2_3: {
                    if (!follower.isBusy()) {
                        setDrivePowerIntake2();
                        follower.followPath(paths.IntakePPPose, true);
                        phase = Phase.DRIVE_INTAKE_PP;
                    }
                    break;
                }

                case DRIVE_INTAKE_PP: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        intakeAdvanceRemaining = 1;

                        preAdvanceRemainingRow2 = computePreAdvanceRow2(tidToUse);

                        setDrivePowerNormal();
                        follower.followPath(paths.LaunchSecondRowPose, true);

                        phase = Phase.DRIVE_LAUNCH_ROW2_AND_INDEX_AFTER_PP;
                    }
                    break;
                }

                case DRIVE_LAUNCH_ROW2_AND_INDEX_AFTER_PP: {
                    intakeSettleTimerS += dt;

                    if (intakeSettleTimerS >= INTAKE_SETTLE_S) {
                        if (intakeAdvanceRemaining > 0
                                && !indexer.isAutoRunning()
                                && !indexer.isMoving()
                                && !indexer.isPreAdvancing()
                                && !indexer.isIntakeAdvancing()) {
                            indexer.startIntakeAdvanceOneSlot();
                            intakeAdvanceRemaining--;
                        }
                    }

                    if (preAdvanceRemainingRow2 > 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()) {
                        indexer.startPreAdvanceOneSlot();
                        preAdvanceRemainingRow2--;
                    }

                    if (!follower.isBusy()
                            && preAdvanceRemainingRow2 == 0
                            && intakeAdvanceRemaining == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()
                            && !indexer.isIntakeAdvancing()) {

                        intakeStop(); // stop before shooting (no reverse)

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
                        follower.followPath(paths.AlignIntake1Pose, true);
                        phase = Phase.DRIVE_ALIGN_INTAKE1;
                    }
                    break;
                }

                // =========================================================
                // ===== Intake 1 + Launch Row1 (main) ======================
                // =========================================================
                case DRIVE_ALIGN_INTAKE1: {
                    if (!follower.isBusy()) {
                        setDrivePowerNormal();
                        intakeStart();

                        follower.followPath(paths.Intake1SweepToG11Pose, true);
                        phase = Phase.DRIVE_INTAKE1_SWEEP_TO_G11;
                    }
                    break;
                }

                case DRIVE_INTAKE1_SWEEP_TO_G11: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        intakeAdvanceRemaining = 2; // pre-advance count after G11

                        preAdvanceRemainingRow1 = computePreAdvanceRow1(tidToUse);

                        setDrivePowerNormal();
                        follower.followPath(paths.LaunchFirstRowPose, true);

                        phase = Phase.DRIVE_LAUNCH_ROW1_AND_PREADV_AFTER_G11;
                    }
                    break;
                }

                case DRIVE_LAUNCH_ROW1_AND_PREADV_AFTER_G11: {
                    intakeSettleTimerS += dt;

                    if (intakeSettleTimerS >= INTAKE_SETTLE_S) {
                        if (intakeAdvanceRemaining > 0
                                && !indexer.isAutoRunning()
                                && !indexer.isMoving()
                                && !indexer.isPreAdvancing()
                                && !indexer.isIntakeAdvancing()) {
                            indexer.startPreAdvanceOneSlot();
                            intakeAdvanceRemaining--;
                        }
                    }

                    if (preAdvanceRemainingRow1 > 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()) {
                        indexer.startPreAdvanceOneSlot();
                        preAdvanceRemainingRow1--;
                    }

                    if (!follower.isBusy()
                            && intakeAdvanceRemaining == 0
                            && preAdvanceRemainingRow1 == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()
                            && !indexer.isIntakeAdvancing()) {

                        intakeStop();

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
                        // NEW: go do the y=12 intake1-like sequence
                        setDrivePowerNormal();
                        follower.followPath(paths.ReturnToAlignIntake1LowPose, true);
                        phase = Phase.DRIVE_ALIGN_INTAKE1_LOW;
                    }
                    break;
                }

                // =========================================================
                // ===== NEW Intake 1-like @ y=12 + Launch Row1 again =======
                // =========================================================
                case DRIVE_ALIGN_INTAKE1_LOW: {
                    if (!follower.isBusy()) {
                        setDrivePowerNormal();
                        intakeStart();

                        follower.followPath(paths.Intake1LowSweepToG11Pose, true);
                        phase = Phase.DRIVE_INTAKE1_LOW_SWEEP_TO_G11;
                    }
                    break;
                }

                case DRIVE_INTAKE1_LOW_SWEEP_TO_G11: {
                    if (!follower.isBusy()) {
                        intakeSettleTimerS = 0.0;
                        intakeAdvanceRemaining = 2; // pre-advance count after low G11

                        preAdvanceRemainingRow1 = computePreAdvanceRow1(tidToUse);

                        setDrivePowerNormal();
                        follower.followPath(paths.LaunchFirstRowFromLowPose, true);

                        phase = Phase.DRIVE_LAUNCH_ROW1_LOW_AND_PREADV_AFTER_G11;
                    }
                    break;
                }

                case DRIVE_LAUNCH_ROW1_LOW_AND_PREADV_AFTER_G11: {
                    intakeSettleTimerS += dt;

                    if (intakeSettleTimerS >= INTAKE_SETTLE_S) {
                        if (intakeAdvanceRemaining > 0
                                && !indexer.isAutoRunning()
                                && !indexer.isMoving()
                                && !indexer.isPreAdvancing()
                                && !indexer.isIntakeAdvancing()) {
                            indexer.startPreAdvanceOneSlot();
                            intakeAdvanceRemaining--;
                        }
                    }

                    if (preAdvanceRemainingRow1 > 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()) {
                        indexer.startPreAdvanceOneSlot();
                        preAdvanceRemainingRow1--;
                    }

                    if (!follower.isBusy()
                            && intakeAdvanceRemaining == 0
                            && preAdvanceRemainingRow1 == 0
                            && !indexer.isMoving()
                            && !indexer.isAutoRunning()
                            && !indexer.isPreAdvancing()
                            && !indexer.isIntakeAdvancing()) {

                        intakeStop();

                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_FIRST_ROW_LOW;
                    }
                    break;
                }

                case ARRIVED_SPINUP_FIRST_ROW_LOW: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_FIRST_ROW_LOW;
                    }
                    break;
                }

                case FIRE_THREE_FIRST_ROW_LOW: {
                    if (!launchRow1LowStarted) {
                        indexer.startAutoLaunchAllThree();
                        launchRow1LowStarted = true;
                    }

                    if (!indexer.isAutoRunning()) {
                        setDrivePowerNormal();
                        follower.followPath(paths.ParkPose, true);
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
            telemetry.addData("Using TID", tidToUse);
            telemetry.addData("FW Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("FW Right RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("FW Left RPM", "%.0f", flywheel.getMeasuredLeftRpm());
            telemetry.addData("Indexer Auto", indexer.isAutoRunning());
            telemetry.addData("Indexer Moving", indexer.isMoving());
            telemetry.addData("Indexer PreAdv", indexer.isPreAdvancing());
            telemetry.addData("Indexer IntakeAdv", indexer.isIntakeAdvancing());
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("AdvRemaining", intakeAdvanceRemaining);
            telemetry.addData("Pose", poseStr(follower.getPose()));
            telemetry.update();
        }

        // Safety
        intakeStop();
        indexer.setCamOpen(false);
        flywheel.stop();
        PoseStorage.lastPose = follower.getPose();
    }

    // ===== Drive power helpers =====
    private void setDrivePowerNormal()  { follower.setMaxPower(MAX_POWER_NORMAL); }
    private void setDrivePowerIntake2() { follower.setMaxPower(MAX_POWER_INTAKE2); }

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

    // =========================================================
    // ======================= PATHS ===========================
    // =========================================================
    public static class Paths {
        public PathChain LaunchPreloadsPose;

        // Intake2
        public PathChain AlignIntake2Pose;
        public PathChain IntakeGPPose;
        public PathChain AlignIntake2Pose2;
        public PathChain AlignIntake2Pose3;
        public PathChain IntakePPPose;
        public PathChain LaunchSecondRowPose;

        // Intake1 main
        public PathChain AlignIntake1Pose;
        public PathChain Intake1SweepToG11Pose;
        public PathChain LaunchFirstRowPose;

        // NEW: Intake1 low
        public PathChain ReturnToAlignIntake1LowPose;
        public PathChain Intake1LowSweepToG11Pose;
        public PathChain LaunchFirstRowFromLowPose;

        public PathChain ParkPose;

        public Paths(Follower follower) {

            LaunchPreloadsPose = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, LAUNCH_PRELOADS_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_START_TO_PRELOADS_START_DEG),
                            Math.toRadians(H_START_TO_PRELOADS_END_DEG))
                    .build();

            AlignIntake2Pose = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_PRELOADS_POSE, ALIGN_INTAKE2_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_PRELOADS_TO_ALIGN2_START_DEG),
                            Math.toRadians(H_PRELOADS_TO_ALIGN2_END_DEG))
                    .build();

            IntakeGPPose = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE2_POSE, INTAKE_GP_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            AlignIntake2Pose2 = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_GP_POSE, ALIGN_INTAKE2_POSE_2))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            AlignIntake2Pose3 = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE2_POSE_2, ALIGN_INTAKE2_POSE_3))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakePPPose = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE2_POSE_3, INTAKE_PP_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            LaunchSecondRowPose = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_PP_POSE, LAUNCH_SECOND_ROW_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_SECOND_RETURN_START_DEG),
                            Math.toRadians(H_SECOND_RETURN_END_DEG))
                    .build();

            AlignIntake1Pose = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_SECOND_ROW_POSE, ALIGN_INTAKE1_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_ROW2_TO_ALIGN1_START_DEG),
                            Math.toRadians(H_ROW2_TO_ALIGN1_END_DEG))
                    .build();

            Intake1SweepToG11Pose = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE1_POSE, INTAKE_G11_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            LaunchFirstRowPose = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_G11_POSE, LAUNCH_FIRST_ROW_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_ROW1_RETURN_START_DEG),
                            Math.toRadians(H_ROW1_RETURN_END_DEG))
                    .build();

            // NEW: from Row1 launch pose -> AlignIntake1Low (y=12)
            ReturnToAlignIntake1LowPose = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_FIRST_ROW_POSE, ALIGN_INTAKE1_LOW_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_ROW1_TO_ALIGN1LOW_START_DEG),
                            Math.toRadians(H_ROW1_TO_ALIGN1LOW_END_DEG))
                    .build();

            // NEW: low sweep
            Intake1LowSweepToG11Pose = follower.pathBuilder()
                    .addPath(new BezierLine(ALIGN_INTAKE1_LOW_POSE, INTAKE_G11_LOW_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // NEW: launch from low G11 back to same Row1 launch pose
            LaunchFirstRowFromLowPose = follower.pathBuilder()
                    .addPath(new BezierLine(INTAKE_G11_LOW_POSE, LAUNCH_FIRST_ROW_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_ROW1_RETURN_START_DEG),
                            Math.toRadians(H_ROW1_RETURN_END_DEG))
                    .build();

            ParkPose = follower.pathBuilder()
                    .addPath(new BezierLine(LAUNCH_FIRST_ROW_POSE, PARK_POSE))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(H_PARK_START_DEG),
                            Math.toRadians(H_PARK_END_DEG))
                    .build();
        }
    }
}