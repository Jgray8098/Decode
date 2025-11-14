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

@Autonomous(name = "RedCloseAuto", group = "Comp")
public class RedCloseAuto extends LinearOpMode {

    // ===== Limelight pipelines & TIDs (same as blue) =====
    private static final int PIPE_OBELISK_1 = 2; // Tag 21 => GPP
    private static final int PIPE_OBELISK_2 = 3; // Tag 22 => PGP
    private static final int PIPE_OBELISK_3 = 4; // Tag 23 => PPG

    private static final int TID_GPP = 21;
    private static final int TID_PGP = 22;
    private static final int TID_PPG = 23;

    // ===== Flywheel (CLOSE) gates =====
    private static final double SPINUP_MIN_WAIT_S = 0.60;
    private static final double SPINUP_TIMEOUT_S  = 2.00;
    private static final double RPM_TOL           = 60.0;

    // ===== Turn settle gate (before first volley) =====
    // Mirrored from Blue (Blue ~155° end turn; Red target is 30°)
    private static final double TURN1_TARGET_DEG      = 30.0;
    private static final double TURN1_HEADING_TOL_DEG = 2.0;   // ±2°
    private static final double TURN1_MIN_SETTLE_S    = 0.30;  // 300 ms
    private static final double TURN1_TIMEOUT_S       = 1.00;  // safety fallback

    // ===== Intake controls =====
    private static final double INTAKE_POWER_IN       = 1.0;
    private static final double INTAKE_BURST_S        = 3.00; // first intake down-leg extra time
    private static final double INTAKE_EXTRA_HOLD_S   = 0.60; // hold after arriving at each intake pose

    // ===== Intake reverse "burp" controls =====
    private static final double INTAKE_POWER_REV      = -0.7;  // reverse power
    private static final double REVERSE_PULSE_S       = 0.25;  // how long to reverse
    private static final double REVERSE_DELAY_S       = 0.75;  // delay after starting drive-back before reverse starts

    // ===== Path2 safety (in-place heading change) =====
    private static final double PATH2_MAX_WAIT_S = 0.75;

    // ===== Devices =====
    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private Indexer indexer;
    private Flywheel flywheel;
    private DcMotor intakeMotor;

    // ===== Pedro =====
    private Follower follower;
    private Paths paths;

    // Start pose (mirrored from Blue: (20.65,121.61,54°) -> (123.35,121.61,126°))
    private static final Pose START_POSE = new Pose(123.35, 121.61, Math.toRadians(126));

    // Tag detection
    private int activePipeline = PIPE_OBELISK_1;
    private int detectedTid = -1;
    private long lastPipeSwapNs = 0L;

    // Indexer pre-advance
    private int preAdvanceTotal = 0;
    private int preAdvanceRemaining = 0;

    // Intake control
    private boolean intakeActive = false;
    private double intakeTimerS = 0.0;          // used by first intake burst
    private double intakeHoldAfterPathS = 0.0;  // used by both intake legs

    // Intake reverse state
    private boolean reversePulseActive = false;
    private boolean reversePulseArmed  = false;
    private double reversePulseTimerS  = 0.0;
    private double reverseDelayTimerS  = 0.0;

    // Spin-up timing
    private double spinupElapsedS = 0.0;

    // Path2 wait timer
    private double path2WaitS = 0.0;

    // Turn settle timers
    private double turn1ElapsedS = 0.0;
    private double turn1StableS  = 0.0;

    private enum Phase {
        DRIVE_PATH1_SCAN,          // to scanning pose
        DETECT_TAG,                // cycle 2/3/4 until 21/22/23
        DRIVE_PATH2_TO_LAUNCH,     // to launch pose; pre-advance while driving
        TURN_SETTLE_1,             // ensure heading settled at 30° before spin-up
        ARRIVED_SPINUP_WAIT_1,     // spinup gate
        FIRE_THREE_1,              // volley #1
        DRIVE_PATH3_ALIGN,         // to intake align (leg 1)
        INTAKE_PATH4_BURST,        // follow Path4 while intaking (timed)
        DRIVE_PATH5_TO_LAUNCH,     // back to launch (reverse burp #1)
        ARRIVED_SPINUP_WAIT_2,     // spinup gate
        FIRE_THREE_2,              // volley #2

        // NEW: second intake + third launch + park
        DRIVE_PATH7_ALIGN2,        // to second intake align
        INTAKE_PATH8_BURST2,       // to second intake pose + hold
        DRIVE_PATH9_TO_LAUNCH2,    // back to launch (reverse burp #2)
        ARRIVED_SPINUP_WAIT_3,     // spinup gate
        FIRE_THREE_3,              // volley #3
        DRIVE_PATH10_PARK,         // final park
        DONE
    }
    private Phase phase = Phase.DRIVE_PATH1_SCAN;

    // launch triggers (prevent retriggering)
    private boolean launch1Started = false;
    private boolean launch2Started = false;
    private boolean launch3Started = false;

    @Override
    public void runOpMode() {
        // ---- Hardware ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);
        llVision.setPipeline(activePipeline);
        llVision.start(100);

        indexer = new Indexer("Indexer", "camServo");
        indexer.init(hardwareMap);
        indexer.hardZero();   // start Auto from encoder = 0

        flywheel = new Flywheel("flywheelRight", "flywheelLeft");
        flywheel.init(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0.0);

        // ---- Pedro ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower); // build AFTER setting starting pose

        telemetry.addLine("RedCloseAuto: Ready. Will detect tags after Path1 during RUN.");
        telemetry.update();

        // ===== INIT loop =====
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Start Pose", poseStr(START_POSE));
            telemetry.update();
            sleep(30);
        }

        // ===== RUN =====
        long lastNs = System.nanoTime();

        // Start CLOSE flywheel and Path1 to scan pose
        flywheel.setState(Flywheel.State.CLOSE);
        follower.followPath(paths.Path1, true);

        while (opModeIsActive() && phase != Phase.DONE) {
            double dt = (System.nanoTime() - lastNs) / 1e9;
            lastNs = System.nanoTime();

            // Update subsystems
            llVision.poll();
            flywheel.update(dt);
            indexer.update(dt);
            follower.update();
            intakeUpdate(dt);          // forward intake burst logic (first cycle)
            maybeTriggerReverseBurp(dt); // reverse burp logic when armed

            switch (phase) {
                case DRIVE_PATH1_SCAN: {
                    if (!follower.isBusy()) {
                        activePipeline = PIPE_OBELISK_1;
                        llVision.setPipeline(activePipeline);
                        lastPipeSwapNs = System.nanoTime();
                        phase = Phase.DETECT_TAG;
                    }
                    break;
                }

                case DETECT_TAG: {
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
                            preAdvanceTotal = computePreAdvanceFromTid(detectedTid);
                            preAdvanceRemaining = preAdvanceTotal;

                            follower.followPath(paths.Path2, true);
                            path2WaitS = 0.0;
                            phase = Phase.DRIVE_PATH2_TO_LAUNCH;
                        }
                    }
                    break;
                }

                case DRIVE_PATH2_TO_LAUNCH: {
                    if (preAdvanceRemaining > 0 && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemaining--;
                    }

                    path2WaitS += dt;

                    boolean path2Done = (!follower.isBusy() || path2WaitS >= PATH2_MAX_WAIT_S);
                    if (path2Done && preAdvanceRemaining == 0 && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        turn1ElapsedS = 0.0;
                        turn1StableS  = 0.0;
                        phase = Phase.TURN_SETTLE_1;
                    }
                    break;
                }

                case TURN_SETTLE_1: {
                    double headingDeg = Math.toDegrees(follower.getPose().getHeading());
                    double err = angleErrorDeg(headingDeg, TURN1_TARGET_DEG);

                    turn1ElapsedS += dt;
                    if (Math.abs(err) <= TURN1_HEADING_TOL_DEG) {
                        turn1StableS += dt;
                    } else {
                        turn1StableS = 0.0;
                    }

                    if (turn1StableS >= TURN1_MIN_SETTLE_S || turn1ElapsedS >= TURN1_TIMEOUT_S) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_WAIT_1;
                    }
                    break;
                }

                case ARRIVED_SPINUP_WAIT_1: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_1;
                    }
                    break;
                }

                case FIRE_THREE_1: {
                    if (!launch1Started) {
                        indexer.startAutoLaunchAllThree();
                        launch1Started = true;
                    }
                    if (!indexer.isAutoRunning()) {
                        follower.followPath(paths.Path3, true);
                        phase = Phase.DRIVE_PATH3_ALIGN;
                    }
                    break;
                }

                case DRIVE_PATH3_ALIGN: {
                    if (!follower.isBusy()) {
                        intakeStart();
                        follower.followPath(paths.Path4, true);
                        phase = Phase.INTAKE_PATH4_BURST;
                    }
                    break;
                }

                case INTAKE_PATH4_BURST: {
                    if (!follower.isBusy() || !intakeActive) {
                        intakeStop();
                        follower.followPath(paths.Path5, true);
                        // Arm reverse burp for the first "back to launch" drive
                        armReverseBurp();
                        phase = Phase.DRIVE_PATH5_TO_LAUNCH;
                    }
                    break;
                }

                case DRIVE_PATH5_TO_LAUNCH: {
                    if (!follower.isBusy()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_WAIT_2;
                    }
                    break;
                }

                case ARRIVED_SPINUP_WAIT_2: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_2;
                    }
                    break;
                }

                case FIRE_THREE_2: {
                    if (!launch2Started) {
                        indexer.startAutoLaunchAllThree();
                        launch2Started = true;
                    }
                    if (!indexer.isAutoRunning()) {
                        // === NEW second intake cycle ===
                        follower.followPath(paths.Path7, true); // to Align2
                        phase = Phase.DRIVE_PATH7_ALIGN2;
                    }
                    break;
                }

                case DRIVE_PATH7_ALIGN2: {
                    if (!follower.isBusy()) {
                        intakeStart();
                        intakeHoldAfterPathS = 0.0;
                        follower.followPath(paths.Path8, true); // to Intake2
                        phase = Phase.INTAKE_PATH8_BURST2;
                    }
                    break;
                }

                case INTAKE_PATH8_BURST2: {
                    if (!follower.isBusy()) {
                        intakeHoldAfterPathS += dt;
                        if (intakeHoldAfterPathS >= INTAKE_EXTRA_HOLD_S) {
                            intakeStop();
                            follower.followPath(paths.Path9, true); // back to launch
                            // Arm reverse burp for second "back to launch" drive
                            armReverseBurp();
                            phase = Phase.DRIVE_PATH9_TO_LAUNCH2;
                        }
                    }
                    break;
                }

                case DRIVE_PATH9_TO_LAUNCH2: {
                    if (!follower.isBusy()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_WAIT_3;
                    }
                    break;
                }

                case ARRIVED_SPINUP_WAIT_3: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_THREE_3;
                    }
                    break;
                }

                case FIRE_THREE_3: {
                    if (!launch3Started) {
                        indexer.startAutoLaunchAllThree();
                        launch3Started = true;
                    }
                    if (!indexer.isAutoRunning()) {
                        follower.followPath(paths.Path10, true); // final park
                        phase = Phase.DRIVE_PATH10_PARK;
                    }
                    break;
                }

                case DRIVE_PATH10_PARK: {
                    if (!follower.isBusy()) {
                        phase = Phase.DONE;
                    }
                    break;
                }

                case DONE:
                default:
                    break;
            }

            // Telemetry
            telemetry.addData("Phase", phase);
            telemetry.addData("ActivePipe", activePipeline);
            telemetry.addData("HasTarget", llVision.hasTarget());
            telemetry.addData("TID", llVision.getTid());
            telemetry.addData("Detected TID", detectedTid);
            telemetry.addData("Pre-Adv total/rem", "%d / %d", preAdvanceTotal, preAdvanceRemaining);
            telemetry.addData("FW Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("FW Right RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("FW Left RPM", "%.0f", flywheel.getMeasuredLeftRpm());
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Intake Hold (s)", "%.2f", intakeHoldAfterPathS);
            telemetry.addData("Reverse Armed", reversePulseArmed);
            telemetry.addData("Reverse Active", reversePulseActive);
            telemetry.addData("Path2 Wait (s)", "%.2f", path2WaitS);
            telemetry.addData("Turn1 Elapsed/Stable (s)", "%.2f / %.2f", turn1ElapsedS, turn1StableS);
            telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Pose", poseStr(follower.getPose()));
            telemetry.update();
        }

        // Safety
        intakeStop();
        indexer.setCamOpen(false);
        flywheel.stop();
    }

    // ===== Helpers =====

    private boolean flywheelReady() {
        double target = flywheel.getTargetRpm();
        double errR = Math.abs(flywheel.getMeasuredRightRpm() - target);
        double errL = Math.abs(flywheel.getMeasuredLeftRpm() - target);
        return target > 0 && errR < RPM_TOL && errL < RPM_TOL && (spinupElapsedS >= SPINUP_MIN_WAIT_S);
    }

    private void intakeStart() {
        intakeActive = true;
        intakeTimerS = 0.0;
        // If a reverse pulse is currently active, let it control power.
        if (!reversePulseActive) {
            intakeMotor.setPower(INTAKE_POWER_IN);
        }
    }

    private void intakeUpdate(double dt) {
        if (!intakeActive) return;
        intakeTimerS += dt;
        // Only the FIRST intake leg uses the timed burst; second leg uses hold-after-arrival only.
        if (phase == Phase.INTAKE_PATH4_BURST && intakeTimerS >= INTAKE_BURST_S) {
            intakeStop();
        }
    }

    private void intakeStop() {
        intakeActive = false;
        intakeTimerS = 0.0;
        intakeHoldAfterPathS = 0.0;
        // If reverse pulse is not active, stop the motor.
        if (!reversePulseActive) {
            intakeMotor.setPower(0.0);
        }
    }

    // ----- Reverse-burp helpers -----

    private void armReverseBurp() {
        reversePulseArmed  = true;
        reversePulseActive = false;
        reversePulseTimerS = 0.0;
        reverseDelayTimerS = 0.0;
        // motor will be off until delay elapses and we start reverse
    }

    private void intakeReverseStartPulse() {
        reversePulseActive = true;
        reversePulseTimerS = 0.0;
        // Only drive reverse if forward intake isn't currently requested
        if (!intakeActive) {
            intakeMotor.setPower(INTAKE_POWER_REV);
        }
    }

    /** Called every loop; handles delay and pulse timing when armed. */
    private void maybeTriggerReverseBurp(double dt) {
        // If a reverse pulse is running, count it down
        if (reversePulseActive) {
            reversePulseTimerS += dt;
            if (reversePulseTimerS >= REVERSE_PULSE_S) {
                reversePulseActive = false;
                reversePulseTimerS = 0.0;
                // Return motor to 0 only if intake is not forward-running
                if (!intakeActive) {
                    intakeMotor.setPower(0.0);
                }
            }
            return;
        }

        // If armed but not yet started, run the delay
        if (reversePulseArmed) {
            reverseDelayTimerS += dt;
            if (reverseDelayTimerS >= REVERSE_DELAY_S) {
                reversePulseArmed  = false;
                reverseDelayTimerS = 0.0;
                intakeReverseStartPulse();
            }
        }
    }

    /** Map tag → pre-advance 0/1/2. */
    private int computePreAdvanceFromTid(int tid) {
        if (tid == TID_PPG) return 0;
        if (tid == TID_PGP) return 1;
        if (tid == TID_GPP) return 2;
        return 0;
    }

    private static String poseStr(Pose p) {
        return String.format("(%.2f, %.2f, %.1f°)",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    /** Smallest signed angle error between two headings (degrees). */
    private static double angleErrorDeg(double currentDeg, double targetDeg) {
        double err = (targetDeg - currentDeg + 540.0) % 360.0 - 180.0;
        return err;
    }

    // ===== Mirrored paths & headings + new Path7–Path10 =====
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {
            final double EPS = 0.01;

            // Path1 (mirror of Blue Path1): (123.35,121.61,126°) → (96,96,115°)
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123.35, 121.61, Math.toRadians(126)),
                            new Pose( 96.00,  96.00, Math.toRadians(115))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(115))
                    .build();

            // Path2 (mirror): (96,96,115°) → (96+eps,96, 30°)
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.00,  96.00, Math.toRadians(115)),
                            new Pose( 96.00 + EPS, 96.00, Math.toRadians(30))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(30))
                    .build();

            // Path3 (mirror): to intake align #1 (~116.36,109.25,270°)
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.00,  96.00, Math.toRadians(30)),
                            new Pose(116.36, 109.25, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                    .build();

            // Path4 (mirror): down to intake #1 (116.36,86.86,270°)
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(116.36, 109.25, Math.toRadians(270)),
                            new Pose(116.36,  86.86, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path5 (mirror): back to launch (96,96,45°)
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(116.36,  86.86, Math.toRadians(270)),
                            new Pose( 96.00,  96.00, Math.toRadians(45))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(45))
                    .build();

            // Path6 (kept for reference; not used now)
            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.00,  96.00, Math.toRadians(45)),
                            new Pose(115.04,  73.35, Math.toRadians(280))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(280))
                    .build();

            // ===== NEW MIRRORED SEQUENCE =====
            // Approx Align2 mirror
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.000, 96.000, Math.toRadians(45)),
                            new Pose(114.362, 80.000, Math.toRadians(265))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(265))
                    .build();

            // Intake2 down
            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(114.362, 75.000, Math.toRadians(265)),
                            new Pose(114.362, 62.000, Math.toRadians(265))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(265))
                    .build();

            // Back to launch (96,96,45°)
            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(114.362, 62.000, Math.toRadians(265)),
                            new Pose( 96.000, 96.000, Math.toRadians(45))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(45))
                    .build();

            // Final park
            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.000, 96.000, Math.toRadians(45)),
                            new Pose(115.381, 75.282, Math.toRadians(90))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }
    }
}