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
    // Mirrored from Blue (Path2 ending heading was 150° -> mirror is 30°)
    private static final double TURN1_TARGET_DEG      = 30.0;
    private static final double TURN1_HEADING_TOL_DEG = 2.0;   // ±2°
    private static final double TURN1_MIN_SETTLE_S    = 0.30;  // 300 ms
    private static final double TURN1_TIMEOUT_S       = 1.00;  // safety fallback

    // ===== Intake (Path4 burst) =====
    private static final double INTAKE_POWER_IN = 1.0;
    private static final double INTAKE_BURST_S  = 3.00;

    // ===== Path2 safety (in-place heading change) =====
    private static final double PATH2_MAX_WAIT_S = 0.75; // timeout guard if follower stays busy

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
    private double intakeTimerS = 0.0;

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
        ARRIVED_SPINUP_WAIT_1,     // close-range spinup gate
        FIRE_THREE_1,              // 3-ball #1
        DRIVE_PATH3_ALIGN,         // to intake align
        INTAKE_PATH4_BURST,        // follow Path4 while intaking (timed)
        DRIVE_PATH5_TO_LAUNCH,     // back to launch
        ARRIVED_SPINUP_WAIT_2,     // spinup gate again
        FIRE_THREE_2,              // 3-ball #2
        DRIVE_PATH6_PARK,          // park
        DONE
    }
    private Phase phase = Phase.DRIVE_PATH1_SCAN;

    // launch triggers (prevent retriggering)
    private boolean launch1Started = false;
    private boolean launch2Started = false;

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
            intakeUpdate(dt);

            switch (phase) {
                case DRIVE_PATH1_SCAN: {
                    if (!follower.isBusy()) {
                        // arrive at scan position → start cycling pipelines
                        activePipeline = PIPE_OBELISK_1;
                        llVision.setPipeline(activePipeline);
                        lastPipeSwapNs = System.nanoTime();
                        phase = Phase.DETECT_TAG;
                    }
                    break;
                }

                case DETECT_TAG: {
                    // Cycle pipelines 2→3→4 every ~50ms until we see 21/22/23
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

                            follower.followPath(paths.Path2, true); // to launch (epsilon move + turn)
                            path2WaitS = 0.0;
                            phase = Phase.DRIVE_PATH2_TO_LAUNCH;
                        }
                    }
                    break;
                }

                case DRIVE_PATH2_TO_LAUNCH: {
                    // kick off next pre-advance if needed and we're free
                    if (preAdvanceRemaining > 0 && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemaining--;
                    }

                    path2WaitS += dt;

                    // Only proceed once Path2 is done AND pre-advances are completed
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
                        // start intake and follow Path4 (downward)
                        intakeStart();
                        follower.followPath(paths.Path4, true);
                        phase = Phase.INTAKE_PATH4_BURST;
                    }
                    break;
                }

                case INTAKE_PATH4_BURST: {
                    // Stop intake after burst time or once path completes
                    if (!follower.isBusy() || !intakeActive) {
                        intakeStop();
                        follower.followPath(paths.Path5, true);
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
                        follower.followPath(paths.Path6, true);
                        phase = Phase.DRIVE_PATH6_PARK;
                    }
                    break;
                }

                case DRIVE_PATH6_PARK: {
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
        intakeMotor.setPower(INTAKE_POWER_IN);
    }

    private void intakeUpdate(double dt) {
        if (!intakeActive) return;
        intakeTimerS += dt;
        if (intakeTimerS >= INTAKE_BURST_S) {
            intakeStop();
        }
    }

    private void intakeStop() {
        intakeActive = false;
        intakeTimerS = 0.0;
        intakeMotor.setPower(0.0);
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

    // ===== Mirrored paths & headings (mirror x across 72: x' = 144 - x; heading' = 180° - heading) =====
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            final double EPS = 0.01;

            // Blue Path1: (20.65,121.61, 54°) → (48,96, 65°)
            // Red Path1 : (123.35,121.61,126°) → (96,96,115°)
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123.35, 121.61, Math.toRadians(126)),
                            new Pose( 96.00,  96.00, Math.toRadians(115))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(115))
                    .build();

            // Blue Path2: (48,96, 65°) → (48,96, 150°) (epsilon)
            // Red Path2 : (96,96,115°) → (96+eps,96, 30°)
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.00,  96.00, Math.toRadians(115)),
                            new Pose( 96.00 + EPS, 96.00, Math.toRadians(30))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(30))
                    .build();

            // Blue Path3: (48,96,150°) → (26.64,109.25,270°)
            // Red Path3 : (96,96, 30°) → (117.36,109.25,270°)
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.00,  96.00, Math.toRadians(30)),
                            new Pose(117.36, 109.25, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                    .build();

            // Blue Path4: (26.64,109.25,270°) → (26.64,86.86,270°)
            // Red Path4 : (117.36,109.25,270°) → (117.36,86.86,270°)
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(117.36, 109.25, Math.toRadians(270)),
                            new Pose(117.36,  86.86, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Blue Path5: (26.64,86.86,270°) → (48,96,135°)
            // Red Path5 : (117.36,86.86,270°) → (96,96,45°)
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(117.36,  86.86, Math.toRadians(270)),
                            new Pose( 96.00,  96.00, Math.toRadians(45))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(45))
                    .build();

            // Blue Path6: (48,96,135°) → (28.96,73.35,260°)
            // Red Path6 : (96,96, 45°) → (115.04,73.35,280°)
            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose( 96.00,  96.00, Math.toRadians(45)),
                            new Pose(115.04,  73.35, Math.toRadians(280))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(280))
                    .build();
        }
    }
}
