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

//@Autonomous(name = "BlueCloseAuto", group = "Comp")
public class BlueCloseAuto extends LinearOpMode {

    // ===== Limelight pipelines & TIDs =====
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
    private static final double TURN1_TARGET_DEG      = 135.0; // Path2 ending heading
    private static final double TURN1_HEADING_TOL_DEG = 2.0;   // ±2°
    private static final double TURN1_MIN_SETTLE_S    = 0.30;  // 300 ms
    private static final double TURN1_TIMEOUT_S       = 1.00;  // safety fallback

    // ===== Intake controls =====
    private static final double INTAKE_POWER_IN        = 1.0;
    private static final double INTAKE_BURST_S         = 3.00; // first intake leg extra time
    private static final double INTAKE_EXTRA_HOLD_S    = 0.60; // extra hold after arriving at each intake pose

    // Reverse "burp" pulse (to avoid overfilling when returning to launch)
    private static final double REVERSE_POWER_OUT      = -1.0;
    private static final double REVERSE_PULSE_S        = 0.35; // duration of the reverse pulse itself

    // Delay + distance gates before reverse "burp" actually starts
    private static final double REVERSE_ARM_DELAY_S    = 0.35; // wait this long into the return path
    private static final double REVERSE_MIN_TRAVEL_IN  = 6.0;  // and after moving this many inches

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

    // Start pose (exact)
    private static final Pose START_POSE = new Pose(20.65, 121.61, Math.toRadians(54));

    // Tag detection (during RUN, after Path1)
    private int activePipeline = PIPE_OBELISK_1;
    private int detectedTid = -1;
    private long lastPipeSwapNs = 0L;

    // Indexer pre-advance
    private int preAdvanceTotal = 0;
    private int preAdvanceRemaining = 0;

    // Intake forward control
    private boolean intakeActive = false;
    private double intakeTimerS = 0.0;          // used by first intake burst
    private double intakeHoldAfterPathS = 0.0;  // used by both intake legs

    // Intake reverse pulse control
    private boolean reversePulseActive = false;
    private double  reversePulseTimerS = 0.0;

    // Reverse-pulse arming (delay + distance into return path)
    private boolean reverseArmed = false;
    private double  reverseArmTimerS = 0.0;
    private Pose    reverseArmStartPose = null;

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
        TURN_SETTLE_1,             // ensure heading settled at 135° before spin-up
        ARRIVED_SPINUP_WAIT_1,     // close-range spinup gate
        FIRE_THREE_1,              // 3-ball volley #1
        DRIVE_PATH3_ALIGN,         // to intake align (leg 1)
        INTAKE_PATH4_BURST,        // follow Path4 while intaking (timed)
        DRIVE_PATH5_TO_LAUNCH,     // back to launch (return #1, reverse burp armed)
        ARRIVED_SPINUP_WAIT_2,     // spinup gate
        FIRE_THREE_2,              // 3-ball volley #2

        // Second intake + third launch sequence:
        DRIVE_PATH7_ALIGN2,        // to second intake align (26.638, 75, 270)
        INTAKE_PATH8_BURST2,       // to second intake pose (26.638, 62, 270) + hold
        DRIVE_PATH9_TO_LAUNCH2,    // back to launch (return #2, reverse burp armed)
        ARRIVED_SPINUP_WAIT_3,     // spinup gate
        FIRE_THREE_3,              // 3-ball volley #3

        DRIVE_PATH10_PARK,         // final park at (21.619, 75.282, 90)
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
        indexer.hardZero();   // ensure Auto starts from encoder = 0

        flywheel = new Flywheel("flywheelRight", "flywheelLeft");
        flywheel.init(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0.0);

        // ---- Pedro ----
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower); // build AFTER setting starting pose

        telemetry.addLine("CloseRangeAuto: Ready. Will detect tags after Path1 during RUN.");
        telemetry.update();

        // ===== INIT loop (no tag detection—can’t see yet) =====
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
            intakeUpdate(dt);            // forward-intake timers
            intakeReversePulseUpdate(dt); // reverse-burp timer

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

                    // Only proceed once Path2 complete/timed-out AND all pre-advances are done
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
                        // Arm reverse burp; will trigger after delay + distance while driving Path5
                        reverseArmed = true;
                        reverseArmTimerS = 0.0;
                        reverseArmStartPose = new Pose(
                                follower.getPose().getX(),
                                follower.getPose().getY(),
                                follower.getPose().getHeading()
                        );
                        follower.followPath(paths.Path5, true);
                        phase = Phase.DRIVE_PATH5_TO_LAUNCH;
                    }
                    break;
                }

                case DRIVE_PATH5_TO_LAUNCH: {
                    // Trigger reverse burp once we're safely into the path
                    if (reverseArmed && !reversePulseActive) {
                        reverseArmTimerS += dt;
                        double traveled = distanceInches(follower.getPose(), reverseArmStartPose);
                        if (reverseArmTimerS >= REVERSE_ARM_DELAY_S && traveled >= REVERSE_MIN_TRAVEL_IN) {
                            intakeReversePulseStart();
                            reverseArmed = false;
                        }
                    }

                    if (!follower.isBusy()) {
                        reverseArmed = false; // disarm if unused
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
                        // === Second intake cycle ===
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
                        // small hold at intake to be sure balls are captured
                        intakeHoldAfterPathS += dt;
                        if (intakeHoldAfterPathS >= INTAKE_EXTRA_HOLD_S) {
                            intakeStop();
                            // Arm reverse burp for the second return path (Path9)
                            reverseArmed = true;
                            reverseArmTimerS = 0.0;
                            reverseArmStartPose = new Pose(
                                    follower.getPose().getX(),
                                    follower.getPose().getY(),
                                    follower.getPose().getHeading()
                            );
                            follower.followPath(paths.Path9, true); // back to launch pose
                            phase = Phase.DRIVE_PATH9_TO_LAUNCH2;
                        }
                    }
                    break;
                }

                case DRIVE_PATH9_TO_LAUNCH2: {
                    // Trigger reverse burp once we're safely into the path
                    if (reverseArmed && !reversePulseActive) {
                        reverseArmTimerS += dt;
                        double traveled = distanceInches(follower.getPose(), reverseArmStartPose);
                        if (reverseArmTimerS >= REVERSE_ARM_DELAY_S && traveled >= REVERSE_MIN_TRAVEL_IN) {
                            intakeReversePulseStart();
                            reverseArmed = false;
                        }
                    }

                    if (!follower.isBusy()) {
                        reverseArmed = false; // disarm if unused
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
            telemetry.addData("Intake Active (fwd)", intakeActive);
            telemetry.addData("Reverse Armed", reverseArmed);
            telemetry.addData("Reverse Active", reversePulseActive);
            telemetry.addData("Reverse Arm t (s)", "%.2f", reverseArmTimerS);
            telemetry.addData("Path2 Wait (s)", "%.2f", path2WaitS);
            telemetry.addData("Turn1 Elapsed/Stable (s)", "%.2f / %.2f", turn1ElapsedS, turn1StableS);
            telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Pose", poseStr(follower.getPose()));
            telemetry.update();
        }

        // Safety
        intakeAllStop();
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
        // Only the FIRST intake leg uses the timed burst; second leg uses hold-after-arrival only.
        if (phase == Phase.INTAKE_PATH4_BURST && intakeTimerS >= INTAKE_BURST_S) {
            intakeStop();
        }
    }

    private void intakeStop() {
        intakeActive = false;
        intakeTimerS = 0.0;
        intakeHoldAfterPathS = 0.0;
        if (!reversePulseActive) { // don't stomp reverse pulse if it's running
            intakeMotor.setPower(0.0);
        }
    }

    private void intakeAllStop() {
        intakeActive = false;
        reversePulseActive = false;
        reverseArmed = false;
        intakeTimerS = 0.0;
        intakeHoldAfterPathS = 0.0;
        intakeMotor.setPower(0.0);
    }

    // Reverse-pulse controls
    private void intakeReversePulseStart() {
        reversePulseActive = true;
        reversePulseTimerS = 0.0;
        intakeMotor.setPower(REVERSE_POWER_OUT);
    }

    private void intakeReversePulseUpdate(double dt) {
        if (!reversePulseActive) return;
        reversePulseTimerS += dt;
        if (reversePulseTimerS >= REVERSE_PULSE_S) {
            reversePulseActive = false;
            // Stop motor *only* if we aren't also intaking forward
            if (!intakeActive) {
                intakeMotor.setPower(0.0);
            } else {
                intakeMotor.setPower(INTAKE_POWER_IN);
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

    private static double distanceInches(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    /** Smallest signed angle error between two headings (degrees). */
    private static double angleErrorDeg(double currentDeg, double targetDeg) {
        double err = (targetDeg - currentDeg + 540.0) % 360.0 - 180.0;
        return err;
    }

    // ===== Paths (added Path7–Path10) =====
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {
            // Path1: (20.65,121.61, 54°) → (48,96, 65°)
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20.65, 121.61, Math.toRadians(54)),
                            new Pose(48.00,  96.00,  Math.toRadians(65))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(65))
                    .build();

            // Path2: (48,96, 65°) → (48,96, 135°)  (epsilon move so follower completes)
            final double EPS = 0.01; // 0.01 inch nudge
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.00, 96.00, Math.toRadians(65)),
                            new Pose(48.00 + EPS, 96.00, Math.toRadians(155)) // use 155° to ensure turn completion
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(155))
                    .build();

            // Path3: (48,96, 135°) → (26.64,109.25, 270°)
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.00,  96.00,  Math.toRadians(155)),
                            new Pose(26.64, 109.25,  Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(270))
                    .build();

            // Path4: (26.64,109.25, 270°) → (26.64,86.86, 270°)
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(26.64, 109.25, Math.toRadians(270)),
                            new Pose(26.64,  86.86, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path5: (26.64,86.86, 270°) → (48,96, 135°)
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(26.64,  86.86, Math.toRadians(270)),
                            new Pose(48.00,  96.00, Math.toRadians(135))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                    .build();

            // Path6 (old park step) is kept but unused now
            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.00,  96.00,  Math.toRadians(135)),
                            new Pose(28.96,  73.35,  Math.toRadians(260))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(260))
                    .build();

            // Path7: to second intake align (26.638, 75, 270°) – slight overshoot to 85 to be safe then down
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.00, 96.00, Math.toRadians(135)),
                            new Pose(26.638, 85.000, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                    .build();

            // Path8: to second intake pose (26.638, 62, 270°)
            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(26.638, 85.000, Math.toRadians(270)),
                            new Pose(26.638, 62.000, Math.toRadians(270))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path9: back to launch (48, 96, 135°)
            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(26.638, 62.000, Math.toRadians(270)),
                            new Pose(48.000, 96.000,  Math.toRadians(135))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                    .build();

            // Path10: final park (21.619, 75.282, 90°)
            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.000, 96.000,  Math.toRadians(135)),
                            new Pose(27, 75.282,  Math.toRadians(90))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();
        }
    }
}