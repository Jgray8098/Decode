package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.Indexer;
import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;

// --- PedroPathing imports ---
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedFarAuto", group = "Comp")
public class RedFarAuto extends LinearOpMode {

    // === Limelight pipelines ===
    private static final int PIPE_OBELISK_1 = 2; // Tag 21 = GPP
    private static final int PIPE_OBELISK_2 = 3; // Tag 22 = PGP
    private static final int PIPE_OBELISK_3 = 4; // Tag 23 = PPG

    // === AprilTag IDs ===
    private static final int TID_GPP = 21; // green-purple-purple
    private static final int TID_PGP = 22; // purple-green-purple
    private static final int TID_PPG = 23; // purple-purple-green

    // === Spin-up parameters ===
    private static final double SPINUP_MIN_WAIT_S = 0.75;
    private static final double SPINUP_TIMEOUT_S  = 2.50;
    private static final double RPM_TOL           = 75.0;

    // === Mirrored poses (inches, radians) ===
    private static final Pose START_POSE        = new Pose(82.04,  9.27, Math.toRadians(90.0));   // mirror of (61.96,9.27,90)
    private static final Pose TARGET_POSE       = new Pose(83.39, 22.94, Math.toRadians(60.0));   // mirror of (60.61,22.94,120)

    private static final Pose INTAKE_ALIGN_POSE = new Pose(117.94, 22.00, Math.toRadians(90.0));  // mirror of (26.06,22.00,90)
    private static final Pose INTAKE_POSE       = new Pose(117.94, 32.82, Math.toRadians(90.0));  // mirror of (26.06,32.82,90)
    private static final Pose PARK_POSE         = new Pose(117.94, 46.94, Math.toRadians(90.0));  // mirror of (26.06,46.94,90)

    // === Intake controls ===
    private static final double INTAKE_POWER_IN      = 1.0;
    private static final double INTAKE_EXTRA_HOLD_S  = 0.60; // after reaching INTAKE_POSE

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private Indexer indexer;
    private Flywheel flywheel;
    private DcMotor intakeMotor;

    // Pedro follower / paths
    private Follower follower;
    private PathChain pathToShot;
    private PathChain pathToAlign;
    private PathChain pathToIntake;
    private PathChain pathBackToShot;
    private PathChain pathToPark;

    // INIT cycling state
    private int activePipeline = PIPE_OBELISK_1;

    // Live readouts
    private boolean curHasTarget = false;
    private int curTid = -1;
    private String curPatternText = "NONE";

    // Last valid detection to use at START
    private int lastSeenTid = -1;
    private String lastSeenPatternText = "NONE";
    private long lastSeenNs = 0L;

    // Intake timers
    private boolean intakeActive = false;
    private double intakeHoldAfterPathS = 0.0;

    // Run-phase state machine
    private enum Phase {
        START_AND_DRIVE,          // drive to first shot; pre-advance while driving
        ARRIVED_SPINUP_WAIT_1,    // spin up @TARGET_POSE
        FIRE_THREE_1,             // first 3-ball volley
        DRIVE_ALIGN,              // to INTAKE_ALIGN_POSE
        INTAKE_MOVE,              // intake ON and drive to INTAKE_POSE (+ extra hold)
        DRIVE_BACK_TO_SHOT,       // drive back to TARGET_POSE
        ARRIVED_SPINUP_WAIT_2,    // spin up for second volley
        FIRE_THREE_2,             // second 3-ball volley
        DRIVE_PARK,               // drive to PARK_POSE
        DONE
    }
    private Phase phase = Phase.START_AND_DRIVE;

    @Override
    public void runOpMode() {
        // --- Devices ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);
        activePipeline = PIPE_OBELISK_1;
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

        // --- Pedro follower setup ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        pathToShot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, TARGET_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), TARGET_POSE.getHeading())
                .build();

        pathToAlign = follower.pathBuilder()
                .addPath(new BezierLine(TARGET_POSE, INTAKE_ALIGN_POSE))
                .setLinearHeadingInterpolation(TARGET_POSE.getHeading(), INTAKE_ALIGN_POSE.getHeading())
                .build();

        pathToIntake = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_ALIGN_POSE, INTAKE_POSE))
                .setLinearHeadingInterpolation(INTAKE_ALIGN_POSE.getHeading(), INTAKE_POSE.getHeading())
                .build();

        pathBackToShot = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE, TARGET_POSE))
                .setLinearHeadingInterpolation(INTAKE_POSE.getHeading(), TARGET_POSE.getHeading())
                .build();

        pathToPark = follower.pathBuilder()
                .addPath(new BezierLine(TARGET_POSE, PARK_POSE))
                .setLinearHeadingInterpolation(TARGET_POSE.getHeading(), PARK_POSE.getHeading())
                .build();

        telemetry.addLine("INIT: cycling pipelines 2/3/4; showing LIVE AprilTag + last valid.");
        telemetry.update();

        // ===== INIT LOOP (live preview) =====
        while (!isStarted() && !isStopRequested()) {
            llVision.poll();

            // Current reading
            curHasTarget = llVision.hasTarget();
            curTid = curHasTarget ? llVision.getTid() : -1;
            curPatternText = mapTidToPatternText(curTid);

            // Record last valid tag for use at Start
            if (curTid == TID_GPP || curTid == TID_PGP || curTid == TID_PPG) {
                lastSeenTid = curTid;
                lastSeenPatternText = curPatternText;
                lastSeenNs = System.nanoTime();
            }

            telemetry.addData("Active Pipeline", activePipeline);
            telemetry.addData("Current  hasTarget", curHasTarget);
            telemetry.addData("Current  TID", (curTid < 0) ? "NONE" : curTid);
            telemetry.addData("Current  Pattern", curPatternText);
            telemetry.addData("LastValid TID", (lastSeenTid < 0) ? "NONE" : lastSeenTid);
            telemetry.addData("LastValid Pattern", lastSeenPatternText);
            telemetry.addData("LastValid Age (s)", "%.2f", ageSeconds(lastSeenNs));
            telemetry.addData("Pose Start", poseStr(START_POSE));
            telemetry.addData("Pose Shot",  poseStr(TARGET_POSE));
            telemetry.addData("Pose Align", poseStr(INTAKE_ALIGN_POSE));
            telemetry.addData("Pose Intake",poseStr(INTAKE_POSE));
            telemetry.addData("Pose Park",  poseStr(PARK_POSE));
            telemetry.update();

            // Keep cycling so DS sees the randomization live
            activePipeline = cyclePipeline(activePipeline);
            llVision.setPipeline(activePipeline);

            sleep(50);
        }

        // ===== RUN (after Start) =====
        long lastNs = System.nanoTime();

        // Choose the tag to use (fallback to PPG if none)
        final int tidToUse =
                (lastSeenTid == TID_GPP || lastSeenTid == TID_PGP || lastSeenTid == TID_PPG)
                        ? lastSeenTid : TID_PPG;

        // Pre-advance steps based on tag
        final int preAdvanceTotal = computePreAdvanceFromTid(tidToUse);
        int preAdvanceRemaining = preAdvanceTotal; // non-blocking while we drive

        // Spin-up timing (after ARRIVAL to shot pose)
        double spinupElapsed = 0.0;

        // Launch trigger guards
        boolean launch1Started = false;
        boolean launch2Started = false;

        // Kick off flywheel + Pedro drive at once
        flywheel.setState(Flywheel.State.LONG);
        follower.followPath(pathToShot, true);

        while (opModeIsActive() && phase != Phase.DONE) {
            llVision.poll();

            double dt = (System.nanoTime() - lastNs) / 1e9;
            lastNs = System.nanoTime();

            // Always update mechanisms each loop
            flywheel.update(dt);
            indexer.update(dt);
            follower.update();

            switch (phase) {
                case START_AND_DRIVE: {
                    // Pre-advance indexer non-blocking while driving
                    if (preAdvanceRemaining > 0 && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemaining--;
                    }

                    // Wait for arrival at the shot pose
                    if (!follower.isBusy()) {
                        spinupElapsed = 0.0; // reset spin-up AFTER arrival
                        phase = Phase.ARRIVED_SPINUP_WAIT_1;
                    }
                    break;
                }

                case ARRIVED_SPINUP_WAIT_1: {
                    spinupElapsed += dt;

                    double target = flywheel.getTargetRpm();
                    double errR = Math.abs(flywheel.getMeasuredRightRpm() - target);
                    double errL = Math.abs(flywheel.getMeasuredLeftRpm() - target);
                    boolean rpmOk = target > 0 && errR < RPM_TOL && errL < RPM_TOL;

                    if ((spinupElapsed >= SPINUP_MIN_WAIT_S && rpmOk) ||
                            (spinupElapsed >= SPINUP_TIMEOUT_S)) {
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
                        follower.followPath(pathToAlign, true);
                        phase = Phase.DRIVE_ALIGN;
                    }
                    break;
                }

                case DRIVE_ALIGN: {
                    if (!follower.isBusy()) {
                        // Start intake and move to the intake position
                        intakeStart();
                        intakeHoldAfterPathS = 0.0;
                        follower.followPath(pathToIntake, true);
                        phase = Phase.INTAKE_MOVE;
                    }
                    break;
                }

                case INTAKE_MOVE: {
                    // Keep intake on while moving; once we arrive, hold a bit longer
                    if (!follower.isBusy()) {
                        intakeHoldAfterPathS += dt;
                        if (intakeHoldAfterPathS >= INTAKE_EXTRA_HOLD_S) {
                            intakeStop();
                            follower.followPath(pathBackToShot, true);
                            phase = Phase.DRIVE_BACK_TO_SHOT;
                        }
                    }
                    break;
                }

                case DRIVE_BACK_TO_SHOT: {
                    if (!follower.isBusy()) {
                        spinupElapsed = 0.0;
                        phase = Phase.ARRIVED_SPINUP_WAIT_2;
                    }
                    break;
                }

                case ARRIVED_SPINUP_WAIT_2: {
                    spinupElapsed += dt;

                    double target = flywheel.getTargetRpm();
                    double errR = Math.abs(flywheel.getMeasuredRightRpm() - target);
                    double errL = Math.abs(flywheel.getMeasuredLeftRpm() - target);
                    boolean rpmOk = target > 0 && errR < RPM_TOL && errL < RPM_TOL;

                    if ((spinupElapsed >= SPINUP_MIN_WAIT_S && rpmOk) ||
                            (spinupElapsed >= SPINUP_TIMEOUT_S)) {
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
                        follower.followPath(pathToPark, true);
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

            // Telemetry
            telemetry.addData("Phase", phase);
            telemetry.addData("Tag Used", tidToUse);
            telemetry.addData("Pre-Adv Total/Remain", "%d / %d", preAdvanceTotal, preAdvanceRemaining);
            telemetry.addData("FW Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("FW Right RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("FW Left RPM", "%.0f", flywheel.getMeasuredLeftRpm());
            telemetry.addData("Spinup Elapsed (s)", "%.2f", spinupElapsed);
            telemetry.addData("Indexer Auto", indexer.isAutoRunning());
            telemetry.addData("Indexer Moving", indexer.isMoving());
            telemetry.addData("Pedro Pose", poseStr(follower.getPose()));
            telemetry.addData("Pedro Busy", follower.isBusy());
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Intake Hold (s)", "%.2f", intakeHoldAfterPathS);
            telemetry.update();
        }

        // Safety
        intakeStop();
        indexer.setCamOpen(false);
        flywheel.stop();
    }

    // ===== Intake helpers =====
    private void intakeStart() {
        intakeActive = true;
        intakeMotor.setPower(INTAKE_POWER_IN);
    }

    private void intakeStop() {
        intakeActive = false;
        intakeMotor.setPower(0.0);
    }

    // ===== Vision helpers =====
    private String mapTidToPatternText(int tid) {
        if (tid == TID_GPP) return "GPP (green-purple-purple)";
        if (tid == TID_PGP) return "PGP (purple-green-purple)";
        if (tid == TID_PPG) return "PPG (purple-purple-green)";
        return "NONE";
    }

    /** Cycle 2 -> 3 -> 4 -> 2 ... */
    private int cyclePipeline(int current) {
        if (current == PIPE_OBELISK_1) return PIPE_OBELISK_2;
        else if (current == PIPE_OBELISK_2) return PIPE_OBELISK_3;
        else return PIPE_OBELISK_1;
    }

    /** Map tag to how many forward slots we must advance before firing. */
    private int computePreAdvanceFromTid(int tid) {
        // Preload order is P,P,G with P at firing position at start.
        // Tag 23 = PPG → already correct → advance 0
        // Tag 22 = PGP → advance 1
        // Tag 21 = GPP → advance 2
        if (tid == TID_PPG) return 0;
        if (tid == TID_PGP) return 1;
        if (tid == TID_GPP) return 2;
        return 0;
    }

    // ===== Misc helpers =====
    private static String poseStr(Pose p) {
        return String.format("(%.2f, %.2f, %.1f°)",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    private static double ageSeconds(long tNs) {
        if (tNs == 0L) return Double.POSITIVE_INFINITY;
        return (System.nanoTime() - tNs) / 1e9;
    }
}
