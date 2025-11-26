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

//@Autonomous(name = "RedFarAuto", group = "Comp")
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

    // === Poses (inches, radians) — RED is a mirror of BLUE across x=72 or field center ===
    private static final Pose START_POSE  = new Pose(82.04,  9.27, Math.toRadians(90.0));   // mirror of (61.96,9.27,90)
    private static final Pose TARGET_POSE = new Pose(83.39, 22.94, Math.toRadians(60.0));   // mirror of (60.61,22.94,120)

    // Per-volley heading offsets (mirrored sign from Blue: Blue used -7°, Red starts with +7°)
    private static final double SECOND_SHOT_HEADING_OFFSET_DEG = +0; // CCW is positive
    private static final Pose TARGET_POSE_2 = new Pose(
            TARGET_POSE.getX(),
            TARGET_POSE.getY(),
            TARGET_POSE.getHeading() + Math.toRadians(SECOND_SHOT_HEADING_OFFSET_DEG)
    );

    private static final double THIRD_SHOT_HEADING_OFFSET_DEG = +0; // start same, tune ±1–2°
    private static final Pose TARGET_POSE_3 = new Pose(
            TARGET_POSE.getX(),
            TARGET_POSE.getY(),
            TARGET_POSE.getHeading() + Math.toRadians(THIRD_SHOT_HEADING_OFFSET_DEG)
    );

    // First intake cycle (mirror of Blue)
    private static final Pose INTAKE_ALIGN_POSE_1 = new Pose(117.94, 22.00, Math.toRadians(90.0));
    private static final Pose INTAKE_POSE_1       = new Pose(117.94, 32.82, Math.toRadians(90.0));

    // Second intake cycle + final park (mirror of Blue)
    private static final Pose INTAKE_ALIGN_POSE_2 = new Pose(117.94, 40.00, Math.toRadians(90.0));
    private static final Pose INTAKE_POSE_2       = new Pose(117.94, 60.00, Math.toRadians(90.0));
    private static final Pose FINAL_PARK_POSE     = new Pose(117.94, 65.00, Math.toRadians(90.0));

    // === Intake controls ===
    private static final double INTAKE_POWER_IN      = 1.0;
    private static final double INTAKE_EXTRA_HOLD_S  = 0.60; // short hold after arriving at intake poses

    // Reverse “burp” to prevent overfill on return paths
    private static final double REVERSE_POWER_OUT     = -1.0;
    private static final double REVERSE_PULSE_S       = 0.35;
    private static final double REVERSE_ARM_DELAY_S   = 0.35; // time into the return path before burp can start
    private static final double REVERSE_MIN_TRAVEL_IN = 6.0;  // and after moving this many inches

    // === Devices ===
    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private Indexer indexer;
    private Flywheel flywheel;
    private DcMotor intakeMotor;

    // Pedro follower / paths
    private Follower follower;
    private PathChain pathToShot;
    private PathChain pathToAlign1;
    private PathChain pathToIntake1;
    private PathChain pathBackToShot_2nd;   // return w/ TARGET_POSE_2 heading
    private PathChain pathToAlign2;
    private PathChain pathToIntake2;
    private PathChain pathBackToShot_3rd;   // return w/ TARGET_POSE_3 heading
    private PathChain pathToFinalPark;

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

    // Reverse “burp” state
    private boolean reversePulseActive = false;
    private double  reversePulseTimerS = 0.0;
    private boolean reverseArmed       = false;
    private double  reverseArmTimerS   = 0.0;
    private Pose    reverseArmStartPose = null;

    // Run-phase state machine
    private enum Phase {
        START_AND_DRIVE,          // drive to first shot; pre-advance while driving
        ARRIVED_SPINUP_WAIT_1,    // spin up @TARGET_POSE
        FIRE_THREE_1,             // first 3-ball volley

        // Intake #1
        DRIVE_ALIGN_1,            // to INTAKE_ALIGN_POSE_1
        INTAKE_MOVE_1,            // intake ON to INTAKE_POSE_1 (+ extra hold)
        DRIVE_BACK_TO_SHOT_2ND,   // back to TARGET_POSE (XY) with TARGET_POSE_2 heading (reverse burp armed)
        ARRIVED_SPINUP_WAIT_2,    // spin up for second volley
        FIRE_THREE_2,             // second 3-ball volley

        // Intake #2
        DRIVE_ALIGN_2,            // to INTAKE_ALIGN_POSE_2
        INTAKE_MOVE_2,            // intake ON to INTAKE_POSE_2 (+ extra hold)
        DRIVE_BACK_TO_SHOT_3RD,   // back to TARGET_POSE (XY) with TARGET_POSE_3 heading (reverse burp armed)
        ARRIVED_SPINUP_WAIT_3,    // spin up for third volley
        FIRE_THREE_3,             // third 3-ball volley

        DRIVE_FINAL_PARK,         // to FINAL_PARK_POSE
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

        // Paths
        pathToShot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, TARGET_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), TARGET_POSE.getHeading())
                .build();

        pathToAlign1 = follower.pathBuilder()
                .addPath(new BezierLine(TARGET_POSE, INTAKE_ALIGN_POSE_1))
                .setLinearHeadingInterpolation(TARGET_POSE.getHeading(), INTAKE_ALIGN_POSE_1.getHeading())
                .build();

        pathToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_ALIGN_POSE_1, INTAKE_POSE_1))
                .setLinearHeadingInterpolation(INTAKE_ALIGN_POSE_1.getHeading(), INTAKE_POSE_1.getHeading())
                .build();

        // Return for 2nd volley with heading offset
        pathBackToShot_2nd = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE_1, TARGET_POSE))
                .setLinearHeadingInterpolation(INTAKE_POSE_1.getHeading(), TARGET_POSE_2.getHeading())
                .build();

        // Second intake cycle
        pathToAlign2 = follower.pathBuilder()
                .addPath(new BezierLine(TARGET_POSE, INTAKE_ALIGN_POSE_2))
                .setLinearHeadingInterpolation(TARGET_POSE.getHeading(), INTAKE_ALIGN_POSE_2.getHeading())
                .build();

        pathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_ALIGN_POSE_2, INTAKE_POSE_2))
                .setLinearHeadingInterpolation(INTAKE_ALIGN_POSE_2.getHeading(), INTAKE_POSE_2.getHeading())
                .build();

        // Return for 3rd volley with (possibly different) heading offset
        pathBackToShot_3rd = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE_2, TARGET_POSE))
                .setLinearHeadingInterpolation(INTAKE_POSE_2.getHeading(), TARGET_POSE_3.getHeading())
                .build();

        pathToFinalPark = follower.pathBuilder()
                .addPath(new BezierLine(TARGET_POSE, FINAL_PARK_POSE))
                .setLinearHeadingInterpolation(TARGET_POSE.getHeading(), FINAL_PARK_POSE.getHeading())
                .build();

        telemetry.addLine("INIT: cycling pipelines 2/3/4; showing LIVE AprilTag + last valid.");
        telemetry.update();

        // ===== INIT LOOP (live preview) =====
        while (!isStarted() && !isStopRequested()) {
            llVision.poll();

            curHasTarget = llVision.hasTarget();
            curTid = curHasTarget ? llVision.getTid() : -1;
            curPatternText = mapTidToPatternText(curTid);

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
            telemetry.addData("Pose Shot1", poseStr(TARGET_POSE));
            telemetry.addData("Pose Shot2 (offset)", poseStr(TARGET_POSE_2));
            telemetry.addData("Pose Shot3 (offset)", poseStr(TARGET_POSE_3));
            telemetry.addData("Pose Align1", poseStr(INTAKE_ALIGN_POSE_1));
            telemetry.addData("Pose Intake1", poseStr(INTAKE_POSE_1));
            telemetry.addData("Pose Align2", poseStr(INTAKE_ALIGN_POSE_2));
            telemetry.addData("Pose Intake2", poseStr(INTAKE_POSE_2));
            telemetry.addData("Pose Final Park", poseStr(FINAL_PARK_POSE));
            telemetry.update();

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

        // Spin-up timing
        double spinupElapsed = 0.0;

        // Launch trigger guards
        boolean launch1Started = false;
        boolean launch2Started = false;
        boolean launch3Started = false;

        // Start flywheel and head to first shot
        flywheel.setState(Flywheel.State.LONG);
        follower.followPath(pathToShot, true);

        while (opModeIsActive() && phase != Phase.DONE) {
            llVision.poll();

            double dt = (System.nanoTime() - lastNs) / 1e9;
            lastNs = System.nanoTime();

            flywheel.update(dt);
            indexer.update(dt);
            follower.update();
            //intakeReversePulseUpdate(dt); // manage reverse-burp timing

            switch (phase) {
                case START_AND_DRIVE: {
                    if (preAdvanceRemaining > 0 && !indexer.isMoving() && !indexer.isAutoRunning()) {
                        indexer.advanceOneSlot();
                        preAdvanceRemaining--;
                    }
                    if (!follower.isBusy()) {
                        spinupElapsed = 0.0;
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

                    if ((spinupElapsed >= SPINUP_MIN_WAIT_S && rpmOk) || (spinupElapsed >= SPINUP_TIMEOUT_S)) {
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
                        follower.followPath(pathToAlign1, true);
                        phase = Phase.DRIVE_ALIGN_1;
                    }
                    break;
                }

                case DRIVE_ALIGN_1: {
                    if (!follower.isBusy()) {
                        intakeStart();
                        intakeHoldAfterPathS = 0.0;
                        follower.followPath(pathToIntake1, true);
                        phase = Phase.INTAKE_MOVE_1;
                    }
                    break;
                }

                case INTAKE_MOVE_1: {
                    if (!follower.isBusy()) {
                        intakeHoldAfterPathS += dt;
                        if (intakeHoldAfterPathS >= INTAKE_EXTRA_HOLD_S) {
                            intakeStop();
                            // Arm reverse-burp for return #1 (to second volley)
                            armReverseBurp();
                            follower.followPath(pathBackToShot_2nd, true);
                            phase = Phase.DRIVE_BACK_TO_SHOT_2ND;
                        }
                    }
                    break;
                }

                case DRIVE_BACK_TO_SHOT_2ND: {
                    maybeTriggerReverseBurp(dt);
                    if (!follower.isBusy()) {
                        disarmReverseBurp();
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

                    if ((spinupElapsed >= SPINUP_MIN_WAIT_S && rpmOk) || (spinupElapsed >= SPINUP_TIMEOUT_S)) {
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
                        follower.followPath(pathToAlign2, true);
                        phase = Phase.DRIVE_ALIGN_2;
                    }
                    break;
                }

                case DRIVE_ALIGN_2: {
                    if (!follower.isBusy()) {
                        intakeStart();
                        intakeHoldAfterPathS = 0.0;
                        follower.followPath(pathToIntake2, true);
                        phase = Phase.INTAKE_MOVE_2;
                    }
                    break;
                }

                case INTAKE_MOVE_2: {
                    if (!follower.isBusy()) {
                        intakeHoldAfterPathS += dt;
                        if (intakeHoldAfterPathS >= INTAKE_EXTRA_HOLD_S) {
                            intakeStop();
                            // Arm reverse-burp for return #2 (to third volley)
                            armReverseBurp();
                            follower.followPath(pathBackToShot_3rd, true);
                            phase = Phase.DRIVE_BACK_TO_SHOT_3RD;
                        }
                    }
                    break;
                }

                case DRIVE_BACK_TO_SHOT_3RD: {
                    maybeTriggerReverseBurp(dt);
                    if (!follower.isBusy()) {
                        disarmReverseBurp();
                        spinupElapsed = 0.0;
                        phase = Phase.ARRIVED_SPINUP_WAIT_3;
                    }
                    break;
                }

                case ARRIVED_SPINUP_WAIT_3: {
                    spinupElapsed += dt;
                    double target = flywheel.getTargetRpm();
                    double errR = Math.abs(flywheel.getMeasuredRightRpm() - target);
                    double errL = Math.abs(flywheel.getMeasuredLeftRpm() - target);
                    boolean rpmOk = target > 0 && errR < RPM_TOL && errL < RPM_TOL;

                    if ((spinupElapsed >= SPINUP_MIN_WAIT_S && rpmOk) || (spinupElapsed >= SPINUP_TIMEOUT_S)) {
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
                        follower.followPath(pathToFinalPark, true);
                        phase = Phase.DRIVE_FINAL_PARK;
                    }
                    break;
                }

                case DRIVE_FINAL_PARK: {
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
            telemetry.addData("Pedro Pose", poseStr(follower.getPose()));
            telemetry.addData("Pedro Busy", follower.isBusy());

            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Hold After Path (s)", "%.2f", intakeHoldAfterPathS);
            telemetry.addData("Rev Armed", reverseArmed);
            telemetry.addData("Rev Active", reversePulseActive);
            telemetry.addData("Rev Arm t (s)", "%.2f", reverseArmTimerS);
            telemetry.addData("2nd Shot Offset (deg)", "%.1f", SECOND_SHOT_HEADING_OFFSET_DEG);
            telemetry.addData("3rd  Shot Offset (deg)", "%.1f", THIRD_SHOT_HEADING_OFFSET_DEG);
            telemetry.update();
        }

        // Safety
        intakeAllStop();
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
        // Only stop the motor if we are not pulsing reverse
        if (!reversePulseActive) {
            intakeMotor.setPower(0.0);
        }
        intakeHoldAfterPathS = 0.0;
    }

    private void intakeAllStop() {
        intakeActive = false;
        reversePulseActive = false;
        reverseArmed = false;
        intakeMotor.setPower(0.0);
        intakeHoldAfterPathS = 0.0;
        reverseArmTimerS = 0.0;
        reversePulseTimerS = 0.0;
    }

    // ===== Reverse “burp” helpers =====

    private void armReverseBurp() {
        reverseArmed = true;
        reverseArmTimerS = 0.0;
        reverseArmStartPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        );
    }

    private void disarmReverseBurp() {
        reverseArmed = false;
        reverseArmTimerS = 0.0;
    }

    private void maybeTriggerReverseBurp(double dt) {
        // Update arming window
        if (reverseArmed && !reversePulseActive) {
            reverseArmTimerS += dt;
            double traveled = distanceInches(follower.getPose(), reverseArmStartPose);
            if (reverseArmTimerS >= REVERSE_ARM_DELAY_S && traveled >= REVERSE_MIN_TRAVEL_IN) {
                // Fire reverse pulse
                reversePulseActive = true;
                reversePulseTimerS = 0.0;
                intakeMotor.setPower(REVERSE_POWER_OUT);
                reverseArmed = false; // one-shot
            }
        }

        // Update active reverse pulse
        if (reversePulseActive) {
            reversePulseTimerS += dt;
            if (reversePulseTimerS >= REVERSE_PULSE_S) {
                reversePulseActive = false;
                // stop motor (we’re not forward-intaking while returning)
                intakeMotor.setPower(0.0);
            }
        }
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

    private static double distanceInches(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    private static double ageSeconds(long tNs) {
        if (tNs == 0L) return Double.POSITIVE_INFINITY;
        return (System.nanoTime() - tNs) / 1e9;
    }
}