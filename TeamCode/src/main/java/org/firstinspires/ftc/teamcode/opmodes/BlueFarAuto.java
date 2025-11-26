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

//@Autonomous(name = "BlueFarAuto", group = "Comp")
public class BlueFarAuto extends LinearOpMode {

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

    // === Poses (inches, radians) ===
    private static final Pose START_POSE  = new Pose(61.96,  9.27, Math.toRadians(90.0));
    private static final Pose TARGET_POSE = new Pose(60.61, 22.94, Math.toRadians(120.0));

    // Second-shot heading offset (applies only to the second volley)
    private static final double SECOND_SHOT_HEADING_OFFSET_DEG = -7.0; // tweak on-field
    private static final Pose TARGET_POSE_2 = new Pose(
            TARGET_POSE.getX(),
            TARGET_POSE.getY(),
            TARGET_POSE.getHeading() + Math.toRadians(SECOND_SHOT_HEADING_OFFSET_DEG)
    );

    // Third-shot heading offset (applies only to the third volley)
    // Negative = rotate CW (aim more right); positive = CCW (aim more left).
    private static final double THIRD_SHOT_HEADING_OFFSET_DEG = -7.0; // tune ±1–2°
    private static final Pose TARGET_POSE_3 = new Pose(
            TARGET_POSE.getX(),
            TARGET_POSE.getY(),
            TARGET_POSE.getHeading() + Math.toRadians(THIRD_SHOT_HEADING_OFFSET_DEG)
    );

    // First intake cycle
    private static final Pose INTAKE_ALIGN_POSE_1 = new Pose(26.06, 22.00, Math.toRadians(90.0));
    private static final Pose INTAKE_POSE_1       = new Pose(26.06, 32.82, Math.toRadians(90.0));

    // Second intake cycle + final park
    private static final Pose INTAKE_ALIGN_POSE_2 = new Pose(26.06, 40.00, Math.toRadians(90.0));
    private static final Pose INTAKE_POSE_2       = new Pose(26.06, 60.00, Math.toRadians(90.0));
    private static final Pose FINAL_PARK_POSE     = new Pose(30, 65.00, Math.toRadians(90.0));

    // === Intake controls ===
    private static final double INTAKE_POWER_IN      = 1.0;
    private static final double INTAKE_EXTRA_HOLD_S  = 0.60; // short hold after arriving at intake poses

    // NEW: reverse “burp” to shed extras on the way back to launch
    private static final double INTAKE_REVERSE_POWER   = -0.6;
    private static final double INTAKE_REVERSE_PULSE_S = 0.35;

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
    private PathChain pathBackToShot_2nd;   // back to TARGET_POSE (XY), aim with TARGET_POSE_2 heading
    private PathChain pathToAlign2;
    private PathChain pathToIntake2;
    private PathChain pathBackToShot_3rd;   // back to TARGET_POSE (XY), aim with TARGET_POSE_3 heading
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

    // NEW: reverse pulse state
    private boolean reversePulseActive = false;
    private double reversePulseTimerS  = 0.0;

    // Run-phase state machine
    private enum Phase {
        START_AND_DRIVE,
        ARRIVED_SPINUP_WAIT_1,
        FIRE_THREE_1,
        DRIVE_ALIGN_1,
        INTAKE_MOVE_1,
        DRIVE_BACK_TO_SHOT_2ND,
        ARRIVED_SPINUP_WAIT_2,
        FIRE_THREE_2,
        DRIVE_ALIGN_2,
        INTAKE_MOVE_2,
        DRIVE_BACK_TO_SHOT_3RD,
        ARRIVED_SPINUP_WAIT_3,
        FIRE_THREE_3,
        DRIVE_FINAL_PARK,
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

        pathBackToShot_2nd = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE_1, TARGET_POSE))
                .setLinearHeadingInterpolation(INTAKE_POSE_1.getHeading(), TARGET_POSE_2.getHeading())
                .build();

        pathToAlign2 = follower.pathBuilder()
                .addPath(new BezierLine(TARGET_POSE, INTAKE_ALIGN_POSE_2))
                .setLinearHeadingInterpolation(TARGET_POSE.getHeading(), INTAKE_ALIGN_POSE_2.getHeading())
                .build();

        pathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_ALIGN_POSE_2, INTAKE_POSE_2))
                .setLinearHeadingInterpolation(INTAKE_ALIGN_POSE_2.getHeading(), INTAKE_POSE_2.getHeading())
                .build();

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

        final int tidToUse =
                (lastSeenTid == TID_GPP || lastSeenTid == TID_PGP || lastSeenTid == TID_PPG)
                        ? lastSeenTid : TID_PPG;

        final int preAdvanceTotal = computePreAdvanceFromTid(tidToUse);
        int preAdvanceRemaining = preAdvanceTotal;

        double spinupElapsed = 0.0;

        boolean launch1Started = false;
        boolean launch2Started = false;
        boolean launch3Started = false;

        flywheel.setState(Flywheel.State.LONG);
        follower.followPath(pathToShot, true);

        while (opModeIsActive() && phase != Phase.DONE) {
            llVision.poll();

            double dt = (System.nanoTime() - lastNs) / 1e9;
            lastNs = System.nanoTime();

            flywheel.update(dt);
            indexer.update(dt);
            follower.update();
            intakeReverseUpdate(dt); // <<< keep reverse-pulse timing updated

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
                            intakeStop(); // stop forward intake before reverse pulse
                            follower.followPath(pathBackToShot_2nd, true);
                            intakeReversePulseStart(); // <<< BURP while driving to shot 2
                            phase = Phase.DRIVE_BACK_TO_SHOT_2ND;
                        }
                    }
                    break;
                }

                case DRIVE_BACK_TO_SHOT_2ND: {
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
                            intakeStop(); // stop forward intake before reverse pulse
                            follower.followPath(pathBackToShot_3rd, true);
                            intakeReversePulseStart(); // <<< BURP while driving to shot 3
                            phase = Phase.DRIVE_BACK_TO_SHOT_3RD;
                        }
                    }
                    break;
                }

                case DRIVE_BACK_TO_SHOT_3RD: {
                    if (!follower.isBusy()) {
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
            telemetry.addData("Spinup Elapsed (s)", "%.2f", spinupElapsed);
            telemetry.addData("Indexer Auto", indexer.isAutoRunning());
            telemetry.addData("Indexer Moving", indexer.isMoving());
            telemetry.addData("Pedro Pose", poseStr(follower.getPose()));
            telemetry.addData("Pedro Busy", follower.isBusy());
            telemetry.addData("Second Shot Offset (deg)", "%.1f", SECOND_SHOT_HEADING_OFFSET_DEG);
            telemetry.addData("Third  Shot Offset (deg)", "%.1f", THIRD_SHOT_HEADING_OFFSET_DEG);
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Intake Hold (s)", "%.2f", intakeHoldAfterPathS);
            telemetry.addData("Reverse Pulse", reversePulseActive);
            telemetry.addData("Reverse Timer (s)", "%.2f", reversePulseTimerS);
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
        intakeMotor.setPower(0.0);
    }

    // Reverse “burp” controls
    private void intakeReversePulseStart() {
        reversePulseActive = true;
        reversePulseTimerS  = 0.0;
        // ensure forward intake is off
        intakeActive = false;
        intakeMotor.setPower(INTAKE_REVERSE_POWER);
    }

    private void intakeReverseUpdate(double dt) {
        if (!reversePulseActive) return;
        reversePulseTimerS += dt;
        if (reversePulseTimerS >= INTAKE_REVERSE_PULSE_S) {
            // stop the motor after pulse
            reversePulseActive = false;
            intakeMotor.setPower(0.0);
        }
    }

    private void intakeAllStop() {
        intakeActive = false;
        reversePulseActive = false;
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