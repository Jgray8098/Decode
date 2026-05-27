package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue15Close", group = "Comp")
public class Blue15Close extends LinearOpMode {

    // ===== Flywheel gates =====
    private static final double SPINUP_MIN_WAIT_S = 0.60;
    private static final double SPINUP_TIMEOUT_S  = 2.00;
    private static final double RPM_TOL           = 60.0;
    private static final double PRELOAD_LAUNCH_SETTLE_S = 0.50;

    // ===== Drive power =====
    private static final double MAX_POWER_NORMAL = 1.0;

    // ===== Intake / return staging timing =====
    private static final double RETURN_FEED_TIME_S = 2.0;

    // ===== Intake settle timing =====
    private static final double GATE_INTAKE_SETTLE_S = 1.5;

    // ===== Hood positions =====
    private static final double HOOD_CLOSE_POS = 0.15;
    private static final double HOOD_LONG_POS  = 0.40;

    // ===== Editable field poses =====
    private static final Pose START_POSE = new Pose(19.222, 121.254, Math.toRadians(143));

    private static final Pose PRELOAD_LAUNCH_POSE = new Pose(48.141, 90.232, Math.toRadians(131));

    private static final Pose FIRST_ROW_ALIGN_POSE   = new Pose(42.384, 62.330, Math.toRadians(180));
    private static final Pose FIRST_ROW_INTAKE_POSE  = new Pose(15.486, 58.357, Math.toRadians(180));
    private static final Pose FIRST_ROW_LAUNCH_POSE  = new Pose(58.330, 85.141, Math.toRadians(135));

    private static final Pose GATE_INTAKE1_POSE = new Pose(14.205, 59.549, Math.toRadians(155));
    private static final Pose GATE_LAUNCH1_POSE = new Pose(58.357, 85.005, Math.toRadians(135));

    private static final Pose GATE_INTAKE2_POSE = new Pose(14.081, 59.589, Math.toRadians(155));
    private static final Pose GATE_LAUNCH2_POSE = new Pose(58.146, 85.065, Math.toRadians(135));

    private static final Pose SECOND_ROW_ALIGN_POSE  = new Pose(41.335, 85.886, Math.toRadians(180));
    private static final Pose SECOND_ROW_INTAKE_POSE = new Pose(19.957, 85.524, Math.toRadians(180));

    private static final Pose SECOND_ROW_LAUNCH_AND_PARK_POSE = new Pose(55.476, 99.092, Math.toRadians(146));

    // ===== Devices =====
    private Indexer indexer;
    private Flywheel flywheel;
    private DcMotor intakeMotor;

    // ===== Pedro =====
    private Follower follower;
    private Paths paths;

    // ===== Launch guards =====
    private boolean launchPreloadsStarted = false;
    private boolean launchCycle1Started = false;
    private boolean launchCycle2Started = false;
    private boolean launchCycle3Started = false;
    private boolean launchCycle4Started = false;

    // ===== Timing =====
    private double spinupElapsedS = 0.0;
    private double preloadSettleTimerS = 0.0;
    private double gateIntakeSettleTimerS = 0.0;

    // ===== Return feed / pre-advance control =====
    private boolean returnFeedActive = false;
    private double returnFeedTimerS = 0.0;
    private boolean returnPreAdvanceStarted = false;

    private enum Phase {
        DRIVE_PRELOAD_LAUNCH,
        ARRIVED_SPINUP_PRELOADS,
        FIRE_PRELOADS,

        DRIVE_FIRST_ROW_ALIGN,
        DRIVE_FIRST_ROW_INTAKE,
        DRIVE_FIRST_ROW_ALIGN_AND_LAUNCH,
        ARRIVED_SPINUP_CYCLE1,
        FIRE_CYCLE1,

        DRIVE_GATE_INTAKE1,
        WAIT_GATE_INTAKE1_SETTLE,
        DRIVE_GATE_INTAKE1_ALIGN_AND_LAUNCH,
        ARRIVED_SPINUP_CYCLE2,
        FIRE_CYCLE2,

        DRIVE_GATE_INTAKE2,
        WAIT_GATE_INTAKE2_SETTLE,
        DRIVE_GATE_INTAKE2_ALIGN_AND_LAUNCH,
        ARRIVED_SPINUP_CYCLE3,
        FIRE_CYCLE3,

        DRIVE_SECOND_ROW_ALIGN,
        DRIVE_SECOND_ROW_INTAKE,
        DRIVE_SECOND_ROW_ALIGN_LAUNCH_AND_PARK,
        ARRIVED_SPINUP_CYCLE4,
        FIRE_CYCLE4,

        DONE
    }

    private Phase phase = Phase.DRIVE_PRELOAD_LAUNCH;

    @Override
    public void runOpMode() {
        indexer = new Indexer("Indexer", "camServo");
        indexer.init(hardwareMap);
        indexer.hardZero();

        flywheel = new Flywheel("flywheelRight", "flywheelLeft", "hoodServo");
        flywheel.init(hardwareMap);
        flywheel.setHoodPositions(HOOD_CLOSE_POS, HOOD_LONG_POS);
        flywheel.enableHoodControl(true);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0.0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(MAX_POWER_NORMAL);
        paths = new Paths(follower);

        telemetry.addLine("Blue15Close ready");
        telemetry.addData("Start Pose", "(%.3f, %.3f, %.1f deg)",
                START_POSE.getX(), START_POSE.getY(), Math.toDegrees(START_POSE.getHeading()));
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Hood pos (init)", "%.2f", safeHoodPos());
            telemetry.update();
            sleep(30);
        }
        if (isStopRequested()) return;

        long lastNs = System.nanoTime();

        flywheel.setState(Flywheel.State.CLOSE);

        follower.followPath(paths.preloadLaunch, true);
        phase = Phase.DRIVE_PRELOAD_LAUNCH;

        while (opModeIsActive() && phase != Phase.DONE) {
            double dt = (System.nanoTime() - lastNs) / 1e9;
            lastNs = System.nanoTime();

            flywheel.update(dt);
            indexer.update(dt);
            follower.update();
            updateReturnFeed(dt);

            switch (phase) {
                case DRIVE_PRELOAD_LAUNCH: {
                    if (!follower.isBusy()) {
                        spinupElapsedS = 0.0;
                        preloadSettleTimerS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_PRELOADS;
                    }
                    break;
                }

                case ARRIVED_SPINUP_PRELOADS: {
                    spinupElapsedS += dt;
                    preloadSettleTimerS += dt;

                    boolean aimSettled = (preloadSettleTimerS >= PRELOAD_LAUNCH_SETTLE_S) && !follower.isBusy();
                    if ((flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) && aimSettled) {
                        phase = Phase.FIRE_PRELOADS;
                    }
                    break;
                }

                case FIRE_PRELOADS: {
                    if (!launchPreloadsStarted) {
                        indexer.startAutoLaunchAllThreeContinuous();
                        launchPreloadsStarted = true;
                    }

                    if (!indexer.isAutoRunning()) {
                        follower.followPath(paths.firstRowAlign, true);
                        phase = Phase.DRIVE_FIRST_ROW_ALIGN;
                    }
                    break;
                }

                case DRIVE_FIRST_ROW_ALIGN: {
                    if (!follower.isBusy()) {
                        intakeStart();
                        follower.followPath(paths.firstRowIntake, true);
                        phase = Phase.DRIVE_FIRST_ROW_INTAKE;
                    }
                    break;
                }

                case DRIVE_FIRST_ROW_INTAKE: {
                    if (!follower.isBusy()) {
                        intakeStop();
                        startReturnFeed();
                        follower.followPath(paths.firstRowAlignAndLaunch, true);
                        phase = Phase.DRIVE_FIRST_ROW_ALIGN_AND_LAUNCH;
                    }
                    break;
                }

                case DRIVE_FIRST_ROW_ALIGN_AND_LAUNCH: {
                    if (!follower.isBusy() && returnFeedDone()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_CYCLE1;
                    }
                    break;
                }

                case ARRIVED_SPINUP_CYCLE1: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_CYCLE1;
                    }
                    break;
                }

                case FIRE_CYCLE1: {
                    if (!launchCycle1Started) {
                        indexer.startAutoLaunchAllThreeContinuous();
                        launchCycle1Started = true;
                    }

                    if (!indexer.isAutoRunning()) {
                        intakeStart();
                        follower.followPath(paths.gateIntake1, true);
                        phase = Phase.DRIVE_GATE_INTAKE1;
                    }
                    break;
                }

                case DRIVE_GATE_INTAKE1: {
                    if (!follower.isBusy()) {
                        gateIntakeSettleTimerS = 0.0;
                        phase = Phase.WAIT_GATE_INTAKE1_SETTLE;
                    }
                    break;
                }

                case WAIT_GATE_INTAKE1_SETTLE: {
                    gateIntakeSettleTimerS += dt;

                    if (gateIntakeSettleTimerS >= GATE_INTAKE_SETTLE_S) {
                        intakeStop();
                        startReturnFeed();
                        follower.followPath(paths.gateIntake1AlignAndLaunch, true);
                        phase = Phase.DRIVE_GATE_INTAKE1_ALIGN_AND_LAUNCH;
                    }
                    break;
                }

                case DRIVE_GATE_INTAKE1_ALIGN_AND_LAUNCH: {
                    if (!follower.isBusy() && returnFeedDone()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_CYCLE2;
                    }
                    break;
                }

                case ARRIVED_SPINUP_CYCLE2: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_CYCLE2;
                    }
                    break;
                }

                case FIRE_CYCLE2: {
                    if (!launchCycle2Started) {
                        indexer.startAutoLaunchAllThreeContinuous();
                        launchCycle2Started = true;
                    }

                    if (!indexer.isAutoRunning()) {
                        intakeStart();
                        follower.followPath(paths.gateIntake2, true);
                        phase = Phase.DRIVE_GATE_INTAKE2;
                    }
                    break;
                }

                case DRIVE_GATE_INTAKE2: {
                    if (!follower.isBusy()) {
                        gateIntakeSettleTimerS = 0.0;
                        phase = Phase.WAIT_GATE_INTAKE2_SETTLE;
                    }
                    break;
                }

                case WAIT_GATE_INTAKE2_SETTLE: {
                    gateIntakeSettleTimerS += dt;

                    if (gateIntakeSettleTimerS >= GATE_INTAKE_SETTLE_S) {
                        intakeStop();
                        startReturnFeed();
                        follower.followPath(paths.gateIntake2AlignAndLaunch, true);
                        phase = Phase.DRIVE_GATE_INTAKE2_ALIGN_AND_LAUNCH;
                    }
                    break;
                }

                case DRIVE_GATE_INTAKE2_ALIGN_AND_LAUNCH: {
                    if (!follower.isBusy() && returnFeedDone()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_CYCLE3;
                    }
                    break;
                }

                case ARRIVED_SPINUP_CYCLE3: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_CYCLE3;
                    }
                    break;
                }

                case FIRE_CYCLE3: {
                    if (!launchCycle3Started) {
                        indexer.startAutoLaunchAllThreeContinuous();
                        launchCycle3Started = true;
                    }

                    if (!indexer.isAutoRunning()) {
                        follower.followPath(paths.secondRowAlign, true);
                        phase = Phase.DRIVE_SECOND_ROW_ALIGN;
                    }
                    break;
                }

                case DRIVE_SECOND_ROW_ALIGN: {
                    if (!follower.isBusy()) {
                        intakeStart();
                        follower.followPath(paths.secondRowIntake, true);
                        phase = Phase.DRIVE_SECOND_ROW_INTAKE;
                    }
                    break;
                }

                case DRIVE_SECOND_ROW_INTAKE: {
                    if (!follower.isBusy()) {
                        intakeStop();
                        startReturnFeed();
                        follower.followPath(paths.secondRowAlignLaunchAndPark, true);
                        phase = Phase.DRIVE_SECOND_ROW_ALIGN_LAUNCH_AND_PARK;
                    }
                    break;
                }

                case DRIVE_SECOND_ROW_ALIGN_LAUNCH_AND_PARK: {
                    if (!follower.isBusy() && returnFeedDone()) {
                        spinupElapsedS = 0.0;
                        phase = Phase.ARRIVED_SPINUP_CYCLE4;
                    }
                    break;
                }

                case ARRIVED_SPINUP_CYCLE4: {
                    spinupElapsedS += dt;
                    if (flywheelReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                        phase = Phase.FIRE_CYCLE4;
                    }
                    break;
                }

                case FIRE_CYCLE4: {
                    if (!launchCycle4Started) {
                        indexer.startAutoLaunchAllThreeContinuous();
                        launchCycle4Started = true;
                    }

                    if (!indexer.isAutoRunning()) {
                        phase = Phase.DONE;
                    }
                    break;
                }

                case DONE:
                default:
                    break;
            }

            telemetry.addData("Phase", phase);

            telemetry.addData("FW Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("FW Right RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("FW Left RPM", "%.0f", flywheel.getMeasuredLeftRpm());
            telemetry.addData("Hood pos", "%.2f", safeHoodPos());
            telemetry.addData("Spinup Elapsed", "%.2f", spinupElapsedS);

            telemetry.addData("Gate Settle Timer", "%.2f", gateIntakeSettleTimerS);

            telemetry.addData("ReturnFeedActive", returnFeedActive);
            telemetry.addData("ReturnFeedTimer", "%.2f", returnFeedTimerS);
            telemetry.addData("ReturnPreAdvanceStarted", returnPreAdvanceStarted);

            telemetry.addData("Indexer moving", indexer.isMoving());
            telemetry.addData("Indexer preAdv", indexer.isPreAdvancing());
            telemetry.addData("Indexer intakeAdv", indexer.isIntakeAdvancing());
            telemetry.addData("Indexer auto", indexer.isAutoRunning());

            telemetry.update();
        }

        intakeStop();
        indexer.setCamOpen(false);
        flywheel.stop();
        PoseStorage.lastPose = follower.getPose();
    }

    // ===== Helpers =====
    private boolean flywheelReady() {
        double target = flywheel.getTargetRpm();
        double errR = Math.abs(flywheel.getMeasuredRightRpm() - target);
        double errL = Math.abs(flywheel.getMeasuredLeftRpm() - target);
        return target > 0 && errR < RPM_TOL && errL < RPM_TOL && (spinupElapsedS >= SPINUP_MIN_WAIT_S);
    }

    private void intakeStart() {
        intakeMotor.setPower(1.0);
    }

    private void intakeStop() {
        intakeMotor.setPower(0.0);
    }

    private void startReturnFeed() {
        returnFeedActive = true;
        returnFeedTimerS = 0.0;
        returnPreAdvanceStarted = false;

        intakeMotor.setPower(1.0);

        if (!indexer.isAutoRunning()
                && !indexer.isMoving()
                && !indexer.isPreAdvancing()
                && !indexer.isIntakeAdvancing()) {

            indexer.startPreAdvanceSlots(2);
            returnPreAdvanceStarted = true;
        }
    }

    private void updateReturnFeed(double dt) {
        if (!returnFeedActive) return;

        returnFeedTimerS += dt;

        if (returnFeedTimerS < RETURN_FEED_TIME_S) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        if (!returnPreAdvanceStarted
                && !indexer.isAutoRunning()
                && !indexer.isMoving()
                && !indexer.isPreAdvancing()
                && !indexer.isIntakeAdvancing()) {

            indexer.startPreAdvanceSlots(2);
            returnPreAdvanceStarted = true;
        }

        if (returnFeedTimerS >= RETURN_FEED_TIME_S
                && returnPreAdvanceStarted
                && !indexer.isMoving()
                && !indexer.isAutoRunning()
                && !indexer.isPreAdvancing()
                && !indexer.isIntakeAdvancing()) {

            returnFeedActive = false;
            returnFeedTimerS = 0.0;
            intakeMotor.setPower(0.0);
        }
    }

    private boolean returnFeedDone() {
        return !returnFeedActive;
    }

    private double safeHoodPos() {
        try {
            return flywheel.getHoodPosition();
        } catch (Exception ignored) {
            return -1.0;
        }
    }

    // ===== Paths =====
    public static class Paths {
        public PathChain preloadLaunch;
        public PathChain firstRowAlign;
        public PathChain firstRowIntake;
        public PathChain firstRowAlignAndLaunch;
        public PathChain gateIntake1;
        public PathChain gateIntake1AlignAndLaunch;
        public PathChain gateIntake2;
        public PathChain gateIntake2AlignAndLaunch;
        public PathChain secondRowAlign;
        public PathChain secondRowIntake;
        public PathChain secondRowAlignLaunchAndPark;

        public Paths(Follower follower) {
            preloadLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, PRELOAD_LAUNCH_POSE))
                    .setLinearHeadingInterpolation(
                            START_POSE.getHeading(),
                            PRELOAD_LAUNCH_POSE.getHeading()
                    )
                    .build();

            firstRowAlign = follower.pathBuilder()
                    .addPath(new BezierLine(PRELOAD_LAUNCH_POSE, FIRST_ROW_ALIGN_POSE))
                    .setLinearHeadingInterpolation(
                            PRELOAD_LAUNCH_POSE.getHeading(),
                            FIRST_ROW_ALIGN_POSE.getHeading()
                    )
                    .build();

            firstRowIntake = follower.pathBuilder()
                    .addPath(new BezierLine(FIRST_ROW_ALIGN_POSE, FIRST_ROW_INTAKE_POSE))
                    .setLinearHeadingInterpolation(
                            FIRST_ROW_ALIGN_POSE.getHeading(),
                            FIRST_ROW_INTAKE_POSE.getHeading()
                    )
                    .build();

            firstRowAlignAndLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(FIRST_ROW_INTAKE_POSE, FIRST_ROW_LAUNCH_POSE))
                    .setLinearHeadingInterpolation(
                            FIRST_ROW_INTAKE_POSE.getHeading(),
                            FIRST_ROW_LAUNCH_POSE.getHeading()
                    )
                    .build();

            gateIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(FIRST_ROW_LAUNCH_POSE, GATE_INTAKE1_POSE))
                    .setLinearHeadingInterpolation(
                            FIRST_ROW_LAUNCH_POSE.getHeading(),
                            GATE_INTAKE1_POSE.getHeading()
                    )
                    .build();

            gateIntake1AlignAndLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(GATE_INTAKE1_POSE, GATE_LAUNCH1_POSE))
                    .setLinearHeadingInterpolation(
                            GATE_INTAKE1_POSE.getHeading(),
                            GATE_LAUNCH1_POSE.getHeading()
                    )
                    .build();

            gateIntake2 = follower.pathBuilder()
                    .addPath(new BezierLine(GATE_LAUNCH1_POSE, GATE_INTAKE2_POSE))
                    .setLinearHeadingInterpolation(
                            GATE_LAUNCH1_POSE.getHeading(),
                            GATE_INTAKE2_POSE.getHeading()
                    )
                    .build();

            gateIntake2AlignAndLaunch = follower.pathBuilder()
                    .addPath(new BezierLine(GATE_INTAKE2_POSE, GATE_LAUNCH2_POSE))
                    .setLinearHeadingInterpolation(
                            GATE_INTAKE2_POSE.getHeading(),
                            GATE_LAUNCH2_POSE.getHeading()
                    )
                    .build();

            secondRowAlign = follower.pathBuilder()
                    .addPath(new BezierLine(GATE_LAUNCH2_POSE, SECOND_ROW_ALIGN_POSE))
                    .setLinearHeadingInterpolation(
                            GATE_LAUNCH2_POSE.getHeading(),
                            SECOND_ROW_ALIGN_POSE.getHeading()
                    )
                    .build();

            secondRowIntake = follower.pathBuilder()
                    .addPath(new BezierLine(SECOND_ROW_ALIGN_POSE, SECOND_ROW_INTAKE_POSE))
                    .setLinearHeadingInterpolation(
                            SECOND_ROW_ALIGN_POSE.getHeading(),
                            SECOND_ROW_INTAKE_POSE.getHeading()
                    )
                    .build();

            secondRowAlignLaunchAndPark = follower.pathBuilder()
                    .addPath(new BezierLine(SECOND_ROW_INTAKE_POSE, SECOND_ROW_LAUNCH_AND_PARK_POSE))
                    .setLinearHeadingInterpolation(
                            SECOND_ROW_INTAKE_POSE.getHeading(),
                            SECOND_ROW_LAUNCH_AND_PARK_POSE.getHeading()
                    )
                    .build();
        }
    }
}