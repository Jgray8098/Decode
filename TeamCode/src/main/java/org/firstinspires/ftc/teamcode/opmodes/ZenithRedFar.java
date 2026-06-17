package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Mark2LaunchSequence;
import org.firstinspires.ftc.teamcode.control.Mark2ManualLauncherController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

@Autonomous(name = "ZenithRedFar", group = "Mark2")
public class ZenithRedFar extends LinearOpMode {

    private static final double MAX_POWER_NORMAL = 0.80;
    private static final double CORNER_SLOW_MAX_POWER = 0.60;
    private static final double RPM_READY_FRACTION = 0.90;
    private static final double SPINUP_TIMEOUT_S = 2.00;
    private static final double LAUNCH_TIMEOUT_S = 6.00;
    private static final double POST_LAUNCH_HOLD_S = 0.0;
    private static final double CORNER_ALIGN_SETTLE_S = 0.30;
    private static final double CORNER_INTAKE_SETTLE_S = 1.0;
    private static final double OVERFLOW_INTAKE_SETTLE_S = 0.75;

    private static final double FAR_AIM_POSITION = 0.09;
    private static final double FAR_LAUNCH_RPM = Mark2ManualLauncherController.FAR_ZONE_RPM;
    private static final double FAR_HOOD_POSITION = Mark2ManualLauncherController.FAR_ZONE_HOOD_POSITION;

    // ===== Path Tuning =====
    // Red side mirrors ZenithBlueFar across the field centerline:
    // x = 144 - blueX, y = blueY, heading = 180 - blueHeading.
    private static final double START_HEADING_DEG = 0.0;
    private static final Pose START_POSE = pose(81.853, 7.772, START_HEADING_DEG);

    private static final Pose PRELOAD_LAUNCH = point(87.435, 17.277);
    private static final double PRELOAD_LAUNCH_HEADING_DEG = 0.0;

    private static final Pose ROW3_ALIGN = point(99.744, 35.316);
    private static final double ROW3_ALIGN_HEADING_DEG = 0.0;
    private static final Pose ROW3_INTAKE = point(126.353, 35.637);
    private static final double ROW3_INTAKE_HEADING_DEG = 0.0;
    private static final Pose ROW3_LAUNCH = point(91.395, 17.326);
    private static final double ROW3_LAUNCH_HEADING_DEG = 4.0;

    private static final Pose CORNER_ALIGN = point(126.050, 21.350);
    private static final double CORNER_ALIGN_HEADING_DEG = 325.0;
    private static final Pose CORNER_INTAKE = point(132.372, 12.419);
    private static final double CORNER_INTAKE_HEADING_DEG = 0.0;
    private static final Pose CORNER_LAUNCH = point(91.247, 17.107);
    private static final double CORNER_LAUNCH_HEADING_DEG = 0.0;

    private static final Pose OVERFLOW1_START = point(125.451, 11.633);
    private static final double OVERFLOW1_START_HEADING_DEG = 35.0;
    private static final Pose OVERFLOW1_INTAKE = point(131.256, 35.521);
    private static final double OVERFLOW1_INTAKE_HEADING_DEG = 80.0;
    private static final Pose OVERFLOW1_LAUNCH = point(91.372, 17.228);
    private static final double OVERFLOW1_LAUNCH_HEADING_DEG = 0.0;

    private static final Pose OVERFLOW2_START = point(125.614, 11.786);
    private static final double OVERFLOW2_START_HEADING_DEG = 35.0;
    private static final Pose OVERFLOW2_INTAKE = point(131.214, 35.549);
    private static final double OVERFLOW2_INTAKE_HEADING_DEG = 80.0;
    private static final Pose OVERFLOW2_LAUNCH = point(91.386, 17.460);
    private static final double OVERFLOW2_LAUNCH_HEADING_DEG = 0.0;

    private static final Pose PARK = point(99.544, 18.093);
    private static final double PARK_HEADING_DEG = 0.0;

    private Follower follower;
    private Paths paths;
    private Mark2Intake intake;
    private Mark2Launcher launcher;
    private Mark2LaunchSequence launchSequence;

    private String phase = "init";
    private double launchTargetRpm = 0.0;
    private double launchAimPosition = FAR_AIM_POSITION;
    private long lastNs;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_POWER_NORMAL);
        paths = new Paths(follower);

        intake = new Mark2Intake(hardwareMap, false);
        launcher = new Mark2Launcher(hardwareMap);
        launchSequence = new Mark2LaunchSequence(launcher, intake);

        launcher.setAimPosition(FAR_AIM_POSITION);
        launcher.resetFeeder();

        telemetry.addLine("ZenithRedFar ready");
        telemetry.addData("Start", poseText(START_POSE));
        telemetry.addLine("Start pose will be set when Play is pressed");
        telemetry.addData("Init aim", "%.2f", FAR_AIM_POSITION);
        telemetry.addData("Feed gate", "idle");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopAll();
            return;
        }

        lastNs = System.nanoTime();
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(MAX_POWER_NORMAL);
        intake.HoldPosition();
        prepareFarLauncher();

        followDriveOnly(paths.LaunchPreloads, "LaunchPreloads");
        launchAndHold("LaunchPreloads");

        prepareFarLauncher();
        followDriveOnly(paths.IntakeRow3Align, "IntakeRow3Align");
        followWithBeamBreakIntake(paths.IntakeRow3, "IntakeRow3", 0.0);
        followDriveOnly(paths.LaunchRow3, "LaunchRow3");
        launchAndHold("LaunchRow3");

        prepareFarLauncher();
        followCornerBeamBreakIntake();
        followDriveOnly(paths.LaunchCorner, "LaunchCorner");
        launchAndHold("LaunchCorner");

        prepareFarLauncher();
        followBeamBreakIntakePair(
                paths.IntakeOverflow10, "IntakeOverflow10",
                paths.IntakeOverflow11, "IntakeOverflow11");
        followDriveOnly(paths.LaunchOverflow1, "LaunchOverflow1");
        launchAndHold("LaunchOverflow1");

        prepareFarLauncher();
        followBeamBreakIntakePair(
                paths.IntakeOverflow20, "IntakeOverflow20",
                paths.IntakeOverflow21, "IntakeOverflow21");
        followDriveOnly(paths.LaunchOverflow2, "LaunchOverflow2");
        launchAndHold("LaunchOverflow2");

        followDriveOnly(paths.Park, "Park");

        phase = "done";
        postTelemetry();
        saveTeleOpPose();
        stopAll();
    }

    private void saveTeleOpPose() {
        Pose finalPose = follower.getPose();
        PoseStorage.lastPose = pose(finalPose.getX(), finalPose.getY(), PARK_HEADING_DEG);
    }

    private void followDriveOnly(PathChain path, String label) {
        phase = "Drive " + label;
        follower.setMaxPower(MAX_POWER_NORMAL);
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            double dt = nextDt();
            follower.update();
            updateLauncher(dt);
            postTelemetry();
        }
    }

    private void followWithBeamBreakIntake(PathChain path, String label, double settleSeconds) {
        followWithBeamBreakIntake(path, label, settleSeconds, MAX_POWER_NORMAL);
    }

    private void followWithBeamBreakIntake(
            PathChain path, String label, double settleSeconds, double maxPower) {
        runBeamBreakIntakePath(path, label, maxPower, true);
        settleWithBeamBreakIntake(label, settleSeconds);
        stopIntakePath();
    }

    private void followBeamBreakIntakePair(
            PathChain firstPath, String firstLabel,
            PathChain secondPath, String secondLabel) {
        runBeamBreakIntakePath(firstPath, firstLabel, MAX_POWER_NORMAL, true);
        runBeamBreakIntakePath(secondPath, secondLabel, MAX_POWER_NORMAL, false);
        settleWithBeamBreakIntake(secondLabel, OVERFLOW_INTAKE_SETTLE_S);
        stopIntakePath();
    }

    private void followCornerBeamBreakIntake() {
        runBeamBreakIntakePath(paths.IntakeCornerAlign, "IntakeCornerAlign", MAX_POWER_NORMAL, true);
        settleWithBeamBreakIntake("IntakeCornerAlign", CORNER_ALIGN_SETTLE_S);
        runBeamBreakIntakePath(paths.IntakeCornerSlow, "IntakeCornerSlow", CORNER_SLOW_MAX_POWER, false);
        settleWithBeamBreakIntake("IntakeCornerSlow", CORNER_INTAKE_SETTLE_S);
        stopIntakePath();
    }

    private void runBeamBreakIntakePath(PathChain path, String label, double maxPower, boolean resetLatch) {
        phase = "Intake " + label;
        if (resetLatch) {
            intake.resetBeamBreakBallLatch();
        }
        follower.setMaxPower(maxPower);
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            double dt = nextDt();
            follower.update();
            updateLauncher(dt);
            intake.PickUpDifferential(dt);
            postTelemetry();
        }
    }

    private void settleWithBeamBreakIntake(String label, double settleSeconds) {
        double settleElapsedS = 0.0;
        while (opModeIsActive() && settleElapsedS < settleSeconds) {
            double dt = nextDt();
            settleElapsedS += dt;
            follower.update();
            updateLauncher(dt);
            intake.PickUpDifferential(dt);
            phase = String.format("Settle %s %.1f/%.1fs", label, settleElapsedS, settleSeconds);
            postTelemetry();
        }
    }

    private void stopIntakePath() {
        intake.Stop();
        follower.setMaxPower(MAX_POWER_NORMAL);
    }

    private void prepareFarLauncher() {
        launchTargetRpm = FAR_LAUNCH_RPM;
        launchAimPosition = FAR_AIM_POSITION;
        launcher.setFlywheelTargetRpm(launchTargetRpm);
        launcher.setHoodPosition(FAR_HOOD_POSITION);
        launcher.setAimPosition(launchAimPosition);
        launcher.resetFeeder();
    }

    private void launchAndHold(String label) {
        phase = "Launch " + label;
        prepareFarLauncher();
        launchSequence.cancel();

        double elapsedS = 0.0;
        double spinupElapsedS = 0.0;
        double holdElapsedS = 0.0;
        boolean feedingStarted = false;

        while (opModeIsActive() && elapsedS < LAUNCH_TIMEOUT_S) {
            double dt = nextDt();
            elapsedS += dt;
            follower.update();
            updateLauncher(dt);

            if (!feedingStarted) {
                spinupElapsedS += dt;
                if (launcherReady() || spinupElapsedS >= SPINUP_TIMEOUT_S) {
                    feedingStarted = launchSequence.startIfFlywheelRunning(launchTargetRpm > 0.0);
                }
            } else if (launchSequence.isActive()) {
                launchSequence.update(dt);
            } else {
                holdElapsedS += dt;
                if (holdElapsedS >= POST_LAUNCH_HOLD_S) {
                    break;
                }
            }

            postTelemetry();
        }

        launchSequence.cancel();
    }

    private boolean launcherReady() {
        return launchTargetRpm > 0.0
                && launcher.getMeasuredRpm() >= launchTargetRpm * RPM_READY_FRACTION;
    }

    private void updateLauncher(double dtSec) {
        if (launchTargetRpm > 0.0) {
            launcher.setAimPosition(launchAimPosition);
            launcher.updateMeasuredRpm(dtSec);
        }
    }

    private double nextDt() {
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;
        return dt;
    }

    private void stopAll() {
        if (intake != null) {
            intake.Stop();
        }
        if (launchSequence != null) {
            launchSequence.cancel();
        }
        if (launcher != null) {
            launcher.stop();
        }
    }

    private void postTelemetry() {
        Pose pose = follower != null ? follower.getPose() : null;
        telemetry.addData("Phase", phase);
        telemetry.addData("Pose", pose != null ? poseText(pose) : "none");
        telemetry.addData("Launch seq", launchSequence != null ? launchSequence.getState() : "none");
        telemetry.addData("Target RPM", "%.0f", launchTargetRpm);
        telemetry.addData("Measured RPM", "%.0f", launcher != null ? launcher.getMeasuredRpm() : 0.0);
        telemetry.addData("Aim target", "%.2f", launchAimPosition);
        telemetry.addData("Aim actual", "%.2f", launcher != null ? launcher.getAimPosition() : 0.0);
        telemetry.addData("Beam raw", intake != null && intake.isBeamBreakDetected() ? "DETECTED" : "clear");
        telemetry.addData("Beam latch", intake != null && intake.isBeamBreakBallLatched() ? "BALL HELD" : "clear");
        telemetry.addData("Beam seat", intake != null && intake.isBeamBreakSeatDelayActive() ? "running" : "off");
        telemetry.update();
    }

    private static String poseText(Pose pose) {
        return String.format("(%.1f, %.1f, %.1f deg)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, headingRad(headingDeg));
    }

    private static Pose point(double x, double y) {
        return new Pose(x, y);
    }

    private static double headingRad(double headingDeg) {
        return Math.toRadians(headingDeg);
    }

    public static class Paths {
        public PathChain LaunchPreloads;
        public PathChain IntakeRow3Align;
        public PathChain IntakeRow3;
        public PathChain LaunchRow3;
        public PathChain IntakeCornerAlign;
        public PathChain IntakeCornerSlow;
        public PathChain LaunchCorner;
        public PathChain IntakeOverflow10;
        public PathChain IntakeOverflow11;
        public PathChain LaunchOverflow1;
        public PathChain IntakeOverflow20;
        public PathChain IntakeOverflow21;
        public PathChain LaunchOverflow2;
        public PathChain Park;

        public Paths(Follower follower) {
            LaunchPreloads = follower.pathBuilder().addPath(
                    new BezierLine(START_POSE, PRELOAD_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(START_HEADING_DEG),
                            headingRad(PRELOAD_LAUNCH_HEADING_DEG))
                    .build();

            IntakeRow3Align = follower.pathBuilder().addPath(
                    new BezierLine(PRELOAD_LAUNCH, ROW3_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(PRELOAD_LAUNCH_HEADING_DEG),
                            headingRad(ROW3_ALIGN_HEADING_DEG))
                    .build();

            IntakeRow3 = follower.pathBuilder().addPath(
                    new BezierLine(ROW3_ALIGN, ROW3_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW3_ALIGN_HEADING_DEG),
                            headingRad(ROW3_INTAKE_HEADING_DEG))
                    .build();

            LaunchRow3 = follower.pathBuilder().addPath(
                    new BezierLine(ROW3_INTAKE, ROW3_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW3_INTAKE_HEADING_DEG),
                            headingRad(ROW3_LAUNCH_HEADING_DEG))
                    .build();

            IntakeCornerAlign = follower.pathBuilder().addPath(
                    new BezierLine(ROW3_LAUNCH, CORNER_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW3_LAUNCH_HEADING_DEG),
                            headingRad(CORNER_ALIGN_HEADING_DEG))
                    .build();

            IntakeCornerSlow = follower.pathBuilder().addPath(
                    new BezierLine(CORNER_ALIGN, CORNER_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(CORNER_ALIGN_HEADING_DEG),
                            headingRad(CORNER_INTAKE_HEADING_DEG))
                    .build();

            LaunchCorner = follower.pathBuilder().addPath(
                    new BezierLine(CORNER_INTAKE, CORNER_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(CORNER_INTAKE_HEADING_DEG),
                            headingRad(CORNER_LAUNCH_HEADING_DEG))
                    .build();

            IntakeOverflow10 = follower.pathBuilder().addPath(
                    new BezierLine(CORNER_LAUNCH, OVERFLOW1_START)
            ).setLinearHeadingInterpolation(
                            headingRad(CORNER_LAUNCH_HEADING_DEG),
                            headingRad(OVERFLOW1_START_HEADING_DEG))
                    .build();

            IntakeOverflow11 = follower.pathBuilder().addPath(
                    new BezierLine(OVERFLOW1_START, OVERFLOW1_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(OVERFLOW1_START_HEADING_DEG),
                            headingRad(OVERFLOW1_INTAKE_HEADING_DEG))
                    .build();

            LaunchOverflow1 = follower.pathBuilder().addPath(
                    new BezierLine(OVERFLOW1_INTAKE, OVERFLOW1_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(OVERFLOW1_INTAKE_HEADING_DEG),
                            headingRad(OVERFLOW1_LAUNCH_HEADING_DEG))
                    .build();

            IntakeOverflow20 = follower.pathBuilder().addPath(
                    new BezierLine(OVERFLOW1_LAUNCH, OVERFLOW2_START)
            ).setLinearHeadingInterpolation(
                            headingRad(OVERFLOW1_LAUNCH_HEADING_DEG),
                            headingRad(OVERFLOW2_START_HEADING_DEG))
                    .build();

            IntakeOverflow21 = follower.pathBuilder().addPath(
                    new BezierLine(OVERFLOW2_START, OVERFLOW2_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(OVERFLOW2_START_HEADING_DEG),
                            headingRad(OVERFLOW2_INTAKE_HEADING_DEG))
                    .build();

            LaunchOverflow2 = follower.pathBuilder().addPath(
                    new BezierLine(OVERFLOW2_INTAKE, OVERFLOW2_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(OVERFLOW2_INTAKE_HEADING_DEG),
                            headingRad(OVERFLOW2_LAUNCH_HEADING_DEG))
                    .build();

            Park = follower.pathBuilder().addPath(
                    new BezierLine(OVERFLOW2_LAUNCH, PARK)
            ).setLinearHeadingInterpolation(
                            headingRad(OVERFLOW2_LAUNCH_HEADING_DEG),
                            headingRad(PARK_HEADING_DEG))
                    .build();
        }
    }
}
