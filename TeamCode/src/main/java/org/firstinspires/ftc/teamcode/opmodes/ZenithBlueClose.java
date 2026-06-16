package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.Mark2AutoLaunchSettings;
import org.firstinspires.ftc.teamcode.control.Mark2LaunchSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

@Autonomous(name = "ZenithBlueClose", group = "Mark2")
public class ZenithBlueClose extends LinearOpMode {

    private static final double MAX_POWER_NORMAL = 0.85;
    private static final double GATE_INTAKE_MAX_POWER = 0.35;
    private static final double RPM_READY_FRACTION = 0.90;
    private static final double SPINUP_TIMEOUT_S = 2.00;
    private static final double LAUNCH_TIMEOUT_S = 6.00;
    private static final double POST_LAUNCH_HOLD_S = 0.0;
    private static final double GATE_INTAKE_SETTLE_S = 1.0;

    private static final double ROW_AIM_POSITION = 0.50;
    private static final double GATE_AIM_POSITION = 0.86;

    // ===== Path Tuning =====
    private static final double START_HEADING_DEG = 323.0;
    private static final Pose START_POSE = pose(19.450, 119.623, START_HEADING_DEG);

    private static final Pose PRELOAD_LAUNCH = point(55.393, 85.593);
    private static final double PRELOAD_LAUNCH_HEADING_DEG = 318.0;

    private static final Pose ROW2_ALIGN = point(47.118, 57.300);
    private static final double ROW2_ALIGN_HEADING_DEG = 180.0;
    private static final Pose ROW2_INTAKE = point(21.913, 57.0);
    private static final double ROW2_INTAKE_HEADING_DEG = 180.0;
    private static final Pose ROW2_LAUNCH = point(55.238, 85.504);
    private static final double ROW2_LAUNCH_HEADING_DEG = 220.0;

    private static final Pose GATE_ALIGN = point(25.856, 55.540);
    private static final double GATE_ALIGN_HEADING_DEG = 140.0;
    private static final Pose GATE_INTAKE = point(14.860, 55.628);
    private static final double GATE_INTAKE_HEADING_DEG = 150.0;
    private static final Pose GATE_CLEAR = point(22.000, 60.000);
    private static final double GATE_CLEAR_HEADING_DEG = 220.0;
    private static final Pose GATE1_LAUNCH = point(55.549, 85.135);

    private static final Pose GATE2_LAUNCH = point(55.333, 85.523);

    private static final Pose GATE3_LAUNCH = point(55.297, 85.502);

    private static final double GATE_LAUNCH_HEADING_DEG = 220.0;

    private static final Pose ROW1_INTAKE = point(26.797, 83.290);
    private static final double ROW1_INTAKE_HEADING_DEG = 180.0;
    private static final Pose ROW1_LAUNCH = point(55.5, 100.0);
    private static final double ROW1_LAUNCH_HEADING_DEG = 226.0;

    private Follower follower;
    private Paths paths;
    private Mark2Intake intake;
    private Mark2Launcher launcher;
    private Mark2LaunchSequence launchSequence;

    private String phase = "init";
    private double launchTargetRpm = 0.0;
    private double launchAimPosition = ROW_AIM_POSITION;
    private long lastNs;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_POWER_NORMAL);
        paths = new Paths(follower);

        intake = new Mark2Intake(hardwareMap, false);
        launcher = new Mark2Launcher(hardwareMap);
        launchSequence = new Mark2LaunchSequence(launcher, intake);

        launcher.setAimPosition(ROW_AIM_POSITION);
        launcher.resetFeeder();

        telemetry.addLine("ZenithBlueClose ready");
        telemetry.addData("Start", poseText(START_POSE));
        telemetry.addLine("Start pose will be set when Play is pressed");
        telemetry.addData("Init aim", "%.2f", ROW_AIM_POSITION);
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
        prepareLauncher(ROW_AIM_POSITION);

        followDriveOnly(paths.LaunchPreloads, "LaunchPreloads");
        launchAndHold(ROW_AIM_POSITION, "LaunchPreloads");
        prepareLauncher(GATE_AIM_POSITION);

        followDriveOnly(paths.AlignRow2, "AlignRow2");
        followWithBeamBreakIntake(paths.IntakeRow2, "IntakeRow2", 0.0);
        followDriveOnly(paths.LaunchRow2, "LaunchRow2");
        launchAndHold(GATE_AIM_POSITION, "LaunchRow2");

        prepareLauncher(GATE_AIM_POSITION);
        followDriveOnly(paths.Gate1Align, "Gate1Align");
        followWithBeamBreakIntake(
                paths.Gate1Intake, "Gate1Intake", GATE_INTAKE_SETTLE_S, GATE_INTAKE_MAX_POWER);
        followDriveOnly(paths.Gate1Clear, "Gate1Clear");
        followDriveOnly(paths.Gate1Launch, "Gate1Launch");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate1");

        prepareLauncher(GATE_AIM_POSITION);
        followDriveOnly(paths.Gate2Align, "Gate2Align");
        followWithBeamBreakIntake(
                paths.Gate2Intake, "Gate2Intake", GATE_INTAKE_SETTLE_S, GATE_INTAKE_MAX_POWER);
        followDriveOnly(paths.Gate2Clear, "Gate2Clear");
        followDriveOnly(paths.Gate2Launch, "Gate2Launch");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate2");

        prepareLauncher(GATE_AIM_POSITION);
        followDriveOnly(paths.Gate3Align, "Gate3Align");
        followWithBeamBreakIntake(
                paths.Gate3Intake, "Gate3Intake", GATE_INTAKE_SETTLE_S, GATE_INTAKE_MAX_POWER);
        followDriveOnly(paths.Gate3Clear, "Gate3Clear");
        followDriveOnly(paths.Gate3Launch, "Gate3Launch");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate3");

        prepareLauncher(
                GATE_AIM_POSITION,
                Mark2AutoLaunchSettings.AUTO_ROW1_RPM,
                Mark2AutoLaunchSettings.AUTO_ROW1_HOOD_POSITION);
        followWithBeamBreakIntake(paths.IntakeRow1, "IntakeRow1", 0.0);
        followDriveOnly(paths.LaunchRow1, "LaunchRow1");
        launchAndHold(
                GATE_AIM_POSITION,
                "LaunchRow1",
                Mark2AutoLaunchSettings.AUTO_ROW1_RPM,
                Mark2AutoLaunchSettings.AUTO_ROW1_HOOD_POSITION);

        phase = "done";
        postTelemetry();
        saveTeleOpPose();
        stopAll();
    }

    private void saveTeleOpPose() {
        Pose finalPose = follower.getPose();
        PoseStorage.lastPose = pose(finalPose.getX(), finalPose.getY(), ROW1_LAUNCH_HEADING_DEG);
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
        phase = "Intake " + label;
        intake.resetBeamBreakBallLatch();
        follower.setMaxPower(maxPower);
        follower.followPath(path, true);

        while (opModeIsActive() && follower.isBusy()) {
            double dt = nextDt();
            follower.update();
            updateLauncher(dt);
            intake.PickUpDifferential(dt);
            postTelemetry();
        }

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

        intake.Stop();
        follower.setMaxPower(MAX_POWER_NORMAL);
    }

    private void prepareLauncher(double aimPosition) {
        prepareLauncher(
                aimPosition,
                Mark2AutoLaunchSettings.AUTO_CLOSE_RPM,
                Mark2AutoLaunchSettings.AUTO_CLOSE_HOOD_POSITION);
    }

    private void prepareLauncher(double aimPosition, double targetRpm, double hoodPosition) {
        launchTargetRpm = targetRpm;
        launchAimPosition = aimPosition;
        launcher.setFlywheelTargetRpm(launchTargetRpm);
        launcher.setHoodPosition(hoodPosition);
        launcher.setAimPosition(launchAimPosition);
        launcher.resetFeeder();
    }

    private void launchAndHold(double aimPosition, String label) {
        launchAndHold(
                aimPosition,
                label,
                Mark2AutoLaunchSettings.AUTO_CLOSE_RPM,
                Mark2AutoLaunchSettings.AUTO_CLOSE_HOOD_POSITION);
    }

    private void launchAndHold(double aimPosition, String label, double targetRpm, double hoodPosition) {
        phase = "Launch " + label;
        prepareLauncher(aimPosition, targetRpm, hoodPosition);
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
        public PathChain AlignRow2;
        public PathChain IntakeRow2;
        public PathChain LaunchRow2;
        public PathChain Gate1Align;
        public PathChain Gate1Intake;
        public PathChain Gate1Clear;
        public PathChain Gate1Launch;
        public PathChain Gate2Align;
        public PathChain Gate2Intake;
        public PathChain Gate2Clear;
        public PathChain Gate2Launch;
        public PathChain Gate3Align;
        public PathChain Gate3Intake;
        public PathChain Gate3Clear;
        public PathChain Gate3Launch;
        public PathChain IntakeRow1;
        public PathChain LaunchRow1;

        public Paths(Follower follower) {
            LaunchPreloads = follower.pathBuilder().addPath(
                    new BezierLine(START_POSE, PRELOAD_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(START_HEADING_DEG),
                            headingRad(PRELOAD_LAUNCH_HEADING_DEG))
                    .build();

            AlignRow2 = follower.pathBuilder().addPath(
                    new BezierLine(PRELOAD_LAUNCH, ROW2_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(PRELOAD_LAUNCH_HEADING_DEG),
                            headingRad(ROW2_ALIGN_HEADING_DEG))
                    .build();

            IntakeRow2 = follower.pathBuilder().addPath(
                    new BezierLine(ROW2_ALIGN, ROW2_INTAKE)
            ).setTangentHeadingInterpolation()
                    .build();

            LaunchRow2 = follower.pathBuilder().addPath(
                    new BezierLine(ROW2_INTAKE, ROW2_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW2_INTAKE_HEADING_DEG),
                            headingRad(ROW2_LAUNCH_HEADING_DEG))
                    .build();

            Gate1Align = follower.pathBuilder().addPath(
                    new BezierLine(ROW2_LAUNCH, GATE_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW2_LAUNCH_HEADING_DEG),
                            headingRad(GATE_ALIGN_HEADING_DEG))
                    .build();

            Gate1Intake = follower.pathBuilder().addPath(
                    new BezierLine(GATE_ALIGN, GATE_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_ALIGN_HEADING_DEG),
                            headingRad(GATE_INTAKE_HEADING_DEG))
                    .build();

            Gate1Clear = follower.pathBuilder().addPath(
                    new BezierLine(GATE_INTAKE, GATE_CLEAR)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_HEADING_DEG),
                            headingRad(GATE_CLEAR_HEADING_DEG))
                    .build();

            Gate1Launch = follower.pathBuilder().addPath(
                    new BezierLine(GATE_CLEAR, GATE1_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_CLEAR_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            Gate2Align = follower.pathBuilder().addPath(
                    new BezierLine(GATE1_LAUNCH, GATE_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_LAUNCH_HEADING_DEG),
                            headingRad(GATE_ALIGN_HEADING_DEG))
                    .build();

            Gate2Intake = follower.pathBuilder().addPath(
                    new BezierLine(GATE_ALIGN, GATE_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_ALIGN_HEADING_DEG),
                            headingRad(GATE_INTAKE_HEADING_DEG))
                    .build();

            Gate2Clear = follower.pathBuilder().addPath(
                    new BezierLine(GATE_INTAKE, GATE_CLEAR)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_HEADING_DEG),
                            headingRad(GATE_CLEAR_HEADING_DEG))
                    .build();

            Gate2Launch = follower.pathBuilder().addPath(
                    new BezierLine(GATE_CLEAR, GATE2_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_CLEAR_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            Gate3Align = follower.pathBuilder().addPath(
                    new BezierLine(GATE2_LAUNCH, GATE_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_LAUNCH_HEADING_DEG),
                            headingRad(GATE_ALIGN_HEADING_DEG))
                    .build();

            Gate3Intake = follower.pathBuilder().addPath(
                    new BezierLine(GATE_ALIGN, GATE_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_ALIGN_HEADING_DEG),
                            headingRad(GATE_INTAKE_HEADING_DEG))
                    .build();

            Gate3Clear = follower.pathBuilder().addPath(
                    new BezierLine(GATE_INTAKE, GATE_CLEAR)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_HEADING_DEG),
                            headingRad(GATE_CLEAR_HEADING_DEG))
                    .build();

            Gate3Launch = follower.pathBuilder().addPath(
                    new BezierLine(GATE_CLEAR, GATE3_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_CLEAR_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            IntakeRow1 = follower.pathBuilder().addPath(
                    new BezierLine(GATE3_LAUNCH, ROW1_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_LAUNCH_HEADING_DEG),
                            headingRad(ROW1_INTAKE_HEADING_DEG))
                    .build();

            LaunchRow1 = follower.pathBuilder().addPath(
                    new BezierLine(ROW1_INTAKE, ROW1_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW1_INTAKE_HEADING_DEG),
                            headingRad(ROW1_LAUNCH_HEADING_DEG))
                    .build();
        }
    }
}
