package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
    private static final double GATE1_INTAKE_MAX_POWER = 0.35;
    private static final double RPM_READY_FRACTION = 0.90;
    private static final double SPINUP_TIMEOUT_S = 2.00;
    private static final double LAUNCH_TIMEOUT_S = 6.00;
    private static final double POST_LAUNCH_HOLD_S = 0.0;
    private static final double GATE_INTAKE_SETTLE_S = 1.50;

    private static final double ROW_AIM_POSITION = 0.50;
    private static final double GATE_AIM_POSITION = 0.86;

    // ===== Path Tuning =====
    private static final double START_HEADING_DEG = 323.0;
    private static final Pose START_POSE = pose(19.450, 119.623, START_HEADING_DEG);

    private static final Pose PRELOAD_LAUNCH = point(54.393, 85.593);
    private static final double PRELOAD_LAUNCH_HEADING_DEG = 318.0;

    private static final Pose ROW2_INTAKE_CONTROL = point(77.684, 56.987);
    private static final Pose ROW2_INTAKE = point(21.913, 56.959);
    private static final double ROW2_INTAKE_HEADING_DEG = 200.0;
    private static final Pose ROW2_LAUNCH = point(55.500, 85.200);
    private static final double ROW2_LAUNCH_HEADING_DEG = 220.0;

    private static final Pose GATE1_ALIGN = point(25.856, 55.540);
    private static final double GATE1_ALIGN_HEADING_DEG = 140.0;
    private static final Pose GATE1_INTAKE = point(14.860, 55.628);
    private static final double GATE1_INTAKE_HEADING_DEG = 140.0;
    private static final Pose GATE1_LAUNCH = point(55.549, 85.135);

    private static final Pose GATE2_INTAKE_CONTROL = point(28.626, 52.448);
    private static final Pose GATE2_INTAKE = point(13.656, 59.517);
    private static final Pose GATE2_LAUNCH = point(55.333, 85.523);

    private static final Pose GATE3_INTAKE_CONTROL = point(28.565, 52.549);
    private static final Pose GATE3_INTAKE = point(13.693, 59.558);
    private static final Pose GATE3_LAUNCH = point(55.297, 85.502);

    private static final Pose GATE4_INTAKE_CONTROL = point(28.593, 52.601);
    private static final Pose GATE4_INTAKE = point(13.833, 59.559);
    private static final Pose GATE4_LAUNCH = point(55.436, 85.618);

    private static final double GATE_INTAKE_LAUNCH_HEADING_DEG = 140.0;
    private static final double GATE_LAUNCH_HEADING_DEG = 220.0;

    private static final Pose ROW1_INTAKE = point(23.797, 83.290);
    private static final double ROW1_INTAKE_HEADING_DEG = 180.0;
    private static final Pose ROW1_LAUNCH = point(56.682, 103.156);
    private static final double ROW1_LAUNCH_HEADING_DEG = 220.0;

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

        followWithBeamBreakIntake(paths.IntakeRow2, "IntakeRow2", 0.0);
        followDriveOnly(paths.LaunchRow2, "LaunchRow2");
        launchAndHold(GATE_AIM_POSITION, "LaunchRow2");

        prepareLauncher(GATE_AIM_POSITION);
        followDriveOnly(paths.Gate1Align, "Gate1Align");
        followWithBeamBreakIntake(
                paths.Gate1Intake, "Gate1Intake", GATE_INTAKE_SETTLE_S, GATE1_INTAKE_MAX_POWER);
        followDriveOnly(paths.Gate1Launch, "Gate1Launch");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate1");

        prepareLauncher(GATE_AIM_POSITION);
        followWithBeamBreakIntake(paths.IntakeGate2, "IntakeGate2", GATE_INTAKE_SETTLE_S);
        followDriveOnly(paths.LaunchGate2, "LaunchGate2");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate2");

        prepareLauncher(GATE_AIM_POSITION);
        followWithBeamBreakIntake(paths.IntakeGate3, "IntakeGate3", GATE_INTAKE_SETTLE_S);
        followDriveOnly(paths.LaunchGate3, "LaunchGate3");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate3");

        prepareLauncher(GATE_AIM_POSITION);
        followWithBeamBreakIntake(paths.IntakeGate4, "IntakeGate4", GATE_INTAKE_SETTLE_S);
        followDriveOnly(paths.LaunchGate4, "LaunchGate4");
        launchAndHold(GATE_AIM_POSITION, "LaunchGate4");

        prepareLauncher(GATE_AIM_POSITION);
        followWithBeamBreakIntake(paths.IntakeRow1, "IntakeRow1", 0.0);
        followDriveOnly(paths.LaunchRow1, "LaunchRow1");
        launchAndHold(GATE_AIM_POSITION, "LaunchRow1");

        phase = "done";
        postTelemetry();
        PoseStorage.lastPose = follower.getPose();
        stopAll();
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
        launchTargetRpm = Mark2AutoLaunchSettings.AUTO_CLOSE_RPM;
        launchAimPosition = aimPosition;
        launcher.setFlywheelTargetRpm(launchTargetRpm);
        launcher.setHoodPosition(Mark2AutoLaunchSettings.AUTO_CLOSE_HOOD_POSITION);
        launcher.setAimPosition(launchAimPosition);
        launcher.resetFeeder();
    }

    private void launchAndHold(double aimPosition, String label) {
        phase = "Launch " + label;
        prepareLauncher(aimPosition);
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
        public PathChain IntakeRow2;
        public PathChain LaunchRow2;
        public PathChain Gate1Align;
        public PathChain Gate1Intake;
        public PathChain Gate1Launch;
        public PathChain IntakeGate2;
        public PathChain LaunchGate2;
        public PathChain IntakeGate3;
        public PathChain LaunchGate3;
        public PathChain IntakeGate4;
        public PathChain LaunchGate4;
        public PathChain IntakeRow1;
        public PathChain LaunchRow1;

        public Paths(Follower follower) {
            LaunchPreloads = follower.pathBuilder().addPath(
                    new BezierLine(START_POSE, PRELOAD_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(START_HEADING_DEG),
                            headingRad(PRELOAD_LAUNCH_HEADING_DEG))
                    .build();

            IntakeRow2 = follower.pathBuilder().addPath(
                    new BezierCurve(PRELOAD_LAUNCH, ROW2_INTAKE_CONTROL, ROW2_INTAKE)
            ).setTangentHeadingInterpolation()
                    .build();

            LaunchRow2 = follower.pathBuilder().addPath(
                    new BezierLine(ROW2_INTAKE, ROW2_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW2_INTAKE_HEADING_DEG),
                            headingRad(ROW2_LAUNCH_HEADING_DEG))
                    .build();

            Gate1Align = follower.pathBuilder().addPath(
                    new BezierLine(ROW2_LAUNCH, GATE1_ALIGN)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW2_LAUNCH_HEADING_DEG),
                            headingRad(GATE1_ALIGN_HEADING_DEG))
                    .build();

            Gate1Intake = follower.pathBuilder().addPath(
                    new BezierLine(GATE1_ALIGN, GATE1_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE1_ALIGN_HEADING_DEG),
                            headingRad(GATE1_INTAKE_HEADING_DEG))
                    .build();

            Gate1Launch = follower.pathBuilder().addPath(
                    new BezierLine(GATE1_INTAKE, GATE1_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_LAUNCH_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            IntakeGate2 = follower.pathBuilder().addPath(
                    new BezierCurve(GATE1_LAUNCH, GATE2_INTAKE_CONTROL, GATE2_INTAKE)
            ).setTangentHeadingInterpolation()
                    .build();

            LaunchGate2 = follower.pathBuilder().addPath(
                    new BezierLine(GATE2_INTAKE, GATE2_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_LAUNCH_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            IntakeGate3 = follower.pathBuilder().addPath(
                    new BezierCurve(GATE2_LAUNCH, GATE3_INTAKE_CONTROL, GATE3_INTAKE)
            ).setTangentHeadingInterpolation()
                    .build();

            LaunchGate3 = follower.pathBuilder().addPath(
                    new BezierLine(GATE3_INTAKE, GATE3_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_LAUNCH_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            IntakeGate4 = follower.pathBuilder().addPath(
                    new BezierCurve(GATE3_LAUNCH, GATE4_INTAKE_CONTROL, GATE4_INTAKE)
            ).setTangentHeadingInterpolation()
                    .build();

            LaunchGate4 = follower.pathBuilder().addPath(
                    new BezierLine(GATE4_INTAKE, GATE4_LAUNCH)
            ).setLinearHeadingInterpolation(
                            headingRad(GATE_INTAKE_LAUNCH_HEADING_DEG),
                            headingRad(GATE_LAUNCH_HEADING_DEG))
                    .build();

            IntakeRow1 = follower.pathBuilder().addPath(
                    new BezierLine(GATE4_LAUNCH, ROW1_INTAKE)
            ).setLinearHeadingInterpolation(
                            headingRad(ROW1_INTAKE_HEADING_DEG),
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
