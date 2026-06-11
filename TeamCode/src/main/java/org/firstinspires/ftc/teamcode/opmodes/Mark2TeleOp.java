package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.Mark2AutoLauncherController;
import org.firstinspires.ftc.teamcode.control.Mark2ManualLauncherController;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;
import org.firstinspires.ftc.teamcode.utility.Mark2TargetLock;

import java.util.Locale;

/**
 * Mark2 primary TeleOp.
 *
 * Gamepad 1:
 *   Right stick Y / X - translate
 *   Left stick X      - rotate
 *   Left trigger      - snail mode
 *   X press           - toggle selected alliance
 *   Y press           - reset Pinpoint pose and field-centric reference to selected alliance start
 *   B press           - toggle robot-centric / field-centric driving
 *   Right bumper      - run launch-feed sequence, only if flywheel is on
 *
 * Gamepad 2:
 *   Dpad Up press     - toggle close launcher preset
 *   Dpad Down press   - toggle far launcher preset
 *   Y press/hold      - return feeder gate to idle and run intake forward
 *   X hold            - intake reverse, only when launch sequence is idle
 *   Left bumper       - toggle automatic RPM/hood from Pinpoint distance
 *   Right bumper      - toggle turret lock on alliance goal target
 *   Left stick X      - aim servo position, only when turret lock is off
 */
@TeleOp(name = "Mark2 TeleOp", group = "Mark2")
public class Mark2TeleOp extends OpMode {

    private Mark2Drivetrain drivetrain;
    private Mark2Intake intake;
    private Mark2Launcher launcher;
    private Mark2ManualLauncherController manualLauncher;
    private Mark2AutoLauncherController autoLauncher;
    private Mark2TargetLock targetLock;

    // Change Mark2TargetLock.DEFAULT_ALLIANCE before a match to choose the initial alliance.
    private static final Mark2TargetLock.Alliance INITIAL_ALLIANCE = Mark2TargetLock.DEFAULT_ALLIANCE;

    private static final double BLUE_START_X_IN = 14.1;
    private static final double BLUE_START_Y_IN = 76.46;
    private static final double BLUE_START_HEADING_DEG = 180.0;

    private static final double RED_START_X_IN = 127.8;
    private static final double RED_START_Y_IN = 76.46;
    private static final double RED_START_HEADING_DEG = 0.0;

    private long lastNs;

    private boolean prevX1 = false;
    private boolean prevY1 = false;
    private boolean prevB1 = false;

    private String lastPoseReset = "not reset";
    private Mark2TargetLock.Alliance selectedAlliance = INITIAL_ALLIANCE;
    private boolean prevRB2 = false;
    private boolean prevLB2 = false;
    private boolean targetLockEnabled = false;
    private boolean autoLauncherEnabled = false;

    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);
        intake = new Mark2Intake(hardwareMap);
        launcher = new Mark2Launcher(hardwareMap);
        manualLauncher = new Mark2ManualLauncherController(launcher, intake);
        autoLauncher = new Mark2AutoLauncherController(launcher);

        targetLock = new Mark2TargetLock();

        drivetrain.setFieldCentricEnabled(true);
        setFieldCentricReferenceToSelectedAllianceStart();

        launcher.resetFeeder();

        lastNs = System.nanoTime();

        telemetry.addLine("Mark2 TeleOp - Initialized");
        telemetry.addLine("GP1: Drive   GP2: Intake + Launcher");
        telemetry.addLine("GP1 X=Alliance  Y=Reset pose  B=Drive mode");
        telemetry.addLine("GP1 RB=Launch feed");
        telemetry.addLine("GP2 DUp=Close toggle DDn=Far toggle");
        telemetry.addLine("GP2 Y=Intake+GateIdle X=Reverse");
        telemetry.addLine("GP2 LB=Auto RPM/hood  RB=Turret lock");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        handleSelectedAllianceToggle();

        telemetry.addLine("Mark2 TeleOp - Init");
        telemetry.addData("Alliance", String.format(Locale.US, "%s (GP1 X to %s)",
                selectedAlliance.name(), oppositeAlliance(selectedAlliance).name()));
        telemetry.addLine("Field-centric starts enabled");
        telemetry.addLine("GP1 Y resets pose after start");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;

        handleSelectedAllianceToggle();

        boolean y1 = gamepad1.y;
        if (y1 && !prevY1) {
            resetPoseToSelectedAllianceStart();
        }
        prevY1 = y1;

        // GP1 B toggles robot-centric / field-centric driving without changing pose or heading reference.
        boolean b1 = gamepad1.b;
        if (b1 && !prevB1) {
            drivetrain.setFieldCentricEnabled(!drivetrain.isFieldCentricEnabled());
        }
        prevB1 = b1;

        drivetrain.driveSafe(gamepad1);

        boolean launchFeedPressed = gamepad1.right_bumper;

        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2) {
            setAutoLauncherEnabled(!autoLauncherEnabled);
        }
        prevLB2 = lb2;

        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2) {
            targetLockEnabled = !targetLockEnabled;
        }
        prevRB2 = rb2;

        boolean useAutoLauncher = autoLauncherEnabled && drivetrain.hasPinpoint();
        boolean manualAimEnabled = !targetLockEnabled;
        if (useAutoLauncher) {
            Pose2D pose = drivetrain.getPose();
            autoLauncher.updateForPose(pose, selectedAlliance, dt);
            manualLauncher.updateWithExternalLauncherSetpoint(
                    gamepad2, dt, autoLauncher.isAtSpeed(), manualAimEnabled, launchFeedPressed);
        } else {
            manualLauncher.update(gamepad2, dt, manualAimEnabled, launchFeedPressed);
        }

        if (targetLockEnabled && drivetrain.hasPinpoint()) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = drivetrain.getPose();
            targetLock.lockToGoal(launcher, pose, selectedAlliance);
        }

        showTelemetry();
    }

    @Override
    public void stop() {
        setAutoLauncherEnabled(false);
        if (manualLauncher != null) {
            manualLauncher.stop();
        }
        if (intake != null) {
            intake.Stop();
        }
    }

    private void showTelemetry() {
        boolean useAutoLauncher = autoLauncherEnabled && drivetrain.hasPinpoint();

        telemetry.addLine("-- Driver --------------------------");
        telemetry.addData("  Alliance", String.format(Locale.US, "%s (GP1 X to %s)",
                selectedAlliance.name(), oppositeAlliance(selectedAlliance).name()));
        telemetry.addData("  Pose reset", lastPoseReset);
        telemetry.addData("  Snail mode", gamepad1.left_trigger > 0 ? "ACTIVE (60%)" : "off");
        telemetry.addData("  Field-centric", drivetrain.hasPinpoint()
                ? (drivetrain.isFieldCentric() ? "FIELD (GP1 B=robot)" : "ROBOT (GP1 B=field)")
                : "unavailable (no Pinpoint)");

        if (drivetrain.hasPinpoint()) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = drivetrain.getPose();
            telemetry.addLine("-- Odometry ------------------------");
            telemetry.addData("  X (in)", String.format(Locale.US, "%.2f",
                    pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
            telemetry.addData("  Y (in)", String.format(Locale.US, "%.2f",
                    pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
            telemetry.addData("  Heading (°)", String.format(Locale.US, "%.1f",
                    pose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)));
        }

        telemetry.addLine("-- Launcher ------------------------");
        telemetry.addData("  Mode", launcherModeText(useAutoLauncher));
        telemetry.addData("  Flywheel", launcher.getTargetRpm() > 0.0 ? "ON" : "off");
        telemetry.addData("  Launch seq", manualLauncher.getLaunchSequenceStateName());
        if (useAutoLauncher) {
            telemetry.addData("  Auto distance", String.format(Locale.US, "%.1f in",
                    autoLauncher.getLastDistanceInches()));
            telemetry.addData("  Auto RPM", String.format(Locale.US, "%.0f",
                    autoLauncher.getTargetRpm()));
            telemetry.addData("  Auto hood", String.format(Locale.US, "%.3f",
                    autoLauncher.getTargetHoodPosition()));
        } else {
            telemetry.addData("  Selected zone", manualLauncher.getSelectedZoneName());
            telemetry.addData("  Zone RPM", String.format(Locale.US, "%.0f",
                    manualLauncher.getSelectedZoneRpm()));
            telemetry.addData("  Zone hood", String.format(Locale.US, "%.3f",
                    manualLauncher.getSelectedZoneHoodPosition()));
        }
        telemetry.addData("  Target RPM", String.format(Locale.US, "%.0f",
                launcher.getTargetRpm()));
        telemetry.addData("  Measured RPM", String.format(Locale.US, "%.0f",
                launcher.getMeasuredRpm()));
        telemetry.addData("  Motor RPM", String.format(Locale.US, "%.0f / %.0f",
                launcher.getMeasuredRpmOne(), launcher.getMeasuredRpmTwo()));
        telemetry.addData("  Motor power", String.format(Locale.US, "%.2f / %.2f",
                launcher.getFlywheelPowerOne(), launcher.getFlywheelPowerTwo()));
        telemetry.addData("  Aim pos", String.format(Locale.US, "%.3f",
                launcher.getAimPosition()));
        telemetry.addData("  Manual aim", targetLockEnabled ? "locked out" : "GP2 left stick");

        telemetry.addData("  Target lock", targetLockEnabled
                ? (drivetrain.hasPinpoint() ? "ON (GP2 RB to off)" : "ON (Pinpoint unavailable)")
                : "off (GP2 RB to on)");
        telemetry.addData("  Target alliance", selectedAlliance.name());
        Mark2TargetLock.FieldPoint target = targetLock.getGoal(selectedAlliance);
        if (drivetrain.hasPinpoint()) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = drivetrain.getPose();
            telemetry.addData("  Target distance", String.format(Locale.US, "%.1f in",
                    targetLock.computeDistanceToGoalInches(pose, selectedAlliance)));
        } else {
            telemetry.addData("  Target distance", "unavailable (no Pinpoint)");
        }
        telemetry.addData("  Target XY (in)", String.format(Locale.US, "(%.1f, %.1f)",
                target.xInches, target.yInches));
        if (targetLockEnabled && drivetrain.hasPinpoint()) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = drivetrain.getPose();
            telemetry.addData("  Lock turret deg", String.format(Locale.US, "%.1f",
                    targetLock.computeDesiredTurretDeg(pose, selectedAlliance)));
        }

        telemetry.addData("  Hood pos", String.format(Locale.US, "%.3f",
                nanToZero(launcher.getHoodPosition())));
        telemetry.addData("  Feeder pos", String.format(Locale.US, "%.3f",
                nanToZero(launcher.getFeederPosition())));

        telemetry.addLine("-- Intake --------------------------");
        telemetry.addData("  Servo pos", String.format(Locale.US, "%.3f",
                intake.getServoPosition()));
        telemetry.addData("  Running", intakeStatus());

        telemetry.update();
    }

    private void resetPoseToSelectedAllianceStart() {
        double xInches;
        double yInches;
        double headingDegrees;

        if (selectedAlliance == Mark2TargetLock.Alliance.BLUE) {
            xInches = BLUE_START_X_IN;
            yInches = BLUE_START_Y_IN;
            headingDegrees = BLUE_START_HEADING_DEG;
        } else {
            xInches = RED_START_X_IN;
            yInches = RED_START_Y_IN;
            headingDegrees = RED_START_HEADING_DEG;
        }

        drivetrain.setStartingPose(xInches, yInches, headingDegrees);
        drivetrain.setFieldCentricHeadingReference(headingDegrees);

        lastPoseReset = drivetrain.hasPinpoint()
                ? String.format(Locale.US, "%s %.1f, %.2f, %.0f deg",
                        selectedAlliance.name(), xInches, yInches, headingDegrees)
                : "Pinpoint unavailable";
    }

    private void handleSelectedAllianceToggle() {
        boolean x1 = gamepad1.x;
        if (x1 && !prevX1) {
            selectedAlliance = oppositeAlliance(selectedAlliance);
            setFieldCentricReferenceToSelectedAllianceStart();
        }
        prevX1 = x1;
    }

    private void setFieldCentricReferenceToSelectedAllianceStart() {
        double headingDegrees = selectedAlliance == Mark2TargetLock.Alliance.BLUE
                ? BLUE_START_HEADING_DEG
                : RED_START_HEADING_DEG;
        drivetrain.setFieldCentricHeadingReference(headingDegrees);
    }

    private void setAutoLauncherEnabled(boolean enabled) {
        if (enabled == autoLauncherEnabled) {
            return;
        }

        autoLauncherEnabled = enabled;
        if (enabled) {
            manualLauncher.stop();
        } else if (autoLauncher != null) {
            autoLauncher.stop();
        }
    }

    private String launcherModeText(boolean useAutoLauncher) {
        if (useAutoLauncher) {
            return "AUTO distance (GP2 LB to manual)";
        }
        if (autoLauncherEnabled) {
            return "AUTO requested, no Pinpoint";
        }
        return "MANUAL presets (GP2 LB to auto)";
    }

    private static Mark2TargetLock.Alliance oppositeAlliance(Mark2TargetLock.Alliance alliance) {
        return alliance == Mark2TargetLock.Alliance.BLUE
                ? Mark2TargetLock.Alliance.RED
                : Mark2TargetLock.Alliance.BLUE;
    }

    private static double nanToZero(double v) {
        return Double.isNaN(v) ? 0.0 : v;
    }

    private String intakeStatus() {
        if (manualLauncher.isLaunchSequenceRunningIntake()) {
            return "FORWARD (launch sequence)";
        }
        if (manualLauncher.isLaunchSequenceActive()) {
            return "waiting for launch intake";
        }
        if (gamepad2.y) {
            return "FORWARD (differential)";
        }
        return gamepad2.x ? "REVERSE (arm up)" : "stopped/hold";
    }
}
