package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 *   Left bumper       - reset Pinpoint pose to selected alliance start
 *   Right bumper      - toggle alliance flip
 *   Right trigger     - toggle field-centric driving (resets heading reference on enable)
 *   Back button       - re-zero field-centric heading reference
 *
 * Gamepad 2:
 *   Dpad Up press     - toggle close launcher preset
 *   Dpad Down press   - toggle far launcher preset
 *   B press           - run launch-feed sequence, only if flywheel is on
 *   Y press/hold      - return feeder gate to idle and run intake forward
 *   X hold            - intake reverse, only when launch sequence is idle
 *   Left stick X      - aim servo position
 *   A press           - toggle turret lock on alliance goal target
 */
@TeleOp(name = "Mark2 TeleOp", group = "Mark2")
public class Mark2TeleOp extends OpMode {

    private Mark2Drivetrain drivetrain;
    private Mark2Intake intake;
    private Mark2Launcher launcher;
    private Mark2ManualLauncherController manualLauncher;
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

    private boolean prevRB1 = false;
    private boolean prevLB1 = false;
    private boolean prevRT1 = false;   // GP1 right trigger — field-centric toggle
    private boolean prevBack1 = false; // GP1 back button  — re-zero field-centric heading

    private String lastPoseReset = "not reset";
    private Mark2TargetLock.Alliance selectedAlliance = INITIAL_ALLIANCE;
    private boolean prevA2 = false;
    private boolean targetLockEnabled = false;

    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);
        intake = new Mark2Intake(hardwareMap);
        launcher = new Mark2Launcher(hardwareMap);
        manualLauncher = new Mark2ManualLauncherController(launcher, intake);

        targetLock = new Mark2TargetLock();

        if (selectedAlliance == Mark2TargetLock.Alliance.BLUE) {
            drivetrain.toggleAllianceFlip();
        }

        launcher.resetFeeder();

        lastNs = System.nanoTime();

        telemetry.addLine("Mark2 TeleOp - Initialized");
        telemetry.addLine("GP1: Drive   GP2: Intake + Launcher");
        telemetry.addLine("GP1 LB=Reset pose to alliance start");
        telemetry.addLine("GP2 DUp=Close toggle DDn=Far toggle");
        telemetry.addLine("GP2 B=Launch Y=Intake+GateIdle X=Reverse");
        telemetry.addLine("GP2 A=Toggle turret target lock");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;

        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            drivetrain.toggleAllianceFlip();
            selectedAlliance = oppositeAlliance(selectedAlliance);
        }
        prevRB1 = rb1;

        boolean lb1 = gamepad1.left_bumper;
        if (lb1 && !prevLB1) {
            resetPoseToSelectedAllianceStart();
        }
        prevLB1 = lb1;

        // GP1 right trigger (rising edge) — toggle field-centric driving
        boolean rt1 = gamepad1.right_trigger > 0.5;
        if (rt1 && !prevRT1) {
            drivetrain.toggleFieldCentric();
        }
        prevRT1 = rt1;

        // GP1 back button (rising edge) — re-zero the field-centric heading reference
        boolean back1 = gamepad1.back;
        if (back1 && !prevBack1) {
            drivetrain.resetFieldCentricHeading();
        }
        prevBack1 = back1;

        drivetrain.driveSafe(gamepad1);

        manualLauncher.update(gamepad2, dt);

        boolean a2 = gamepad2.a;
        if (a2 && !prevA2) {
            targetLockEnabled = !targetLockEnabled;
        }
        prevA2 = a2;

        if (targetLockEnabled && drivetrain.hasPinpoint()) {
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D pose = drivetrain.getPose();
            targetLock.lockToGoal(launcher, pose, selectedAlliance);
        }

        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("-- Driver --------------------------");
        telemetry.addData("  Alliance", String.format(Locale.US, "%s (RB to %s)",
                selectedAlliance.name(), oppositeAlliance(selectedAlliance).name()));
        telemetry.addData("  Pose reset", lastPoseReset);
        telemetry.addData("  Snail mode", gamepad1.left_trigger > 0 ? "ACTIVE (60%)" : "off");
        telemetry.addData("  Field-centric", drivetrain.hasPinpoint()
                ? (drivetrain.isFieldCentric() ? "ON  (RT=disable, Back=re-zero)" : "off (RT to enable)")
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
        telemetry.addData("  Mode", "MANUAL");
        telemetry.addData("  Flywheel", manualLauncher.isFlywheelRunning() ? "ON" : "off");
        telemetry.addData("  Launch seq", manualLauncher.getLaunchSequenceStateName());
        telemetry.addData("  Selected zone", manualLauncher.getSelectedZoneName());
        telemetry.addData("  Zone RPM", String.format(Locale.US, "%.0f",
                manualLauncher.getSelectedZoneRpm()));
        telemetry.addData("  Zone hood", String.format(Locale.US, "%.3f",
                manualLauncher.getSelectedZoneHoodPosition()));
        telemetry.addData("  Target RPM", String.format(Locale.US, "%.0f",
                manualLauncher.getTargetRpmCommand()));
        telemetry.addData("  Measured RPM", String.format(Locale.US, "%.0f",
                launcher.getMeasuredRpm()));
        telemetry.addData("  Motor RPM", String.format(Locale.US, "%.0f / %.0f",
                launcher.getMeasuredRpmOne(), launcher.getMeasuredRpmTwo()));
        telemetry.addData("  Motor power", String.format(Locale.US, "%.2f / %.2f",
                launcher.getFlywheelPowerOne(), launcher.getFlywheelPowerTwo()));
        telemetry.addData("  Aim pos", String.format(Locale.US, "%.3f",
                launcher.getAimPosition()));

        telemetry.addData("  Target lock", targetLockEnabled
                ? (drivetrain.hasPinpoint() ? "ON (A to off)" : "ON (Pinpoint unavailable)")
                : "off (A to on)");
        telemetry.addData("  Target alliance", selectedAlliance.name());
        Mark2TargetLock.FieldPoint target = targetLock.getGoal(selectedAlliance);
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
        drivetrain.resetFieldCentricHeading();

        lastPoseReset = drivetrain.hasPinpoint()
                ? String.format(Locale.US, "%s %.1f, %.2f, %.0f deg",
                        selectedAlliance.name(), xInches, yInches, headingDegrees)
                : "Pinpoint unavailable";
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
