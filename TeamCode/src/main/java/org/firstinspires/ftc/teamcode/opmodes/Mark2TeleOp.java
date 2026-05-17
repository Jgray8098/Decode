package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher.LauncherState;
import org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames;

import java.util.Locale;

/**
 * Mark2 primary TeleOp.
 *
 * Replaces ColorSensorTele.  All vision (Limelight), color-sensor (ArtifactTracker /
 * MotifStorage), PedroPathing (Follower), and old-mechanism (Flywheel / Indexer /
 * MecanumDrive) dependencies have been removed.  Only the three Mark2 subsystems
 * are used: {@link Mark2Drivetrain}, {@link Mark2Intake}, and {@link Mark2Launcher}.
 *
 * ── Control scheme ────────────────────────────────────────────────────────────
 *
 *  Gamepad 1 (driver)
 *  ─────────────────
 *  Left  stick Y / X   Forward / strafe (expo + deadzone applied in Drivetrain)
 *  Right stick X       Rotate
 *
 *  Gamepad 2 (operator)
 *  ────────────────────
 *  Y (held)            Intake — PickUp (full forward power)
 *  X (held)            Intake — Hold   (soft hold power)
 *  A (held) / default  Intake — Stop   (motors off, servo stow)
 *
 *  Left  bumper (held) Aim robot at goal: calls drivetrain.aim(GOAL_X, GOAL_Y)
 *                      each loop.  Normal drive is suppressed while aiming.
 *  Right bumper (edge) Fire: compute live distance from odometry to goal, call
 *                      launcher.shoot(distance) — uses the InterpolatingTreeMap
 *                      to select the correct RPM and hood angle automatically.
 *
 *  B (edge)            Full launcher stop (motors off, servos retract → IDLE)
 *
 *  D-pad Up   (edge)   Fallback: shoot at CLOSE_SHOT_DISTANCE preset
 *  D-pad Down (edge)   Fallback: shoot at FAR_SHOT_DISTANCE preset
 *
 *  After each shot the feeder is automatically retracted and the motors keep
 *  spinning (resetFeeder()) so the operator can fire the next ball immediately
 *  with another RB press.  Press B to spin everything down between volleys.
 *
 * ── TODO ──────────────────────────────────────────────────────────────────────
 *  • Set GOAL_X / GOAL_Y to the real field coordinates of the launch target.
 *  • Tune CLOSE_SHOT_DISTANCE and FAR_SHOT_DISTANCE fallback presets.
 *  • Verify Pinpoint config name matches the robot controller ("pinpoint").
 * ─────────────────────────────────────────────────────────────────────────────
 */
@TeleOp(name = "Mark2 TeleOp", group = "Mark2")
public class Mark2TeleOp extends OpMode {

    // ── Hardware config name — see Mark2HardwareMapNames ──────────────────────

    // ── Launch target field position (inches) — TUNE ─────────────────────────
    /**
     * Field X coordinate of the launch target (goal / basket).
     * Used by the auto-aim (left bumper) and live-distance shot (right bumper).
     * Update both values once the real target position is known.
     */
    private static final double GOAL_X = 72.0;   // TUNE — center of field as placeholder
    private static final double GOAL_Y = 72.0;   // TUNE

    // ── Fallback shot-distance presets (inches) — D-pad ─────────────────────
    /** D-pad Up fallback: close shot preset. */
    private static final double CLOSE_SHOT_DISTANCE =  60.0;   // TUNE
    /** D-pad Down fallback: far / long shot preset. */
    private static final double FAR_SHOT_DISTANCE   = 120.0;   // TUNE

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Mark2Drivetrain mark2Drivetrain;
    private Mark2Intake mark2Intake;
    private Mark2Launcher mark2Launcher;

    // ── Loop timing ───────────────────────────────────────────────────────────
    private long lastNs;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevRB2      = false;
    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;
    private boolean prevB2       = false;

    // ── Runtime state ─────────────────────────────────────────────────────────
    /** Distance to goal computed at the moment right bumper was pressed. */
    private double lastShotDistance = 0.0;
    /** True while left bumper is held and aim() is actively running. */
    private boolean aimActive = false;

    // ═════════════════════════════════════════════════════════════════════════
    // OpMode lifecycle
    // ═════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        I2cDeviceSynchSimple pinpointClient =
                hardwareMap.get(I2cDeviceSynchSimple.class, Mark2HardwareMapNames.PINPOINT);

        mark2Drivetrain = new Mark2Drivetrain(hardwareMap, pinpointClient);
        mark2Intake = new Mark2Intake(hardwareMap);
        mark2Launcher = new Mark2Launcher(hardwareMap);

        lastNs = System.nanoTime();

        telemetry.addLine("Mark2 TeleOp — Initialized");
        telemetry.addLine("GP1: Drive    GP2: Intake + Launcher");
        telemetry.addLine("─────────────────────────────────────");
        telemetry.addLine("GP2 Y=PickUp  X=Hold  A=Stop");
        telemetry.addLine("GP2 LB=Aim  RB=Fire(dist)  B=StopLauncher");
        telemetry.addLine("GP2 DUp=CloseShot(fallback)  DDn=FarShot(fallback)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ── Timing ────────────────────────────────────────────────────────────
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;

        // ── Drive / Aim ───────────────────────────────────────────────────────
        // Left bumper held → aim robot at goal (suppresses normal drive).
        // Releasing left bumper returns full control to the driver instantly.
        aimActive = gamepad2.left_bumper;
        if (aimActive) {
            mark2Drivetrain.aim(GOAL_X, GOAL_Y);
        } else {
            mark2Drivetrain.driveTeleop(gamepad1);
        }

        // ── Intake ────────────────────────────────────────────────────────────
        //   Priority: Y (PickUp) > X (Hold) > default (Stop)
        if (gamepad2.y) {
            mark2Intake.PickUp();
        } else if (gamepad2.x) {
            mark2Intake.Hold();
        } else {
            mark2Intake.Stop();
        }

        // ── Launcher — button edge detection ─────────────────────────────────
        boolean rb2      = gamepad2.right_bumper;
        boolean dpadUp   = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        boolean b2       = gamepad2.b;

        // Right bumper — live-distance shot using InterpolatingTreeMap
        if (rb2 && !prevRB2) {
            lastShotDistance = distanceToGoal();
            mark2Launcher.shoot(lastShotDistance);
        }

        // D-pad fallbacks
        if (dpadUp && !prevDpadUp) {
            lastShotDistance = CLOSE_SHOT_DISTANCE;
            mark2Launcher.shoot(CLOSE_SHOT_DISTANCE);
        }
        if (dpadDown && !prevDpadDown) {
            lastShotDistance = FAR_SHOT_DISTANCE;
            mark2Launcher.shoot(FAR_SHOT_DISTANCE);
        }

        // B — full stop
        if (b2 && !prevB2) {
            mark2Launcher.stop();
        }

        prevRB2      = rb2;
        prevDpadUp   = dpadUp;
        prevDpadDown = dpadDown;
        prevB2       = b2;

        // ── Launcher state machine ────────────────────────────────────────────
        mark2Launcher.update(dt);

        // Auto-retract feeder, keep motors spinning → ready for next shot immediately.
        // Press B to shut everything down between volleys.
        if (mark2Launcher.getState() == LauncherState.DONE) {
            mark2Launcher.resetFeeder();
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        showTelemetry();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Helpers
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Compute the straight-line distance (inches) from the robot's current
     * odometry position to {@link #GOAL_X} / {@link #GOAL_Y}.
     */
    private double distanceToGoal() {
        Pose2D pose = mark2Drivetrain.getPose();
        double dx = GOAL_X - pose.getX(DistanceUnit.INCH);
        double dy = GOAL_Y - pose.getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }

    private void showTelemetry() {
        // Pose + distance to goal
        Pose2D pose = mark2Drivetrain.getPose();
        telemetry.addData("X (in)",          "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)",          "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading",         "%.1f°", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Dist to goal",    String.format(Locale.US, "%.1f in", distanceToGoal()));
        telemetry.addData("Aiming",          aimActive ? "YES — rotating to goal" : "no");

        telemetry.addLine("─── Launcher ───────────────────────────");
        telemetry.addData("State",            mark2Launcher.getState());
        telemetry.addData("Last shot dist",   String.format(Locale.US, "%.1f in", lastShotDistance));
        telemetry.addData("Target RPM",       String.format(Locale.US, "%.0f", mark2Launcher.getTargetRpm()));
        telemetry.addData("Measured RPM",     String.format(Locale.US, "%.0f", mark2Launcher.getMeasuredRpm()));
        telemetry.addData("Target Power",     String.format(Locale.US, "%.3f", mark2Launcher.getTargetPower()));
        telemetry.addData("Hood pos",         String.format(Locale.US, "%.3f", nanToZero(mark2Launcher.getHoodPosition())));
        telemetry.addData("Feeder pos",       String.format(Locale.US, "%.3f", nanToZero(mark2Launcher.getFeederPosition())));
        telemetry.addData("At speed?",        mark2Launcher.isAtSpeed());

        telemetry.addLine("─── Intake servo ───────────────────────");
        telemetry.addData("Intake servo",     String.format(Locale.US, "%.3f", nanToZero(mark2Intake.getServoPosition())));

        telemetry.addLine("─── Controls ───────────────────────────");
        telemetry.addLine("GP2 Y=PickUp  X=Hold  A=Stop");
        telemetry.addLine("GP2 LB=Aim  RB=Fire  B=StopLauncher");
        telemetry.addLine("GP2 DUp/DDn=Fallback presets");

        telemetry.update();
    }

    /** Guard against the SDK returning Double.NaN before any setPosition() is called. */
    private static double nanToZero(double value) {
        return Double.isNaN(value) ? 0.0 : value;
    }
}

