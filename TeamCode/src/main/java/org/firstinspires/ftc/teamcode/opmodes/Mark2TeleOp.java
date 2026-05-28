package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher.LauncherState;

import java.util.Locale;

/**
 * Mark2 primary TeleOp.
 *
 * ── CONTROL MAP ───────────────────────────────────────────────────────────────
 *
 *  Gamepad 1  (Driver)
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │  Right stick Y / X  →  translate (forward / strafe)         │
 *  │  Left  stick X      →  rotate                               │
 *  │  Left  trigger hold →  snail mode (60 % power cap)          │
 *  │  Right bumper       →  toggle alliance-flip                  │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *  Gamepad 2  (Operator)
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  INTAKE                                                                 │
 *  │    Y  (hold)        →  intake forward (motor 1 full, motor 2 × 1/3)    │
 *  │    X  (hold)        →  intake reverse, arm raised to stowed position    │
 *  │    (release Y/X)    →  motors stop, servo moves to hold position        │
 *  │                                                                         │
 *  │  LAUNCHER                                                               │
 *  │    Dpad Up          →  select CLOSE shot distance preset                │
 *  │    Dpad Down        →  select FAR shot distance preset                  │
 *  │    B  (hold)        →  spin up and fire; stops when released            │
 *  │    A  (press)       →  cut launcher power immediately                   │
 *  │                                                                         │
 *  │  FEEDER (manual, only when launcher is IDLE)                            │
 *  │    Left stick X     →  aim servo position                              │
 *  │                         full left  = AIM_MIN_POS (0.05)               │
 *  │                         full right = AIM_MAX_POS (0.95)               │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 * ── TODO ──────────────────────────────────────────────────────────────────────
 *  • Tune CLOSE_SHOT_DISTANCE and FAR_SHOT_DISTANCE to real field distances.
 *  • Tune FEEDER_MIN_POS / FEEDER_MAX_POS to physical servo limits.
 *  • Re-enable Pinpoint odometry once hardware is installed.
 * ─────────────────────────────────────────────────────────────────────────────
 */
@TeleOp(name = "Mark2 TeleOp", group = "Mark2")
public class Mark2TeleOp extends OpMode {

    // ── Launcher distance presets (inches) ────────────────────────────────────
    /** Dpad Up preset — close shot. */
    private static final double CLOSE_SHOT_DISTANCE = 60.0;    // TUNE
    /** Dpad Down preset — far / long shot.  Reduced from 120 to limit max power. */
    private static final double FAR_SHOT_DISTANCE   = 72.0;    // TUNE

    // ── Aim servo manual-control limits ──────────────────────────────────────
    /** Minimum (leftmost) position for manual aim servo control. */
    private static final double AIM_MIN_POS = 0.05;
    /** Maximum (rightmost) position for manual aim servo control. */
    private static final double AIM_MAX_POS = 0.95;
    /** Stick deadzone applied to the aim control axis on GP2. */
    private static final double AIM_STICK_DEADZONE = 0.05;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Mark2Drivetrain drivetrain;
    private Mark2Intake     intake;
    private Mark2Launcher   launcher;

    // ── Loop timing ───────────────────────────────────────────────────────────
    private long lastNs;

    // ── Match state ───────────────────────────────────────────────────────────
    /** Distance selected by the operator via Dpad.  Defaults to close preset. */
    private double selectedDistance = CLOSE_SHOT_DISTANCE;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevRB1      = false;
    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;

    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);   // no Pinpoint — not yet on robot
        intake     = new Mark2Intake(hardwareMap);
        launcher   = new Mark2Launcher(hardwareMap);     // full hardware (all servos)

        lastNs = System.nanoTime();

        telemetry.addLine("Mark2 TeleOp — Initialized");
        telemetry.addLine("GP1: Drive   GP2: Intake + Launcher");
        telemetry.addLine("GP2  Y=Intake  DUp/DDn=SelectDist  B=Fire");
        telemetry.addLine("GP2  LeftStickX=Feeder (IDLE only)");
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {

        // ── Timing ────────────────────────────────────────────────────────────
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;

        // ── GP1 Drivetrain ────────────────────────────────────────────────────
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) drivetrain.toggleAllianceFlip();
        prevRB1 = rb1;

        drivetrain.driveSafe(gamepad1);   // right stick = translate, left stick X = rotate
                                          // left trigger = snail mode (60 % cap)

        // ── GP2 Intake ────────────────────────────────────────────────────────
        if (gamepad2.y) {
            // Motor 1 at full INTAKE_POWER, motor 2 at 1/3 power, servo to intake position
            intake.PickUpDifferential();
        } else if (gamepad2.x) {
            // Reverse with arm raised — eject / unjam; arm returns to hold when released
            intake.ReverseArm();
        } else {
            // Servo moves to hold position; motors stop
            intake.HoldPosition();
        }

        // ── GP2 Launcher — distance selection (Dpad) ─────────────────────────
        boolean dpadUp   = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        if (dpadUp   && !prevDpadUp)   selectedDistance = CLOSE_SHOT_DISTANCE;
        if (dpadDown && !prevDpadDown) selectedDistance = FAR_SHOT_DISTANCE;

        prevDpadUp   = dpadUp;
        prevDpadDown = dpadDown;

        // ── GP2 Launcher — fire while B held; A cuts power immediately ────────
        if (gamepad2.a) {
            // A — hard stop: cut motors and retract feeder regardless of state
            launcher.stop();
        } else if (gamepad2.b) {
            // B held — start the sequence when idle; state machine runs autonomously
            if (launcher.getState() == LauncherState.IDLE) {
                launcher.shoot(selectedDistance);
            }
        } else {
            // B released — stop launcher if it was running
            if (launcher.getState() != LauncherState.IDLE) {
                launcher.stop();
            }
        }

        // ── GP2 Aim servo (left stick X — always active) ──────────────────────
        double aimStick = gamepad2.left_stick_x;
        if (Math.abs(aimStick) > AIM_STICK_DEADZONE) {
            // Map [-1, +1]  →  [AIM_MIN_POS, AIM_MAX_POS]
            double aimPos = (aimStick + 1.0) / 2.0
                    * (AIM_MAX_POS - AIM_MIN_POS)
                    + AIM_MIN_POS;
            launcher.setAimPosition(aimPos);
        }

        // ── Launcher state machine ────────────────────────────────────────────
        launcher.update(dt);

        // Auto-retract feeder once shot is complete; keeps motors spinning so
        // the operator can fire again immediately with another B press.
        if (launcher.getState() == LauncherState.DONE) {
            launcher.resetFeeder();
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        showTelemetry();
    }

    // ═════════════════════════════════════════════════════════════════════════
    private void showTelemetry() {
        telemetry.addLine("── Driver ──────────────────────────");
        telemetry.addData("  Alliance flip", drivetrain.isAllianceFlipped()
                ? "FLIPPED (RB to restore)" : "normal  (RB to flip)");
        telemetry.addData("  Snail mode", gamepad1.left_trigger > 0 ? "ACTIVE (60%)" : "off");

        telemetry.addLine("── Launcher ────────────────────────");
        telemetry.addData("  State",         launcher.getState().name());
        telemetry.addData("  Selected dist", String.format(Locale.US, "%.0f in  (DUp=%.0f  DDn=%.0f)",
                selectedDistance, CLOSE_SHOT_DISTANCE, FAR_SHOT_DISTANCE));
        telemetry.addData("  Target RPM",    String.format(Locale.US, "%.0f", launcher.getTargetRpm()));
        telemetry.addData("  Measured RPM",  String.format(Locale.US, "%.0f", launcher.getMeasuredRpm()));
        telemetry.addData("  At speed",      launcher.isAtSpeed() ? "YES" : "no");
        telemetry.addData("  Aim pos",       String.format(Locale.US, "%.3f", launcher.getAimPosition()));
        telemetry.addData("  Hood pos",      String.format(Locale.US, "%.3f",
                nanToZero(launcher.getHoodPosition())));
        telemetry.addData("  Feeder pos",    String.format(Locale.US, "%.3f",
                nanToZero(launcher.getFeederPosition())));

        telemetry.addLine("── Intake ──────────────────────────");
        telemetry.addData("  Servo pos",     String.format(Locale.US, "%.3f",
                intake.getServoPosition()));
        telemetry.addData("  Running",       gamepad2.y ? "FORWARD (differential)"
                : (gamepad2.x ? "REVERSE (arm up)" : "stopped/hold"));

        telemetry.update();
    }

    private static double nanToZero(double v) {
        return Double.isNaN(v) ? 0.0 : v;
    }
}

