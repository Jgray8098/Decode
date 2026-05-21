package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

/**
 * Mark2InitialTesting
 *
 * A safe, low-power OpMode for first-time bench and field testing of the
 * main subsystems.  Nothing runs at full power; every subsystem can be
 * exercised independently to verify wiring, motor direction, and servo
 * positions.
 *
 * ── STARTUP SAFETY ───────────────────────────────────────────────────────────
 *  On INIT the subsystem constructors are called.  No servo is moved until
 *  the driver physically inspects the robot and presses GP1 START.
 *
 * ── HARDWARE STATUS ──────────────────────────────────────────────────────────
 *  Drivetrain  : motors LIVE  (no Pinpoint / IMU)
 *  Intake      : motors + servo LIVE
 *  Launcher    : motors LIVE  — servos NOT installed, all servo calls disabled
 *
 * ── CONTROL MAP ──────────────────────────────────────────────────────────────
 *
 *  Gamepad 1  (Drivetrain — capped at 40 % power)
 *  ┌─────────────────────────────────────────────────────┐
 *  │  Left  stick Y / X  →  translate (forward / strafe) │
 *  │  Right stick X      →  rotate                       │
 *  └─────────────────────────────────────────────────────┘
 *
 *  Gamepad 2  (Intake + Launcher motors)
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  INTAKE                                                                 │
 *  │    Y  (hold)        →  intake forward (PickUp)                          │
 *  │    A  (hold)        →  intake reverse                                   │
 *  │    (neither)        →  intake stopped                                   │
 *  │    LB  press        →  intake servo position  − 0.02  (nudge down)      │
 *  │    RB  press        →  intake servo position  + 0.02  (nudge up)        │
 *  │                                                                         │
 *  │  LAUNCHER MOTORS                                                        │
 *  │    Right trigger > 0.5  (hold)  →  spin motors at 30 %                 │
 *  │    (trigger released)           →  motors stop                          │
 *  │                                                                         │
 *  │  LAUNCHER SERVOS  [DISABLED — hardware not yet installed]               │
 *  │    Dpad Up/Down         →  hood servo nudge    (disabled)               │
 *  │    Dpad Right/Left      →  feeder servo nudge  (disabled)               │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 * ── TUNING WORKFLOW ──────────────────────────────────────────────────────────
 *  1. Use right trigger to verify launcher motor direction at low power.
 *  2. Use LB/RB to find intake servo positions.
 *  3. Once launcher servos are installed, change hasServos=false → true and
 *     uncomment the Dpad servo nudge blocks below.
 */
@TeleOp(name = "Mark2InitialTesting", group = "Test")
public class Mark2InitialTesting extends OpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Mark2Launcher launcher;
    private Mark2Intake   intake;
    private Mark2Drivetrain drivetrain;

    // ── Hardware config note ──────────────────────────────────────────────────
    // Pinpoint / IMU not currently installed — using no-arg drivetrain constructor.

    // ── Test constants ────────────────────────────────────────────────────────
    /** Motor power used when the right trigger spins the launcher during testing. */
    private static final double LAUNCHER_TEST_POWER = 0.30;
    /** How much a single bumper press shifts a servo position. */
    private static final double SERVO_NUDGE_STEP    = 0.02;

    // ── Servo position tracking ───────────────────────────────────────────────
    private double hoodServoPos   = 0.0;   // tracking only — servos not installed
    private double feederServoPos = 0.0;   // tracking only — servos not installed
    private double intakeServoPos = 0.0;

    // ── Safety gate ───────────────────────────────────────────────────────────
    private boolean confirmed = false;

    // ── dt timing ─────────────────────────────────────────────────────────────
    private long lastNs = 0;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLB2, prevRB2;
    private boolean prevStartGp1 = false;

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);        // no Pinpoint / IMU
        intake     = new Mark2Intake(hardwareMap);
        launcher   = new Mark2Launcher(hardwareMap, false);   // false = motors only, no servos

        intakeServoPos = nanToZero(intake.getServoPosition());
        // hoodServoPos / feederServoPos left at 0.0 — servos not installed
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init_loop() {
        telemetry.addLine("══ SAFETY CHECK — inspect robot before confirming ══");
        telemetry.addLine("");

        telemetry.addLine("Servo status after INIT:");
        telemetry.addLine("  Hood servo (1 & 2)  — NOT installed");
        telemetry.addLine("  Feeder servo        — NOT installed");
        telemetry.addData("  Intake servo",
                Double.isNaN(intake.getServoPosition())
                        ? "NOT commanded — physically untouched"
                        : String.format(Locale.US, "commanded to %.3f by constructor",
                                intake.getServoPosition()));

        telemetry.addLine("");
        telemetry.addLine("Inspect checklist:");
        telemetry.addLine("  [ ] Intake servo is within its mechanical range");
        telemetry.addLine("  [ ] No linkage is bound or under unexpected stress");
        telemetry.addLine("  [ ] Motors can spin freely");
        telemetry.addLine("");

        if (confirmed) {
            telemetry.addLine("✓  CONFIRMED — press START on Driver Station to begin");
        } else {
            telemetry.addLine(">> Press GP1 START to confirm and unlock operation <<");
        }

        boolean startNow = gamepad1.start;
        if (startNow && !prevStartGp1) {
            confirmed = true;
            lastNs = System.nanoTime();   // seed dt timer at confirmation
        }
        prevStartGp1 = startNow;

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {

        // ── Safety gate ───────────────────────────────────────────────────────
        if (!confirmed) {
            telemetry.addLine("⚠  Not confirmed — return to INIT and press GP1 START");
            telemetry.update();
            return;
        }

        // ── dt timing ─────────────────────────────────────────────────────────
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;

        // ── Drivetrain ────────────────────────────────────────────────────────
        drivetrain.driveSafe(gamepad1);

        // ── Intake motors ─────────────────────────────────────────────────────
        if      (gamepad2.y) intake.PickUp();
        else if (gamepad2.a) intake.Reverse();
        else                 intake.Stop();

        // ── Intake servo nudge (LB / RB) ─────────────────────────────────────
        boolean lb2 = gamepad2.left_bumper;
        boolean rb2 = gamepad2.right_bumper;
        if (lb2 && !prevLB2) {
            intakeServoPos = clamp(intakeServoPos - SERVO_NUDGE_STEP, 0.0, 1.0);
            intake.setServoPosition(intakeServoPos);
        }
        if (rb2 && !prevRB2) {
            intakeServoPos = clamp(intakeServoPos + SERVO_NUDGE_STEP, 0.0, 1.0);
            intake.setServoPosition(intakeServoPos);
        }
        prevLB2 = lb2;
        prevRB2 = rb2;

        // ── Launcher motors (hold right trigger) ──────────────────────────────
        if (gamepad2.right_trigger > 0.5) {
            launcher.testSpinMotors(LAUNCHER_TEST_POWER);
        } else {
            launcher.testSpinMotors(0.0);
        }

        // ── Launcher state machine ────────────────────────────────────────────
        launcher.update(dt);

        // ── Launcher servo nudge — DISABLED (servos not installed) ───────────
        // Uncomment once hood and feeder servos are physically installed AND
        // Mark2Launcher constructor is changed to hasServos = true.
        /*
        boolean dpadUp   = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        if (dpadUp && !prevDpadUp) {
            hoodServoPos = clamp(hoodServoPos + SERVO_NUDGE_STEP, 0.0, 1.0);
            launcher.setHoodPosition(hoodServoPos);
        }
        if (dpadDown && !prevDpadDown) {
            hoodServoPos = clamp(hoodServoPos - SERVO_NUDGE_STEP, 0.0, 1.0);
            launcher.setHoodPosition(hoodServoPos);
        }
        boolean dpadRight = gamepad2.dpad_right;
        boolean dpadLeft  = gamepad2.dpad_left;
        if (dpadRight && !prevDpadRight) {
            feederServoPos = clamp(feederServoPos + SERVO_NUDGE_STEP, 0.0, 1.0);
            launcher.setFeederPosition(feederServoPos);
        }
        if (dpadLeft && !prevDpadLeft) {
            feederServoPos = clamp(feederServoPos - SERVO_NUDGE_STEP, 0.0, 1.0);
            launcher.setFeederPosition(feederServoPos);
        }
        prevDpadUp    = dpadUp;   prevDpadDown  = dpadDown;
        prevDpadRight = dpadRight; prevDpadLeft  = dpadLeft;
        */

        // ── Telemetry ─────────────────────────────────────────────────────────
        Pose2D pose = drivetrain.getPose();

        telemetry.addLine("── Drivetrain ──────────────────");
        telemetry.addData("  X (in)",      "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("  Y (in)",      "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("  Heading (°)", "%.1f", pose.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("── Launcher (motors only) ──────");
        telemetry.addData("  State",          launcher.getState().name());
        telemetry.addData("  Motor power cmd","%.2f",
                gamepad2.right_trigger > 0.5 ? LAUNCHER_TEST_POWER : 0.0);
        telemetry.addData("  Measured RPM",   "%.0f", launcher.getMeasuredRpm());
        telemetry.addLine("  Hood / feeder servos — NOT installed");

        telemetry.addLine("── Intake ──────────────────────");
        telemetry.addData("  Intake servo pos", "%.3f", intakeServoPos);
        telemetry.addData("  Motor running",
                gamepad2.y ? "FORWARD" : (gamepad2.a ? "REVERSE" : "stopped"));

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double nanToZero(double value) {
        return Double.isNaN(value) ? 0.0 : value;
    }
}

