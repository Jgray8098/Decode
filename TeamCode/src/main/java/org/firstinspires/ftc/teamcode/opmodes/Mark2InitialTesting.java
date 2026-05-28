package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.Mark2ManualLauncherController;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

import java.util.Locale;

/**
 * Mark2InitialTesting
 *
 * A safe, low-power OpMode for first-time bench and field testing of the
 * main subsystems.  Nothing runs at full power; every subsystem can be
 * exercised independently to verify wiring, motor direction, and servo
 * positions.
 *
 * ── STARTUP SAFETY ───────────────────────────────────────────────────────────
 *  On INIT the subsystem constructors are called.  Intake, launcher, and turret
 *  servos are moved to their known center positions.
 *
 * ── HARDWARE STATUS ──────────────────────────────────────────────────────────
 *  Drivetrain  : motors LIVE  (no Pinpoint / IMU)
 *  Intake      : motors + dual arm servos LIVE
 *  Launcher    : motors + hood servo + dual gate servos LIVE
 *  Turret Aim  : dual aim servos LIVE
 *
 * ── CONTROL MAP ──────────────────────────────────────────────────────────────
 *
 *  Gamepad 1  (Drivetrain — capped at safe power inside Mark2Drivetrain)
 *  ┌──────────────────────────────────────────────────────────────┐
 *  │  Right stick Y / X  →  translate (forward / strafe)         │
 *  │  Left  stick X      →  rotate                               │
 *  │  Left  trigger hold →  snail mode (60 % power cap)          │
 *  │  Right bumper       →  toggle alliance-flip (reverses fwd   │
 *  │                         + strafe for opposite-side driving) │
 *  │  START              →  confirm safety during INIT           │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *  Gamepad 2  (Intake + Launcher + Turret Aim)
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  INTAKE                                                                 │
 *  │    Y  (hold)        →  intake forward (PickUp)                          │
 *  │    A  (hold)        →  intake reverse                                   │
 *  │    (neither)        →  intake stopped                                   │
 *  │    LB  press        →  intake servos position  − 0.02  (nudge down)     │
 *  │    RB  press        →  intake servos position  + 0.02  (nudge up)       │
 *  │                                                                         │
 *  │  LAUNCHER MOTORS                                                        │
 *  │    X press                   →  select CLOSE zone (2000 RPM, hood 0.20) │
 *  │    B press                   →  select FAR zone (2800 RPM, hood 0.70)   │
 *  │    Right trigger > 0.5 hold  →  spin motors at selected zone RPM        │
 *  │    (trigger released)           →  motors stop                          │
 *  │                                                                         │
 *  │  LAUNCHER SERVOS                                                        │
 *  │    Dpad Up/Down         →  hood servo nudge                             │
 *  │    Dpad Right/Left      →  gate servos nudge                            │
 *  │                                                                         │
 *  │  TURRET AIM                                                             │
 *  │    Left stick X         →  aim servos position                          │
 *  │                           full left = 0.0, center = 0.5, right = 1.0    │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 * ── TUNING WORKFLOW ──────────────────────────────────────────────────────────
 *  1. Use X/B to select close/far launcher presets, then hold right trigger.
 *  2. Use LB/RB to find intake servo positions.
 *  3. Use Dpad Up/Down to find hood servo positions.
 *  4. Use Dpad Left/Right to find gate servo positions.
 *  5. Use GP2 left stick X to verify turret aim servo travel.
 */
//@TeleOp(name = "Mark2InitialTesting", group = "Test")
public class Mark2InitialTesting extends OpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Mark2Launcher launcher;
    private Mark2ManualLauncherController manualLauncher;
    private Mark2Intake intake;
    private Mark2Drivetrain drivetrain;

    // ── Hardware config note ──────────────────────────────────────────────────
    // Pinpoint / IMU not currently installed — using no-arg drivetrain constructor.

    // ── Test constants ────────────────────────────────────────────────────────
    /** How much a single bumper or dpad press shifts a servo position. */
    private static final double SERVO_NUDGE_STEP = 0.02;
    /** Safe centered position used during initial bring-up. */
    private static final double SERVO_CENTER_POS = 0.50;
    // ── Servo position tracking ───────────────────────────────────────────────
    private double intakeServoPos = SERVO_CENTER_POS;

    // ── Safety gate ───────────────────────────────────────────────────────────
    private boolean confirmed = false;

    // ── dt timing ─────────────────────────────────────────────────────────────
    private long lastNs = 0;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevLB2, prevRB2;
    private boolean prevStartGp1 = false;
    private boolean prevRB1      = false;   // GP1 right bumper — alliance flip toggle


    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);       // no Pinpoint / IMU
        intake = new Mark2Intake(hardwareMap);
        launcher = new Mark2Launcher(hardwareMap, true);     // true = motors + servos
        manualLauncher = new Mark2ManualLauncherController(launcher);

        // Initialize intake servos to safe centered positions
        intake.setServoPosition(SERVO_CENTER_POS);
        intakeServoPos = SERVO_CENTER_POS;

        // Initialize launcher servos to safe centered positions
        manualLauncher.centerServos();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init_loop() {
        telemetry.addLine("══ SAFETY CHECK — inspect robot before confirming ══");
        telemetry.addLine("");

        telemetry.addLine("Servo status after INIT:");

        telemetry.addData("  Intake servos",
                Double.isNaN(intake.getServoPosition())
                        ? "NOT commanded — physically untouched"
                        : String.format(Locale.US, "commanded to %.3f by constructor",
                        intake.getServoPosition()));

        telemetry.addData("  Turret aim servos",
                String.format(Locale.US, "commanded to %.3f", launcher.getAimPosition()));

        telemetry.addData("  Hood servo",
                Double.isNaN(launcher.getHoodPosition())
                        ? "NOT commanded"
                        : String.format(Locale.US, "currently %.3f", launcher.getHoodPosition()));

        telemetry.addData("  Gate servos",
                Double.isNaN(launcher.getFeederPosition())
                        ? "NOT commanded"
                        : String.format(Locale.US, "currently %.3f", launcher.getFeederPosition()));

        telemetry.addLine("");
        telemetry.addLine("Inspect checklist:");
        telemetry.addLine("  [ ] Intake servos are within their mechanical range");
        telemetry.addLine("  [ ] Turret servos are centered and not binding");
        telemetry.addLine("  [ ] Hood servo is within its mechanical range");
        telemetry.addLine("  [ ] Gate servos are within their mechanical range");
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
        // GP1 RB (rising edge) — toggle alliance-flip (reverses forward + strafe)
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) drivetrain.toggleAllianceFlip();
        prevRB1 = rb1;

        drivetrain.driveSafe(gamepad1);

        // ── Intake motors ─────────────────────────────────────────────────────
        if (gamepad2.y) {
            intake.PickUp();
        } else if (gamepad2.a) {
            intake.Reverse();
        } else {
            intake.Stop();
        }

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

        // ── Manual launcher bring-up controls ─────────────────────────────────
        manualLauncher.update(gamepad2, dt);

        // ── Telemetry ─────────────────────────────────────────────────────────
        Pose2D pose = drivetrain.getPose();

        telemetry.addLine("── Drivetrain ──────────────────");
        telemetry.addData("  X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("  Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("  Heading (°)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("  Alliance flip", drivetrain.isAllianceFlipped() ? "FLIPPED (GP1 RB to restore)" : "normal  (GP1 RB to flip)");
        telemetry.addData("  Snail mode",    gamepad1.left_trigger > 0 ? "ACTIVE (60%)" : "off");

        telemetry.addLine("── Launcher ────────────────────");
        telemetry.addData("  Mode", "MANUAL");
        telemetry.addData("  Selected zone", manualLauncher.getSelectedZoneName());
        telemetry.addData("  Zone RPM", "%.0f", manualLauncher.getSelectedZoneRpm());
        telemetry.addData("  Zone hood", "%.3f", manualLauncher.getSelectedZoneHoodPosition());
        telemetry.addData("  Target RPM", "%.0f", manualLauncher.getTargetRpmCommand());
        telemetry.addData("  Measured RPM", "%.0f", launcher.getMeasuredRpm());
        telemetry.addData("  Hood servo pos", "%.3f", manualLauncher.getHoodServoPos());
        telemetry.addData("  Gate servo pos", "%.3f", manualLauncher.getFeederServoPos());

        telemetry.addLine("── Intake ──────────────────────");
        telemetry.addData("  Intake servo pos", "%.3f", intakeServoPos);
        telemetry.addData("  Motor running",
                gamepad2.y ? "FORWARD" : (gamepad2.a ? "REVERSE" : "stopped"));

        telemetry.addLine("── Turret Aim ──────────────────");
        telemetry.addData("  GP2 left stick X", "%.2f", gamepad2.left_stick_x);
        telemetry.addData("  Aim servo pos",    "%.3f", launcher.getAimPosition());

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

