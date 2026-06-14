package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.Mark2LaunchSequence;
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
 *  │  Right trigger      →  toggle field-centric driving         │
 *  │                         (resets heading reference on enable) │
 *  │  Back button        →  re-zero field-centric heading        │
 *  │  START              →  confirm safety during INIT           │
 *  └──────────────────────────────────────────────────────────────┘
 *
 *  Gamepad 2  (Intake + Launcher + Turret Aim)
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  INTAKE                                                                 │
 *  │    Y  (press/hold)  →  gate idle + intake forward (PickUp)              │
 *  │    A  (hold)        →  intake reverse                                   │
 *  │    (neither)        →  intake stopped                                   │
 *  │    LB  press        →  intake servos position  − 0.02  (nudge down)     │
 *  │    RB  press        →  intake servos position  + 0.02  (nudge up)       │
 *  │                                                                         │
 *  │  LAUNCHER MOTORS                                                        │
 *  │    X press                   →  nudge target RPM down by 50             │
 *  │    B press                   →  nudge target RPM up by 50               │
 *  │    Right trigger press       →  toggle motors on/off                    │
 *  │    (turning on)                 →  reset target to 1000 RPM             │
 *  │    Left trigger press        →  run launch sequence if motors are on    │
 *  │                                                                         │
 *  │  LAUNCHER SERVOS                                                        │
 *  │    Dpad Up/Down         →  hood servo nudge                             │
 *  │    Gate servos          →  controlled by launch sequence + Y reset      │
 *  │                                                                         │
 *  │  TURRET AIM                                                             │
 *  │    Left stick X         →  aim servos position                          │
 *  │                           full left = 0.0, center = 0.5, right = 1.0    │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 * ── TUNING WORKFLOW ──────────────────────────────────────────────────────────
 *  1. Use right trigger to toggle motors on at 1000 RPM, then X/B to nudge RPM.
 *     Press left trigger to run the launch sequence once flywheels are on.
 *  2. Use LB/RB to find intake servo positions.
 *  3. Use Dpad Up/Down to find hood servo positions.
 *  4. Use GP2 left stick X to verify turret aim servo travel.
 */
@TeleOp(name = "Mark2InitialTesting", group = "Test")
public class Mark2InitialTesting extends OpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Mark2Launcher launcher;
    private Mark2LaunchSequence launchSequence;
    private Mark2Intake intake;
    private Mark2Drivetrain drivetrain;

    // ── Hardware config note ──────────────────────────────────────────────────
    // Pinpoint / IMU not currently installed — using no-arg drivetrain constructor.

    // ── Test constants ────────────────────────────────────────────────────────
    /** How much a single bumper or dpad press shifts a servo position. */
    private static final double SERVO_NUDGE_STEP = 0.02;
    /** Launcher RPM change per X/B press. */
    private static final double RPM_NUDGE_STEP = 50.0;
    /** RPM used whenever GP2 right trigger turns the flywheels on. */
    private static final double LAUNCHER_SPINUP_RPM = 1000.0;
    /** Safe centered position used during initial bring-up. */
    private static final double SERVO_CENTER_POS = 0.50;
    // ── Servo position tracking ───────────────────────────────────────────────
    private double intakeServoPos = SERVO_CENTER_POS;
    private double hoodServoPos = Mark2Launcher.clipHoodPosition(SERVO_CENTER_POS);
    private double launcherTargetRpm = LAUNCHER_SPINUP_RPM;
    private boolean launcherMotorsRunning = false;

    // ── Safety gate ───────────────────────────────────────────────────────────
    private boolean confirmed = false;

    // ── dt timing ─────────────────────────────────────────────────────────────
    private long lastNs = 0;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevLB2, prevRB2;
    private boolean prevX2, prevB2, prevRT2, prevLT2, prevY2;
    private boolean prevDpadUp2, prevDpadDown2;
    private boolean prevStartGp1 = false;
    private boolean prevRB1      = false;   // GP1 right bumper — alliance flip toggle
    private boolean prevRT1      = false;   // GP1 right trigger — field-centric toggle
    private boolean prevBack1    = false;   // GP1 back button  — re-zero field-centric heading


    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);       // no Pinpoint / IMU
        intake = new Mark2Intake(hardwareMap);
        launcher = new Mark2Launcher(hardwareMap, true);     // true = motors + servos
        launchSequence = new Mark2LaunchSequence(launcher, intake);

        // Initialize intake servos to safe centered positions
        intake.setServoPosition(SERVO_CENTER_POS);
        intakeServoPos = SERVO_CENTER_POS;

        // Initialize launcher servos to safe centered positions
        hoodServoPos = Mark2Launcher.clipHoodPosition(SERVO_CENTER_POS);
        launcher.setHoodPosition(hoodServoPos);
        launcher.resetFeeder();
        launcher.setAimPosition(SERVO_CENTER_POS);
        launcherTargetRpm = LAUNCHER_SPINUP_RPM;
        launcherMotorsRunning = false;
        launcher.stopFlywheels();
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

        // GP1 right trigger (rising edge) — toggle field-centric driving
        boolean rt1 = gamepad1.right_trigger > 0.5;
        if (rt1 && !prevRT1) drivetrain.toggleFieldCentric();
        prevRT1 = rt1;

        // GP1 back button (rising edge) — re-zero field-centric heading reference
        boolean back1 = gamepad1.back;
        if (back1 && !prevBack1) drivetrain.resetFieldCentricHeading();
        prevBack1 = back1;

        drivetrain.driveSafe(gamepad1);

        // ── Intake motors ─────────────────────────────────────────────────────
        boolean y2 = gamepad2.y;
        if (y2 && !prevY2) {
            launchSequence.resetFeeder();
        }
        prevY2 = y2;

        if (!launchSequence.isActive()) {
            if (y2) {
                intake.PickUp();
            } else if (gamepad2.a) {
                intake.Reverse();
            } else {
                intake.Stop();
            }
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
        boolean rt2 = gamepad2.right_trigger > 0.5;
        if (rt2 && !prevRT2) {
            launcherMotorsRunning = !launcherMotorsRunning;
            if (launcherMotorsRunning) {
                launcherTargetRpm = LAUNCHER_SPINUP_RPM;
            } else {
                launcher.stopFlywheels();
                if (launchSequence.isActive()) {
                    launchSequence.cancel();
                }
            }
        }
        prevRT2 = rt2;

        boolean lt2 = gamepad2.left_trigger > 0.5;
        if (lt2 && !prevLT2) {
            launchSequence.startIfFlywheelRunning(
                    launcherMotorsRunning && launcherTargetRpm > 0.0);
        }
        prevLT2 = lt2;

        boolean x2 = gamepad2.x;
        boolean b2 = gamepad2.b;

        if (x2 && !prevX2) {
            launcherTargetRpm = Math.max(0.0, launcherTargetRpm - RPM_NUDGE_STEP);
        }

        if (b2 && !prevB2) {
            launcherTargetRpm += RPM_NUDGE_STEP;
        }

        prevX2 = x2;
        prevB2 = b2;

        // GP2 dpad up/down nudges the hood servo within Mark2Launcher's safe hood limits.
        boolean dpadUp2 = gamepad2.dpad_up;
        boolean dpadDown2 = gamepad2.dpad_down;

        if (dpadUp2 && !prevDpadUp2) {
            hoodServoPos = Mark2Launcher.clipHoodPosition(hoodServoPos + SERVO_NUDGE_STEP);
            launcher.setHoodPosition(hoodServoPos);
        }

        if (dpadDown2 && !prevDpadDown2) {
            hoodServoPos = Mark2Launcher.clipHoodPosition(hoodServoPos - SERVO_NUDGE_STEP);
            launcher.setHoodPosition(hoodServoPos);
        }

        prevDpadUp2 = dpadUp2;
        prevDpadDown2 = dpadDown2;

        if (launcherMotorsRunning) {
            launcher.setFlywheelTargetRpm(launcherTargetRpm);
        }
        launcher.updateMeasuredRpm(dt);
        launcher.setAimFromStick(gamepad2.left_stick_x, dt);
        launchSequence.update(dt);

        // ── Telemetry ─────────────────────────────────────────────────────────
        Pose2D pose = drivetrain.getPose();

        telemetry.addLine("── Drivetrain ──────────────────");
        telemetry.addData("  X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("  Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("  Heading (°)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("  Alliance flip", drivetrain.isAllianceFlipped() ? "FLIPPED (GP1 RB to restore)" : "normal  (GP1 RB to flip)");
        telemetry.addData("  Snail mode",    gamepad1.left_trigger > 0 ? "ACTIVE (60%)" : "off");
        telemetry.addData("  Pinpoint",      drivetrain.hasPinpoint() ? "CONNECTED" : "not found");
        telemetry.addData("  Field-centric", drivetrain.hasPinpoint()
                ? (drivetrain.isFieldCentric() ? "ON  (GP1 RT=disable, Back=re-zero)" : "off (GP1 RT to enable)")
                : "unavailable (no Pinpoint)");

        telemetry.addLine("── Launcher ────────────────────");
        telemetry.addData("  Mode", "DIRECT TEST");
        telemetry.addData("  Motors", launcherMotorsRunning ? "ON  (GP2 RT to off)" : "off (GP2 RT to 1000 RPM)");
        telemetry.addData("  Set RPM", "%.0f", launcherTargetRpm);
        telemetry.addData("  Target RPM", "%.0f", launcher.getTargetRpm());
        telemetry.addData("  Measured RPM", "%.0f", launcher.getMeasuredRpm());
        telemetry.addData("  Launch seq", launchSequence.getState().name());
        telemetry.addData("  Hood servo pos", "%.3f", launcher.getHoodPosition());
        telemetry.addData("  Gate servo pos", "%.3f", launcher.getFeederPosition());

        telemetry.addLine("── Intake ──────────────────────");
        telemetry.addData("  Intake servo pos", "%.3f", intakeServoPos);
        telemetry.addData("  Motor running",
                launchSequence.isRunningIntake()
                        ? "FORWARD (launch sequence)"
                        : (launchSequence.isActive()
                        ? "waiting for launch intake"
                        : (y2 ? "FORWARD" : (gamepad2.a ? "REVERSE" : "stopped"))));

        telemetry.addLine("── Turret Aim ──────────────────");
        telemetry.addData("  GP2 left stick X", "%.2f", gamepad2.left_stick_x);
        telemetry.addData("  Aim servo pos",    "%.3f", launcher.getAimPosition());

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void stop() {
        launcherMotorsRunning = false;
        if (launchSequence != null && launchSequence.isActive()) {
            launchSequence.cancel();
        }
        if (launcher != null) {
            launcher.stopFlywheels();
        }
        if (intake != null) {
            intake.Stop();
        }
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

