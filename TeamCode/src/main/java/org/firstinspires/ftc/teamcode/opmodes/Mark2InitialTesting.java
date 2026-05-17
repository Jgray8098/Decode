package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

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
 * A safe, low-power OpMode for first-time bench and field testing of the three
 * main subsystems.  Nothing runs at full power; every subsystem can be exercised
 * independently to verify wiring, motor direction, and servo positions.
 *
 * ── STARTUP SAFETY ───────────────────────────────────────────────────────────
 *  On INIT the subsystem constructors are called.  The Launcher constructor
 *  moves the feeder servo to its idle position (0.0); all other servos are
 *  intentionally left at their physical resting position and are NOT commanded
 *  until the driver explicitly confirms safety.
 *
 *  After pressing INIT on the Driver Station:
 *    1. Physically inspect the robot — confirm every servo is within its
 *       mechanical range and no linkage is bound or under stress.
 *    2. The telemetry screen lists which servos were moved and which were not.
 *    3. Press GP1 START to confirm and unlock the OpMode for normal operation.
 *  The OpMode is completely inert until that confirmation is received.
 *
 * ── CONTROL MAP ──────────────────────────────────────────────────────────────
 *
 *  Gamepad 1  (Drivetrain — capped at 40 % power)
 *  ┌─────────────────────────────────────────────────────┐
 *  │  Left  stick Y / X  →  translate (forward / strafe) │
 *  │  Right stick X      →  rotate                       │
 *  └─────────────────────────────────────────────────────┘
 *
 *  Gamepad 2  (Launcher + Intake)
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  INTAKE                                                                 │
 *  │    Y  (hold)        →  intake forward (PickUp)                          │
 *  │    A  (hold)        →  intake reverse                                   │
 *  │    (neither)        →  intake stopped                                   │
 *  │    LB  press        →  intake servo position  − 0.02  (nudge down)      │
 *  │    RB  press        →  intake servo position  + 0.02  (nudge up)        │
 *  │                                                                         │
 *  │  LAUNCHER                                                               │
 *  │    Right trigger > 0.5  (hold)  →  spin motors at 30 %                 │
 *  │    (trigger released)           →  motors stop                          │
 *  │    Dpad Up    press  →  hood servo position   + 0.02  (nudge up)        │
 *  │    Dpad Down  press  →  hood servo position   − 0.02  (nudge down)      │
 *  │    Dpad Right press  →  feeder servo position + 0.02  (nudge forward)   │
 *  │    Dpad Left  press  →  feeder servo position − 0.02  (nudge back)      │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 * ── TUNING WORKFLOW ──────────────────────────────────────────────────────────
 *  1. Use Dpad Up/Down to find the correct hood servo positions for each
 *     distance, then copy the displayed values into InterpolatingTreeMap in
 *     Launcher.java.
 *  2. Use Dpad Left/Right to find FEEDER_SERVO_FEED_POSITION and
 *     FEEDER_SERVO_IDLE_POSITION.
 *  3. Use LB/RB to find the three INTAKE_SERVO_* positions.
 *  4. Confirm motor directions with Y / A / right trigger before full testing.
 */
@TeleOp(name = "Mark2InitialTesting", group = "Test")
public class Mark2InitialTesting extends OpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private Mark2Launcher launcher;
    private Mark2Intake intake;
    private Mark2Drivetrain drivetrain;

    /**
     * Hardware-map name of the GoBILDA Pinpoint I²C device.
     * Must match exactly what is configured in the robot controller app.
     */
    private static final String PINPOINT_DEVICE_NAME = "pinpoint";

    // ── Test constants ────────────────────────────────────────────────────────
    /** Motor power used when the right trigger spins the launcher during testing. */
    private static final double LAUNCHER_TEST_POWER = 0.30;
    /** How much a single dpad or bumper press shifts a servo position. */
    private static final double SERVO_NUDGE_STEP    = 0.02;

    // ── Servo position tracking (last value commanded; shown in telemetry) ────
    // Initialised from the subsystem getters after construction so these always
    // reflect what the hardware was actually told — not an assumed default.
    private double hoodServoPos   = 0.0;
    private double feederServoPos = 0.0;
    private double intakeServoPos = 0.0;

    // ── Safety gate ───────────────────────────────────────────────────────────
    /**
     * Set to {@code true} only after the driver presses GP1 START in
     * {@code init_loop()}.  The {@code loop()} method is completely inert
     * until this flag is raised.
     */
    private boolean confirmed = false;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLB2, prevRB2;
    private boolean prevStartGp1 = false;

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        I2cDeviceSynchSimple pinpointClient =
                hardwareMap.get(I2cDeviceSynchSimple.class, PINPOINT_DEVICE_NAME);

        drivetrain = new Mark2Drivetrain(hardwareMap, pinpointClient);
        intake = new Mark2Intake(hardwareMap);
        launcher = new Mark2Launcher(hardwareMap);

        // ── DO NOT call setPosition() here. ──────────────────────────────────
        // The Launcher constructor already commands the feeder servo to its idle
        // position (0.0) — intentional and documented.  The hood servos and the
        // intake servo are deliberately left at their physical resting position
        // until the driver confirms safety in init_loop().
        //
        // Read back what the constructors actually commanded so our tracking
        // variables are in sync with the hardware from the very first telemetry
        // update.  getPosition() returns NaN for servos that have not yet been
        // commanded; nanToZero() converts that to 0.0 for safe display/use.
        hoodServoPos   = nanToZero(launcher.getHoodPosition());
        feederServoPos = nanToZero(launcher.getFeederPosition());
        intakeServoPos = nanToZero(intake.getServoPosition());
    }

    // ─────────────────────────────────────────────────────────────────────────
    /**
     * Called repeatedly between INIT and START.
     *
     * <p>Displays the current servo state — distinguishing between servos that
     * were commanded by a constructor and servos that were intentionally left
     * untouched — and waits for the driver to physically inspect the robot and
     * press GP1 START to confirm everything is safe.
     *
     * <p>Nothing moves while this method is running except what the constructors
     * already commanded during {@code init()}.
     */
    @Override
    public void init_loop() {
        telemetry.addLine("══ SAFETY CHECK — inspect robot before confirming ══");
        telemetry.addLine("");

        telemetry.addLine("Servo status after INIT:");
        telemetry.addData("  Hood servo (1 & 2)",
                Double.isNaN(launcher.getHoodPosition())
                        ? "NOT commanded — physically untouched"
                        : String.format(Locale.US, "commanded to %.3f by constructor", launcher.getHoodPosition()));
        telemetry.addData("  Feeder servo",
                Double.isNaN(launcher.getFeederPosition())
                        ? "NOT commanded — physically untouched"
                        : String.format(Locale.US, "commanded to %.3f by constructor", launcher.getFeederPosition()));
        telemetry.addData("  Intake servo",
                Double.isNaN(intake.getServoPosition())
                        ? "NOT commanded — physically untouched"
                        : String.format(Locale.US, "commanded to %.3f by constructor", intake.getServoPosition()));

        telemetry.addLine("");
        telemetry.addLine("Inspect checklist:");
        telemetry.addLine("  [ ] All servos are within their mechanical range");
        telemetry.addLine("  [ ] No linkage is bound or under unexpected stress");
        telemetry.addLine("  [ ] Motors can spin freely");
        telemetry.addLine("");

        if (confirmed) {
            telemetry.addLine("✓  CONFIRMED — press START on Driver Station to begin");
        } else {
            telemetry.addLine(">> Press GP1 START to confirm and unlock operation <<");
        }

        // Rising-edge detect on GP1 start
        boolean startNow = gamepad1.start;
        if (startNow && !prevStartGp1) {
            confirmed = true;
        }
        prevStartGp1 = startNow;

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {

        // ── Safety gate ───────────────────────────────────────────────────────
        // If the driver has not confirmed via init_loop(), refuse to do anything.
        // This can also trigger if loop() is somehow called before init_loop()
        // has been acknowledged (shouldn't happen in normal FTC flow, but safe).
        if (!confirmed) {
            telemetry.addLine("⚠  Not confirmed — return to INIT and press GP1 START");
            telemetry.update();
            return;
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        drivetrain.driveSafe(gamepad1);

        // ── Intake motors ─────────────────────────────────────────────────────
        if      (gamepad2.y) intake.PickUp();
        else if (gamepad2.a) intake.Reverse();
        else                 intake.Stop();

        // ── Intake servo nudge (LB / RB, one step per press) ─────────────────
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

        // ── Launcher motors (hold right trigger to spin) ──────────────────────
        if (gamepad2.right_trigger > 0.5) {
            launcher.testSpinMotors(LAUNCHER_TEST_POWER);
        } else {
            // Stop motors only — leave servos where they are for position inspection
            launcher.testSpinMotors(0.0);
        }

        // ── Hood servo nudge (Dpad Up / Down, one step per press) ────────────
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

        // ── Feeder servo nudge (Dpad Right / Left, one step per press) ───────
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

        prevDpadUp    = dpadUp;
        prevDpadDown  = dpadDown;
        prevDpadRight = dpadRight;
        prevDpadLeft  = dpadLeft;

        // ── Telemetry ─────────────────────────────────────────────────────────
        Pose2D pose = drivetrain.getPose();

        telemetry.addLine("── Drivetrain ──────────────────");
        telemetry.addData("  X (in)",      "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("  Y (in)",      "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("  Heading (°)", "%.1f", pose.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("── Launcher ────────────────────");
        telemetry.addData("  Hood servo pos",   "%.3f", hoodServoPos);
        telemetry.addData("  Feeder servo pos", "%.3f", feederServoPos);
        telemetry.addData("  Motor power cmd",  "%.2f", LAUNCHER_TEST_POWER);
        telemetry.addData("  Measured RPM",     "%.0f", launcher.getMeasuredRpm());

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

    /**
     * Return {@code value} unchanged, or {@code 0.0} if it is NaN.
     *
     * <p>FTC's {@code Servo.getPosition()} returns {@code Double.NaN} (SDK
     * constant {@code Servo.NO_INSTRUCTION}) for any servo that has not yet
     * received a {@code setPosition()} call.  Using NaN as a starting point
     * for nudge arithmetic would propagate NaN to every subsequent command,
     * so we map it to 0.0 — the conventional safe/home position — instead.</p>
     */
    private static double nanToZero(double value) {
        return Double.isNaN(value) ? 0.0 : value;
    }
}

