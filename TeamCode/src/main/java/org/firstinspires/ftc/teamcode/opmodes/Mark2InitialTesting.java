package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Mark2InitialTesting
 *
 * A safe, low-power OpMode for first-time bench and field testing of the three
 * main subsystems.  Nothing runs at full power; every subsystem can be exercised
 * independently to verify wiring, motor direction, and servo positions.
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
    private Launcher  launcher;
    private Intake    intake;
    private Drivetrain drivetrain;

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

    // ── Servo position state (commanded values; shown in telemetry) ───────────
    private double hoodServoPos   = 0.0;
    private double feederServoPos = 0.0;
    private double intakeServoPos = 0.0;

    // ── Button edge-detection ─────────────────────────────────────────────────
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLB2, prevRB2;

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        I2cDeviceSynchSimple pinpointClient =
                hardwareMap.get(I2cDeviceSynchSimple.class, PINPOINT_DEVICE_NAME);

        drivetrain = new Drivetrain(hardwareMap, pinpointClient);
        intake     = new Intake(hardwareMap);
        launcher   = new Launcher(hardwareMap);

        // Push initial servo positions so the telemetry values are valid from the start
        launcher.setHoodPosition(hoodServoPos);
        launcher.setFeederPosition(feederServoPos);
        intake.setServoPosition(intakeServoPos);

        telemetry.addLine("══ Mark2InitialTesting ready ══");
        telemetry.addLine("GP1 sticks  →  drive (40 % power)");
        telemetry.addLine("GP2 Y / A   →  intake fwd / rev");
        telemetry.addLine("GP2 LB / RB →  intake servo − / +");
        telemetry.addLine("GP2 R-trigger (hold)  →  launcher spin (30 %)");
        telemetry.addLine("GP2 Dpad Up/Down  →  hood servo +/−");
        telemetry.addLine("GP2 Dpad Right/Left  →  feeder servo +/−");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {

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
}

