package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Mark2ManualLauncherController;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;

import java.util.Locale;

/**
 * Mark2 primary TeleOp.
 *
 * Gamepad 1:
 *   Right stick Y / X - translate
 *   Left stick X      - rotate
 *   Left trigger      - snail mode
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
 */
@TeleOp(name = "Mark2 TeleOp", group = "Mark2")
public class Mark2TeleOp extends OpMode {

    private Mark2Drivetrain drivetrain;
    private Mark2Intake intake;
    private Mark2Launcher launcher;
    private Mark2ManualLauncherController manualLauncher;

    private long lastNs;

    private boolean prevRB1 = false;
    private boolean prevRT1 = false;   // GP1 right trigger — field-centric toggle
    private boolean prevBack1 = false; // GP1 back button  — re-zero field-centric heading

    @Override
    public void init() {
        drivetrain = new Mark2Drivetrain(hardwareMap);
        intake = new Mark2Intake(hardwareMap);
        launcher = new Mark2Launcher(hardwareMap);
        manualLauncher = new Mark2ManualLauncherController(launcher, intake);

        launcher.resetFeeder();

        lastNs = System.nanoTime();

        telemetry.addLine("Mark2 TeleOp - Initialized");
        telemetry.addLine("GP1: Drive   GP2: Intake + Launcher");
        telemetry.addLine("GP2 DUp=Close toggle DDn=Far toggle");
        telemetry.addLine("GP2 B=Launch Y=Intake+GateIdle X=Reverse");
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
        }
        prevRB1 = rb1;

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

        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("-- Driver --------------------------");
        telemetry.addData("  Alliance flip", drivetrain.isAllianceFlipped()
                ? "FLIPPED (RB to restore)" : "normal  (RB to flip)");
        telemetry.addData("  Snail mode", gamepad1.left_trigger > 0 ? "ACTIVE (60%)" : "off");
        telemetry.addData("  Field-centric", drivetrain.isFieldCentric()
                ? "ON  (RT=disable, Back=re-zero)" : "off (RT to enable)");

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
