package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.control.Mark2AutoLauncherController;
import org.firstinspires.ftc.teamcode.control.Mark2AutoLauncherController.LauncherState;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mark2Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames;

/**
 * Abstract base for all Mark2 autonomous OpModes.
 *
 * <p>Provides blocking {@link #navigate} and {@link #shootFrom} helpers that
 * drive the robot using the Pinpoint odometry-backed PID in {@link Mark2Drivetrain}
 * and fire the robot using the {@link Mark2AutoLauncherController} state machine.  Subclasses
 * only need to implement {@link #setup()} (hardware/pose init) and
 * {@link #runPath()} (the autonomous sequence).</p>
 *
 * <p>No vision, no Limelight, no PedroPathing — all position data comes from
 * the GoBilda Pinpoint via the Drivetrain subsystem's IMU/odometry.</p>
 */
public abstract class Mark2AutoBase extends LinearOpMode {

    // ── Hardware name — see Mark2HardwareMapNames ─────────────────────────────

    // ── Tuning ────────────────────────────────────────────────────────────────
    /** Maximum time to wait for a shot to complete before continuing. */
    protected static final double SHOOT_TIMEOUT_S      = 6.0;
    /** Default time to run intake in reverse after picking up game elements. */
    protected static final double OUTTAKE_REVERSE_S    = 1.0;

    // ── Subsystems ────────────────────────────────────────────────────────────
    protected Mark2Drivetrain mark2Drivetrain;
    protected Mark2Intake mark2Intake;
    protected Mark2Launcher mark2Launcher;
    protected Mark2AutoLauncherController mark2AutoLauncher;

    // ── Internal state ────────────────────────────────────────────────────────
    /** Label shown in telemetry while the robot works through its path. */
    protected String currentPhase = "init";

    /** Timestamp of the most recent {@link #nextDt()} call (nanoseconds). */
    private long lastNs;

    /** Remaining seconds of non-blocking intake reverse (0 = not active). */
    private double reverseRemainingS = 0.0;

    // =========================================================================
    // LinearOpMode entry point
    // =========================================================================

    @Override
    public final void runOpMode() {
        setup();

        waitForStart();

        // Re-stamp time so the first dt is ~0 rather than including wait time.
        lastNs = System.nanoTime();

        if (!isStopRequested()) {
            runPath();
        }

        // Safety: always stop intake and launcher when the OpMode ends.
        mark2Intake.Stop();
        mark2AutoLauncher.stop();
    }

    // =========================================================================
    // Abstract interface — implemented by each concrete auto class
    // =========================================================================

    /**
     * Called once before {@code waitForStart()}.
     * Initialize subsystems, set the robot's starting pose via
     * {@link #initSubsystems(double, double, double)}, and add any
     * pre-match telemetry lines.
     */
    protected abstract void setup();

    /**
     * The full autonomous path sequence.  Runs immediately after the match
     * starts.  Use the blocking helpers ({@link #navigate}, {@link #shootFrom})
     * to build the sequence step by step.
     */
    protected abstract void runPath();

    // =========================================================================
    // Subsystem initialization helper
    // =========================================================================

    /**
     * Instantiate all three subsystems and seed the Pinpoint odometry with
     * the robot's known starting position on the field.
     *
     * @param startX         Starting field X (inches).
     * @param startY         Starting field Y (inches).
     * @param startRotDeg    Starting heading (degrees, CCW positive).
     */
    protected void initSubsystems(double startX, double startY, double startRotDeg) {
        //I2cDeviceSynchSimple pinpointClient =
        //        hardwareMap.get(I2cDeviceSynchSimple.class, Mark2HardwareMapNames.PINPOINT);
        //mark2Drivetrain = new Mark2Drivetrain(hardwareMap, pinpointClient);
        mark2Drivetrain.setStartingPose(startX, startY, startRotDeg);

        mark2Intake = new Mark2Intake(hardwareMap);
        mark2Launcher = new Mark2Launcher(hardwareMap);
        mark2AutoLauncher = new Mark2AutoLauncherController(mark2Launcher, mark2Intake);
    }

    // =========================================================================
    // Navigation helpers
    // =========================================================================

    /**
     * Drive to a field pose (blocking).
     * The robot stays at the last commanded motor power until it arrives;
     * any active intake-reverse timer continues to tick during the drive.
     *
     * @param x      Target field X position (inches).
     * @param y      Target field Y position (inches).
     * @param rotDeg Target heading (degrees, CCW positive, field frame).
     */
    protected void navigate(double x, double y, double rotDeg) {
        currentPhase = String.format("Nav→(%.0f,%.0f,%.0f°)", x, y, rotDeg);
        while (opModeIsActive()) {
            double dt = nextDt();
            tickReverse(dt);
            mark2AutoLauncher.update(dt);   // no-op when launcher is IDLE
            if (mark2Drivetrain.driveToPosition(x, y, rotDeg, dt)) break;
            postTelemetry();
        }
        postTelemetry();
    }

    /**
     * Start the intake, then navigate to the given position.
     * The intake keeps spinning after this call returns; call
     * {@link #stopIntake()} or {@link #startReverseFor(double)} when done.
     */
    protected void navigateWithIntake(double x, double y, double rotDeg) {
        mark2Intake.PickUp();
        navigate(x, y, rotDeg);
    }

    /** Immediately stop the intake and cancel any active reverse timer. */
    protected void stopIntake() {
        reverseRemainingS = 0.0;
        mark2Intake.Stop();
    }

    // =========================================================================
    // Shooting helper
    // =========================================================================

    /**
     * Spin up the launcher for the given distance, wait until it is at speed,
     * trigger the feeder servo, then stop everything.  Blocks for up to
     * {@value #SHOOT_TIMEOUT_S} seconds.
     *
     * @param distanceInches Distance from the robot to the target (inches).
     *                       Interpolated against the setpoint table in Launcher.
     */
    protected void shootFrom(double distanceInches) {
        currentPhase = String.format("Shoot (%.0f in)", distanceInches);
        mark2AutoLauncher.shoot(distanceInches);

        double elapsed = 0.0;
        while (opModeIsActive()
                && mark2AutoLauncher.getState() != LauncherState.DONE
                && elapsed < SHOOT_TIMEOUT_S) {
            double dt = nextDt();
            elapsed += dt;
            mark2AutoLauncher.update(dt);
            postTelemetry();
        }
        mark2AutoLauncher.stop();
    }

    // =========================================================================
    // Non-blocking intake reverse
    // =========================================================================

    /**
     * Begin running the intake in reverse for {@code seconds} seconds
     * (non-blocking).  The reverse timer is ticked inside every
     * {@link #navigate(double, double, double)} call, so the reverse
     * runs concurrently with the subsequent drive leg.
     *
     * @param seconds How long to run in reverse.
     */
    protected void startReverseFor(double seconds) {
        reverseRemainingS = seconds;
        mark2Intake.Reverse();
    }

    private void tickReverse(double dt) {
        if (reverseRemainingS > 0.0) {
            reverseRemainingS -= dt;
            if (reverseRemainingS <= 0.0) {
                reverseRemainingS = 0.0;
                mark2Intake.Stop();
            }
        }
    }

    // =========================================================================
    // Internal timing
    // =========================================================================

    /** Returns elapsed time since the last call (seconds) and re-stamps the clock. */
    private double nextDt() {
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1.0e9;
        lastNs = now;
        return dt;
    }

    // =========================================================================
    // Telemetry
    // =========================================================================

    /** Post standard per-loop telemetry. */
    protected void postTelemetry() {
        Pose2D pose = mark2Drivetrain.getPose();
        telemetry.addData("Phase",       currentPhase);
        telemetry.addData("X (in)",      "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)",      "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading",     "%.1f°", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Launcher",    mark2AutoLauncher.getState());
        telemetry.addData("Target RPM",  "%.0f", mark2AutoLauncher.getTargetRpm());
        telemetry.addData("Meas. RPM",   "%.0f", mark2AutoLauncher.getMeasuredRpm());
        telemetry.addData("Rev timer",   "%.2f s", reverseRemainingS);
        telemetry.update();
    }
}

