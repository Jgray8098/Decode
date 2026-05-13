package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Launcher.LauncherState;

/**
 * Abstract base for all Mark2 autonomous OpModes.
 *
 * <p>Provides blocking {@link #navigate} and {@link #shootFrom} helpers that
 * drive the robot using the Pinpoint odometry-backed PID in {@link Drivetrain}
 * and fire the robot using the {@link Launcher} state machine.  Subclasses
 * only need to implement {@link #setup()} (hardware/pose init) and
 * {@link #runPath()} (the autonomous sequence).</p>
 *
 * <p>No vision, no Limelight, no PedroPathing — all position data comes from
 * the GoBilda Pinpoint via the Drivetrain subsystem's IMU/odometry.</p>
 */
public abstract class Mark2AutoBase extends LinearOpMode {

    // ── Hardware name ─────────────────────────────────────────────────────────
    /** Config name for the GoBilda Pinpoint in the robot controller. */
    protected static final String PINPOINT_NAME = "pinpoint";

    // ── Tuning ────────────────────────────────────────────────────────────────
    /** Maximum time to wait for a shot to complete before continuing. */
    protected static final double SHOOT_TIMEOUT_S      = 6.0;
    /** Default time to run intake in reverse after picking up game elements. */
    protected static final double OUTTAKE_REVERSE_S    = 1.0;

    // ── Subsystems ────────────────────────────────────────────────────────────
    protected Drivetrain drivetrain;
    protected Intake     intake;
    protected Launcher   launcher;

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
        intake.Stop();
        launcher.stop();
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
        I2cDeviceSynchSimple pinpointClient =
                hardwareMap.get(I2cDeviceSynchSimple.class, PINPOINT_NAME);
        drivetrain = new Drivetrain(hardwareMap, pinpointClient);
        drivetrain.setStartingPose(startX, startY, startRotDeg);

        intake   = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
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
            launcher.update(dt);   // no-op when launcher is IDLE
            if (drivetrain.driveToPosition(x, y, rotDeg, dt)) break;
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
        intake.PickUp();
        navigate(x, y, rotDeg);
    }

    /** Immediately stop the intake and cancel any active reverse timer. */
    protected void stopIntake() {
        reverseRemainingS = 0.0;
        intake.Stop();
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
        launcher.shoot(distanceInches);

        double elapsed = 0.0;
        while (opModeIsActive()
                && launcher.getState() != LauncherState.DONE
                && elapsed < SHOOT_TIMEOUT_S) {
            double dt = nextDt();
            elapsed += dt;
            launcher.update(dt);
            postTelemetry();
        }
        launcher.stop();
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
        intake.Reverse();
    }

    private void tickReverse(double dt) {
        if (reverseRemainingS > 0.0) {
            reverseRemainingS -= dt;
            if (reverseRemainingS <= 0.0) {
                reverseRemainingS = 0.0;
                intake.Stop();
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
        Pose2D pose = drivetrain.getPose();
        telemetry.addData("Phase",       currentPhase);
        telemetry.addData("X (in)",      "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)",      "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading",     "%.1f°", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Launcher",    launcher.getState());
        telemetry.addData("Target RPM",  "%.0f", launcher.getTargetRpm());
        telemetry.addData("Meas. RPM",   "%.0f", launcher.getMeasuredRpm());
        telemetry.addData("Rev timer",   "%.2f s", reverseRemainingS);
        telemetry.update();
    }
}

