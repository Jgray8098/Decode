package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.InterpolatingTreeMap;
import org.firstinspires.ftc.teamcode.utility.LaunchSetpoint;


/**
 * Launcher subsystem.
 *
 * Usage (inside an OpMode):
 *
 *   // init()
 *   launcher = new Launcher(hardwareMap);
 *
 *   // loop()
 *   if (triggerShot) launcher.shoot(distanceInches);
 *   launcher.update(dt);   // MUST be called every loop
 *   if (launcher.getState() == Launcher.State.DONE) launcher.stop();
 */
public class Launcher {


    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    public enum LauncherState {
        /** Motors off, servos retracted. */
        IDLE,
        /** Motors running; waiting for measured RPM to reach the ready threshold. */
        SPINNING_UP,
        /** RPM threshold met; feeder servo (servoThree) has been triggered. */
        FEEDING,
        /** Shot complete. Call stop() or resetFeeder() to return to IDLE. */
        DONE
    }

    // -------------------------------------------------------------------------
    // Hardware names
    // -------------------------------------------------------------------------
    private static final String LAUNCHER_MOTOR_ONE_NAME   = "LauncherMotorOne";
    private static final String LAUNCHER_MOTOR_TWO_NAME   = "LauncherMotorTwo";
    private static final String LAUNCHER_SERVO_ONE_NAME   = "LauncherServoOne";
    private static final String LAUNCHER_SERVO_TWO_NAME   = "LauncherServoTwo";
    private static final String LAUNCHER_SERVO_THREE_NAME = "LauncherServoThree";

    // -------------------------------------------------------------------------
    // Servo positions  (tune!)
    // -------------------------------------------------------------------------
    /** Hood servo resting position (servos one & two) when launcher is idle. */
    private static final double HOOD_SERVO_RESET_POSITION  = 0.0;
    /** ServoThree "push" position — feeds ball into launch mechanism. */
    private static final double FEEDER_SERVO_FEED_POSITION = 1.0;
    /** ServoThree retracted / resting position. */
    private static final double FEEDER_SERVO_IDLE_POSITION = 0.0;

    /** Encoder ticks per output-shaft revolution for your GoBilda motor model. */
    private static final double MOTOR_TICKS_PER_REV = 537.7;   // 312 RPM Yellow Jacket — change to match your model

    /**
     * Free-spin RPM at the motor OUTPUT shaft at nominal voltage (12 V).
     * This equals the rated RPM printed on the GoBilda motor label.
     */
    private static final double MOTOR_FREE_RPM = 312.0;         // change to match your model

    /**
     * External gear / belt / chain ratio between the motor output shaft and
     * the launcher wheel.
     *   > 1.0  →  launcher wheel spins FASTER than the motor output shaft
     *   = 1.0  →  direct drive
     *   < 1.0  →  launcher wheel spins SLOWER (reduction)
     * Example: 24-tooth motor sprocket driving a 12-tooth wheel sprocket → 2.0
     */
    private static final double EXTERNAL_GEAR_RATIO = 1.0;

    /** Estimated free-spin RPM at the launcher wheel at nominal voltage. */
    private static final double LAUNCHER_FREE_RPM = MOTOR_FREE_RPM * EXTERNAL_GEAR_RATIO;

    /**
     * Fraction of the target RPM that counts as "at speed".
     * 0.90 = fire once the launcher is ≥ 90 % of its target RPM.
     */
    private static final double RPM_READY_FRACTION = 0.90;

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx launcherMotorOne;
    private DcMotorEx launcherMotorTwo;
    private Servo hoodPositionServo1;
    private Servo hoodPositionServo2;
    private Servo feedLauncherServo;

    // -------------------------------------------------------------------------
    // Distance → motor power setpoints
    //   Key   = distance to target (inches)
    //   Value = motor power  [0.0 – 1.0]
    //
    //   Add / adjust entries to match your robot's real-world performance.
    // -------------------------------------------------------------------------
    private final InterpolatingTreeMap powerMap = new InterpolatingTreeMap();

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    private LauncherState state = LauncherState.IDLE;
    private double targetPower  = 0.0;
    private double targetRpm    = 0.0;
    private double measuredRpm  = 0.0;

    // Fallback tick-based velocity
    private int lastPosOne = 0;
    private int lastPosTwo = 0;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
    public Launcher(HardwareMap hardwareMap) {
        launcherMotorOne  = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_ONE_NAME);
        launcherMotorTwo  = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_TWO_NAME);
        hoodPositionServo1 = hardwareMap.servo.get(LAUNCHER_SERVO_ONE_NAME);
        hoodPositionServo2 = hardwareMap.servo.get(LAUNCHER_SERVO_TWO_NAME);
        feedLauncherServo = hardwareMap.servo.get(LAUNCHER_SERVO_THREE_NAME);

        launcherMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodPositionServo2.setDirection(Servo.Direction.REVERSE);

        launcherMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastPosOne = launcherMotorOne.getCurrentPosition();
        lastPosTwo = launcherMotorTwo.getCurrentPosition();

        feedLauncherServo.setPosition(FEEDER_SERVO_IDLE_POSITION);

        // --- Distance (inches) → LaunchSetpoint (rpm, hoodPosition)  (tune!) ---
        //   rpm          = target launcher-wheel RPM for this shot distance
        //   hoodPosition = servo position [0.0–1.0] for servos one & two
        //                  (0.0 = lowest angle, 1.0 = highest angle)
        powerMap.put(24.0,  new LaunchSetpoint(2000.0, 0.10));
        powerMap.put(48.0,  new LaunchSetpoint(2500.0, 0.20));
        powerMap.put(72.0,  new LaunchSetpoint(3000.0, 0.30));
        powerMap.put(96.0,  new LaunchSetpoint(3800.0, 0.42));
        powerMap.put(120.0, new LaunchSetpoint(4500.0, 0.55));
        powerMap.put(144.0, new LaunchSetpoint(5400.0, 0.70));
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Begin a shot sequence for the given distance.
     * The motors will spin up and servoThree will fire automatically once the
     * launcher reaches speed.  Call {@link #update(double)} every loop iteration.
     *
     * <p>Calling shoot() while a shot is already in progress is a no-op;
     * call {@link #stop()} first to reset.</p>
     *
     * @param distanceFromTargetInches distance to the target in inches.
     */
    public void shoot(double distanceFromTargetInches) {
        if (state != LauncherState.IDLE && state != LauncherState.DONE) return;

        LaunchSetpoint setpoint = powerMap.get(distanceFromTargetInches);
        targetRpm   = setpoint.rpm;
        // Derive open-loop power proportionally from the target RPM.
        targetPower = Math.min(targetRpm / LAUNCHER_FREE_RPM, 1.0);

        launcherMotorOne.setPower(targetPower);
        launcherMotorTwo.setPower(targetPower);

        // Set hood angle from the interpolated setpoint
        hoodPositionServo1.setPosition(setpoint.hoodPosition);
        hoodPositionServo2.setPosition(setpoint.hoodPosition);
        feedLauncherServo.setPosition(FEEDER_SERVO_IDLE_POSITION); // feeder stays back until at speed

        state = LauncherState.SPINNING_UP;
    }

    /**
     * Drive the launcher state machine.  <b>Must be called every OpMode loop iteration.</b>
     *
     * @param dtSec seconds elapsed since the previous loop call.
     *              Compute with {@code (System.nanoTime() - lastNs) / 1e9}.
     */
    public void update(double dtSec) {
        switch (state) {

            case SPINNING_UP: {
                // Measure average RPM across both motors
                double tpsOne = estimateTps(launcherMotorOne, true,  dtSec);
                double tpsTwo = estimateTps(launcherMotorTwo, false, dtSec);
                measuredRpm = (tpsToRpm(tpsOne) + tpsToRpm(tpsTwo)) / 2.0;

                if (measuredRpm >= targetRpm * RPM_READY_FRACTION) {
                    // Launcher is at speed — trigger the feeder servo
                    feedLauncherServo.setPosition(FEEDER_SERVO_FEED_POSITION);
                    state = LauncherState.FEEDING;
                }
                break;
            }

            case FEEDING:
                // Feeder servo command has been sent; mark the shot as complete.
                // The motors keep running so the caller can decide when to stop.
                state = LauncherState.DONE;
                break;

            case DONE:
            case IDLE:
            default:
                break;
        }
    }

    /**
     * Stop motors, retract all servos, and return to {@link LauncherState#IDLE}.
     * Call this after the shot is confirmed complete (state == DONE).
     */
    public void stop() {
        launcherMotorOne.setPower(0);
        launcherMotorTwo.setPower(0);
        hoodPositionServo1.setPosition(HOOD_SERVO_RESET_POSITION);
        hoodPositionServo2.setPosition(HOOD_SERVO_RESET_POSITION);
        feedLauncherServo.setPosition(FEEDER_SERVO_IDLE_POSITION);
        state = LauncherState.IDLE;
        targetPower = 0.0;
        targetRpm   = 0.0;
        measuredRpm = 0.0;
    }

    /**
     * Retract the feeder servo and return to {@link LauncherState#IDLE} without
     * stopping the launch motors.  Useful when you want to fire multiple balls
     * in quick succession by calling {@link #shoot(double)} again immediately.
     */
    public void resetFeeder() {
        feedLauncherServo.setPosition(FEEDER_SERVO_IDLE_POSITION);
        state = LauncherState.IDLE;
    }

    // -------------------------------------------------------------------------
    // Telemetry / state getters
    // -------------------------------------------------------------------------
    public LauncherState getState()        { return state; }
    public double getMeasuredRpm()  { return measuredRpm; }
    public double getTargetRpm()    { return targetRpm; }
    public double getTargetPower()  { return targetPower; }
    /** Returns {@code true} once the launcher has reached the ready RPM threshold. */
    public boolean isAtSpeed()      { return measuredRpm >= targetRpm * RPM_READY_FRACTION; }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    /**
     * Estimate motor shaft velocity in ticks/sec.
     * Prefers the SDK {@code getVelocity()} reading; falls back to a
     * tick-delta calculation if the control hub returns zero.
     */
    private double estimateTps(DcMotorEx motor, boolean isMotorOne, double dtSec) {
        double v = motor.getVelocity();
        if (Math.abs(v) > 1e-3) return Math.abs(v);

        // Fallback: manual tick delta
        int pos    = motor.getCurrentPosition();
        int last   = isMotorOne ? lastPosOne : lastPosTwo;
        int dTicks = pos - last;
        if (isMotorOne) lastPosOne = pos; else lastPosTwo = pos;
        return Math.abs(dTicks / Math.max(dtSec, 1e-4));
    }

    /** Convert GoBilda output-shaft ticks/sec → launcher-wheel RPM. */
    private static double tpsToRpm(double tps) {
        // tps is at the motor OUTPUT shaft (GoBilda encoder is post-gearbox).
        // Convert to output-shaft RPM, then scale by the external gear ratio.
        return (tps * 60.0 / MOTOR_TICKS_PER_REV) * EXTERNAL_GEAR_RATIO;
    }
}
