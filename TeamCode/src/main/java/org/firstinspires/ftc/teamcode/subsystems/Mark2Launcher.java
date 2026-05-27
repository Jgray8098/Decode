package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.InterpolatingTreeMap;
import org.firstinspires.ftc.teamcode.utility.LaunchSetpoint;

import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.LAUNCHER_MOTOR_ONE;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.LAUNCHER_MOTOR_TWO;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.GATE_SERVO_LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.GATE_SERVO_RIGHT;


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
public class Mark2Launcher {


    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    public enum LauncherState {
        /** Motors off, servos retracted. */
        IDLE,
        /** Motors running; waiting for measured RPM to reach the ready threshold. */
        SPINNING_UP,
        /** RPM threshold met; feeder gate servos have been triggered. */
        FEEDING,
        /** Shot complete. Call stop() or resetFeeder() to return to IDLE. */
        DONE
    }

    // Hardware-map names live in Mark2HardwareMapNames — imported as static above.

    // -------------------------------------------------------------------------
    // Servo positions  (tune!)
    // -------------------------------------------------------------------------
    /** Hood servo resting position when launcher is idle. */
    private static final double HOOD_SERVO_RESET_POSITION  = 0.0;
    /** Gate servos "push" position — feeds ball into launch mechanism. */
    private static final double FEEDER_SERVO_FEED_POSITION = 0.24;
    /** Gate servos retracted / resting position. */
    private static final double FEEDER_SERVO_IDLE_POSITION = 0.58;


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
    private Servo hoodPositionServo;
    private Servo gateServoLeft;
    private Servo gateServoRight;

    /** True when all three servos were successfully initialised. False = motors only. */
    private final boolean hasServos;

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

    /**
     * Full constructor — requires all motors AND servos to be present in
     * the hardware configuration.  Use this once all hardware is installed.
     */
    public Mark2Launcher(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /**
     * Partial constructor for staged hardware bring-up.
     *
     * @param hardwareMap the OpMode hardware map
     * @param hasServos   {@code true}  = full init (motors + servos)<br>
     *                    {@code false} = motors only; servo fields are left null
     *                    and every servo call in this class is skipped safely.
     */
    public Mark2Launcher(HardwareMap hardwareMap, boolean hasServos) {
        this.hasServos = hasServos;

        launcherMotorOne = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_ONE);
        launcherMotorTwo = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR_TWO);

        if (hasServos) {
            hoodPositionServo = hardwareMap.servo.get(HOOD_SERVO);
            gateServoLeft     = hardwareMap.servo.get(GATE_SERVO_LEFT);
            gateServoRight    = hardwareMap.servo.get(GATE_SERVO_RIGHT);

            // Servos are intentionally NOT commanded here.
            // No servo position is assumed safe until the driver confirms safety
            // in Mark2InitialTesting.init_loop().
        }

        launcherMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastPosOne = launcherMotorOne.getCurrentPosition();
        lastPosTwo = launcherMotorTwo.getCurrentPosition();


        // --- Distance (inches) → LaunchSetpoint (rpm, hoodPosition)  (tune!) ---
        //   rpm          = target launcher-wheel RPM for this shot distance
        //   hoodPosition = servo position [0.0–1.0] for hood servo
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

    public void shoot(double distanceFromTargetInches) {
        if (state != LauncherState.IDLE && state != LauncherState.DONE) return;

        LaunchSetpoint setpoint = powerMap.get(distanceFromTargetInches);
        targetRpm   = setpoint.rpm;
        targetPower = Math.min(targetRpm / LAUNCHER_FREE_RPM, 1.0);

        launcherMotorOne.setPower(targetPower);
        launcherMotorTwo.setPower(targetPower);

        if (hasServos) {
            hoodPositionServo.setPosition(setpoint.hoodPosition);
            setFeederPosition(FEEDER_SERVO_IDLE_POSITION);
        }

        state = LauncherState.SPINNING_UP;
    }

    public void update(double dtSec) {
        switch (state) {

            case SPINNING_UP: {
                double tpsOne = estimateTps(launcherMotorOne, true,  dtSec);
                double tpsTwo = estimateTps(launcherMotorTwo, false, dtSec);
                measuredRpm = (tpsToRpm(tpsOne) + tpsToRpm(tpsTwo)) / 2.0;

                if (measuredRpm >= targetRpm * RPM_READY_FRACTION) {
                    if (hasServos) setFeederPosition(FEEDER_SERVO_FEED_POSITION);
                    state = LauncherState.FEEDING;
                }
                break;
            }

            case FEEDING:
                state = LauncherState.DONE;
                break;

            case DONE:
            case IDLE:
            default:
                break;
        }
    }

    public void stop() {
        launcherMotorOne.setPower(0);
        launcherMotorTwo.setPower(0);
        if (hasServos) {
            hoodPositionServo.setPosition(HOOD_SERVO_RESET_POSITION);
            setFeederPosition(FEEDER_SERVO_IDLE_POSITION);
        }
        state = LauncherState.IDLE;
        targetPower = 0.0;
        targetRpm   = 0.0;
        measuredRpm = 0.0;
    }

    public void resetFeeder() {
        if (hasServos) setFeederPosition(FEEDER_SERVO_IDLE_POSITION);
        state = LauncherState.IDLE;
    }

    public void testSpinMotors(double power) {
        launcherMotorOne.setPower(power);
        launcherMotorTwo.setPower(power);
    }

    public void setHoodPosition(double position) {
        if (!hasServos) return;
        hoodPositionServo.setPosition(position);
    }

    public void setFeederPosition(double position) {
        if (!hasServos) return;
        gateServoLeft.setPosition(position);
        gateServoRight.setPosition(position);
    }

    public LauncherState getState()        { return state; }
    public double getMeasuredRpm()  { return measuredRpm; }
    public double getTargetRpm()    { return targetRpm; }
    public double getTargetPower()  { return targetPower; }
    /** Last position sent to hood servo. Returns NaN if servos not installed. */
    public double getHoodPosition()   { return hasServos ? hoodPositionServo.getPosition() : Double.NaN; }
    /** Last position sent to the feeder gate servos. Returns NaN if servos not installed. */
    public double getFeederPosition() { return hasServos ? gateServoLeft.getPosition()  : Double.NaN; }
    /** Returns {@code true} if servos are present and wired. */
    public boolean hasServos()        { return hasServos; }
    /** Returns {@code true} once the launcher has reached the ready RPM threshold. */
    public boolean isAtSpeed()      { return measuredRpm >= targetRpm * RPM_READY_FRACTION; }

    private double estimateTps(DcMotorEx motor, boolean isMotorOne, double dtSec) {
        double v = motor.getVelocity();
        if (Math.abs(v) > 1e-3) return Math.abs(v);

        int pos    = motor.getCurrentPosition();
        int last   = isMotorOne ? lastPosOne : lastPosTwo;
        int dTicks = pos - last;
        if (isMotorOne) lastPosOne = pos; else lastPosTwo = pos;
        return Math.abs(dTicks / Math.max(dtSec, 1e-4));
    }

    private static double tpsToRpm(double tps) {
        return (tps * 60.0 / MOTOR_TICKS_PER_REV) * EXTERNAL_GEAR_RATIO;
    }
}
