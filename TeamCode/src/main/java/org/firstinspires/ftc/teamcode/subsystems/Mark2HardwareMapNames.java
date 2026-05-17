package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Centralized hardware-map configuration names for all Mark2 subsystems.
 *
 * <p>Every string in this file must exactly match the device name as it is
 * configured in the Robot Controller app.  Changing a name here affects all
 * subsystems simultaneously, so no subsystem-level string literals need to be
 * updated individually.</p>
 *
 * <p>Usage in a subsystem:</p>
 * <pre>
 *   import static org.firstinspires.ftc.teamcode.subsystems.Mark2HardwareMapNames.*;
 *
 *   motor = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);
 * </pre>
 */
public final class Mark2HardwareMapNames {

    private Mark2HardwareMapNames() {
        // Utility class — do not instantiate
    }

    // ── Drivetrain ────────────────────────────────────────────────────────────
    public static final String FRONT_LEFT_MOTOR  = "LeftFrontMotor";
    public static final String FRONT_RIGHT_MOTOR = "RightFrontMotor";
    public static final String REAR_LEFT_MOTOR   = "LeftBackMotor";
    public static final String REAR_RIGHT_MOTOR  = "RightBackMotor";

    /** Built-in IMU on the Control Hub / Expansion Hub. */
    public static final String IMU_SENSOR        = "imu";

    // ── Odometry (GoBilda Pinpoint) ───────────────────────────────────────────
    public static final String PINPOINT          = "pinpoint";

    // ── Intake ────────────────────────────────────────────────────────────────
    public static final String INTAKE_MOTOR_ONE  = "IntakeMotorOne";
    public static final String INTAKE_MOTOR_TWO  = "IntakeMotorTwo";
    public static final String INTAKE_SERVO      = "IntakeServo";

    // ── Launcher ──────────────────────────────────────────────────────────────
    public static final String LAUNCHER_MOTOR_ONE  = "LauncherMotorOne";
    public static final String LAUNCHER_MOTOR_TWO  = "LauncherMotorTwo";
    /** Hood angle servo #1 (ServoOne). */
    public static final String LAUNCHER_SERVO_ONE   = "LauncherServoOne";
    /** Hood angle servo #2 (ServoTwo) — wired in reverse in firmware. */
    public static final String LAUNCHER_SERVO_TWO   = "LauncherServoTwo";
    /** Feeder / ball-push servo (ServoThree). */
    public static final String LAUNCHER_SERVO_THREE = "LauncherServoThree";
}


