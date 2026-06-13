package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.Nullable;

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
    public static final String INTAKE_MOTOR_ONE  = "IntakeMotorOne"; // Expansion Hub
    public static final String INTAKE_MOTOR_TWO  = "IntakeMotorTwo"; // Expansion Hub
    public static final String INTAKE_SERVO_LEFT      = "IntakeServoLeft"; //Control Hub Servo Port 0
    public static final String INTAKE_SERVO_RIGHT = "IntakeServoRight"; //Expansion Hub Servo Port 5
    public static final String BEAM_BREAK_SENSOR = "BeamBreakSensor";

    // ── Launcher ──────────────────────────────────────────────────────────────
    public static final String LAUNCHER_MOTOR_ONE  = "LauncherMotorOne"; // Right Motor, Control Hub Port 2
    public static final String LAUNCHER_MOTOR_TWO  = "LauncherMotorTwo"; // Left Motor, Control Hub Port 3

    /** Hood angle servo #1 (ServoOne). */
    public static final String HOOD_SERVO   = "HoodServo"; // Control Hub Servo Port 5

    /** Feeder / ball-push servo Left (ServoTwo) — wired in reverse in firmware. */
    public static final String GATE_SERVO_LEFT   = "GateServoLeft"; //Control Hub Servo Port 3

    /** Feeder / ball-push servo Right (ServoThree). */
    public static final String GATE_SERVO_RIGHT = "GateServoRight"; //Control Hub Servo Port 4

    /** Servos used to rotate the turret */
    public static final String AIM_SERVO_LEFT = "AimServoLeft"; //Control Hub Servo Port 1
    public static final String AIM_SERVO_RIGHT = "AimServoRight";  //Control Hub Servo Port 2
}


