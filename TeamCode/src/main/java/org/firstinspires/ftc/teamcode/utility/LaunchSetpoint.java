package org.firstinspires.ftc.teamcode.utility;

/**
 * Immutable data class that pairs the two values the launcher needs for a
 * given shot distance: wheel RPM and hood servo angle.
 */
public class LaunchSetpoint {

    /** Target launcher-wheel RPM for this distance. */
    public final double rpm;

    /**
     * Hood servo position [0.0 – 1.0] shared by the two hood servos.
     * Higher values typically angle the hood upward for longer shots.
     */
    public final double hoodPosition;

    public LaunchSetpoint(double rpm, double hoodPosition) {
        this.rpm          = rpm;
        this.hoodPosition = hoodPosition;
    }

    /** Linear interpolation between two setpoints at parameter t ∈ [0, 1]. */
    public static LaunchSetpoint lerp(LaunchSetpoint a, LaunchSetpoint b, double t) {
        return new LaunchSetpoint(
            a.rpm          + t * (b.rpm          - a.rpm),
            a.hoodPosition + t * (b.hoodPosition - a.hoodPosition)
        );
    }
}

