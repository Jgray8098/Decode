package org.firstinspires.ftc.teamcode.utility;

import java.util.TreeMap;

/**
 * Maps a continuous double key (e.g. distance in inches) to a
 * {@link LaunchSetpoint} via piecewise linear interpolation between the
 * nearest known entries.  Clamps to the boundary setpoint when the key
 * falls outside the populated range.
 */
public class InterpolatingTreeMap {

    private final TreeMap<Double, LaunchSetpoint> map = new TreeMap<>();

    /** Add a known setpoint at the given distance (inches). */
    public void put(double distanceInches, LaunchSetpoint setpoint) {
        map.put(distanceInches, setpoint);
    }

    /** Convenience overload — avoids constructing a {@link LaunchSetpoint} explicitly. */
    public void put(double distanceInches, double rpm, double hoodPosition) {
        map.put(distanceInches, new LaunchSetpoint(rpm, hoodPosition));
    }

    /**
     * Return the linearly-interpolated {@link LaunchSetpoint} for the given
     * distance.  Clamps to the nearest endpoint when outside the known range.
     */
    public LaunchSetpoint get(double distanceInches) {
        if (map.isEmpty()) return new LaunchSetpoint(0.0, 0.0);
        Double lo = map.floorKey(distanceInches);
        Double hi = map.ceilingKey(distanceInches);
        if (lo == null) return map.get(hi);
        if (hi == null) return map.get(lo);
        if (lo.equals(hi)) return map.get(lo);
        double t = (distanceInches - lo) / (hi - lo);
        return LaunchSetpoint.lerp(map.get(lo), map.get(hi), t);
    }
}

