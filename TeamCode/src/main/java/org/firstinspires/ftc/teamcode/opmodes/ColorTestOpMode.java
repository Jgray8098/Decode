package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

//@TeleOp(name="Color Test (Tuning v2)")
public class ColorTestOpMode extends OpMode {

    private RevColorSensorV3 colorSensorFrontR, colorSensorBackR, colorSensorFrontL, colorSensorBackL;

    // ---- Tuning knobs (based on your measurements) ----
    private static final int GAIN = 16;

    // “Ball present / not a hole” brightness gates
    private static final float MIN_BRIGHTNESS_SUM = 0.06f;   // below this = ignore (hole/no ball)
    private static final float BALL_PRESENT_SUM   = 0.08f;   // below this = don't latch color yet

    // Score thresholds (score = gRatio - (rRatio+bRatio))
    private static final float PURPLE_SCORE_MAX = -0.20f;    // <= this => PURPLE
    private static final float GREEN_SCORE_MIN  = -0.05f;    // >= this => GREEN (your green was ~ -0.01)

    // Latching stability (loops in a row)
    private static final int REQUIRED_STABLE_LOOPS = 8;

    // Latched results
    private BallColor rightLatched = BallColor.UNKNOWN;
    private BallColor leftLatched  = BallColor.UNKNOWN;

    // Stability tracking
    private BallColor rightLast = BallColor.UNKNOWN;
    private int rightStable = 0;

    private BallColor leftLast = BallColor.UNKNOWN;
    private int leftStable = 0;

    private enum BallColor { GREEN, PURPLE, UNKNOWN }

    @Override
    public void init() {
        colorSensorFrontR = hardwareMap.get(RevColorSensorV3.class, "colorSensorFrontR");
        colorSensorBackR  = hardwareMap.get(RevColorSensorV3.class, "colorSensorBackR");
        colorSensorFrontL = hardwareMap.get(RevColorSensorV3.class, "colorSensorFrontL");
        colorSensorBackL  = hardwareMap.get(RevColorSensorV3.class, "colorSensorBackL");

        // Force LEDs ON
        setLed(colorSensorFrontR, true);
        setLed(colorSensorBackR, true);
        setLed(colorSensorFrontL, true);
        setLed(colorSensorBackL, true);

        // Set gain
        colorSensorFrontR.setGain(GAIN);
        colorSensorBackR.setGain(GAIN);
        colorSensorFrontL.setGain(GAIN);
        colorSensorBackL.setGain(GAIN);

        telemetry.addLine("Color Test (Tuning v2) Initialized");
        telemetry.addData("GAIN", GAIN);
        telemetry.addData("MIN_BRIGHTNESS_SUM", MIN_BRIGHTNESS_SUM);
        telemetry.addData("BALL_PRESENT_SUM", BALL_PRESENT_SUM);
        telemetry.addData("PURPLE_SCORE_MAX", PURPLE_SCORE_MAX);
        telemetry.addData("GREEN_SCORE_MIN", GREEN_SCORE_MIN);
        telemetry.addData("REQUIRED_STABLE_LOOPS", REQUIRED_STABLE_LOOPS);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Classify each pocket from its two sensors
        PocketReading right = readPocket(colorSensorFrontR, colorSensorBackR);
        PocketReading left  = readPocket(colorSensorFrontL, colorSensorBackL);

        // Latch with stability requirement
        rightLatched = latchWithStability(right.colorIfPresent, right.present, rightLatched,
                true /*right pocket*/);
        leftLatched  = latchWithStability(left.colorIfPresent, left.present, leftLatched,
                false /*left pocket*/);

        // ---- Telemetry: latched + debug for all 4 sensors ----
        telemetry.addData("RIGHT latched", rightLatched);
        telemetry.addData("LEFT  latched", leftLatched);

        telemetry.addLine("=== Pocket Combined ===");
        telemetry.addData("RIGHT present", right.present);
        telemetry.addData("RIGHT avgSum", "%.3f", right.avgSum);
        telemetry.addData("RIGHT avgScore", right.avgScore == null ? "NULL" : String.format("%.3f", right.avgScore));
        telemetry.addData("RIGHT instant", right.colorIfPresent);

        telemetry.addData("LEFT present", left.present);
        telemetry.addData("LEFT avgSum", "%.3f", left.avgSum);
        telemetry.addData("LEFT avgScore", left.avgScore == null ? "NULL" : String.format("%.3f", left.avgScore));
        telemetry.addData("LEFT instant", left.colorIfPresent);

        telemetry.addLine("=== Per-Sensor Debug ===");
        addSensorDebug("FrontR", colorSensorFrontR);
        addSensorDebug("BackR ", colorSensorBackR);
        addSensorDebug("FrontL", colorSensorFrontL);
        addSensorDebug("BackL ", colorSensorBackL);

        telemetry.update();
    }

    // ----------------- Pocket reading + latching -----------------

    private static class PocketReading {
        boolean present;
        float avgSum;
        Float avgScore;          // null if both sensors invalid
        BallColor colorIfPresent; // GREEN/PURPLE/UNKNOWN (UNKNOWN if not present or ambiguous)
    }

    private PocketReading readPocket(RevColorSensorV3 s1, RevColorSensorV3 s2) {
        // Read both sensors
        Reading r1 = readOne(s1);
        Reading r2 = readOne(s2);

        PocketReading out = new PocketReading();

        // Average sums always (even if one is low, this helps you see what's happening)
        out.avgSum = (r1.sum + r2.sum) / 2.0f;

        // Present if either sensor sees strong enough brightness
        out.present = (r1.sum >= BALL_PRESENT_SUM) || (r2.sum >= BALL_PRESENT_SUM);

        // Average only valid scores (valid = sum >= MIN_BRIGHTNESS_SUM)
        float scoreSum = 0f;
        int n = 0;
        if (r1.score != null) { scoreSum += r1.score; n++; }
        if (r2.score != null) { scoreSum += r2.score; n++; }

        out.avgScore = (n == 0) ? null : (scoreSum / n);

        if (!out.present || out.avgScore == null) {
            out.colorIfPresent = BallColor.UNKNOWN;
        } else {
            out.colorIfPresent = classifyFromScore(out.avgScore);
        }

        return out;
    }

    private BallColor latchWithStability(BallColor instant, boolean present, BallColor latched, boolean isRight) {
        // If not present, don't change latch; just reset stability
        if (!present) {
            if (isRight) { rightLast = BallColor.UNKNOWN; rightStable = 0; }
            else { leftLast = BallColor.UNKNOWN; leftStable = 0; }
            return latched;
        }

        // We only want to latch GREEN or PURPLE (never latch UNKNOWN)
        if (instant == BallColor.UNKNOWN) {
            if (isRight) { rightStable = 0; rightLast = BallColor.UNKNOWN; }
            else { leftStable = 0; leftLast = BallColor.UNKNOWN; }
            return latched;
        }

        if (isRight) {
            if (instant == rightLast) rightStable++;
            else { rightLast = instant; rightStable = 1; }

            if (rightStable >= REQUIRED_STABLE_LOOPS) return instant;
            return latched;
        } else {
            if (instant == leftLast) leftStable++;
            else { leftLast = instant; leftStable = 1; }

            if (leftStable >= REQUIRED_STABLE_LOOPS) return instant;
            return latched;
        }
    }

    private BallColor classifyFromScore(float score) {
        // Your measured ranges:
        // purple ~ -0.40, green ~ -0.01
        if (score <= PURPLE_SCORE_MAX) return BallColor.PURPLE;
        if (score >= GREEN_SCORE_MIN)  return BallColor.GREEN;
        return BallColor.UNKNOWN;
    }

    // ----------------- Per-sensor reading + debug -----------------

    private static class Reading {
        float sum;
        Float score; // null if too dim (sum < MIN_BRIGHTNESS_SUM)
        float r, g, b;
    }

    private Reading readOne(RevColorSensorV3 s) {
        NormalizedRGBA c = s.getNormalizedColors();
        Reading r = new Reading();
        r.r = c.red;
        r.g = c.green;
        r.b = c.blue;
        r.sum = c.red + c.green + c.blue;

        if (r.sum < MIN_BRIGHTNESS_SUM) {
            r.score = null;
        } else {
            float gRatio = r.g / r.sum;
            float rbRatio = (r.r + r.b) / r.sum;
            r.score = (gRatio - rbRatio);
        }
        return r;
    }

    private void addSensorDebug(String label, RevColorSensorV3 s) {
        NormalizedRGBA c = s.getNormalizedColors();
        float sum = c.red + c.green + c.blue;

        Float score = null;
        if (sum >= MIN_BRIGHTNESS_SUM) {
            float gRatio = c.green / sum;
            float rbRatio = (c.red + c.blue) / sum;
            score = (gRatio - rbRatio);
        }

        telemetry.addData(label + " rgb", "r=%.3f g=%.3f b=%.3f", c.red, c.green, c.blue);
        telemetry.addData(label + " sum", "%.3f", sum);
        telemetry.addData(label + " score", score == null ? "NULL" : String.format("%.3f", score));
        telemetry.addData(label + " gain", s.getGain());
    }

    private void setLed(Object sensor, boolean on) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(on);
        }
    }
}




