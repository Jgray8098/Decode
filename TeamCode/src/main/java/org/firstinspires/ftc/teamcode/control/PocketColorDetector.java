package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class PocketColorDetector {

    public enum BallColor { GREEN, PURPLE, UNKNOWN }

    private final NormalizedColorSensor a, b;

    // You WILL tune these with telemetry
    private static final float MIN_BRIGHTNESS_SUM = 0.05f;   // reject dim/holes
    private static final float GREEN_SCORE_MIN = 0.10f;      // score above => green
    private static final float PURPLE_SCORE_MAX = -0.10f;    // score below => purple

    // Stability filter
    private BallColor last = BallColor.UNKNOWN;
    private int stableCount = 0;
    private final int requiredStable;

    public PocketColorDetector(NormalizedColorSensor sA, NormalizedColorSensor sB, int requiredStableLoops) {
        this.a = sA;
        this.b = sB;
        this.requiredStable = requiredStableLoops;

        setLed(a, true);
        setLed(b, true);
    }

    private void setLed(NormalizedColorSensor s, boolean on) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(on);
    }

    private Float scoreIfValid(NormalizedColorSensor s) {
        NormalizedRGBA c = s.getNormalizedColors();
        float sum = c.red + c.green + c.blue;
        if (sum < MIN_BRIGHTNESS_SUM) return null;

        float gRatio = c.green / sum;
        float rbRatio = (c.red + c.blue) / sum;

        return (gRatio - rbRatio); // positive => greener, negative => more purple-ish
    }

    private BallColor classifyFromScore(float score) {
        if (score >= GREEN_SCORE_MIN) return BallColor.GREEN;
        if (score <= PURPLE_SCORE_MAX) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    /** Call this every loop. Returns a *latched* result once stable; otherwise UNKNOWN. */
    public BallColor updateAndGetLatched() {
        Float s1 = scoreIfValid(a);
        Float s2 = scoreIfValid(b);

        // If neither sensor has a valid reading, treat as unknown (likely hole/too far/no ball)
        if (s1 == null && s2 == null) {
            resetStability(BallColor.UNKNOWN);
            return BallColor.UNKNOWN;
        }

        // Average only valid scores
        float sum = 0f;
        int n = 0;
        if (s1 != null) { sum += s1; n++; }
        if (s2 != null) { sum += s2; n++; }
        float avgScore = sum / n;

        BallColor now = classifyFromScore(avgScore);

        // Stability check
        if (now == last) stableCount++;
        else {
            last = now;
            stableCount = 1;
        }

        // Only “commit” when stable for requiredStable loops
        if (stableCount >= requiredStable && now != BallColor.UNKNOWN) {
            return now;
        }
        return BallColor.UNKNOWN;
    }

    private void resetStability(BallColor setTo) {
        last = setTo;
        stableCount = 0;
    }
}

