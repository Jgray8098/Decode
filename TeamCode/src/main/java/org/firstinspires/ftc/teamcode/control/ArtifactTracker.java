// ===============================
// ArtifactTracker.java (UPDATED)
// ===============================
package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * ArtifactTracker v2:
 * - Tracks 3 slots (1..3)
 * - Uses two sensor stations: R -> slot1 position, L -> slot2 position
 * - Uses TWO sensors per station and REQUIRES stable color for N loops before claiming a slot
 *
 * Slot rotation on advance forward:
 *   newSlot1 = oldSlot3
 *   newSlot2 = oldSlot1
 *   newSlot3 = oldSlot2
 */
public class ArtifactTracker {

    public enum Color { GREEN, PURPLE, UNKNOWN, EMPTY }

    private final RevColorSensorV3 frontR, backR, frontL, backL;

    // Tuned constants
    private final int gain;
    private final float minBrightnessSum;   // below this -> ignore sample
    private final float ballPresentSum;     // above this -> present gate
    private final float purpleScoreMax;     // <= -> PURPLE
    private final float greenScoreMin;      // >= -> GREEN

    // Stability requirement (match what worked in ColorTestOpMode)
    private final int requiredStableLoops;

    // Slot states (1..3), 0 unused
    private final Color[] slot = new Color[]{ Color.EMPTY, Color.UNKNOWN, Color.UNKNOWN, Color.UNKNOWN };

    private boolean trackingEnabled = false;

    // Per-station stability trackers (R station fills slot1, L station fills slot2)
    private Color rLast = Color.UNKNOWN;
    private int rStable = 0;

    private Color lLast = Color.UNKNOWN;
    private int lStable = 0;

    public ArtifactTracker(
            RevColorSensorV3 frontR, RevColorSensorV3 backR,
            RevColorSensorV3 frontL, RevColorSensorV3 backL,
            int gain,
            float minBrightnessSum,
            float ballPresentSum,
            float purpleScoreMax,
            float greenScoreMin
    ) {
        this(frontR, backR, frontL, backL, gain, minBrightnessSum, ballPresentSum, purpleScoreMax, greenScoreMin, 8);
    }

    public ArtifactTracker(
            RevColorSensorV3 frontR, RevColorSensorV3 backR,
            RevColorSensorV3 frontL, RevColorSensorV3 backL,
            int gain,
            float minBrightnessSum,
            float ballPresentSum,
            float purpleScoreMax,
            float greenScoreMin,
            int requiredStableLoops
    ) {
        this.frontR = frontR;
        this.backR  = backR;
        this.frontL = frontL;
        this.backL  = backL;

        this.gain = gain;
        this.minBrightnessSum = minBrightnessSum;
        this.ballPresentSum = ballPresentSum;
        this.purpleScoreMax = purpleScoreMax;
        this.greenScoreMin = greenScoreMin;
        this.requiredStableLoops = requiredStableLoops;

        // LED + gain
        setLed(frontR, true);
        setLed(backR, true);
        setLed(frontL, true);
        setLed(backL, true);

        frontR.setGain(gain);
        backR.setGain(gain);
        frontL.setGain(gain);
        backL.setGain(gain);

        trackingEnabled = false;
        slot[1] = Color.UNKNOWN;
        slot[2] = Color.UNKNOWN;
        slot[3] = Color.UNKNOWN;
    }

    /** Call after the FIRST 3-shot launch completes (when you assume indexer is empty). */
    public void startTrackingEmpty() {
        trackingEnabled = true;
        slot[1] = Color.EMPTY;
        slot[2] = Color.EMPTY;
        slot[3] = Color.EMPTY;

        resetStationStability();
    }

    public void stopTracking() {
        trackingEnabled = false;
        slot[1] = Color.UNKNOWN;
        slot[2] = Color.UNKNOWN;
        slot[3] = Color.UNKNOWN;

        resetStationStability();
    }

    public boolean isTrackingEnabled() { return trackingEnabled; }

    /**
     * Call every loop.
     * If slot1/slot2 are EMPTY and a ball is present, we wait for N stable classifications
     * before committing GREEN/PURPLE. We never commit UNKNOWN to a slot.
     */
    public void update() {
        if (!trackingEnabled) return;

        PocketReading r = readPocket(frontR, backR);
        PocketReading l = readPocket(frontL, backL);

        // ---- Station R -> Slot 1 ----
        if (slot[1] == Color.EMPTY) {
            Color stable = stableColorForStation(r, true);
            if (stable == Color.GREEN || stable == Color.PURPLE) {
                slot[1] = stable;
            }
        } else {
            if (!r.present) { rLast = Color.UNKNOWN; rStable = 0; }
        }

        // ---- Station L -> Slot 2 ----
        if (slot[2] == Color.EMPTY) {
            Color stable = stableColorForStation(l, false);
            if (stable == Color.GREEN || stable == Color.PURPLE) {
                slot[2] = stable;
            }
        } else {
            if (!l.present) { lLast = Color.UNKNOWN; lStable = 0; }
        }
    }

    /**
     * Call exactly once AFTER an indexer advance completes.
     * Rotates slot memory to match physical movement.
     */
    public void onAdvanceForward() {
        if (!trackingEnabled) return;

        Color old1 = slot[1], old2 = slot[2], old3 = slot[3];
        slot[1] = old3;
        slot[2] = old1;
        slot[3] = old2;

        resetStationStability();
    }

    /** Optional: if you know you fired from slot3, mark it empty. */
    public void onFiredOneFromSlot3() {
        if (!trackingEnabled) return;
        slot[3] = Color.EMPTY;
    }

    public Color getSlot(int slotNum) {
        if (slotNum < 1 || slotNum > 3) return Color.UNKNOWN;
        return slot[slotNum];
    }

    public int count(Color c) {
        int n = 0;
        if (slot[1] == c) n++;
        if (slot[2] == c) n++;
        if (slot[3] == c) n++;
        return n;
    }

    /** Launch order assumption: slot3 fires first, then slot1, then slot2. */
    public Color[] getLaunchOrder() {
        return new Color[]{ slot[3], slot[1], slot[2] };
    }

    // =========================================================
    // Slot3 inference/assumption helpers (for sorting)
    // =========================================================

    /** Only sets slot3 if it is UNKNOWN or EMPTY. */
    public void assumeSlot3(Color assumed) {
        if (!trackingEnabled) return;
        if (assumed != Color.GREEN && assumed != Color.PURPLE) return;

        if (slot[3] == Color.UNKNOWN || slot[3] == Color.EMPTY) {
            slot[3] = assumed;
        }
    }

    /**
     * Infer slot3 assuming the set must be exactly {2 PURPLE, 1 GREEN},
     * using only slots 1 and 2.
     */
    public Color inferSlot3AssumingTwoP1G() {
        if (!trackingEnabled) return Color.UNKNOWN;

        Color s1 = slot[1];
        Color s2 = slot[2];

        if ((s1 != Color.GREEN && s1 != Color.PURPLE) ||
                (s2 != Color.GREEN && s2 != Color.PURPLE)) {
            return Color.UNKNOWN;
        }

        int purple = 0;
        if (s1 == Color.PURPLE) purple++;
        if (s2 == Color.PURPLE) purple++;

        return (purple == 2) ? Color.GREEN : Color.PURPLE;
    }

    // ---------------- Stability logic ----------------

    private Color stableColorForStation(PocketReading pr, boolean isRightStation) {
        if (!pr.present) {
            if (isRightStation) { rLast = Color.UNKNOWN; rStable = 0; }
            else                { lLast = Color.UNKNOWN; lStable = 0; }
            return Color.UNKNOWN;
        }

        if (pr.colorIfPresent == Color.UNKNOWN) {
            if (isRightStation) { rLast = Color.UNKNOWN; rStable = 0; }
            else                { lLast = Color.UNKNOWN; lStable = 0; }
            return Color.UNKNOWN;
        }

        if (isRightStation) {
            if (pr.colorIfPresent == rLast) rStable++;
            else { rLast = pr.colorIfPresent; rStable = 1; }

            return (rStable >= requiredStableLoops) ? pr.colorIfPresent : Color.UNKNOWN;
        } else {
            if (pr.colorIfPresent == lLast) lStable++;
            else { lLast = pr.colorIfPresent; lStable = 1; }

            return (lStable >= requiredStableLoops) ? pr.colorIfPresent : Color.UNKNOWN;
        }
    }

    private void resetStationStability() {
        rLast = Color.UNKNOWN; rStable = 0;
        lLast = Color.UNKNOWN; lStable = 0;
    }

    // ---------------- Sensor reading logic ----------------

    private static class PocketReading {
        boolean present;
        Float avgScore;
        Color colorIfPresent;
    }

    private static class Reading {
        float sum;
        Float score;
    }

    private PocketReading readPocket(RevColorSensorV3 s1, RevColorSensorV3 s2) {
        Reading a = readOne(s1);
        Reading b = readOne(s2);

        PocketReading out = new PocketReading();
        out.present = (a.sum >= ballPresentSum) || (b.sum >= ballPresentSum);

        float scoreSum = 0f;
        int n = 0;
        if (a.score != null) { scoreSum += a.score; n++; }
        if (b.score != null) { scoreSum += b.score; n++; }
        out.avgScore = (n == 0) ? null : (scoreSum / n);

        if (!out.present || out.avgScore == null) out.colorIfPresent = Color.UNKNOWN;
        else out.colorIfPresent = classify(out.avgScore);

        return out;
    }

    private Reading readOne(RevColorSensorV3 s) {
        NormalizedRGBA c = s.getNormalizedColors();
        Reading r = new Reading();
        r.sum = c.red + c.green + c.blue;

        if (r.sum < minBrightnessSum) {
            r.score = null;
        } else {
            float gRatio = c.green / r.sum;
            float rbRatio = (c.red + c.blue) / r.sum;
            r.score = (gRatio - rbRatio);
        }
        return r;
    }

    private Color classify(float score) {
        if (score <= purpleScoreMax) return Color.PURPLE;
        if (score >= greenScoreMin)  return Color.GREEN;
        return Color.UNKNOWN;
    }

    private void setLed(Object sensor, boolean on) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(on);
        }
    }
}