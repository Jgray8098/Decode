package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Mark2 — Blue Far Cycle autonomous.
 *
 * Alliance:  BLUE
 * Start:     Far side (blue alliance wall)
 *
 * Path summary (ported from BlueFarCycle)
 * ─────────────────────────────────────────────────────────────────────────────
 * 1. Drive to preloads launch pose → shoot
 * 2. Intake 2 (fast sweep at y≈15, heading 180°):
 *      align → GP → align2 → align3 → PP
 *    stop intake → fire Row 2
 * 3. Intake 1-main (wide sweep at y≈35, heading 180°):
 *      align → sweep full width to G11
 *    stop intake → fire Row 1
 * 4. Intake 1-low (second cycle at y≈10, heading 180°):
 *      align-low → sweep to G11-low
 *    stop intake → fire Row 1 again (same launch pose)
 * 5. Park
 *
 * Note: this cycle variant does NOT reverse the intake between passes;
 * instead the intake simply stops before each shot.  This matches the
 * original BlueFarCycle behaviour.
 *
 * Tune LAUNCH_DISTANCE_FAR to match your actual target distance.
 */
@Autonomous(name = "Mark2 Blue Far Cycle", group = "Mark2")
public class Mark2BlueFarCycle extends Mark2AutoBase {

    // ─── Starting pose ────────────────────────────────────────────────────────
    private static final double START_X   = 62.006;
    private static final double START_Y   =  9.021;
    private static final double START_ROT = 90.0;      // degrees

    // ─── Launch distance — tune to real target distance ───────────────────────
    private static final double LAUNCH_DISTANCE_FAR = 90.0; // TUNE

    // ─── Field poses (x, y, heading°) ─────────────────────────────────────────
    // Preloads
    private static final double[] LAUNCH_PRELOADS    = {59.152, 22.663, 117.0};

    // ── Intake 2 (fast sweep at y≈15) ──────────────────────────────────────────
    private static final double[] ALIGN_INTAKE2      = {55.000, 15.000, 180.0};
    private static final double[] INTAKE_GP          = {17.454, 15.000, 180.0};
    private static final double[] ALIGN_INTAKE2_2    = {18.951, 16.227, 180.0};
    private static final double[] ALIGN_INTAKE2_3    = {18.969, 12.018, 180.0};
    private static final double[] INTAKE_PP          = {13.675, 10.982, 180.0};

    // Row 2 launch (after Intake 2)
    private static final double[] LAUNCH_ROW2        = {54.000, 21.000, 112.0};

    // ── Intake 1-main (wide sweep at y≈35, align → G11 in one go) ─────────────
    private static final double[] ALIGN_INTAKE1      = {50.000, 34.000, 180.0};
    private static final double[] INTAKE_G11         = {22.793, 35.000, 180.0};

    // Row 1 launch (reused for both Row-1-main and Row-1-low shots)
    private static final double[] LAUNCH_ROW1        = {52.000, 21.000, 112.0};

    // ── Intake 1-low (second pass at y≈10) ────────────────────────────────────
    private static final double[] ALIGN_INTAKE1_LOW  = {50.000, 12.000, 180.0};
    private static final double[] INTAKE_G11_LOW     = {22.793, 10.000, 180.0};

    // Park (from Row-1 launch pose after the low cycle)
    private static final double[] PARK               = {38.810, 15.074,  90.0};

    // =========================================================================
    @Override
    protected void setup() {
        initSubsystems(START_X, START_Y, START_ROT);
        telemetry.addLine("Mark2 Blue Far Cycle — Ready");
        telemetry.addLine("Preloads → Int2 → Row2 → Int1-main → Row1 → Int1-low → Row1 → Park");
        telemetry.addData("Launch dist (in)", LAUNCH_DISTANCE_FAR);
        telemetry.update();
    }

    @Override
    protected void runPath() {

        // ── 1. Preloads → shoot ────────────────────────────────────────────────
        currentPhase = "Drive to preloads";
        navigate(LAUNCH_PRELOADS[0], LAUNCH_PRELOADS[1], LAUNCH_PRELOADS[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 2. Intake 2 (fast) ────────────────────────────────────────────────
        currentPhase = "Intake2 - align";
        navigate(ALIGN_INTAKE2[0], ALIGN_INTAKE2[1], ALIGN_INTAKE2[2]);

        currentPhase = "Intake2 - GP sweep";
        navigateWithIntake(INTAKE_GP[0], INTAKE_GP[1], INTAKE_GP[2]);

        currentPhase = "Intake2 - align2";
        navigate(ALIGN_INTAKE2_2[0], ALIGN_INTAKE2_2[1], ALIGN_INTAKE2_2[2]);

        currentPhase = "Intake2 - align3";
        navigate(ALIGN_INTAKE2_3[0], ALIGN_INTAKE2_3[1], ALIGN_INTAKE2_3[2]);

        currentPhase = "Intake2 - PP sweep";
        navigate(INTAKE_PP[0], INTAKE_PP[1], INTAKE_PP[2]);

        // No outtake reverse in the cycle variant — just stop before shooting
        stopIntake();

        // ── 3. Shoot Row 2 ─────────────────────────────────────────────────────
        currentPhase = "Drive to Row2 launch";
        navigate(LAUNCH_ROW2[0], LAUNCH_ROW2[1], LAUNCH_ROW2[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 4. Intake 1-main (align → G11 in one wide sweep) ───────────────────
        currentPhase = "Intake1-main - align";
        navigate(ALIGN_INTAKE1[0], ALIGN_INTAKE1[1], ALIGN_INTAKE1[2]);

        currentPhase = "Intake1-main - sweep to G11";
        navigateWithIntake(INTAKE_G11[0], INTAKE_G11[1], INTAKE_G11[2]);

        stopIntake();

        // ── 5. Shoot Row 1 (main) ──────────────────────────────────────────────
        currentPhase = "Drive to Row1 launch (main)";
        navigate(LAUNCH_ROW1[0], LAUNCH_ROW1[1], LAUNCH_ROW1[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 6. Intake 1-low (second intake cycle at y≈10) ─────────────────────
        currentPhase = "Intake1-low - align";
        navigate(ALIGN_INTAKE1_LOW[0], ALIGN_INTAKE1_LOW[1], ALIGN_INTAKE1_LOW[2]);

        currentPhase = "Intake1-low - sweep to G11-low";
        navigateWithIntake(INTAKE_G11_LOW[0], INTAKE_G11_LOW[1], INTAKE_G11_LOW[2]);

        stopIntake();

        // ── 7. Shoot Row 1 again (low cycle) ──────────────────────────────────
        currentPhase = "Drive to Row1 launch (low)";
        navigate(LAUNCH_ROW1[0], LAUNCH_ROW1[1], LAUNCH_ROW1[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 8. Park ────────────────────────────────────────────────────────────
        currentPhase = "Park";
        navigate(PARK[0], PARK[1], PARK[2]);

        currentPhase = "Done";
        postTelemetry();
    }
}

