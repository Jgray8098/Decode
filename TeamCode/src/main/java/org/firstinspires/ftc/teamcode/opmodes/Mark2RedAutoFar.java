package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Mark2 — Red Far autonomous.
 *
 * Alliance:  RED
 * Start:     Far side (away from the high goal, red alliance wall)
 *
 * Path summary (order carried over from CurrentRedAutoFar)
 * ─────────────────────────────────────────────────────────
 * 1. Drive to preloads launch pose → shoot (FAR distance)
 * 2. Intake 2 (fast sweep, heading 0°):
 *      align → GP (green+purple) → align2 → align3 → PP (purple+purple)
 *    reverse cleanup while driving back → fire Row 2
 * 3. Intake 1 (precision sweep, heading 0°):
 *      align → P11 → P12 → G11
 *    reverse cleanup while driving back → fire Row 1
 * 4. Park
 *
 * All field coordinates are identical to CurrentRedAutoFar (robot faces 0°
 * on the red side, mirrored from blue's 180°).
 *
 * Tune LAUNCH_DISTANCE_FAR to match your actual target distance.
 */
@Autonomous(name = "Mark2 Red Auto Far", group = "Mark2")
public class Mark2RedAutoFar extends Mark2AutoBase {

    // ─── Starting pose ────────────────────────────────────────────────────────
    private static final double START_X   =  81.994;
    private static final double START_Y   =   9.021;
    private static final double START_ROT =  90.0;    // degrees

    // ─── Launch distance (inches) — tune to real target distance ─────────────
    /** Approximate distance from the far-side launch zone to the high goal. */
    private static final double LAUNCH_DISTANCE_FAR = 90.0; // TUNE

    // ─── Field poses (x, y, heading°) ─────────────────────────────────────────
    // Preloads
    private static final double[] LAUNCH_PRELOADS     = { 84.848,  22.663,  63.0};

    // ── Intake 2 (FIRST after preloads — fast sweep at y≈15, facing 0°) ───────
    private static final double[] ALIGN_INTAKE2       = { 89.000,  15.000,   0.0};
    private static final double[] INTAKE_GP           = {126.546,  15.000,   0.0};
    private static final double[] ALIGN_INTAKE2_2     = {125.049,  16.227,   0.0};
    private static final double[] ALIGN_INTAKE2_3     = {125.031,  12.018,   0.0};
    private static final double[] INTAKE_PP           = {130.325,  10.982,   0.0};

    // Row 2 launch (after Intake 2)
    private static final double[] LAUNCH_ROW2         = { 93.000,  21.000,  68.0};

    // ── Intake 1 (SECOND — precision sweep at y≈32–34, facing 0°) ─────────────
    private static final double[] ALIGN_INTAKE1       = { 94.000,  32.000,   0.0};
    private static final double[] INTAKE_P11          = {106.796,  32.000,   0.0};
    private static final double[] INTAKE_P12          = {111.611,  34.000,   0.0};
    private static final double[] INTAKE_G11          = {116.207,  34.000,   0.0};

    // Row 1 launch (after Intake 1)
    private static final double[] LAUNCH_ROW1         = { 93.000,  21.000,  68.0};

    // Park
    private static final double[] PARK                = {105.190,  15.074,  90.0};

    // =========================================================================
    @Override
    protected void setup() {
        initSubsystems(START_X, START_Y, START_ROT);
        telemetry.addLine("Mark2 Red Auto Far — Ready");
        telemetry.addLine("Order: Preloads → Intake2 (fast) → Row2 → Intake1 (slow) → Row1 → Park");
        telemetry.addData("Launch dist (in)", LAUNCH_DISTANCE_FAR);
        telemetry.update();
    }

    @Override
    protected void runPath() {

        // ── 1. Preloads → shoot ────────────────────────────────────────────────
        currentPhase = "Drive to preloads";
        navigate(LAUNCH_PRELOADS[0], LAUNCH_PRELOADS[1], LAUNCH_PRELOADS[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 2. Intake 2 (fast, first) ──────────────────────────────────────────
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

        startReverseFor(OUTTAKE_REVERSE_S);

        // ── 3. Shoot Row 2 ─────────────────────────────────────────────────────
        currentPhase = "Drive to Row2 launch";
        navigate(LAUNCH_ROW2[0], LAUNCH_ROW2[1], LAUNCH_ROW2[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 4. Intake 1 (slow precision, second) ───────────────────────────────
        currentPhase = "Intake1 - align";
        navigate(ALIGN_INTAKE1[0], ALIGN_INTAKE1[1], ALIGN_INTAKE1[2]);

        currentPhase = "Intake1 - P11";
        navigateWithIntake(INTAKE_P11[0], INTAKE_P11[1], INTAKE_P11[2]);

        currentPhase = "Intake1 - P12";
        navigate(INTAKE_P12[0], INTAKE_P12[1], INTAKE_P12[2]);

        currentPhase = "Intake1 - G11";
        navigate(INTAKE_G11[0], INTAKE_G11[1], INTAKE_G11[2]);

        startReverseFor(OUTTAKE_REVERSE_S);

        // ── 5. Shoot Row 1 ─────────────────────────────────────────────────────
        currentPhase = "Drive to Row1 launch";
        navigate(LAUNCH_ROW1[0], LAUNCH_ROW1[1], LAUNCH_ROW1[2]);
        shootFrom(LAUNCH_DISTANCE_FAR);

        // ── 6. Park ────────────────────────────────────────────────────────────
        currentPhase = "Park";
        navigate(PARK[0], PARK[1], PARK[2]);

        currentPhase = "Done";
        postTelemetry();
    }
}

