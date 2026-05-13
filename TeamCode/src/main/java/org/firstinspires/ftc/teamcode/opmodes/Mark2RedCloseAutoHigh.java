package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Mark2 — Red Close High autonomous.
 *
 * Alliance:  RED
 * Start:     Close side (near the high goal, red alliance wall)
 *
 * Path summary
 * ────────────
 * 1. Drive to preloads launch pose → shoot
 * 2. Intake Row 1  :  align → P11 → P12 → G11  (intake running)
 *    reverse cleanup while driving back → fire Row 1
 * 3. Intake Row 2  :  align → P21 → G21 → P22  (intake running)
 *    reverse cleanup while driving back → fire Row 2
 * 4. Park
 *
 * All field coordinates mirror the original RedCloseAutoHigh poses.
 * The AprilTag scan step has been removed — the robot shoots immediately
 * upon arriving at each launch pose.
 *
 * Tune LAUNCH_DISTANCE_CLOSE to match your actual target distance.
 */
@Autonomous(name = "Mark2 Red Close High", group = "Mark2")
public class Mark2RedCloseAutoHigh extends Mark2AutoBase {

    // ─── Starting pose ────────────────────────────────────────────────────────
    private static final double START_X   = 123.539;
    private static final double START_Y   = 123.153;
    private static final double START_ROT = 126.0;    // degrees

    // ─── Launch distance (inches) — tune to real target distance ─────────────
    /** Approximate distance from the close-side launch zone to the high goal. */
    private static final double LAUNCH_DISTANCE_CLOSE = 48.0; // TUNE

    // ─── Field poses (x, y, heading°) ─────────────────────────────────────────
    // Preloads
    private static final double[] LAUNCH_PRELOADS = {89.0, 97.0, 26.0};

    // Row 1 intake sequence (red side — robot faces 0°, sweeps right)
    private static final double[] ALIGN_INTAKE1  = { 94.568, 84.477,  0.0};
    private static final double[] INTAKE_P11     = {105.166, 84.477,  0.0};
    private static final double[] INTAKE_P12     = {109.957, 84.477,  0.0};
    private static final double[] INTAKE_G11     = {117.976, 84.477,  0.0};

    // Row 1 launch
    private static final double[] LAUNCH_ROW1    = { 92.0,   97.0,   39.0};

    // Row 2 intake sequence
    private static final double[] ALIGN_INTAKE2  = { 96.306, 62.928,  0.0};
    private static final double[] INTAKE_P21     = {102.973, 62.928,  0.0};
    private static final double[] INTAKE_G21     = {108.378, 60.928,  0.0};
    private static final double[] INTAKE_P22     = {116.590, 60.928,  0.0};

    // Row 2 launch
    private static final double[] LAUNCH_ROW2    = { 89.0,   97.0,   38.0};

    // Park
    private static final double[] PARK           = {110.432, 75.475,  0.0};

    // =========================================================================
    @Override
    protected void setup() {
        initSubsystems(START_X, START_Y, START_ROT);
        telemetry.addLine("Mark2 Red Close High — Ready");
        telemetry.addData("Launch dist (in)", LAUNCH_DISTANCE_CLOSE);
        telemetry.update();
    }

    @Override
    protected void runPath() {

        // ── 1. Drive to preloads launch position and shoot ───────────────────
        currentPhase = "Drive to preloads";
        navigate(LAUNCH_PRELOADS[0], LAUNCH_PRELOADS[1], LAUNCH_PRELOADS[2]);
        shootFrom(LAUNCH_DISTANCE_CLOSE);

        // ── 2. Row 1 intake ───────────────────────────────────────────────────
        currentPhase = "Intake Row1 - align";
        navigate(ALIGN_INTAKE1[0], ALIGN_INTAKE1[1], ALIGN_INTAKE1[2]);

        currentPhase = "Intake Row1 - P11";
        navigateWithIntake(INTAKE_P11[0], INTAKE_P11[1], INTAKE_P11[2]);

        currentPhase = "Intake Row1 - P12";
        navigate(INTAKE_P12[0], INTAKE_P12[1], INTAKE_P12[2]);

        currentPhase = "Intake Row1 - G11";
        navigate(INTAKE_G11[0], INTAKE_G11[1], INTAKE_G11[2]);

        startReverseFor(OUTTAKE_REVERSE_S);

        // ── 3. Shoot Row 1 ───────────────────────────────────────────────────
        currentPhase = "Drive to Row1 launch";
        navigate(LAUNCH_ROW1[0], LAUNCH_ROW1[1], LAUNCH_ROW1[2]);
        shootFrom(LAUNCH_DISTANCE_CLOSE);

        // ── 4. Row 2 intake ───────────────────────────────────────────────────
        currentPhase = "Intake Row2 - align";
        navigate(ALIGN_INTAKE2[0], ALIGN_INTAKE2[1], ALIGN_INTAKE2[2]);

        currentPhase = "Intake Row2 - P21";
        navigateWithIntake(INTAKE_P21[0], INTAKE_P21[1], INTAKE_P21[2]);

        currentPhase = "Intake Row2 - G21";
        navigate(INTAKE_G21[0], INTAKE_G21[1], INTAKE_G21[2]);

        currentPhase = "Intake Row2 - P22";
        navigate(INTAKE_P22[0], INTAKE_P22[1], INTAKE_P22[2]);

        startReverseFor(OUTTAKE_REVERSE_S);

        // ── 5. Shoot Row 2 ───────────────────────────────────────────────────
        currentPhase = "Drive to Row2 launch";
        navigate(LAUNCH_ROW2[0], LAUNCH_ROW2[1], LAUNCH_ROW2[2]);
        shootFrom(LAUNCH_DISTANCE_CLOSE);

        // ── 6. Park ───────────────────────────────────────────────────────────
        currentPhase = "Park";
        navigate(PARK[0], PARK[1], PARK[2]);

        currentPhase = "Done";
        postTelemetry();
    }
}

