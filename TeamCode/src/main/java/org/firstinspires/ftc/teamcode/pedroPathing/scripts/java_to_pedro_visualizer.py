"""
java_to_pedro_visualizer.py
============================
Converts Pedro Pathing pose constants from your Java op-modes into the JSON
format used by the Pedro Pathing Path Visualizer
  https://pedro-path-generator.vercel.app/

Usage
-----
  python scripts/java_to_pedro_visualizer.py

Output
------
  scripts/pedro_paths_blue_far.json   ← import this file in the visualizer
  scripts/pedro_paths_red_far.json

How to import
-------------
  1. Open https://pedro-path-generator.vercel.app/
  2. Click the  ≡  (hamburger / menu) button → "Import"
  3. Choose the generated .json file
  4. All paths appear in the visualizer for review / tweaking.

JSON format used
----------------
The visualizer stores an array of path-chain objects.
Each path is a straight BezierLine (two waypoints).
Headings are stored in DEGREES (the visualizer converts to radians internally).
"""

import json, math, os

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def pose(x, y, heading_deg):
    """Return a dict with x/y in inches and heading in degrees."""
    return {"x": x, "y": y, "heading": heading_deg}

def path(name, start, end, h_start_deg=None, h_end_deg=None):
    """
    Build a single-segment path (BezierLine) entry.
    If heading interpolation endpoints are not supplied, the start/end pose
    headings are used directly.
    """
    wp0 = dict(start)
    wp1 = dict(end)
    if h_start_deg is not None:
        wp0["heading"] = h_start_deg
    if h_end_deg is not None:
        wp1["heading"] = h_end_deg
    return {"name": name, "waypoints": [wp0, wp1]}

# ─────────────────────────────────────────────────────────────────────────────
# BLUE FAR  (CurrentBlueAutoFar.java / Mark2BlueAutoFar.java)
# ─────────────────────────────────────────────────────────────────────────────

# ── Poses ────────────────────────────────────────────────────────────────────
BLUE_START            = pose(62.006,  9.021,  90.0)
BLUE_LAUNCH_PRELOADS  = pose(59.152, 22.663, 117.0)

BLUE_ALIGN_INTAKE2    = pose(55.000, 15.000, 180.0)
BLUE_INTAKE_GP        = pose(17.454, 15.000, 180.0)
BLUE_ALIGN_INTAKE2_2  = pose(18.951, 16.227, 180.0)
BLUE_ALIGN_INTAKE2_3  = pose(18.969, 12.018, 180.0)
BLUE_INTAKE_PP        = pose(13.675, 10.982, 180.0)
BLUE_LAUNCH_ROW2      = pose(54.000, 21.000, 112.0)

BLUE_ALIGN_INTAKE1    = pose(50.000, 34.000, 180.0)
BLUE_INTAKE_P11       = pose(37.204, 34.000, 180.0)
BLUE_INTAKE_P12       = pose(32.389, 35.000, 180.0)
BLUE_INTAKE_G11       = pose(27.793, 35.000, 180.0)
BLUE_LAUNCH_ROW1      = pose(52.000, 21.000, 112.0)

BLUE_PARK             = pose(38.810, 15.074,  90.0)

# ── Path chain (ordered as the opmode executes) ───────────────────────────────
BLUE_FAR_PATHS = [
    path("1 Start → LaunchPreloads",
         BLUE_START, BLUE_LAUNCH_PRELOADS,
         h_start_deg=90, h_end_deg=117),

    path("2 LaunchPreloads → AlignIntake2",
         BLUE_LAUNCH_PRELOADS, BLUE_ALIGN_INTAKE2,
         h_start_deg=117, h_end_deg=180),

    path("3 AlignIntake2 → IntakeGP",
         BLUE_ALIGN_INTAKE2, BLUE_INTAKE_GP,
         h_start_deg=180, h_end_deg=180),

    path("4 IntakeGP → AlignIntake2_2",
         BLUE_INTAKE_GP, BLUE_ALIGN_INTAKE2_2,
         h_start_deg=180, h_end_deg=180),

    path("5 AlignIntake2_2 → AlignIntake2_3",
         BLUE_ALIGN_INTAKE2_2, BLUE_ALIGN_INTAKE2_3,
         h_start_deg=180, h_end_deg=180),

    path("6 AlignIntake2_3 → IntakePP",
         BLUE_ALIGN_INTAKE2_3, BLUE_INTAKE_PP,
         h_start_deg=180, h_end_deg=180),

    path("7 IntakePP → LaunchRow2",
         BLUE_INTAKE_PP, BLUE_LAUNCH_ROW2,
         h_start_deg=180, h_end_deg=109),

    path("8 LaunchRow2 → AlignIntake1",
         BLUE_LAUNCH_ROW2, BLUE_ALIGN_INTAKE1,
         h_start_deg=109, h_end_deg=180),

    path("9 AlignIntake1 → IntakeP11",
         BLUE_ALIGN_INTAKE1, BLUE_INTAKE_P11,
         h_start_deg=180, h_end_deg=180),

    path("10 IntakeP11 → IntakeP12",
         BLUE_INTAKE_P11, BLUE_INTAKE_P12,
         h_start_deg=180, h_end_deg=180),

    path("11 IntakeP12 → IntakeG11",
         BLUE_INTAKE_P12, BLUE_INTAKE_G11,
         h_start_deg=180, h_end_deg=180),

    path("12 IntakeG11 → LaunchRow1",
         BLUE_INTAKE_G11, BLUE_LAUNCH_ROW1,
         h_start_deg=180, h_end_deg=111),

    path("13 LaunchRow1 → Park",
         BLUE_LAUNCH_ROW1, BLUE_PARK,
         h_start_deg=111, h_end_deg=90),
]

# ─────────────────────────────────────────────────────────────────────────────
# RED FAR  (CurrentRedAutoFar.java / Mark2RedAutoFar.java)
# ─────────────────────────────────────────────────────────────────────────────

RED_START             = pose( 81.994,  9.021,  90.0)
RED_LAUNCH_PRELOADS   = pose( 84.848, 22.663,  63.0)

RED_ALIGN_INTAKE2     = pose( 89.000, 15.000,   0.0)
RED_INTAKE_GP         = pose(126.546, 15.000,   0.0)
RED_ALIGN_INTAKE2_2   = pose(125.049, 16.227,   0.0)
RED_ALIGN_INTAKE2_3   = pose(125.031, 12.018,   0.0)
RED_INTAKE_PP         = pose(130.325, 10.982,   0.0)
RED_LAUNCH_ROW2       = pose( 93.000, 21.000,  68.0)

RED_ALIGN_INTAKE1     = pose( 94.000, 32.000,   0.0)
RED_INTAKE_P11        = pose(106.796, 32.000,   0.0)
RED_INTAKE_P12        = pose(111.611, 34.000,   0.0)
RED_INTAKE_G11        = pose(116.207, 34.000,   0.0)
RED_LAUNCH_ROW1       = pose( 93.000, 21.000,  68.0)

RED_PARK              = pose(105.190, 15.074,  90.0)

RED_FAR_PATHS = [
    path("1 Start → LaunchPreloads",
         RED_START, RED_LAUNCH_PRELOADS,
         h_start_deg=90, h_end_deg=63),

    path("2 LaunchPreloads → AlignIntake2",
         RED_LAUNCH_PRELOADS, RED_ALIGN_INTAKE2,
         h_start_deg=63, h_end_deg=0),

    path("3 AlignIntake2 → IntakeGP",
         RED_ALIGN_INTAKE2, RED_INTAKE_GP,
         h_start_deg=0, h_end_deg=0),

    path("4 IntakeGP → AlignIntake2_2",
         RED_INTAKE_GP, RED_ALIGN_INTAKE2_2,
         h_start_deg=0, h_end_deg=0),

    path("5 AlignIntake2_2 → AlignIntake2_3",
         RED_ALIGN_INTAKE2_2, RED_ALIGN_INTAKE2_3,
         h_start_deg=0, h_end_deg=0),

    path("6 AlignIntake2_3 → IntakePP",
         RED_ALIGN_INTAKE2_3, RED_INTAKE_PP,
         h_start_deg=0, h_end_deg=0),

    path("7 IntakePP → LaunchRow2",
         RED_INTAKE_PP, RED_LAUNCH_ROW2,
         h_start_deg=0, h_end_deg=71),

    path("8 LaunchRow2 → AlignIntake1",
         RED_LAUNCH_ROW2, RED_ALIGN_INTAKE1,
         h_start_deg=71, h_end_deg=0),

    path("9 AlignIntake1 → IntakeP11",
         RED_ALIGN_INTAKE1, RED_INTAKE_P11,
         h_start_deg=0, h_end_deg=0),

    path("10 IntakeP11 → IntakeP12",
         RED_INTAKE_P11, RED_INTAKE_P12,
         h_start_deg=0, h_end_deg=0),

    path("11 IntakeP12 → IntakeG11",
         RED_INTAKE_P12, RED_INTAKE_G11,
         h_start_deg=0, h_end_deg=0),

    path("12 IntakeG11 → LaunchRow1",
         RED_INTAKE_G11, RED_LAUNCH_ROW1,
         h_start_deg=0, h_end_deg=68),

    path("13 LaunchRow1 → Park",
         RED_LAUNCH_ROW1, RED_PARK,
         h_start_deg=68, h_end_deg=90),
]

# ─────────────────────────────────────────────────────────────────────────────
# BLUE CLOSE  (BlueCloseAutoHigh.java)
# ─────────────────────────────────────────────────────────────────────────────

BLUE_CLOSE_START            = pose(20.461, 123.153,  54.0)
BLUE_CLOSE_APRILTAG_END     = pose(47.099,  95.936,  74.0)
BLUE_CLOSE_LAUNCH_PRELOADS  = pose(53.239,  99.989, 155.0)

BLUE_CLOSE_ALIGN_INTAKE1    = pose(44.432,  88.477, 180.0)
BLUE_CLOSE_INTAKE_P11       = pose(37.834,  88.477, 180.0)
BLUE_CLOSE_INTAKE_P12       = pose(32.043,  87.477, 180.0)
BLUE_CLOSE_INTAKE_G11       = pose(27.024,  87.477, 180.0)
BLUE_CLOSE_LAUNCH_ROW1      = pose(51.046,  98.182, 141.0)

BLUE_CLOSE_ALIGN_INTAKE2    = pose(46.694,  63.928, 180.0)
BLUE_CLOSE_INTAKE_P21       = pose(38.027,  63.928, 180.0)
BLUE_CLOSE_INTAKE_G21       = pose(32.622,  62.928, 180.0)
BLUE_CLOSE_INTAKE_P22       = pose(27.410,  62.928, 180.0)
BLUE_CLOSE_LAUNCH_ROW2      = pose(52.046,  98.182, 144.0)

BLUE_CLOSE_PARK             = pose(34.568,  75.475, 180.0)

BLUE_CLOSE_HIGH_PATHS = [
    path("1 Start → AprilTagPosition",
         BLUE_CLOSE_START, BLUE_CLOSE_APRILTAG_END,
         h_start_deg=54, h_end_deg=74),

    path("2 AprilTagPosition → LaunchPreloads",
         BLUE_CLOSE_APRILTAG_END, BLUE_CLOSE_LAUNCH_PRELOADS,
         h_start_deg=74, h_end_deg=155),

    path("3 LaunchPreloads → AlignIntake1",
         BLUE_CLOSE_LAUNCH_PRELOADS, BLUE_CLOSE_ALIGN_INTAKE1,
         h_start_deg=155, h_end_deg=180),

    path("4 AlignIntake1 → IntakePurple11",
         BLUE_CLOSE_ALIGN_INTAKE1, BLUE_CLOSE_INTAKE_P11,
         h_start_deg=180, h_end_deg=180),

    path("5 IntakePurple11 → IntakePurple12",
         BLUE_CLOSE_INTAKE_P11, BLUE_CLOSE_INTAKE_P12,
         h_start_deg=180, h_end_deg=180),

    path("6 IntakePurple12 → IntakeGreen11",
         BLUE_CLOSE_INTAKE_P12, BLUE_CLOSE_INTAKE_G11,
         h_start_deg=180, h_end_deg=180),

    path("7 IntakeGreen11 → LaunchFirstRow",
         BLUE_CLOSE_INTAKE_G11, BLUE_CLOSE_LAUNCH_ROW1,
         h_start_deg=180, h_end_deg=141),

    path("8 LaunchFirstRow → AlignIntake2",
         BLUE_CLOSE_LAUNCH_ROW1, BLUE_CLOSE_ALIGN_INTAKE2,
         h_start_deg=141, h_end_deg=180),

    path("9 AlignIntake2 → IntakePurple21",
         BLUE_CLOSE_ALIGN_INTAKE2, BLUE_CLOSE_INTAKE_P21,
         h_start_deg=180, h_end_deg=180),

    path("10 IntakePurple21 → IntakeGreen21",
         BLUE_CLOSE_INTAKE_P21, BLUE_CLOSE_INTAKE_G21,
         h_start_deg=180, h_end_deg=180),

    path("11 IntakeGreen21 → IntakePurple22",
         BLUE_CLOSE_INTAKE_G21, BLUE_CLOSE_INTAKE_P22,
         h_start_deg=180, h_end_deg=180),

    path("12 IntakePurple22 → LaunchSecondRow",
         BLUE_CLOSE_INTAKE_P22, BLUE_CLOSE_LAUNCH_ROW2,
         h_start_deg=180, h_end_deg=144),

    path("13 LaunchSecondRow → Park",
         BLUE_CLOSE_LAUNCH_ROW2, BLUE_CLOSE_PARK,
         h_start_deg=144, h_end_deg=180),
]

# ─────────────────────────────────────────────────────────────────────────────
# RED CLOSE  (RedCloseAutoHigh.java)
# ─────────────────────────────────────────────────────────────────────────────

RED_CLOSE_START            = pose(123.539, 123.153, 126.0)
RED_CLOSE_APRILTAG_END     = pose( 94.000,  94.000, 106.0)
RED_CLOSE_LAUNCH_PRELOADS  = pose( 89.000,  97.000,  26.0)

RED_CLOSE_ALIGN_INTAKE1    = pose( 94.568,  84.477,   0.0)
RED_CLOSE_INTAKE_P11       = pose(105.166,  84.477,   0.0)
RED_CLOSE_INTAKE_P12       = pose(109.957,  84.477,   0.0)
RED_CLOSE_INTAKE_G11       = pose(117.976,  84.477,   0.0)
RED_CLOSE_LAUNCH_ROW1      = pose( 92.000,  97.000,  39.0)

RED_CLOSE_ALIGN_INTAKE2    = pose( 96.306,  62.928,   0.0)
RED_CLOSE_INTAKE_P21       = pose(102.973,  62.928,   0.0)
RED_CLOSE_INTAKE_G21       = pose(108.378,  60.928,   0.0)
RED_CLOSE_INTAKE_P22       = pose(116.590,  60.928,   0.0)
RED_CLOSE_LAUNCH_ROW2      = pose( 89.000,  97.000,  38.0)

RED_CLOSE_PARK             = pose(110.432,  75.475,   0.0)

RED_CLOSE_HIGH_PATHS = [
    path("1 Start → AprilTagPosition",
         RED_CLOSE_START, RED_CLOSE_APRILTAG_END,
         h_start_deg=126, h_end_deg=106),

    path("2 AprilTagPosition → LaunchPreloads",
         RED_CLOSE_APRILTAG_END, RED_CLOSE_LAUNCH_PRELOADS,
         h_start_deg=106, h_end_deg=26),

    path("3 LaunchPreloads → AlignIntake1",
         RED_CLOSE_LAUNCH_PRELOADS, RED_CLOSE_ALIGN_INTAKE1,
         h_start_deg=26, h_end_deg=0),

    path("4 AlignIntake1 → IntakePurple11",
         RED_CLOSE_ALIGN_INTAKE1, RED_CLOSE_INTAKE_P11,
         h_start_deg=0, h_end_deg=0),

    path("5 IntakePurple11 → IntakePurple12",
         RED_CLOSE_INTAKE_P11, RED_CLOSE_INTAKE_P12,
         h_start_deg=0, h_end_deg=0),

    path("6 IntakePurple12 → IntakeGreen11",
         RED_CLOSE_INTAKE_P12, RED_CLOSE_INTAKE_G11,
         h_start_deg=0, h_end_deg=0),

    path("7 IntakeGreen11 → LaunchFirstRow",
         RED_CLOSE_INTAKE_G11, RED_CLOSE_LAUNCH_ROW1,
         h_start_deg=0, h_end_deg=39),

    path("8 LaunchFirstRow → AlignIntake2",
         RED_CLOSE_LAUNCH_ROW1, RED_CLOSE_ALIGN_INTAKE2,
         h_start_deg=39, h_end_deg=0),

    path("9 AlignIntake2 → IntakePurple21",
         RED_CLOSE_ALIGN_INTAKE2, RED_CLOSE_INTAKE_P21,
         h_start_deg=0, h_end_deg=0),

    path("10 IntakePurple21 → IntakeGreen21",
         RED_CLOSE_INTAKE_P21, RED_CLOSE_INTAKE_G21,
         h_start_deg=0, h_end_deg=0),

    path("11 IntakeGreen21 → IntakePurple22",
         RED_CLOSE_INTAKE_G21, RED_CLOSE_INTAKE_P22,
         h_start_deg=0, h_end_deg=0),

    path("12 IntakePurple22 → LaunchSecondRow",
         RED_CLOSE_INTAKE_P22, RED_CLOSE_LAUNCH_ROW2,
         h_start_deg=0, h_end_deg=38),

    path("13 LaunchSecondRow → Park",
         RED_CLOSE_LAUNCH_ROW2, RED_CLOSE_PARK,
         h_start_deg=38, h_end_deg=0),
]

# ─────────────────────────────────────────────────────────────────────────────
# Write JSON files
# ─────────────────────────────────────────────────────────────────────────────

def write(filename, paths_list):
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, filename)
    with open(out_path, "w") as f:
        json.dump(paths_list, f, indent=2)
    print(f"  Wrote {out_path}  ({len(paths_list)} paths)")

if __name__ == "__main__":
    print("Pedro Pathing Visualizer JSON generator")
    print("=" * 44)
    write("pedro_paths_blue_far.json", BLUE_FAR_PATHS)
    write("pedro_paths_red_far.json",  RED_FAR_PATHS)
    write("pedro_paths_blue_close_high.json", BLUE_CLOSE_HIGH_PATHS)
    write("pedro_paths_red_close_high.json",  RED_CLOSE_HIGH_PATHS)
    print()
    print("Import steps:")
    print("  1. Open https://pedro-path-generator.vercel.app/")
    print("  2. Click the menu (≡) → Import")
    print("  3. Select one of the generated .json files above")
    print("  4. All 13 paths load as editable segments on the field")

