# Pedro Path Conversion Scripts

These scripts provide two-way conversion between FTC Java opmode paths and Pedro Pathing visualizer/export formats.

## 1) Java opmode -> JSON + .pp

`java_to_pedro_visualizer.py`

- Parses `Pose` constants and `BezierLine` path segments from Java opmodes.
- Writes visualizer `.json` and Pedro export `.pp` in one command.
- Uses ASCII arrows (`->`) in path names.
- Relative input paths are resolved from `pedroPathing/scripts` first, then `pedroPathing/output`.
- Default output is `pedroPathing/output`.

```powershell
python "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\pedroPathing\scripts\java_to_pedro_visualizer.py" "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\opmodes\BlueCloseAutoHigh.java" "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\opmodes\RedCloseAutoHigh.java"
```

```powershell
python "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\pedroPathing\scripts\java_to_pedro_visualizer.py" "..\..\opmodes\BlueCloseAutoHigh.java"
```

## 2) .pp/.json -> Java points

`scripts/pedro_to_java_points.py`

- Parses path sequence from `.pp` (or visualizer `.json`).
- Generates a Java fragment with named `Pose` constants and a `Paths` class.
- Omits imports and top-level class boilerplate (fragment-only output).
- Uses line names like `A -> B` to preserve point naming.
- Derives `PathChain` member names from segment names (falls back only when needed).
- Relative input paths are resolved from `pedroPathing/scripts` first, then `pedroPathing/output`.
- Default Java output is `pedroPathing/output`.
- Default extension is `.javafragment` so generated snippets are not compiled with opmodes.

```powershell
python "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\pedroPathing\scripts\pedro_to_java_points.py" "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\pedroPathing\output\pedro_paths_blue_close_auto_high.pp"
```

```powershell
python "C:\DEV\robot\2026\PioneerMainSeason\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\pedroPathing\scripts\pedro_to_java_points.py" "pedro_paths_blue_close_auto_high.pp" --output-file "fragments\blue_close_points_fragment.javafragment"
```

