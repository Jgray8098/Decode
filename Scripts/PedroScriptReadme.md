# Pedro Path Conversion Scripts

These scripts provide two-way conversion between FTC Java opmode paths and Pedro Pathing `.pp` project files.

Input and output files live in `Scripts\InputAndOutput\`.

---

## Workflow

```
Java opmode  ──►  Convert-JavaToPedroPp.ps1  ──►  .pp file  (open in Pedro visualizer)
                                                        │
                                                        ▼
Java opmode  ◄──  Convert-PedroToJavaPoints.ps1  ◄──  .pp file  (edited in Pedro visualizer)
```

---

## 1) Java opmode → .pp

**`Convert-JavaToPedroPp.ps1`**

Parses `Pose` constants and `BezierLine` path segments (or Mark2-style `double[]` navigate calls)
from a Java opmode and writes a Pedro Pathing `.pp` project file ready to open in the visualizer.

```powershell
# Single file — output goes to Scripts\InputAndOutput\
.\Scripts\Convert-JavaToPedroPp.ps1 .\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\opmodes\BlueCloseAutoHigh.java

# Multiple files
.\Scripts\Convert-JavaToPedroPp.ps1 .\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\opmodes\BlueCloseAutoHigh.java `
                                     .\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\opmodes\RedCloseAutoHigh.java

# Custom output directory
.\Scripts\Convert-JavaToPedroPp.ps1 BlueCloseAutoHigh.java -OutDir .\Scripts\InputAndOutput
```

Output file naming: `pedro_paths_<snake_stem>.pp`

---

## 2) .pp → Java points fragment

**`Convert-PedroToJavaPoints.ps1`**

Reads a `.pp` project file (e.g. exported from the Pedro visualizer or produced by the script above)
and generates a Java fragment with named `Pose` constants and a `Paths` inner class containing
`PathChain` builder calls, ready to paste into an opmode.

```powershell
# Single file — output goes to Scripts\InputAndOutput\
.\Scripts\Convert-PedroToJavaPoints.ps1 .\Scripts\InputAndOutput\pedro_paths_blue_close_auto_high.pp

# All .pp files in InputAndOutput
.\Scripts\Convert-PedroToJavaPoints.ps1 .\Scripts\InputAndOutput\*.pp

# Explicit output file
.\Scripts\Convert-PedroToJavaPoints.ps1 .\Scripts\InputAndOutput\pedro_paths_blue_close_auto_high.pp `
    -OutputFile .\Scripts\InputAndOutput\blue_close_points.javafragment
```

Output file naming: `<input_stem>_points.javafragment`

The `.javafragment` extension keeps generated snippets out of the Gradle compile path.

---

## Notes

- All output defaults to `Scripts\InputAndOutput\` — no `-OutDir` needed when running from the project root.
- Path names use `A -> B` arrow notation to preserve point names through the round-trip.
- `Math.toRadians(...)` headings and bare radian literals are both handled automatically.
