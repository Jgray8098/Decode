# Pedro Path Conversion Scripts

These scripts convert between Java opmode paths and Pedro Pathing `.pp` project
files so you can edit paths visually in the Pedro visualizer and keep Java in sync.

---

## ⚡ Quick Reference — these are the two commands you need

### Java → .pp  *(before opening the Pedro visualizer)*
```powershell
.\Convert-JavaToPedroPp.ps1 Mark2BlueInitialAuto.java
```
Reads the Java opmode, extracts every pose and path, and writes  
`Scripts\InputAndOutput\Mark2BlueInitialAuto.pp`.  
Open that file in the Pedro visualizer to edit the paths.

---

### .pp → Java fragment  *(after saving from the Pedro visualizer)*
```powershell
.\Convert-PedroToJavaPoints.ps1 Mark2BlueInitialAuto.pp
```
Reads the saved `.pp` file and writes  
`Scripts\InputAndOutput\Mark2BlueInitialAuto_points.javafragment`.  
Open that file, select all, and paste it into the marked section of the OpMode.

---

> **Both commands are run from the `Scripts\` directory.**  
> Bare filenames (no path) are resolved automatically:  
> — `.java` files are looked up in `TeamCode\…\opmodes\`  
> — `.pp` files are looked up in `Scripts\InputAndOutput\`

---

## Full workflow

```
Java opmode  ──►  Convert-JavaToPedroPp.ps1  ──►  .pp  (open in Pedro visualizer, edit paths)
                                                      │
                                                      ▼ (save from visualizer)
Java opmode  ◄──  Convert-PedroToJavaPoints.ps1  ◄──  .pp
                  (paste .javafragment into OpMode marked section)
```

---

## Script details

### `Convert-JavaToPedroPp.ps1`

Parses `Pose` constants and `BezierLine` path segments (or Mark2-style `double[]`
navigate calls) from a Java opmode and writes a `.pp` project file.

```powershell
# Bare filename — resolves from TeamCode\...\opmodes\ automatically
.\Convert-JavaToPedroPp.ps1 Mark2BlueInitialAuto.java

# Explicit path
.\Convert-JavaToPedroPp.ps1 ..\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\opmodes\Mark2BlueInitialAuto.java

# Multiple files
.\Convert-JavaToPedroPp.ps1 Mark2BlueInitialAuto.java Mark2BlueCloseAutoHigh.java
```

Output: `Scripts\InputAndOutput\pedro_paths_<snake_name>.pp`

---

### `Convert-PedroToJavaPoints.ps1`

Reads a `.pp` project file and generates a Java fragment with named `Pose`
constants and a `Paths` inner class ready to paste into an OpMode.

```powershell
# Bare filename — resolves from Scripts\InputAndOutput\ automatically
.\Convert-PedroToJavaPoints.ps1 Mark2BlueInitialAuto.pp

# Explicit relative path
.\Convert-PedroToJavaPoints.ps1 .\InputAndOutput\Mark2BlueInitialAuto.pp

# All .pp files at once
.\Convert-PedroToJavaPoints.ps1 .\InputAndOutput\*.pp
```

Output: `Scripts\InputAndOutput\<input_stem>_points.javafragment`

The `.javafragment` extension keeps generated snippets out of the Gradle compile path.

---

## Notes

- All output defaults to `Scripts\InputAndOutput\` — no `-OutDir` flag needed.
- Path names use `A -> B` arrow notation to preserve point names through the round-trip.
- `Math.toRadians(...)` headings and bare radian literals are both handled automatically.
- Tangential segments (no explicit `startDeg`/`endDeg`) fall back to carrying the
  incoming heading forward so the fragment always has numeric values.
