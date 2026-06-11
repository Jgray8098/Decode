<#
.SYNOPSIS
    Convert Pedro .pp project files into named Java Pose points.
.DESCRIPTION
    Reads a Pedro Pathing .pp project file and generates a Java code fragment
    containing named Pose constants and a Paths inner class with PathChain builder
    calls, ready to paste into an opmode.

    When a bare filename (no path separator) is given and the file does not exist
    in the current directory, the script automatically searches the
    Scripts\InputAndOutput folder so you can run:
        .\Convert-PedroToJavaPoints.ps1 Mark2BlueInitialAuto.pp
    from the Scripts\ directory without typing the full path.
.PARAMETER InputFiles
    One or more .pp input files.  Bare filenames are resolved against
    Scripts\InputAndOutput if not found in the working directory.
.PARAMETER OutDir
    Directory to write output .javafragment files.
    Defaults to Scripts\InputAndOutput.
.PARAMETER OutputFile
    Explicit output path (only valid when exactly one InputFile is given).
.EXAMPLE
    .\Convert-PedroToJavaPoints.ps1 Mark2BlueInitialAuto.pp
.EXAMPLE
    .\Convert-PedroToJavaPoints.ps1 .\InputAndOutput\*.pp -OutDir .\InputAndOutput
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory, Position = 0, ValueFromRemainingArguments)]
    [string[]]$InputFiles,

    [Parameter()]
    [string]$OutDir,

    [Parameter()]
    [string]$OutputFile
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:DefaultOutputDir = Join-Path $PSScriptRoot 'InputAndOutput'

# ---------------------------------------------------------------------------
# Shared helper
# ---------------------------------------------------------------------------

function Normalize-Arrow {
    param([string]$Text)
    $Text = $Text -replace [char]0x2192, '->'
    $Text = $Text -replace '\s*->\s*', ' -> '
    return $Text.Trim()
}

# ---------------------------------------------------------------------------
# Name utilities
# ---------------------------------------------------------------------------

function ConvertTo-ConstName {
    <# Equivalent to sanitize_const_name() #>
    param([string]$Raw, [string]$Fallback)
    $text = [regex]::Replace((Normalize-Arrow $Raw).Trim(), '^\d+\s*', '')
    $text = [regex]::Replace($text, '[^A-Za-z0-9]+', '_').Trim('_').ToUpper()
    if (-not $text) { $text = $Fallback }
    if ($text -notmatch '_POSE' -and -not $text.EndsWith('_POSE')) {
        $text = "${text}_POSE"
    }
    return $text
}

function Split-SegmentNames {
    <# Splits "A -> B" or "A to B" (Pedro visualizer style) into two const names. #>
    param([string]$LineName, [int]$Idx)
    $cleaned = [regex]::Replace((Normalize-Arrow $LineName), '^\d+\s*', '').Trim()

    # Prefer '->' separator; also accept ' to ' (case-insensitive, word boundary)
    if ($cleaned -match '->') {
        $parts = $cleaned -split '->', 2
        $left  = ConvertTo-ConstName $parts[0].Trim() ('POINT_{0:D2}' -f $Idx)
        $right = ConvertTo-ConstName $parts[1].Trim() ('POINT_{0:D2}' -f ($Idx + 1))
        return $left, $right
    }
    if ($cleaned -match '(?i)\bto\b') {
        $parts = [regex]::Split($cleaned, '(?i)\s+to\s+', 2)
        if ($parts.Count -eq 2) {
            $left  = ConvertTo-ConstName $parts[0].Trim() ('POINT_{0:D2}' -f $Idx)
            $right = ConvertTo-ConstName $parts[1].Trim() ('POINT_{0:D2}' -f ($Idx + 1))
            return $left, $right
        }
    }
    # No recognisable separator — use generic fallbacks
    $p1 = ConvertTo-ConstName ('POINT_{0:D2}' -f $Idx)  ('POINT_{0:D2}' -f $Idx)
    $p2 = ConvertTo-ConstName $cleaned                   ('POINT_{0:D2}' -f ($Idx + 1))
    return $p1, $p2
}

function ConvertTo-PascalFromConst {
    <# Equivalent to to_pascal_from_const() #>
    param([string]$Name)
    $core  = $Name -replace '_POSE$', ''
    $parts = $core.Split('_') | Where-Object { $_ -ne '' }
    return ($parts | ForEach-Object {
        $_.Substring(0, 1).ToUpper() + $_.Substring(1).ToLower()
    }) -join ''
}

function ConvertTo-CamelCase {
    param([string]$Text)
    if (-not $Text) { return $Text }
    return $Text.Substring(0, 1).ToLower() + $Text.Substring(1)
}

function Get-PathMemberName {
    <# Derives a camelCase PathChain member name from the segment's start/end names. #>
    param($Segment, [int]$Idx, [System.Collections.Generic.HashSet[string]]$Used)

    $startPascal = ConvertTo-PascalFromConst $Segment.start_name
    $endPascal   = ConvertTo-PascalFromConst $Segment.end_name

    # Always prefer the full "startToEnd" form so the path name is self-describing.
    $preferred = ConvertTo-CamelCase "${startPascal}To${endPascal}"
    if ($preferred -and -not $Used.Contains($preferred)) {
        $Used.Add($preferred) | Out-Null
        return $preferred
    }

    # Collision — append a numeric suffix.
    $i = 2
    while ($true) {
        $candidate = "${preferred}${i}"
        if (-not $Used.Contains($candidate)) {
            $Used.Add($candidate) | Out-Null
            return $candidate
        }
        $i++
    }
}

function Build-NamedSegments {
    param([object[]]$ResolvedSegments)
    $used = [System.Collections.Generic.HashSet[string]]::new()
    $out  = [System.Collections.Generic.List[object]]::new()
    for ($i = 0; $i -lt $ResolvedSegments.Count; $i++) {
        $seg        = $ResolvedSegments[$i]
        $memberName = Get-PathMemberName $seg ($i + 1) $used
        $out.Add(@{ member_name = $memberName; segment = $seg })
    }
    return $out
}

# ---------------------------------------------------------------------------
# Pose equality / unique-name registry
# ---------------------------------------------------------------------------

function Test-PoseEqual {
    param($A, $B)
    return ([Math]::Abs($A.x - $B.x) -lt 1e-9) -and
           ([Math]::Abs($A.y - $B.y) -lt 1e-9) -and
           ([Math]::Abs($A.heading_deg - $B.heading_deg) -lt 1e-9)
}

function Ensure-Unique {
    <# Equivalent to ensure_unique() #>
    param([string]$Name, [System.Collections.Specialized.OrderedDictionary]$Registry, $Pose)
    if (-not $Registry.Contains($Name)) {
        $Registry[$Name] = $Pose
        return $Name
    }
    if (Test-PoseEqual $Registry[$Name] $Pose) { return $Name }
    $i = 2
    while ($true) {
        $candidate = "${Name}_${i}"
        if (-not $Registry.Contains($candidate)) {
            $Registry[$candidate] = $Pose
            return $candidate
        }
        if (Test-PoseEqual $Registry[$candidate] $Pose) { return $candidate }
        $i++
    }
}

# ---------------------------------------------------------------------------
# Segment readers
# ---------------------------------------------------------------------------

function Get-OptionalDeg {
    <#
        Safely read a numeric degree property from a PSCustomObject.
        Returns $Fallback when the property is absent (tangential segments
        do not have startDeg / endDeg) or when the value is not numeric.
    #>
    param([object]$Obj, [string]$PropName, [double]$Fallback)
    $props = $Obj.PSObject.Properties
    if (-not $props[$PropName]) { return $Fallback }
    $val = $props[$PropName].Value
    if ($null -eq $val) { return $Fallback }
    try { return [double]$val } catch { return $Fallback }
}

function Read-SegmentsFromPp {
    param([object]$Project)
    $start   = $Project.startPoint
    $lines   = $Project.lines
    $seq     = $Project.sequence
    $lineMap = @{}
    foreach ($line in $lines) { $lineMap[$line.id] = $line }

    $current = @{
        x           = [double]$start.x
        y           = [double]$start.y
        heading_deg = Get-OptionalDeg $start 'startDeg' 0.0
    }

    $segments = [System.Collections.Generic.List[object]]::new()
    $idx = 1

    foreach ($item in $seq) {
        if ($item.kind -ne 'path') { continue }
        $line = $lineMap[$item.lineId]
        if (-not $line) { continue }

        $endp    = $line.endPoint
        # For tangential segments startDeg/endDeg are absent; fall back to the
        # incoming heading so the Java fragment stays numerically consistent.
        $startDeg = Get-OptionalDeg $endp 'startDeg' $current.heading_deg
        $endDeg   = Get-OptionalDeg $endp 'endDeg'   $current.heading_deg

        $end = @{
            x           = [double]$endp.x
            y           = [double]$endp.y
            heading_deg = $endDeg
        }

        $rawName   = [string]$line.name
        $names     = Split-SegmentNames $rawName $idx
        $startName = $names[0]
        $endName   = $names[1]

        $startWithHeading = @{
            x           = $current.x
            y           = $current.y
            heading_deg = $startDeg
        }

        $segments.Add(@{
            name       = Normalize-Arrow $rawName
            start_name = $startName
            end_name   = $endName
            start      = $startWithHeading
            end        = $end
        })

        $current = $end
        $idx++
    }

    if ($segments.Count -eq 0) { throw "No path sequence entries found in .pp file." }
    return $segments.ToArray()
}

# ---------------------------------------------------------------------------
# Java renderer
# ---------------------------------------------------------------------------

function Render-JavaFragment {
    param([string]$ClassName, [object[]]$Segments)

    $namedPoints      = [System.Collections.Specialized.OrderedDictionary]::new()
    $resolvedSegments = [System.Collections.Generic.List[object]]::new()

    foreach ($seg in $Segments) {
        $sName = Ensure-Unique $seg.start_name $namedPoints $seg.start
        $eName = Ensure-Unique $seg.end_name   $namedPoints $seg.end
        $resolvedSegments.Add(@{
            name       = $seg.name
            start_name = $sName
            end_name   = $eName
            start      = $seg.start
            end        = $seg.end
        })
    }

    $namedSegs = Build-NamedSegments $resolvedSegments.ToArray()

    $lines = [System.Collections.Generic.List[string]]::new()

    foreach ($name in $namedPoints.Keys) {
        $pose = $namedPoints[$name]
        $lines.Add(
            "private static final Pose $name = new Pose($($pose.x.ToString('F3')), $($pose.y.ToString('F3')), Math.toRadians($($pose.heading_deg.ToString('F3'))));"
        )
    }

    $lines.Add('')
    $lines.Add('public static class Paths {')
    foreach ($ns in $namedSegs) {
        $lines.Add("    public PathChain $($ns.member_name);")
    }

    $lines.Add('')
    $lines.Add('    public Paths(Follower follower) {')
    foreach ($ns in $namedSegs) {
        $seg = $ns.segment
        $lines.Add("        // $(Normalize-Arrow $seg.name)")
        $lines.Add("        $($ns.member_name) = follower")
        $lines.Add('                .pathBuilder()')
        $lines.Add("                .addPath(new BezierLine($($seg.start_name), $($seg.end_name)))")
        $lines.Add("                .setLinearHeadingInterpolation($($seg.start_name).getHeading(), $($seg.end_name).getHeading())")
        $lines.Add('                .build();')
        $lines.Add('')
    }

    $lines.Add('    }')
    $lines.Add('}')
    $lines.Add('')

    return $lines -join "`n"
}

function ConvertTo-ClassNameFromStem {
    param([string]$Stem)
    $parts = [regex]::Split($Stem, '[^A-Za-z0-9]+')
    return ($parts | Where-Object { $_ } | ForEach-Object {
        $_.Substring(0, 1).ToUpper() + $_.Substring(1)
    }) -join '' | ForEach-Object { $_ + 'Points' }
}

# ---------------------------------------------------------------------------
# File converter
# ---------------------------------------------------------------------------

function Convert-PedroFile {
    param([string]$InputPath, [string]$OutputDir, [string]$OutputFilePath)

    $payload = Get-Content -Path $InputPath -Raw -Encoding UTF8 | ConvertFrom-Json

    $ext = [System.IO.Path]::GetExtension($InputPath).ToLower()
    if ($ext -eq '.pp') {
        $segments = Read-SegmentsFromPp $payload
    } else {
        throw "Unsupported input format for $([System.IO.Path]::GetFileName($InputPath)). Only .pp files are supported."
    }

    $stem      = [System.IO.Path]::GetFileNameWithoutExtension($InputPath)
    $className = ConvertTo-ClassNameFromStem $stem

    if ($OutputFilePath) {
        $outPath = $OutputFilePath
    } else {
        $outPath = Join-Path $OutputDir "$stem`_points.javafragment"
    }

    $java = Render-JavaFragment -ClassName $className -Segments $segments
    [System.IO.File]::WriteAllText($outPath, $java, [System.Text.Encoding]::UTF8)
    return $outPath
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if ($OutputFile -and $InputFiles.Count -ne 1) {
    throw '-OutputFile can only be used with exactly one input file.'
}

$resolvedOutDir = if ($OutDir) {
    if ([System.IO.Path]::IsPathRooted($OutDir)) { $OutDir }
    else { Join-Path (Get-Location) $OutDir }
} else {
    $script:DefaultOutputDir
}

[System.IO.Directory]::CreateDirectory($resolvedOutDir) | Out-Null

foreach ($file in $InputFiles) {
    # First try the path exactly as given (relative or absolute).
    $resolved = Get-Item -Path $file -ErrorAction SilentlyContinue

    # If not found and the input looks like a bare filename (no directory separator),
    # try Scripts\InputAndOutput automatically.
    if (-not $resolved -and $file -notmatch '[/\\]') {
        $fallback = Join-Path $script:DefaultOutputDir $file
        $resolved = Get-Item -Path $fallback -ErrorAction SilentlyContinue
    }

    if (-not $resolved) { $resolved = Get-Item -LiteralPath $file }
    foreach ($item in $resolved) {
        $outFile = if ($OutputFile) {
            if ([System.IO.Path]::IsPathRooted($OutputFile)) { $OutputFile }
            else { Join-Path (Get-Location) $OutputFile }
        } else { $null }

        if ($outFile) {
            [System.IO.Directory]::CreateDirectory(
                [System.IO.Path]::GetDirectoryName($outFile)
            ) | Out-Null
        }

        $outPath = Convert-PedroFile -InputPath $item.FullName `
                                     -OutputDir $resolvedOutDir `
                                     -OutputFilePath $outFile
        Write-Host "Wrote $outPath"
    }
}

