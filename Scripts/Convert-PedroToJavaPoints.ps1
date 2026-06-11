<#
.SYNOPSIS
    Convert Pedro .pp or visualizer .json files into named Java Pose points.
.DESCRIPTION
    Reads a Pedro Pathing .pp project file or a visualizer JSON array and generates
    a Java code fragment containing named Pose constants and a Paths inner class with
    PathChain builder calls, ready to paste into an opmode.
.PARAMETER InputFiles
    One or more .pp or .json input files.
.PARAMETER OutDir
    Directory to write output .javafragment files.
    Defaults to the pedroPathing output folder in TeamCode.
.PARAMETER OutputFile
    Explicit output path (only valid when exactly one InputFile is given).
.EXAMPLE
    .\Convert-PedroToJavaPoints.ps1 pedro_paths_blue_close_auto_high.pp
.EXAMPLE
    .\Convert-PedroToJavaPoints.ps1 .\output\*.pp -OutDir .\output
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

$script:DefaultOutputDir = Join-Path $PSScriptRoot 'output'

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
    <# Equivalent to split_segment_names() #>
    param([string]$LineName, [int]$Idx)
    $cleaned = [regex]::Replace((Normalize-Arrow $LineName), '^\d+\s*', '').Trim()
    if ($cleaned -match '->') {
        $parts = $cleaned -split '->', 2
        $left  = ConvertTo-ConstName $parts[0].Trim() ('POINT_{0:D2}' -f $Idx)
        $right = ConvertTo-ConstName $parts[1].Trim() ('POINT_{0:D2}' -f ($Idx + 1))
        return $left, $right
    }
    $p1 = ConvertTo-ConstName ('POINT_{0:D2}' -f $Idx)       ('POINT_{0:D2}' -f $Idx)
    $p2 = ConvertTo-ConstName $cleaned                        ('POINT_{0:D2}' -f ($Idx + 1))
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
    <# Equivalent to derive_path_member_name() #>
    param($Segment, [int]$Idx, [System.Collections.Generic.HashSet[string]]$Used)

    $startPascal = ConvertTo-PascalFromConst $Segment.start_name
    $endPascal   = ConvertTo-PascalFromConst $Segment.end_name

    $preferred = ConvertTo-CamelCase $endPascal
    if ($preferred -and -not $Used.Contains($preferred)) {
        $Used.Add($preferred) | Out-Null
        return $preferred
    }

    $combined = ConvertTo-CamelCase "${startPascal}To${endPascal}"
    if (-not $Used.Contains($combined)) {
        $Used.Add($combined) | Out-Null
        return $combined
    }

    $fallback = 'segment{0:D2}' -f $Idx
    while ($Used.Contains($fallback)) {
        $Idx++
        $fallback = 'segment{0:D2}' -f $Idx
    }
    $Used.Add($fallback) | Out-Null
    return $fallback
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
        heading_deg = [double]$start.startDeg
    }

    $segments = [System.Collections.Generic.List[object]]::new()
    $idx = 1

    foreach ($item in $seq) {
        if ($item.kind -ne 'path') { continue }
        $line = $lineMap[$item.lineId]
        if (-not $line) { continue }

        $endp = $line.endPoint
        $end  = @{
            x           = [double]$endp.x
            y           = [double]$endp.y
            heading_deg = [double]$endp.endDeg
        }

        $rawName   = [string]$line.name
        $names     = Split-SegmentNames $rawName $idx
        $startName = $names[0]
        $endName   = $names[1]

        $startWithHeading = @{
            x           = $current.x
            y           = $current.y
            heading_deg = [double]$endp.startDeg
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

function Read-SegmentsFromJson {
    param([object[]]$Paths)
    $segments = [System.Collections.Generic.List[object]]::new()

    for ($i = 0; $i -lt $Paths.Count; $i++) {
        $path = $Paths[$i]
        $wps  = $path.waypoints
        if ($wps.Count -lt 2) { continue }
        $startWp = $wps[0]
        $endWp   = $wps[-1]
        $rawName = [string]$path.name
        $idx     = $i + 1
        $names   = Split-SegmentNames $rawName $idx

        $segments.Add(@{
            name       = Normalize-Arrow $rawName
            start_name = $names[0]
            end_name   = $names[1]
            start      = @{ x = [double]$startWp.x; y = [double]$startWp.y; heading_deg = [double]$startWp.heading }
            end        = @{ x = [double]$endWp.x;   y = [double]$endWp.y;   heading_deg = [double]$endWp.heading }
        })
    }

    if ($segments.Count -eq 0) { throw "No valid paths found in .json file." }
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
    } elseif ($payload -is [array]) {
        $segments = Read-SegmentsFromJson $payload
    } else {
        throw "Unsupported input format for $([System.IO.Path]::GetFileName($InputPath))"
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
    # Support glob expansion
    $resolved = Get-Item -Path $file -ErrorAction SilentlyContinue
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

