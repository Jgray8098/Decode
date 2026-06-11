<#
.SYNOPSIS
    Convert Java Pedro opmode files into visualizer JSON and exported .pp files.
.DESCRIPTION
    Parses a Java opmode file that uses the Pedro Pathing library, extracts all
    Pose definitions and pathBuilder chains (or Mark2-style double[] navigate calls),
    and writes a visualizer-compatible .json file and a Pedro Pathing .pp project file.
.PARAMETER JavaFiles
    One or more input Java opmode .java files.
.PARAMETER OutDir
    Directory to write the generated .json and .pp files.
    Defaults to the pedroPathing output folder in TeamCode.
.EXAMPLE
    .\Convert-JavaToVisualizer.ps1 BlueCloseAutoHigh.java
.EXAMPLE
    .\Convert-JavaToVisualizer.ps1 .\opmodes\*.java -OutDir .\output
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory, Position = 0, ValueFromRemainingArguments)]
    [string[]]$JavaFiles,

    [Parameter()]
    [string]$OutDir
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:DefaultOutputDir = Join-Path $PSScriptRoot 'output'

$script:PP_COLORS = @(
    '#ffc516', '#5B9C9A', '#857B75', '#C9DB67',
    '#60a5fa', '#ff6b6b', '#f97316'
)

# ===========================================================================
# JSON helpers
# ===========================================================================

function ConvertTo-CleanJson {
    <# ConvertTo-Json HTML-escapes <, >, &, ' by default. Reverse that. #>
    param([object]$Object, [int]$Depth = 20)
    $json = $Object | ConvertTo-Json -Depth $Depth
    $json = $json -replace '\\u003e', '>'
    $json = $json -replace '\\u003c', '<'
    $json = $json -replace '\\u0026', '&'
    $json = $json -replace "\\u0027", "'"
    return $json
}

# ===========================================================================
# Arrow / name helpers
# ===========================================================================

function Normalize-Arrow {
    param([string]$Text)
    $Text = $Text -replace [char]0x2192, '->'
    $Text = $Text -replace '\s*->\s*', ' -> '
    return $Text.Trim()
}

function ConvertTo-SnakeStem {
    param([string]$Name)
    $snake = [regex]::Replace($Name, '([a-z0-9])([A-Z])', '$1_$2').ToLower()
    $snake = [regex]::Replace($snake, '[^a-z0-9_]+', '_').Trim('_')
    return $snake
}

function ConvertTo-TitleChainName {
    param([string]$Stem)
    $parts = $Stem.Replace('_', ' ').Split(' ') | Where-Object { $_ }
    return ($parts | ForEach-Object {
        $_.Substring(0, 1).ToUpper() + $_.Substring(1)
    }) -join ' '
}

# ===========================================================================
# Float / heading parsers
# ===========================================================================

function ConvertTo-JavaFloat {
    param([string]$Text)
    return [double]($Text.Trim() -replace '[dfDF]$', '')
}

function ConvertTo-HeadingDeg {
    <# Handles Math.toRadians(...) or bare radian/degree literals. #>
    param([string]$Arg)
    $Arg = $Arg.Trim()
    if ($Arg -match '^Math\.toRadians\((.+)\)$') {
        return ConvertTo-JavaFloat $Matches[1]
    }
    $raw = ConvertTo-JavaFloat $Arg
    if ([Math]::Abs($raw) -le (2.0 * [Math]::PI + 0.5)) {
        return $raw * 180.0 / [Math]::PI
    }
    return $raw
}

# ===========================================================================
# Argument splitting (paren-aware)
# ===========================================================================

function Split-JavaArgs {
    param([string]$Text)
    $out   = [System.Collections.Generic.List[string]]::new()
    $token = [System.Text.StringBuilder]::new()
    $depth = 0
    foreach ($ch in $Text.ToCharArray()) {
        if ($ch -eq '(') { $depth++ }
        elseif ($ch -eq ')') { $depth-- }
        if ($ch -eq ',' -and $depth -eq 0) {
            $out.Add($token.ToString().Trim())
            $token.Clear() | Out-Null
            continue
        }
        $token.Append($ch) | Out-Null
    }
    if ($token.Length -gt 0) { $out.Add($token.ToString().Trim()) }
    return $out
}

# ===========================================================================
# Pose literal / definition parsing
# ===========================================================================

function ConvertTo-PoseLiteral {
    <# Parse 'new Pose(x, y[, heading])' -> @{x;y;heading_deg} or $null #>
    param([string]$Expr)
    $Expr = $Expr.Trim()
    if ($Expr -notmatch '^new\s+Pose\((.*)\)$') { return $null }
    $args = Split-JavaArgs $Matches[1]
    if ($args.Count -lt 2) { return $null }
    $x       = ConvertTo-JavaFloat $args[0]
    $y       = ConvertTo-JavaFloat $args[1]
    $heading = if ($args.Count -ge 3) { ConvertTo-HeadingDeg $args[2] } else { 0.0 }
    return @{ x = $x; y = $y; heading_deg = $heading }
}

function Get-PoseDefinitions {
    <# Equivalent to parse_pose_definitions() #>
    param([string]$Text)
    $resolved = [ordered]@{}
    $aliases  = @{}

    $pattern = 'private\s+static\s+final\s+Pose\s+(\w+)\s*=\s*(.+?);'
    foreach ($m in [regex]::Matches($Text, $pattern)) {
        $name = $m.Groups[1].Value
        $expr = $m.Groups[2].Value.Trim()
        $lit  = ConvertTo-PoseLiteral $expr
        if ($lit) {
            $resolved[$name] = $lit
        } elseif ($expr -match '^\w+$') {
            $aliases[$name] = $expr
        }
    }

    # Resolve aliases iteratively
    $changed = $true
    while ($changed) {
        $changed = $false
        foreach ($name in @($aliases.Keys)) {
            $target = $aliases[$name]
            if ($resolved.Contains($target)) {
                $resolved[$name] = $resolved[$target]
                $aliases.Remove($name)
                $changed = $true
            }
        }
    }
    return $resolved
}

# ===========================================================================
# BezierLine extraction
# ===========================================================================

function Find-MatchingParen {
    param([string]$Text, [int]$OpenIdx)
    $depth = 0
    for ($i = $OpenIdx; $i -lt $Text.Length; $i++) {
        $ch = $Text[$i]
        if ($ch -eq '(') { $depth++ }
        elseif ($ch -eq ')') {
            $depth--
            if ($depth -eq 0) { return $i }
        }
    }
    return -1
}

function Get-BezierPairs {
    # Returns a List[hashtable] with .Start and .End keys.
    # Using hashtables (not arrays) prevents PowerShell pipeline unrolling.
    param([string]$Body)
    $pairs  = [System.Collections.Generic.List[hashtable]]::new()
    $token  = 'new BezierLine('
    $start  = 0
    while ($true) {
        $idx = $Body.IndexOf($token, $start)
        if ($idx -eq -1) { break }
        $openIdx  = $idx + $token.Length - 1
        $closeIdx = Find-MatchingParen $Body $openIdx
        if ($closeIdx -eq -1) { break }
        $inner     = $Body.Substring($openIdx + 1, $closeIdx - $openIdx - 1)
        $splitArgs = Split-JavaArgs $inner
        if ($splitArgs.Count -ge 2) {
            $pairs.Add(@{ Start = $splitArgs[0].Trim(); End = $splitArgs[1].Trim() })
        }
        $start = $closeIdx + 1
    }
    return ,$pairs   # leading comma prevents the List itself from being unrolled
}

# ===========================================================================
# Path chain / segment parsing
# ===========================================================================

function Get-PathChains {
    <# Equivalent to parse_segments() #>
    param([string]$Text)
    $chains  = [ordered]@{}
    $chainRe = [regex]::new(
        '(\w+)\s*=\s*follower\s*\.pathBuilder\(\)(.*?)\.build\(\)\s*;',
        [System.Text.RegularExpressions.RegexOptions]::Singleline
    )
    $headingRe = [regex]::new(
        '\.setLinearHeadingInterpolation\(\s*(.+?)\s*,\s*(.+?)\s*\)',
        [System.Text.RegularExpressions.RegexOptions]::Singleline
    )

    foreach ($m in $chainRe.Matches($Text)) {
        $chainName   = $m.Groups[1].Value
        $body        = $m.Groups[2].Value
        $bezierPairs = Get-BezierPairs $body
        $headings    = $headingRe.Matches($body)
        $count       = [Math]::Min($bezierPairs.Count, $headings.Count)
        if ($count -eq 0) { continue }

        $segs = [System.Collections.Generic.List[object]]::new()
        for ($i = 0; $i -lt $count; $i++) {
            $segs.Add(@{
                chain_name   = $chainName
                start_expr   = $bezierPairs[$i].Start
                end_expr     = $bezierPairs[$i].End
                h_start_expr = $headings[$i].Groups[1].Value.Trim()
                h_end_expr   = $headings[$i].Groups[2].Value.Trim()
            })
        }
        $chains[$chainName] = $segs.ToArray()
    }
    return $chains
}

function Get-OrderedSegments {
    <# Equivalent to ordered_segments() #>
    param([string]$Text, [System.Collections.Specialized.OrderedDictionary]$ChainMap)
    $followRe = [regex]::new('followPath\(\s*paths\.(\w+)')
    $order    = [System.Collections.Generic.List[string]]::new()
    $seen     = [System.Collections.Generic.HashSet[string]]::new()

    foreach ($m in $followRe.Matches($Text)) {
        $name = $m.Groups[1].Value
        if ($ChainMap.Contains($name) -and -not $seen.Contains($name)) {
            $order.Add($name)
            $seen.Add($name) | Out-Null
        }
    }
    if ($order.Count -eq 0) {
        foreach ($k in $ChainMap.Keys) { $order.Add($k) }
    }

    $out = [System.Collections.Generic.List[object]]::new()
    foreach ($chain in $order) { foreach ($seg in $ChainMap[$chain]) { $out.Add($seg) } }
    return $out.ToArray()
}

# ===========================================================================
# Pose reference resolver
# ===========================================================================

$script:LiteralCounter = 0

function Resolve-PoseRef {
    param([string]$Expr, [System.Collections.Specialized.OrderedDictionary]$Poses)
    $Expr = $Expr.Trim()
    if ($Poses.Contains($Expr)) { return $Expr, $Poses[$Expr] }
    $lit = ConvertTo-PoseLiteral $Expr
    if ($lit) {
        $script:LiteralCounter++
        $name = 'LITERAL_POINT_{0:D2}' -f $script:LiteralCounter
        return $name, $lit
    }
    throw "Could not resolve pose expression: $Expr"
}

function Resolve-HeadingDeg {
    param([string]$Expr, [System.Collections.Specialized.OrderedDictionary]$Poses, [double]$FallbackDeg)
    $Expr = $Expr.Trim()
    if ($Expr -match '^(\w+)\.getHeading\(\)$' -and $Poses.Contains($Matches[1])) {
        return $Poses[$Matches[1]].heading_deg
    }
    try { return ConvertTo-HeadingDeg $Expr }
    catch { return $FallbackDeg }
}

# ===========================================================================
# Mark2 style parsing (double[] arrays + navigate() calls)
# ===========================================================================

function Get-Mark2Arrays {
    param([string]$Text)
    $points  = [ordered]@{}
    $pattern = 'private\s+static\s+final\s+double\[\]\s+(\w+)\s*=\s*\{([^}]*)\}\s*;'
    foreach ($m in [regex]::Matches($Text, $pattern)) {
        $name   = $m.Groups[1].Value
        $values = ($m.Groups[2].Value -split ',') |
                  Where-Object { $_.Trim() } |
                  ForEach-Object { $_.Trim() }
        if ($values.Count -lt 3) { continue }
        $points[$name] = @{
            x           = ConvertTo-JavaFloat $values[0]
            y           = ConvertTo-JavaFloat $values[1]
            heading_deg = ConvertTo-JavaFloat $values[2]
        }
    }
    return $points
}

function Get-Mark2StartPose {
    param([string]$Text)
    $scalars = @{}
    $pattern = 'private\s+static\s+final\s+double\s+(START_X|START_Y|START_ROT)\s*=\s*([^;]+);'
    foreach ($m in [regex]::Matches($Text, $pattern)) {
        $scalars[$m.Groups[1].Value] = ConvertTo-JavaFloat $m.Groups[2].Value
    }
    if ($scalars.ContainsKey('START_X') -and
        $scalars.ContainsKey('START_Y') -and
        $scalars.ContainsKey('START_ROT')) {
        return 'START_POSE', @{
            x           = $scalars['START_X']
            y           = $scalars['START_Y']
            heading_deg = $scalars['START_ROT']
        }
    }
    return $null, $null
}

function Build-Mark2VisualizerPaths {
    param([string]$Text)
    $named              = Get-Mark2Arrays $Text
    $startName, $startPose = Get-Mark2StartPose $Text
    if ($named.Count -eq 0 -or -not $startPose) { return @() }

    $navigateRe  = [regex]::new(
        '\bnavigate(?:WithIntake)?\(\s*(\w+)\[0\]\s*,\s*\1\[1\]\s*,\s*\1\[2\]\s*\)'
    )
    $currentName = $startName
    $currentPose = $startPose
    $paths       = [System.Collections.Generic.List[object]]::new()
    $idx         = 1

    foreach ($m in $navigateRe.Matches($Text)) {
        $targetName = $m.Groups[1].Value
        if (-not $named.Contains($targetName)) { continue }
        $targetPose = $named[$targetName]
        $paths.Add(@{
            name      = Normalize-Arrow "$idx $currentName -> $targetName"
            waypoints = @(
                @{ x = $currentPose.x; y = $currentPose.y; heading = $currentPose.heading_deg }
                @{ x = $targetPose.x; y = $targetPose.y; heading = $targetPose.heading_deg }
            )
        })
        $currentName = $targetName
        $currentPose = $targetPose
        $idx++
    }
    return $paths.ToArray()
}

# ===========================================================================
# Main path builder
# ===========================================================================

function Build-VisualizerPaths {
    param([string]$Text)
    $poses    = Get-PoseDefinitions $Text
    $chainMap = Get-PathChains $Text
    $segs     = Get-OrderedSegments $Text $chainMap

    if ($segs.Count -eq 0) {
        $mark2 = Build-Mark2VisualizerPaths $Text
        if ($mark2.Count -gt 0) { return $mark2 }
        throw 'No Pedro path segments found in Java file.'
    }

    $script:LiteralCounter = 0
    $paths = [System.Collections.Generic.List[object]]::new()

    for ($idx = 1; $idx -le $segs.Count; $idx++) {
        $seg = $segs[$idx - 1]
        $startName, $startPose = Resolve-PoseRef $seg.start_expr $poses
        $endName,   $endPose   = Resolve-PoseRef $seg.end_expr   $poses
        $hStart = Resolve-HeadingDeg $seg.h_start_expr $poses $startPose.heading_deg
        $hEnd   = Resolve-HeadingDeg $seg.h_end_expr   $poses $endPose.heading_deg

        $paths.Add(@{
            name      = Normalize-Arrow "$idx $startName -> $endName"
            waypoints = @(
                @{ x = $startPose.x; y = $startPose.y; heading = $hStart }
                @{ x = $endPose.x;   y = $endPose.y;   heading = $hEnd   }
            )
        })
    }
    return $paths.ToArray()
}

# ===========================================================================
# .pp conversion (inlined from Convert-JsonToPedroPp.ps1)
# ===========================================================================

function Convert-JsonPathsToPedroProject {
    param([object[]]$Paths, [string]$ChainName)

    $firstWp     = $Paths[0].waypoints[0]
    $lastWpFirst = $Paths[0].waypoints[-1]

    $startPoint = [ordered]@{
        x        = [double]$firstWp.x
        y        = [double]$firstWp.y
        heading  = 'linear'
        startDeg = [double]$firstWp.heading
        endDeg   = [double]$lastWpFirst.heading
        locked   = $false
    }

    $linesList = [System.Collections.Generic.List[object]]::new()
    $lineIds   = [System.Collections.Generic.List[string]]::new()

    for ($i = 0; $i -lt $Paths.Count; $i++) {
        $path    = $Paths[$i]
        $lineId  = 'line-{0:D3}' -f ($i + 1)
        $lineIds.Add($lineId)
        $color   = $script:PP_COLORS[$i % $script:PP_COLORS.Count]
        $startWp = $path.waypoints[0]
        $endWp   = $path.waypoints[-1]

        $linesList.Add([ordered]@{
            id             = $lineId
            name           = Normalize-Arrow $path.name
            endPoint       = [ordered]@{
                x        = [double]$endWp.x
                y        = [double]$endWp.y
                heading  = 'linear'
                startDeg = [double]$startWp.heading
                endDeg   = [double]$endWp.heading
            }
            controlPoints  = @()
            color          = $color
            locked         = $false
            waitBeforeMs   = 0
            waitAfterMs    = 0
            waitBeforeName = ''
            waitAfterName  = ''
        })
    }

    $seqList = [System.Collections.Generic.List[object]]::new()
    foreach ($id in $lineIds) {
        $seqList.Add([ordered]@{ kind = 'path'; lineId = $id })
    }

    return [ordered]@{
        startPoint = $startPoint
        lines      = $linesList.ToArray()
        shapes     = @()
        sequence   = $seqList.ToArray()
        pathChains = @(
            [ordered]@{
                id      = "chain-$ChainName"
                name    = $ChainName
                color   = '#C9DB67'
                lineIds = $lineIds.ToArray()
            }
        )
        version    = '1.2.1'
        timestamp  = (Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ss.fffZ')
    }
}

# ===========================================================================
# Per-file output writer
# ===========================================================================

function Write-Outputs {
    param([string]$JavaFilePath, [string]$OutputDir)

    $source    = [System.IO.File]::ReadAllText($JavaFilePath, [System.Text.Encoding]::UTF8)
    $paths     = Build-VisualizerPaths $source

    $stem      = ConvertTo-SnakeStem ([System.IO.Path]::GetFileNameWithoutExtension($JavaFilePath))
    $baseName  = "pedro_paths_$stem"
    $jsonOut   = Join-Path $OutputDir "$baseName.json"
    $ppOut     = Join-Path $OutputDir "$baseName.pp"

    $jsonText  = ConvertTo-CleanJson $paths
    [System.IO.File]::WriteAllText($jsonOut, ($jsonText + "`n"), [System.Text.Encoding]::UTF8)

    $chainName = ConvertTo-TitleChainName $stem
    $project   = Convert-JsonPathsToPedroProject -Paths $paths -ChainName $chainName
    $ppText    = ConvertTo-CleanJson $project
    [System.IO.File]::WriteAllText($ppOut, ($ppText + "`n"), [System.Text.Encoding]::UTF8)

    return $jsonOut, $ppOut
}

# ===========================================================================
# Main
# ===========================================================================

$resolvedOutDir = if ($OutDir) {
    if ([System.IO.Path]::IsPathRooted($OutDir)) { $OutDir }
    else { Join-Path (Get-Location) $OutDir }
} else {
    $script:DefaultOutputDir
}

[System.IO.Directory]::CreateDirectory($resolvedOutDir) | Out-Null

foreach ($filePattern in $JavaFiles) {
    $items = Get-Item -Path $filePattern -ErrorAction SilentlyContinue
    if (-not $items) { $items = Get-Item -LiteralPath $filePattern }
    foreach ($item in $items) {
        try {
            $jsonOut, $ppOut = Write-Outputs -JavaFilePath $item.FullName -OutputDir $resolvedOutDir
            Write-Host "Wrote $jsonOut"
            Write-Host "Wrote $ppOut"
        } catch {
            Write-Warning "Skipped $($item.Name): $_"
        }
    }
}









