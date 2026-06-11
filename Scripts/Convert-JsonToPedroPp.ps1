<#
.SYNOPSIS
    Convert visualizer JSON path files to Pedro Pathing .pp format.
.DESCRIPTION
    Reads a JSON file containing an array of path waypoints and converts it
    to the Pedro Pathing Visualizer .pp project format.
.PARAMETER InputFile
    Path to the input .json file (array of {name, waypoints:[{x,y,heading}]}).
.PARAMETER OutputFile
    Path to the output .pp file. Defaults to same stem as input with .pp extension.
.PARAMETER ChainName
    Name for the path chain. Defaults to a title-cased version of the file stem.
.EXAMPLE
    .\Convert-JsonToPedroPp.ps1 -InputFile pedro_paths_red_close_high.json
.EXAMPLE
    .\Convert-JsonToPedroPp.ps1 -InputFile .\output\pedro_paths_red.json -ChainName "Red Close High"
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory, Position = 0)]
    [string]$InputFile,

    [Parameter()]
    [string]$OutputFile,

    [Parameter()]
    [string]$ChainName
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# Shared helpers (also sourced by the other scripts)
# ---------------------------------------------------------------------------

$script:PP_COLORS = @(
    '#ffc516', '#5B9C9A', '#857B75', '#C9DB67',
    '#60a5fa', '#ff6b6b', '#f97316'
)

function Normalize-Arrow {
    param([string]$Text)
    # Replace unicode right-arrow U+2192 with ASCII, then normalise spacing
    $Text = $Text -replace [char]0x2192, '->'
    $Text = $Text -replace '\s*->\s*', ' -> '
    return $Text.Trim()
}

function ConvertTo-ChainNameFromStem {
    param([string]$Stem)
    $base  = $Stem -replace '^pedro_paths_', ''
    $parts = $base -split '_' | Where-Object { $_ -ne '' }
    return ($parts | ForEach-Object {
        $_.Substring(0, 1).ToUpper() + $_.Substring(1)
    }) -join ' '
}

function Convert-JsonPathsToPedroProject {
    <#
    .SYNOPSIS
        Core conversion: array of visualizer path objects  ->  .pp project object.
    .PARAMETER Paths
        Array of objects with .name and .waypoints (each with .x .y .heading).
    .PARAMETER ChainName
        Human-readable name for the single PathChain in the project.
    #>
    param(
        [Parameter(Mandatory)][object[]]$Paths,
        [Parameter(Mandatory)][string]$ChainName
    )

    if ($Paths.Count -eq 0) { throw 'No paths provided to Convert-JsonPathsToPedroProject.' }

    $firstPath    = $Paths[0]
    $firstWp      = $firstPath.waypoints[0]
    $lastWpFirst  = $firstPath.waypoints[-1]

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
        $wps     = $path.waypoints
        $startWp = $wps[0]
        $endWp   = $wps[-1]

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

    $project = [ordered]@{
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

    return $project
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

$resolvedInput = Resolve-Path $InputFile

if (-not $ChainName) {
    $stem      = [System.IO.Path]::GetFileNameWithoutExtension($resolvedInput)
    $ChainName = ConvertTo-ChainNameFromStem $stem
}

if (-not $OutputFile) {
    $dir        = [System.IO.Path]::GetDirectoryName($resolvedInput)
    $stem       = [System.IO.Path]::GetFileNameWithoutExtension($resolvedInput)
    $OutputFile = Join-Path $dir "$stem.pp"
}

[System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName(
    [System.IO.Path]::GetFullPath($OutputFile)
)) | Out-Null

$jsonContent = Get-Content -Path $resolvedInput -Raw -Encoding UTF8
$paths       = $jsonContent | ConvertFrom-Json

$project = Convert-JsonPathsToPedroProject -Paths $paths -ChainName $ChainName
$output  = $project | ConvertTo-Json -Depth 20
$output  = $output -replace '\\u003e', '>'
$output  = $output -replace '\\u003c', '<'
$output  = $output -replace '\\u0026', '&'
$output  = $output -replace "\\u0027", "'"
[System.IO.File]::WriteAllText($OutputFile, ($output + "`n"), [System.Text.Encoding]::UTF8)

Write-Host "Wrote $OutputFile"



