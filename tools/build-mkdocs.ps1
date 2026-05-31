param(
    [switch]$Strict
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$SiteDir = Join-Path $RepoRoot "site"

function Copy-DirectoryContents {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Source,
        [Parameter(Mandatory = $true)]
        [string]$Destination
    )

    if (-not (Test-Path -LiteralPath $Source)) {
        return
    }

    if (-not (Test-Path -LiteralPath $Destination)) {
        New-Item -ItemType Directory -Path $Destination | Out-Null
    }

    Copy-Item -Path (Join-Path $Source "*") -Destination $Destination -Recurse -Force
}

function Remove-GeneratedPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }

    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if (-not $resolved.StartsWith($RepoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove path outside repository root: $resolved"
    }

    Remove-Item -LiteralPath $resolved -Recurse -Force
}

Push-Location $RepoRoot
try {
    $mkdocsArgs = @("build")
    if ($Strict) {
        $mkdocsArgs += "--strict"
    }

    & python -m mkdocs @mkdocsArgs

    foreach ($name in @("rust", "c")) {
        $src = Join-Path $RepoRoot $name
        $dst = Join-Path $SiteDir $name
        if (-not (Test-Path -LiteralPath $src)) {
            continue
        }
        if (Test-Path -LiteralPath $dst) {
            Remove-Item -LiteralPath $dst -Recurse -Force
        }
        Copy-Item -LiteralPath $src -Destination $dst -Recurse -Force
    }

    $cname = Join-Path $RepoRoot "CNAME"
    if (Test-Path -LiteralPath $cname) {
        Copy-Item -LiteralPath $cname -Destination (Join-Path $SiteDir "CNAME") -Force
    }

    foreach ($file in @("index.html", "404.html", "sitemap.xml", "sitemap.xml.gz")) {
        $src = Join-Path $SiteDir $file
        if (Test-Path -LiteralPath $src) {
            Copy-Item -LiteralPath $src -Destination (Join-Path $RepoRoot $file) -Force
        }
    }

    foreach ($dir in @("index-zh-rust")) {
        Remove-GeneratedPath -Path (Join-Path $RepoRoot $dir)
    }

    foreach ($dir in @("javascripts", "stylesheets", "search")) {
        $src = Join-Path $SiteDir $dir
        $dst = Join-Path $RepoRoot $dir
        if (-not (Test-Path -LiteralPath $src)) {
            continue
        }
        Remove-GeneratedPath -Path $dst
        Copy-Item -LiteralPath $src -Destination $dst -Recurse -Force
    }

    Copy-DirectoryContents `
        -Source (Join-Path $SiteDir "assets") `
        -Destination (Join-Path $RepoRoot "assets")
}
finally {
    Pop-Location
}
