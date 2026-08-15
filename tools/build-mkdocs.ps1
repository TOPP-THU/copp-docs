param(
    [switch]$Strict
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$SiteDir = Join-Path $RepoRoot "site"
$PagefindSiteDir = Join-Path $RepoRoot "site-pagefind"
$PagefindZhSiteDir = Join-Path $RepoRoot "site-pagefind-zh"

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
    if ($LASTEXITCODE -ne 0) {
        throw "Building the MkDocs site failed with exit code $LASTEXITCODE"
    }

    foreach ($name in @("rust", "c", "cpp", "python", "matlab")) {
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

    $nojekyll = Join-Path $SiteDir ".nojekyll"
    New-Item -ItemType File -Path $nojekyll -Force | Out-Null

    Remove-GeneratedPath -Path $PagefindSiteDir
    & python (Join-Path $PSScriptRoot "prepare_pagefind_site.py") `
        --site $SiteDir `
        --output $PagefindSiteDir `
        --exclude-path index-zh.html
    if ($LASTEXITCODE -ne 0) {
        throw "Preparing the Pagefind staging site failed with exit code $LASTEXITCODE"
    }

    & python -m pagefind `
        --site $PagefindSiteDir `
        --output-path (Join-Path $SiteDir "pagefind") `
        --root-selector html `
        --force-language en `
        --include-characters "_:." `
        --exclude-selectors "nav, header, footer, .md-sidebar, .md-header, .md-footer, .sidebar"
    if ($LASTEXITCODE -ne 0) {
        throw "Building the Pagefind index failed with exit code $LASTEXITCODE"
    }

    Remove-GeneratedPath -Path $PagefindZhSiteDir
    & python (Join-Path $PSScriptRoot "prepare_pagefind_site.py") `
        --site $SiteDir `
        --output $PagefindZhSiteDir `
        --include-path index-zh.html
    if ($LASTEXITCODE -ne 0) {
        throw "Preparing the Chinese Pagefind staging site failed with exit code $LASTEXITCODE"
    }

    & python -m pagefind `
        --site $PagefindZhSiteDir `
        --output-path (Join-Path $SiteDir "pagefind-zh") `
        --root-selector html `
        --force-language zh `
        --include-characters "_:." `
        --exclude-selectors "nav, header, footer, .md-sidebar, .md-header, .md-footer, .sidebar"
    if ($LASTEXITCODE -ne 0) {
        throw "Building the Chinese Pagefind index failed with exit code $LASTEXITCODE"
    }

    $searchIndexJson = Join-Path $SiteDir "search\search_index.json"
    if (Test-Path -LiteralPath $searchIndexJson) {
        $searchIndexJs = Join-Path $SiteDir "search\search_index.js"
        $searchIndex = Get-Content -Raw -Encoding UTF8 $searchIndexJson
        Set-Content -LiteralPath $searchIndexJs -Encoding UTF8 -Value "var __index = $searchIndex;"
    }

    foreach ($file in @("index.html", "index-zh.html", "404.html", "sitemap.xml", "sitemap.xml.gz", ".nojekyll")) {
        $src = Join-Path $SiteDir $file
        if (Test-Path -LiteralPath $src) {
            Copy-Item -LiteralPath $src -Destination (Join-Path $RepoRoot $file) -Force
        }
    }

    foreach ($dir in @("index-zh-rust", "index-zh")) {
        Remove-GeneratedPath -Path (Join-Path $RepoRoot $dir)
    }

    foreach ($dir in @("javascripts", "stylesheets", "search", "pagefind", "pagefind-zh")) {
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
    Remove-GeneratedPath -Path $PagefindSiteDir
    Remove-GeneratedPath -Path $PagefindZhSiteDir
    Pop-Location
}
