param(
    [Parameter(Mandatory = $true, ValueFromRemainingArguments = $true)]
    [string[]]$HtmlPath
)

$DefaultLangs = @("rust", "c")
$Labels = @{
    rust = "Rust"
    c = "C"
}

function Normalize-Lang([string]$Value) {
    return ($Value.Trim().ToLowerInvariant())
}

function Get-Label([string]$Lang) {
    if ($Labels.ContainsKey($Lang)) {
        return $Labels[$Lang]
    }
    return $Lang
}

function Parse-Languages([string]$Value) {
    if ($Value.Contains(":")) {
        $parts = $Value.Split(":", 2)[1]
        $langs = @($parts.Split(",") | ForEach-Object { Normalize-Lang $_ } | Where-Object { $_ })
        if ($langs.Count -gt 0) {
            return $langs
        }
    }
    return $DefaultLangs
}

function Strip-Tags([string]$Html) {
    $text = [regex]::Replace($Html, "<[^>]*>", "")
    $text = [System.Net.WebUtility]::HtmlDecode($text)
    return ([regex]::Replace($text, "\s+", " ")).Trim()
}

function Escape-Attr([string]$Value) {
    return [System.Net.WebUtility]::HtmlEncode($Value)
}

function Get-HeadingLanguage([string]$HeadingHtml, [string[]]$Langs) {
    $text = (Strip-Tags $HeadingHtml).ToLowerInvariant()
    foreach ($lang in $Langs) {
        if ($text -eq $lang -or $text -eq (Get-Label $lang).ToLowerInvariant()) {
            return $lang
        }
    }
    return ""
}

function Build-Tabs([string]$Content, [string[]]$Langs, [int]$Index) {
    $headingPattern = [regex]::new("<h([1-6])\b[^>]*>[\s\S]*?</h\1>", [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    $matches = $headingPattern.Matches($Content)
    $panels = @{}
    foreach ($lang in $Langs) {
        $panels[$lang] = ""
    }

    $order = New-Object "System.Collections.Generic.List[string]"
    $current = ""
    $last = 0
    $prefix = ""

    foreach ($match in $matches) {
        $before = $Content.Substring($last, $match.Index - $last)
        if ($current) {
            $panels[$current] += $before
        } else {
            $prefix += $before
        }

        $lang = Get-HeadingLanguage $match.Value $Langs
        if ($lang) {
            $current = $lang
            if (-not $order.Contains($lang)) {
                $order.Add($lang)
            }
        } elseif ($current) {
            $panels[$current] += $match.Value
        } else {
            $prefix += $match.Value
        }

        $last = $match.Index + $match.Length
    }

    if ($current) {
        $panels[$current] += $Content.Substring($last)
    } else {
        $prefix += $Content.Substring($last)
    }

    $active = @()
    foreach ($lang in $order) {
        $html = $panels[$lang]
        if ((Strip-Tags $html) -or $html.Trim()) {
            $active += $lang
        }
    }

    if ($active.Count -lt 2) {
        return $Content
    }

    $groupId = "copp-tabs-static-$Index"
    $buttons = New-Object System.Text.StringBuilder
    for ($i = 0; $i -lt $active.Count; $i++) {
        $lang = $active[$i]
        $activeClass = if ($i -eq 0) { " is-active" } else { "" }
        $selected = if ($i -eq 0) { "true" } else { "false" }
        $langEsc = Escape-Attr $lang
        $labelEsc = Escape-Attr (Get-Label $lang)
        [void]$buttons.Append("<button type=""button"" class=""copp-tabs__button$activeClass"" role=""tab"" id=""$groupId-tab-$langEsc"" aria-controls=""$groupId-panel-$langEsc"" aria-selected=""$selected"" data-copp-tab-button=""$langEsc"">$labelEsc</button>")
    }

    $panelHtml = New-Object System.Text.StringBuilder
    for ($i = 0; $i -lt $active.Count; $i++) {
        $lang = $active[$i]
        $hidden = if ($i -eq 0) { "" } else { " hidden" }
        $langEsc = Escape-Attr $lang
        [void]$panelHtml.Append("<div class=""copp-tabs__panel"" role=""tabpanel"" id=""$groupId-panel-$langEsc"" aria-labelledby=""$groupId-tab-$langEsc"" data-copp-panel=""$langEsc""$hidden>$($panels[$lang])</div>")
    }

    return "$prefix<section class=""copp-tabs"" data-copp-tabs-widget><div class=""copp-tabs__bar"" role=""tablist"">$buttons</div>$panelHtml</section>"
}

function Transform-Markers([string]$Html) {
    $markerPattern = "(?:<p\b[^>]*>\s*)?<span\b[^>]*\bdata-copp-tabs=([""'])([^""']+)\1[^>]*>\s*</span>(?:\s*</p>)?"
    $markerRegex = [regex]::new($markerPattern, [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    $builder = New-Object System.Text.StringBuilder
    $cursor = 0
    $count = 0

    while ($true) {
        $start = $markerRegex.Match($Html, $cursor)
        if (-not $start.Success) {
            [void]$builder.Append($Html.Substring($cursor))
            break
        }

        $value = $start.Groups[2].Value
        if (-not $value.StartsWith("start")) {
            [void]$builder.Append($Html.Substring($cursor, $start.Index + $start.Length - $cursor))
            $cursor = $start.Index + $start.Length
            continue
        }

        $contentStart = $start.Index + $start.Length
        $end = $markerRegex.Match($Html, $contentStart)
        while ($end.Success -and -not $end.Groups[2].Value.StartsWith("end")) {
            $end = $markerRegex.Match($Html, $end.Index + $end.Length)
        }

        if (-not $end.Success) {
            [void]$builder.Append($Html.Substring($cursor, $start.Index + $start.Length - $cursor))
            $cursor = $start.Index + $start.Length
            continue
        }

        $langs = Parse-Languages $value
        $content = $Html.Substring($contentStart, $end.Index - $contentStart)
        [void]$builder.Append($Html.Substring($cursor, $start.Index - $cursor))
        [void]$builder.Append((Build-Tabs $content $langs $count))
        $count += 1
        $cursor = $end.Index + $end.Length
    }

    return [PSCustomObject]@{
        Html = $builder.ToString()
        Count = $count
    }
}

function Inject-Assets([string]$Html) {
    $result = $Html
    if (-not $result.Contains("copp-tabs.css")) {
        $css = '<link rel="stylesheet" href="assets/copp-tabs.css">'
        $result = [regex]::Replace($result, "</head>", "$css`n</head>", [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    }
    if (-not $result.Contains("copp-tabs.js")) {
        $js = '<script defer src="assets/copp-tabs.js"></script>'
        $result = [regex]::Replace($result, "</body>", "$js`n</body>", [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    }
    return $result
}

foreach ($file in $HtmlPath) {
    $fullPath = (Resolve-Path -LiteralPath $file).Path
    $html = [System.IO.File]::ReadAllText($fullPath, [System.Text.Encoding]::UTF8)
    $transformed = Transform-Markers $html
    $enhanced = Inject-Assets $transformed.Html
    $utf8NoBom = [System.Text.UTF8Encoding]::new($false)
    [System.IO.File]::WriteAllText($fullPath, $enhanced, $utf8NoBom)
    Write-Host "${file}: transformed $($transformed.Count) tab group(s), injected assets"
}
