param(
    [bool]$OpenBrowser = $true
)

$repositoryRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..\..")
$openBrowserValue = $OpenBrowser.ToString().ToLowerInvariant()

Push-Location $repositoryRoot
try {
    lerobot-ik-sim --open_browser=$openBrowserValue
}
finally {
    Pop-Location
}
