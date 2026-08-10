$ErrorActionPreference = "Stop"

$installDirectory = Join-Path $env:LOCALAPPDATA "AmbatuDrone"
$executable = Join-Path $installDirectory "AmbatuDrone.exe"

if (-not (Test-Path -LiteralPath $executable)) {
    throw "AmbatuDrone.exe was not found at $executable"
}

$shell = New-Object -ComObject WScript.Shell
$shortcutLocations = @(
    (Join-Path ([Environment]::GetFolderPath("Desktop")) "AmbatuDrone.lnk"),
    (Join-Path ([Environment]::GetFolderPath("Programs")) "AmbatuDrone.lnk")
)

foreach ($shortcutPath in $shortcutLocations) {
    $shortcut = $shell.CreateShortcut($shortcutPath)
    $shortcut.TargetPath = $executable
    $shortcut.WorkingDirectory = $installDirectory
    $shortcut.Description = "AmbatuDrone controller bridge and flight dashboard"
    $shortcut.Save()
}
