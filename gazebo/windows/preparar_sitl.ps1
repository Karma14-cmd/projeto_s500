param([string]$Runtime = (Join-Path $env:USERPROFILE 'gz-s500'))
$ErrorActionPreference = 'Stop'
if ($Runtime.Contains(' ')) { throw 'Use uma pasta de runtime sem espaços.' }
$taskPixi = Get-Command pixi -ErrorAction SilentlyContinue
if (-not $taskPixi) {
    $taskPixiPath = Join-Path $env:LOCALAPPDATA 'pixi\bin\pixi.exe'
    if (-not (Test-Path -LiteralPath $taskPixiPath)) {
        throw 'Instale Pixi no perfil do usuário: winget install --id prefix-dev.pixi --exact --scope user'
    }
} else { $taskPixiPath = $taskPixi.Source }
New-Item -ItemType Directory -Path $Runtime -Force | Out-Null
foreach ($taskFile in @('pixi.toml','pixi.lock')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $taskFile) -Destination (Join-Path $Runtime $taskFile)
}
& $taskPixiPath install --manifest-path (Join-Path $Runtime 'pixi.toml')
if ($LASTEXITCODE -ne 0) { throw 'Falha ao preparar ambiente Pixi.' }
$taskSitl = Join-Path $Runtime 'ardupilot'
New-Item -ItemType Directory -Path $taskSitl -Force | Out-Null
$taskManifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'binarios_sitl.json') -Raw | ConvertFrom-Json
foreach ($taskFile in $taskManifest.arquivos) {
    $taskDest = Join-Path $taskSitl $taskFile.nome
    if (-not (Test-Path -LiteralPath $taskDest) -or
        (Get-FileHash -LiteralPath $taskDest -Algorithm SHA256).Hash -ne $taskFile.sha256) {
        Invoke-WebRequest -Uri $taskFile.url -OutFile $taskDest
    }
    if ((Get-FileHash -LiteralPath $taskDest -Algorithm SHA256).Hash -ne $taskFile.sha256) {
        throw "Checksum divergente: $($taskFile.nome). O servidor oficial pode ter atualizado o binário."
    }
}
Copy-Item -LiteralPath (Join-Path $taskSitl 'ArduCopter.elf') -Destination (Join-Path $taskSitl 'arducopter.exe')
$taskPrefix = Join-Path $Runtime '.pixi\envs\default'
# Mesa restrito a este ambiente; evita a falha OpenGL observada no driver AMD.
foreach ($taskDll in @('opengl32.dll','libgallium_wgl.dll')) {
    Copy-Item -LiteralPath (Join-Path $taskPrefix "Library\bin\$taskDll") -Destination (Join-Path $taskPrefix "bin\$taskDll")
}
Write-Output 'SITL e Gazebo preparados. Consulte README_voo.md para iniciar.'
