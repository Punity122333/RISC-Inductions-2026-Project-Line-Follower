$ErrorActionPreference = "SilentlyContinue"
Set-Location $PSScriptRoot

$scad = if ($env:OPENSCAD) { $env:OPENSCAD } else { "C:\Program Files\OpenSCAD\openscad.exe" }
$src = "robot.scad"
$out = "out"

if (-not (Test-Path $out)) { New-Item -ItemType Directory -Path $out | Out-Null }

function Invoke-Scad {
    param([string[]]$ScadArgs)
    $proc = Start-Process -FilePath $scad -ArgumentList $ScadArgs -NoNewWindow -Wait -PassThru
    if ($proc.ExitCode -ne 0) { throw "openscad failed ($($proc.ExitCode)): $($ScadArgs -join ' ')" }
}

function Export-Stl {
    param([string]$Id, [string]$Name)
    Write-Host "  part_id=$Id -> $out\$Name"
    Invoke-Scad @("-o", "$out\$Name", "-D", "part_id=$Id", "-D", "show_pose=0", $src)
}

Export-Stl "1" "chassis.stl"
Export-Stl "2" "wheel.stl"
Export-Stl "3" "chassis_collision.stl"
Export-Stl "4" "wheel_collision.stl"

Write-Host "  assembly -> $out\preview.png"
Invoke-Scad @(
    "-o", "$out\preview.png",
    "--imgsize=1400,1050",
    "--projection=p",
    "--colorscheme=Tomorrow",
    "--autocenter", "--viewall",
    "--camera=0,0,0,58,0,26,560",
    "-D", "part_id=0", "-D", "show_pose=0", $src
)

Write-Host "  poses -> $out\pose.echo"
Invoke-Scad @("-o", "$out\pose.echo", "-D", "part_id=5", "-D", "show_pose=1", $src)

Write-Host "  checking"
python check_stl.py "$out\chassis.stl" "$out\wheel.stl" "$out\chassis_collision.stl" "$out\wheel_collision.stl"
if ($LASTEXITCODE -ne 0) { throw "stl check failed" }

Write-Host "done"
