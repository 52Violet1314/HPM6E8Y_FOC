$ninja = "C:/ST/STM32CubeCLT_1.19.0/Ninja/bin/ninja.exe"
$cmds = & $ninja -C build -t commands 2>&1
$link = $cmds[-1]
# build.ninja 里还没有新源文件 (cmake 没重跑), 手工补进去
if (-not (($cmds -join "`n") -match 'app_enc_lut')) {
    $link = $link -replace 'CMakeFiles/app\.dir/src/app/app_encoder\.c\.obj',
        'CMakeFiles/app.dir/src/app/app_encoder.c.obj CMakeFiles/app.dir/src/app/app_enc_lut.c.obj'
}
$inner = $link -replace '^C:\\WINDOWS\\system32\\cmd\.exe /C "', '' -replace '"$', ''
$inner = $inner -replace '^cd \. && ', ''
Set-Content -Path build\_link.bat -Value ("@echo off`r`n" + $inner + "`r`n") -Encoding ascii

Push-Location build
$p = Start-Process -FilePath "cmd.exe" -ArgumentList "/c _link.bat" -Wait -PassThru -NoNewWindow
Pop-Location
"退出码: " + $p.ExitCode
Get-ChildItem build\output\demo.elf, build\output\demo.bin | Select-Object LastWriteTime, Length, Name
