# 从 build.ninja 生成 _relink.bat / _postrun.bat。
# 【为什么需要它】本环境下 cmake 的 re-run 与 ninja 都会挂死 (见 tools/_cc.py 的说明),
# 所以链接命令直接从 build.ninja 里抠出来手工执行。新增源文件后要重跑本脚本,
# 否则新目标文件不会进链接列表 (症状: undefined reference)。
$B = 'C:\Users\14420\Desktop\HPMicro\HPM6E8Y\build'
$gcc = 'D:\sdk_env\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win\bin\riscv32-unknown-elf-gcc.exe'

$line = (Select-String -Path "$B\build.ninja" -Pattern '^build output/demo\.elf: ' | Select-Object -First 1).Line
if (-not $line) { throw 'build.ninja 里找不到 demo.elf 的链接边' }
$objs = @(($line -split '\s+') | Where-Object { $_ -like '*.obj' })

# 【补漏】build.ninja 是上一次 cmake configure 生成的; 之后新增的源文件不在里面,
# 但 obj 已经被 _cc.py 编出来了 —— 这里按"src/ 下每个 .c 都应该有 obj"补进链接列表,
# 否则症状是 undefined reference (明明编过了却链不上)。
$extra = @(
    'CMakeFiles/app.dir/src/app/app_motion.c.obj',
    'CMakeFiles/app.dir/src/bsp/bsp_button.c.obj',
    'CMakeFiles/app.dir/src/dualcore/core1_boot.c.obj',
    # 双核: core1 的镜像数组(CMake 自定义命令生成的 C 文件, 在 build/generated/ 下,
    # 所以 build.ninja 里的 obj 路径是相对路径; 手工链接时必须补进来)
    'CMakeFiles/app.dir/generated/sec_core_img.c.obj'
)
foreach ($e in $extra) {
    $p = Join-Path $B $e
    if ((Test-Path $p) -and -not ($objs -contains $e)) {
        $objs += $e
        Write-Output "patch: + $e"
    }
}

$flagsLine = (Select-String -Path "$B\build.ninja" -Pattern '^\s+FLAGS = ' | Select-Object -First 1).Line
$flags = $flagsLine -replace '^\s+FLAGS = ', ''
$libsLine = (Select-String -Path "$B\build.ninja" -Pattern '^\s+LINK_LIBRARIES = ' | Select-Object -First 1).Line
$libs = $libsLine -replace '^\s+LINK_LIBRARIES = ', ''

Set-Content -Path "$B\_relink.bat" -Encoding ascii -Value (
    "@echo off`r`n" + $gcc + " " + $flags + " " + ($objs -join ' ') + " -o output\demo.elf " + $libs + "`r`n")

$postLine = (Select-String -Path "$B\build.ninja" -Pattern '^\s+POST_BUILD = ' | Select-Object -First 1).Line
$post = $postLine -replace '^\s+POST_BUILD = ', ''
$inner = $post -replace '^C:\\WINDOWS\\system32\\cmd\.exe /C "', '' -replace '"$', ''
$inner = $inner -replace '^cd /D [^&]+&& ', ''
Set-Content -Path "$B\_postrun.bat" -Encoding ascii -Value ("@echo off`r`n" + $inner + "`r`n")

"objects: $($objs.Count)"
"has app_motion: " + [bool]($objs -match 'app_motion')
