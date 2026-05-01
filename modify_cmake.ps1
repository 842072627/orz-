$file = "C:\Users\Administrator\Desktop\test-studio-mapf\Start-Kit-main\build_startkit.bat"
$content = Get-Content $file -Raw
$content = $content -replace 'Start-Kit-main', 'Start-Kit-main' -replace 'set "PROJ=.*"', 'set "PROJ=%~dp0"' -replace '-DPYTHON_LIBRARY', '-DSKIP_PYTHON=ON -DPYTHON_LIBRARY'
Set-Content -Path $file -Value $content -NoNewline
