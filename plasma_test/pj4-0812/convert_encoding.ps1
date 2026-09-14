# 编码转换脚本：从GB2312转换为UTF-8
# 用于解决项目编码格式更改后的乱码问题

param(
    [string]$SourcePath = "./User",
    [string[]]$FileExtensions = @("*.c", "*.h", "*.cpp", "*.hpp")
)

# 设置控制台输出编码为UTF-8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Write-Host "开始转换编码格式..." -ForegroundColor Green
Write-Host "源路径: $SourcePath" -ForegroundColor Yellow
Write-Host "文件类型: $($FileExtensions -join ', ')" -ForegroundColor Yellow

# 统计变量
$totalFiles = 0
$convertedFiles = 0
$errorFiles = 0

# 遍历所有指定类型的文件
foreach ($extension in $FileExtensions) {
    $files = Get-ChildItem -Path $SourcePath -Filter $extension -Recurse -File
    
    foreach ($file in $files) {
        $totalFiles++
        Write-Host "处理文件: $($file.FullName)" -ForegroundColor Cyan
        
        try {
            # 使用GB2312编码读取文件内容
            $gb2312Encoding = [System.Text.Encoding]::GetEncoding("GB2312")
            $content = [System.IO.File]::ReadAllText($file.FullName, $gb2312Encoding)
            
            # 使用UTF-8编码保存文件内容（带BOM）
            $utf8Encoding = New-Object System.Text.UTF8Encoding($true)
            [System.IO.File]::WriteAllText($file.FullName, $content, $utf8Encoding)
            
            Write-Host "  ✓ 转换成功" -ForegroundColor Green
            $convertedFiles++
        }
        catch {
            Write-Host "  ✗ 转换失败: $($_.Exception.Message)" -ForegroundColor Red
            $errorFiles++
        }
    }
}

# 输出统计结果
Write-Host "`n转换完成！" -ForegroundColor Green
Write-Host "总文件数: $totalFiles" -ForegroundColor White
Write-Host "成功转换: $convertedFiles" -ForegroundColor Green
Write-Host "转换失败: $errorFiles" -ForegroundColor Red

if ($errorFiles -eq 0) {
    Write-Host "`n所有文件转换成功！现在可以正常查看中文注释了。" -ForegroundColor Green
} else {
    Write-Host "`n部分文件转换失败，请检查文件权限或编码格式。" -ForegroundColor Yellow
}

# 暂停以查看结果
Write-Host "`n按任意键继续..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")