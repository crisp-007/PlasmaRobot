# Encoding conversion script: GB2312 to UTF-8
# Fix Chinese comment encoding issues in STM32 project

Write-Host "=== STM32 Project Encoding Fix Tool ===" -ForegroundColor Green
Write-Host "Converting C/H files in User directory..." -ForegroundColor Yellow

$userPath = "./User"
$fileCount = 0
$successCount = 0

# Get all C and H files
$files = Get-ChildItem -Path $userPath -Include "*.c", "*.h" -Recurse

foreach ($file in $files) {
    $fileCount++
    Write-Host "[$fileCount] Processing: $($file.Name)" -NoNewline
    
    try {
        # Read file with GB2312 encoding (Default)
        $content = Get-Content -Path $file.FullName -Encoding Default
        
        # Save as UTF-8 encoding
        $content | Out-File -FilePath $file.FullName -Encoding UTF8
        
        Write-Host " Success" -ForegroundColor Green
        $successCount++
    }
    catch {
        Write-Host " Failed" -ForegroundColor Red
    }
}

Write-Host "`nConversion completed!" -ForegroundColor Green
Write-Host "Total files: $fileCount" -ForegroundColor White
Write-Host "Successfully converted: $successCount" -ForegroundColor Green

if ($successCount -eq $fileCount) {
    Write-Host "`nAll files converted successfully! Chinese comments should display correctly now." -ForegroundColor Green
    Write-Host "Please reopen VS Code to refresh file contents." -ForegroundColor Yellow
} else {
    Write-Host "`nSome files failed to convert. Please check file permissions." -ForegroundColor Yellow
}

Read-Host "`nPress Enter to exit"