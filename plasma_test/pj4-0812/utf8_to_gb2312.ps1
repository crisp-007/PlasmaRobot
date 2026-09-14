# Encoding conversion script: UTF-8 to GB2312
# Convert UTF-8 files back to GB2312 encoding in STM32 project

Write-Host "=== STM32 Project UTF-8 to GB2312 Converter ===" -ForegroundColor Green
Write-Host "Converting C/H files from UTF-8 to GB2312..." -ForegroundColor Yellow

$userPath = "./User"
$fileCount = 0
$successCount = 0

# Get all C and H files
$files = Get-ChildItem -Path $userPath -Include "*.c", "*.h" -Recurse

foreach ($file in $files) {
    $fileCount++
    Write-Host "[$fileCount] Processing: $($file.Name)" -NoNewline
    
    try {
        # Read file with UTF-8 encoding
        $content = Get-Content -Path $file.FullName -Encoding UTF8
        
        # Save as GB2312 encoding (Default)
        $content | Out-File -FilePath $file.FullName -Encoding Default
        
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
    Write-Host "`nAll files converted successfully! Files are now in GB2312 encoding." -ForegroundColor Green
    Write-Host "Please reopen VS Code to refresh file contents." -ForegroundColor Yellow
} else {
    Write-Host "`nSome files failed to convert. Please check file permissions." -ForegroundColor Yellow
}

Read-Host "`nPress Enter to exit"