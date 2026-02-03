# PowerShell скрипт для сборки всех файлов проекта
# Улучшенная версия с поддержкой UTF-8

param(
    [string]$OutputFile = "project_all_files.txt",
    [switch]$IncludeLibs = $false
)

Write-Host "Сборка всех файлов проекта CoreLayer..." -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

# Удаляем старый файл
if (Test-Path $OutputFile) {
    Remove-Item $OutputFile -Force
}

# Создаем заголовок
$header = @"
# CoreLayer - Полный код проекта
# Сгенерировано: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
# ==========================================

"@

$header | Out-File -FilePath $OutputFile -Encoding UTF8

# Функция для добавления файла
function Add-FileToOutput {
    param(
        [string]$FilePath,
        [string]$OutputFile
    )
    
    if (-not (Test-Path $FilePath)) {
        return
    }
    
    Write-Host "Добавляю: $FilePath" -ForegroundColor Yellow
    
    $relativePath = $FilePath -replace [regex]::Escape((Get-Location).Path + "\"), ""
    
    $content = @"

### Файл: $relativePath
``````cpp
$(Get-Content $FilePath -Raw -Encoding UTF8)
``````

"@
    
    $content | Out-File -FilePath $OutputFile -Append -Encoding UTF8
}

# Основные файлы
Write-Host "`nДобавляю основные файлы..." -ForegroundColor Cyan
Add-FileToOutput "main.cpp" $OutputFile
Add-FileToOutput "CMakeLists.txt" $OutputFile
Add-FileToOutput "README.md" $OutputFile

# Заголовочные файлы ядра
Write-Host "`nДобавляю заголовочные файлы ядра..." -ForegroundColor Cyan
"`n## Основные заголовочные файлы`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8

Get-ChildItem "include/core" -Filter "*.h" -ErrorAction SilentlyContinue | ForEach-Object {
    Add-FileToOutput $_.FullName $OutputFile
}
Get-ChildItem "include/core" -Filter "*.hpp" -ErrorAction SilentlyContinue | ForEach-Object {
    Add-FileToOutput $_.FullName $OutputFile
}
Get-ChildItem "include/core" -Filter "*.tpp" -ErrorAction SilentlyContinue | ForEach-Object {
    Add-FileToOutput $_.FullName $OutputFile
}

# Исходный код
Write-Host "`nДобавляю исходный код..." -ForegroundColor Cyan
"`n## Исходный код`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8

$sourceExtensions = @("*.h", "*.hpp", "*.cpp", "*.tpp")
foreach ($ext in $sourceExtensions) {
    Get-ChildItem "src" -Filter $ext -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
        Add-FileToOutput $_.FullName $OutputFile
    }
}

# Тесты
Write-Host "`nДобавляю тесты..." -ForegroundColor Cyan
"`n## Тесты`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8

foreach ($ext in $sourceExtensions) {
    Get-ChildItem "tests" -Filter $ext -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
        Add-FileToOutput $_.FullName $OutputFile
    }
}

# Конфигурационные файлы
Write-Host "`nДобавляю конфигурацию..." -ForegroundColor Cyan
"`n## Конфигурация`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8

$configExtensions = @("*.json", "*.xml", "*.cfg", "*.ini")
foreach ($ext in $configExtensions) {
    Get-ChildItem "config" -Filter $ext -ErrorAction SilentlyContinue | ForEach-Object {
        Add-FileToOutput $_.FullName $OutputFile
    }
}

# Скрипты визуализации
Write-Host "`nДобавляю скрипты визуализации..." -ForegroundColor Cyan
"`n## Скрипты визуализации`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8

$scriptExtensions = @("*.py", "*.bat", "*.sh")
foreach ($ext in $scriptExtensions) {
    Get-ChildItem "visualization" -Filter $ext -ErrorAction SilentlyContinue | ForEach-Object {
        Add-FileToOutput $_.FullName $OutputFile
    }
}

# Документация
Write-Host "`nДобавляю документацию..." -ForegroundColor Cyan
"`n## Документация`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8

Get-ChildItem "." -Filter "*.md" -ErrorAction SilentlyContinue | ForEach-Object {
    if ($_.Name -ne "README.md") {  # README уже добавлен
        Add-FileToOutput $_.FullName $OutputFile
    }
}

Get-ChildItem "docs" -Filter "*.md" -Recurse -ErrorAction SilentlyContinue | ForEach-Object {
    Add-FileToOutput $_.FullName $OutputFile
}

# Библиотеки (опционально)
if ($IncludeLibs) {
    Write-Host "`nДобавляю библиотеки..." -ForegroundColor Cyan
    "`n## Библиотеки`n" | Out-File -FilePath $OutputFile -Append -Encoding UTF8
    
    foreach ($ext in $sourceExtensions) {
        Get-ChildItem "libs" -Filter $ext -Recurse -ErrorAction SilentlyContinue | Select-Object -First 20 | ForEach-Object {
            Add-FileToOutput $_.FullName $OutputFile
        }
    }
}

# Статистика
$fileSize = (Get-Item $OutputFile).Length
$fileSizeKB = [math]::Round($fileSize / 1024, 2)
$fileSizeMB = [math]::Round($fileSize / 1024 / 1024, 2)

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "Готово! Все файлы собраны в: $OutputFile" -ForegroundColor Green
Write-Host "Размер файла: $fileSize байт ($fileSizeKB KB / $fileSizeMB MB)" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

# Показать содержимое папок
Write-Host "`nСтатистика по папкам:" -ForegroundColor Cyan
@("src", "include", "tests", "config", "visualization") | ForEach-Object {
    if (Test-Path $_) {
        $count = (Get-ChildItem $_ -Recurse -File | Measure-Object).Count
        Write-Host "  $_/: $count файлов" -ForegroundColor White
    }
}