# collect_project.ps1
# Сборка всех файлов проекта с поддержкой кириллицы (Windows-1251)

param(
    [string]$RootDir = (Get-Location).Path,
    [string]$OutputFile = (Join-Path $RootDir "project_all_files.txt")
)

$extensions = @('.cpp', '.h', '.tpp', '.txt', '.md')
$excludeDirs = @('build', 'bin', 'obj', 'Debug', 'Release', '.git', '.vs', 'CMakeFiles', 'cmake-build-debug', 'libs')
$excludeFiles = @('CMakeCache.txt')

$timestamp = Get-Date -Format "dd.MM.yyyy HH:mm:ss,fff"

# Создаём кодировку UTF-8 с BOM для корректного сохранения кириллицы
$utf8WithBom = New-Object System.Text.UTF8Encoding($true)

$header = @"
=== Сборка всех файлов проекта CoreLayer === 
Дата: $timestamp 
Корневая директория проекта: $RootDir\ 


"@

# Записываем заголовок с BOM
[System.IO.File]::WriteAllText($OutputFile, $header, $utf8WithBom)

$foundCount = 0
$notFoundCount = 0

# Функция проверки исключений
function Test-Excluded {
    param([string]$Path, [string]$Root)
    
    $relative = $Path.Substring($Root.Length + 1).Replace('\', '/').ToLower()
    
    foreach ($dir in $excludeDirs) {
        if ($relative -like "$dir/*" -or $relative -eq $dir -or $relative -like "*/$dir/*") {
            return $true
        }
    }
    
    foreach ($file in $excludeFiles) {
        if ($relative -like $file.ToLower()) {
            return $true
        }
    }
    
    return $false
}

# Получаем все файлы
Get-ChildItem -Path $RootDir -Recurse -File -ErrorAction SilentlyContinue | Where-Object {
    $ext = $_.Extension.ToLower()
    $matchExt = $extensions -contains $ext
    
    if (-not $matchExt) { return $false }
    
    if (Test-Excluded -Path $_.FullName -Root $RootDir) {
        return $false
    }
    
    return $true
} | Sort-Object FullName | ForEach-Object {
    $file = $_
    $relativePath = $file.FullName.Substring($RootDir.Length + 1)
    
    try {
        # Читаем файл в кодировке Windows-1251 (стандарт для русских проектов в VS)
        $win1251 = [System.Text.Encoding]::GetEncoding(1251)
        $content = [System.IO.File]::ReadAllText($file.FullName, $win1251)
        
        $separator = @"
============================================= 
ФАЙЛ: $relativePath 
============================================= 

"@
        [System.IO.File]::AppendAllText($OutputFile, $separator, $utf8WithBom)
        [System.IO.File]::AppendAllText($OutputFile, $content, $utf8WithBom)
        
        $footer = @"

=== КОНЕЦ ФАЙЛА: $relativePath === 


"@
        [System.IO.File]::AppendAllText($OutputFile, $footer, $utf8WithBom)
        
        $foundCount++
    }
    catch {
        $notFoundCount++
        Write-Warning "Не удалось прочитать: $relativePath"
    }
}

$summary = @"
============================================= 
=== СБОРКА ЗАВЕРШЕНА === 
Файлов найдено: $foundCount 
Файлов не найдено: $notFoundCount 
Корневая директория: $RootDir\ 
Итоговый файл: $OutputFile 
============================================= 
"@

[System.IO.File]::AppendAllText($OutputFile, $summary, $utf8WithBom)

Write-Host "`n✅ Сборка завершена!" -ForegroundColor Green
Write-Host "📁 Найдено файлов: $foundCount" -ForegroundColor Cyan
Write-Host "📄 Результат: $OutputFile`n" -ForegroundColor Yellow