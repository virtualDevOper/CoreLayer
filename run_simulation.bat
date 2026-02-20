@echo off
echo Building CoreLayer simulation...
cmake --build build --config Release --target CoreLayer
if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    pause
    exit /b 1
)

echo.
echo Running simulation...
.\build\Release\CoreLayer.exe
if %ERRORLEVEL% NEQ 0 (
    echo Simulation failed!
    pause
    exit /b 1
)

echo.
echo Simulation completed successfully!
echo Results saved to: data/output/results_data/
pause
