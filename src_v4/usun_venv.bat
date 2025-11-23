@echo off
chcp 65001 >nul
echo ========================================
echo  Manualne usuniecie venv
echo ========================================
echo.

echo Biezacy katalog: %CD%
echo.

if not exist "venv\" (
    echo [INFO] venv nie istnieje w tym katalogu
    echo.
    pause
    exit /b 0
)

echo [INFO] venv istnieje
echo.
echo UWAGA: To usunie cale srodowisko wirtualne!
echo Bedzie trzeba uruchomic install.bat ponownie.
echo.

set /p confirm=Czy na pewno chcesz usunac venv? (t/n) [n]: 
if /i not "%confirm%"=="t" (
    echo.
    echo Anulowano
    pause
    exit /b 0
)

echo.
echo Usuwam venv...
rd /s /q venv 2>nul

timeout /t 2 /nobreak >nul

if exist "venv\" (
    echo [WARN] Nie mozna usunac normalnie
    echo.
    echo Proba 2: Usuwanie plikow...
    del /f /s /q venv\*.* 2>nul
    
    timeout /t 2 /nobreak >nul
    
    if exist "venv\" (
        echo [WARN] Nadal nie mozna usunac
        echo.
        echo Mozliwe przyczyny:
        echo 1. Jakis program uzywa plikow z venv
        echo 2. Python jest uruchomiony z venv
        echo 3. VS Code lub inne IDE ma otwarty venv
        echo.
        echo Zamknij wszystkie programy i sprobuj ponownie
    ) else (
        echo [OK] Usunieto (proba 2)
    )
) else (
    echo [OK] venv usunieto
)

echo.
echo Aby zainstalowac ponownie:
echo   install.bat
echo.
pause