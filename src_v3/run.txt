@echo off
REM Skrypt uruchomieniowy dla systemu symulacji drona - Windows

echo ========================================
echo  System Symulacji Drona
echo  Adam Nowak
echo ========================================
echo.

REM Sprawdz czy Python jest zainstalowany
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python nie jest zainstalowany
    echo   Pobierz z https://www.python.org/
    pause
    exit /b 1
)

for /f "tokens=*" %%i in ('python --version') do set PYTHON_VER=%%i
echo [OK] Python: %PYTHON_VER%

REM Sprawdz czy istnieje venv
if not exist "venv\" (
    echo.
    echo [WARN] Nie znaleziono srodowiska wirtualnego.
    echo   Czy chcesz je utworzyc? T/N
    set /p response=
    if /i "%response%"=="T" (
        echo Tworze srodowisko wirtualne...
        python -m venv venv
        echo [OK] Utworzono venv
    )
)

REM Aktywuj venv jesli istnieje
if exist "venv\Scripts\activate.bat" (
    echo Aktywuje srodowisko wirtualne...
    call venv\Scripts\activate.bat
    echo [OK] Srodowisko aktywne
) else (
    echo [WARN] Kontynuuje bez srodowiska wirtualnego
)

REM Sprawdz czy zainstalowane sa pakiety
python -c "import osmnx" >nul 2>&1
if errorlevel 1 (
    echo.
    echo [WARN] Brak wymaganych pakietow.
    echo   Instaluje zaleznosci z requirements.txt...
    python -m pip install --upgrade pip
    pip install -r requirements.txt
    if errorlevel 1 (
        echo [ERROR] Blad instalacji pakietow!
        pause
        exit /b 1
    )
    echo [OK] Pakiety zainstalowane
)

echo.
echo ========================================
echo  Uruchamiam symulacje...
echo ========================================
echo.

cd src
python main.py
cd ..

REM Dezaktywuj venv
if exist "venv\Scripts\deactivate.bat" (
    call venv\Scripts\deactivate.bat
)

pause
