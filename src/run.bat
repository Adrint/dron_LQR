@echo off
REM Skrypt uruchomieniowy dla systemu symulacji drona - Windows

echo ========================================
echo  System Symulacji Drona
echo  Politechnika Warszawska
echo ========================================
echo.

REM Sprawdź czy Python jest zainstalowany
python --version >nul 2>&1
if errorlevel 1 (
    echo [91mX Python nie jest zainstalowany![0m
    echo   Pobierz z https://www.python.org/
    pause
    exit /b 1
)

for /f "tokens=*" %%i in ('python --version') do set PYTHON_VER=%%i
echo [92m√[0m Python: %PYTHON_VER%

REM Sprawdź czy istnieje venv
if not exist "venv\" (
    echo.
    echo [93m! Nie znaleziono srodowiska wirtualnego.[0m
    echo   Czy chcesz je utworzyc? (T/N)
    set /p response=
    if /i "%response%"=="T" (
        echo Tworze srodowisko wirtualne...
        python -m venv venv
        echo [92m√[0m Utworzono venv
    )
)

REM Aktywuj venv jeśli istnieje
if exist "venv\Scripts\activate.bat" (
    echo Aktywuje srodowisko wirtualne...
    call venv\Scripts\activate.bat
    echo [92m√[0m Srodowisko aktywne
) else (
    echo [93m! Kontynuuje bez srodowiska wirtualnego[0m
)

REM Sprawdź czy zainstalowane są pakiety
python -c "import osmnx" >nul 2>&1
if errorlevel 1 (
    echo.
    echo [93m! Brak wymaganych pakietow.[0m
    echo   Instaluje zaleznosci z requirements.txt...
    python -m pip install --upgrade pip
    pip install -r requirements.txt
    if errorlevel 1 (
        echo [91mX Blad instalacji pakietow![0m
        pause
        exit /b 1
    )
    echo [92m√[0m Pakiety zainstalowane
)

REM Uruchom program główny
echo.
echo ========================================
echo  Uruchamiam symulacje...
echo ========================================
echo.

cd src
python main.py

REM Powrót do katalogu głównego
cd ..

REM Dezaktywuj venv
if exist "venv\Scripts\deactivate.bat" (
    call venv\Scripts\deactivate.bat
)

pause