@echo off
chcp 65001 >nul
REM Uruchamianie - System Symulacji Drona

echo.
echo ========================================
echo  SYSTEM SYMULACJI DRONA
echo  Adam Nowak - Politechnika Warszawska  
echo ========================================
echo.

REM Sprawdz czy venv istnieje
if not exist "venv\Scripts\activate.bat" (
    echo [BLAD] Srodowisko wirtualne nie istnieje!
    echo.
    echo Najpierw uruchom instalacje:
    echo   install.bat
    echo.
    pause
    exit /b 1
)

REM Sprawdz czy main.py istnieje
if not exist "main.py" (
    echo [BLAD] Nie znaleziono main.py!
    echo.
    echo Program wymaga pliku main.py w tym katalogu.
    echo.
    pause
    exit /b 1
)

REM Aktywuj srodowisko
echo Aktywuje srodowisko wirtualne...
call venv\Scripts\activate.bat

REM Sprawdz czy katalog data istnieje
if not exist "data\" (
    echo Tworze katalog data...
    mkdir data
)

REM Uruchom program
echo.
echo Uruchamiam program...
echo ========================================
echo.

python main.py

REM Po zakonczeniu
echo.
echo ========================================
echo Program zakonczyl dzialanie
echo.
pause