@echo off
REM Skrypt instalacyjny dla Windows

echo ========================================
echo  Instalacja: System Symulacji Drona
echo ========================================
echo.

REM Sprawdź Python
echo Sprawdzam Python...
python --version >nul 2>&1
if errorlevel 1 (
    echo [91mX Python nie jest zainstalowany![0m
    echo.
    echo Pobierz Python 3.8 lub nowszy z:
    echo https://www.python.org/downloads/
    echo.
    echo Podczas instalacji zaznacz:
    echo   [X] Add Python to PATH
    pause
    exit /b 1
)

for /f "tokens=*" %%i in ('python --version') do set PYTHON_VER=%%i
echo [92m√[0m %PYTHON_VER%

REM Sprawdź pip
echo Sprawdzam pip...
python -m pip --version >nul 2>&1
if errorlevel 1 (
    echo [91mX pip nie jest zainstalowany![0m
    echo Instaluje pip...
    python -m ensurepip --upgrade
)
echo [92m√[0m pip dostepny

REM Utwórz środowisko wirtualne
echo.
echo Tworze srodowisko wirtualne...
if exist "venv\" (
    echo [93m! Katalog venv juz istnieje[0m
    set /p response=Czy chcesz go usunac i utworzyc ponownie? (T/N): 
    if /i "%response%"=="T" (
        rd /s /q venv
        python -m venv venv
        echo [92m√[0m Utworzono nowe srodowisko
    )
) else (
    python -m venv venv
    echo [92m√[0m Utworzono srodowisko wirtualne
)

REM Aktywuj środowisko
echo Aktywuje srodowisko...
call venv\Scripts\activate.bat
if errorlevel 1 (
    echo [91mX Nie udalo sie aktywowac srodowiska[0m
    pause
    exit /b 1
)

REM Upgrade pip
echo Aktualizuje pip...
python -m pip install --upgrade pip >nul 2>&1
echo [92m√[0m pip zaktualizowany

REM Instaluj zależności
echo.
echo Instaluje zaleznosci (moze to chwile potrwac)...
echo Pakiety do zainstalowania:
type requirements.txt | findstr /v "^#" | findstr /v "^$"
echo.

REM Specjalne traktowanie dla Windows - instaluj po kolei
echo Instalowanie pakietow podstawowych...
pip install numpy scipy matplotlib pandas

echo Instalowanie pakietow geograficznych...
pip install pyproj geopy shapely

echo Instalowanie geopandas i osmnx...
pip install geopandas osmnx

if errorlevel 1 (
    echo.
    echo [91mX Wystapily bledy podczas instalacji[0m
    echo.
    echo Sprobuj zainstalowac recznie problematyczne pakiety:
    echo   pip install pipwin
    echo   pipwin install gdal
    echo   pipwin install fiona
    echo   pip install -r requirements.txt
    pause
    exit /b 1
)

echo [92m√[0m Wszystkie pakiety zainstalowane pomyslnie!

REM Sprawdź kluczowe importy
echo.
echo Weryfikuje instalacje...
python -c "import numpy; print('√ numpy')"
python -c "import matplotlib; print('√ matplotlib')"
python -c "import scipy; print('√ scipy')"
python -c "import osmnx; print('√ osmnx')"
python -c "import geopandas; print('√ geopandas')"

if errorlevel 1 (
    echo [91mX Blad importu modulow[0m
    pause
    exit /b 1
)

echo.
echo [92m√ Wszystkie kluczowe moduly dostepne[0m

REM Utwórz katalog data
if not exist "src\data\" (
    mkdir src\data
    echo [92m√[0m Utworzono katalog src\data
)

REM Podsumowanie
echo.
echo ========================================
echo [92m√ Instalacja zakonczona pomyslnie![0m
echo ========================================
echo.
echo Aby uruchomic program:
echo   1. Aktywuj srodowisko: venv\Scripts\activate
echo   2. Uruchom: cd src ^&^& python main.py
echo.
echo Lub uzyj skryptu:
echo   run.bat
echo.
pause