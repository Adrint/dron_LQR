@echo off
REM Skrypt instalacyjny dla Windows

echo ========================================
echo  Instalacja: System Symulacji Drona
echo ========================================
echo.

REM Sprawdz Python
echo Sprawdzam Python...
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python nie jest zainstalowany!
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
echo [OK] %PYTHON_VER%

REM Sprawdz pip
echo Sprawdzam pip...
python -m pip --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] pip nie jest zainstalowany!
    echo Instaluje pip...
    python -m ensurepip --upgrade
)
echo [OK] pip dostepny

REM Utworz srodowisko wirtualne
echo.
echo Tworze srodowisko wirtualne...
if exist "venv\" (
    echo [WARN] Katalog venv juz istnieje
    set /p response=Czy chcesz go usunac i utworzyc ponownie? (T/N): 
    if /i "%response%"=="T" (
        rd /s /q venv
        python -m venv venv
        echo [OK] Utworzono nowe srodowisko
    )
) else (
    python -m venv venv
    echo [OK] Utworzono srodowisko wirtualne
)

REM Aktywuj srodowisko
echo Aktywuje srodowisko...
call venv\Scripts\activate.bat
if errorlevel 1 (
    echo [ERROR] Nie udalo sie aktywowac srodowiska
    pause
    exit /b 1
)

REM Upgrade pip
echo Aktualizuje pip...
python -m pip install --upgrade pip >nul 2>&1
echo [OK] pip zaktualizowany

REM Instalacja zaleznosci
echo.
echo Instaluje zaleznosci (moze to chwile potrwac)...
echo Pakiety do zainstalowania:
type requirements.txt | findstr /v "^#" | findstr /v "^$"
echo.

echo Instalowanie pakietow podstawowych...
pip install numpy scipy matplotlib pandas

echo Instalowanie pakietow geograficznych...
pip install pyproj geopy shapely

echo Instalowanie geopandas i osmnx...
pip install geopandas osmnx

if errorlevel 1 (
    echo.
    echo [ERROR] Wystapily bledy podczas instalacji
    echo.
    echo Zalecane reczne kroki:
    echo   pip install pipwin
    echo   pipwin install gdal
    echo   pipwin install fiona
    echo   pip install -r requirements.txt
    pause
    exit /b 1
)

echo [OK] Wszystkie pakiety zainstalowane pomyslnie!

REM Weryfikacja importow
echo.
echo Weryfikuje instalacje...
python -c "import numpy; print('[OK] numpy')"
python -c "import matplotlib; print('[OK] matplotlib')"
python -c "import scipy; print('[OK] scipy')"
python -c "import osmnx; print('[OK] osmnx')"
python -c "import geopandas; print('[OK] geopandas')"

if errorlevel 1 (
    echo [ERROR] Blad importu modulow
    pause
    exit /b 1
)

echo.
echo [OK] Wszystkie kluczowe moduly dostepne

REM Utworz katalog data
if not exist "src\data\" (
    mkdir src\data
    echo [OK] Utworzono katalog src\data
)

REM Podsumowanie
echo.
echo ========================================
echo  Instalacja zakonczona pomyslnie!
echo ========================================
echo.
echo Aby uruchomic program:
echo   1. Aktywuj srodowisko: venv\Scripts\activate
echo   2. Uruchom: cd src && python main.py
echo.
echo Lub uzyj skryptu:
echo   run.bat
echo.
pause
