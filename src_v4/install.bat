@echo off
echo.
echo Instalacja - System Symulacji Drona
echo.

REM Krok 1: Python
echo [1/4] Python...
python --version
if errorlevel 1 goto ERROR_PYTHON
echo OK
echo.

REM Krok 2: Venv
echo [2/4] Tworzenie venv...
python -m venv venv
if errorlevel 1 goto ERROR_VENV
echo OK
echo.

REM Krok 3: Aktywacja
echo [3/4] Aktywacja...
call venv\Scripts\activate.bat
if errorlevel 1 goto ERROR_ACTIVATE
echo OK
echo.

REM Krok 4: Pakiety
echo [4/4] Pakiety (5-10 min)...
python -m pip install --upgrade pip --quiet
pip install numpy scipy matplotlib pandas --quiet
pip install pyproj geopy shapely --quiet
pip install geopandas osmnx requests --quiet
echo OK
echo.

echo Sukces!
echo Uruchom: run.bat
echo.
pause
exit /b 0

:ERROR_PYTHON
echo BLAD: Python nie dziala
pause
exit /b 1

:ERROR_VENV
echo BLAD: Nie mozna utworzyc venv
pause
exit /b 1

:ERROR_ACTIVATE
echo BLAD: Nie mozna aktywowac
pause
exit /b 1