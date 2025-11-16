# System Symulacji Drona - Projekt Magisterski

System symulacji lotu quadrocoptera z planowaniem trasy 3D i sterowaniem LQR.

## 📋 Wymagania systemowe

- Python 3.8 lub nowszy
- System operacyjny: Windows, Linux, macOS
- ~500MB wolnego miejsca na dane map

## 🚀 Instalacja

### 1. Sklonuj/Pobierz projekt

```bash
# Rozpakuj archiwum lub sklonuj repozytorium
cd drone-simulation
```

### 2. Utwórz wirtualne środowisko (zalecane)

**Linux/macOS:**
```bash
python3 -m venv venv
source venv/bin/activate
```

**Windows:**
```cmd
python -m venv venv
venv\Scripts\activate
```

### 3. Zainstaluj zależności

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Uwaga dla Windows:** W przypadku problemów z geopandas:
```bash
pip install pipwin
pipwin install gdal
pipwin install fiona
pip install -r requirements.txt
```

## 🎯 Uruchomienie

### Główny program

```bash
cd src
python main.py
```

### Poszczególne moduły (opcjonalnie)

**1. Pobieranie danych map:**
```bash
cd src
python map_1_data.py
```

**2. Wybór punktów A i B:**
```bash
python map_2_select_points.py
```

**3. Tylko symulacja (jeśli trasa już zaplanowana):**
```bash
python control_dron_lqr.py
```

## 📚 Struktura projektu

```
drone-simulation/
├── src/                          # Kod źródłowy
│   ├── main.py                   # Główny program
│   ├── config.py                 # Konfiguracja globalna
│   ├── config_parameters.py      # Parametry drona
│   ├── map_1_data.py            # Pobieranie map OSM
│   ├── map_2_select_points.py   # Wybór punktów
│   ├── map_3_path_planner.py    # Algorytm A* 3D
│   ├── map_4_animate_route.py   # Animacja lotu
│   ├── map_5_visualization.py   # Wizualizacja orientacji
│   ├── control_dron_lqr.py      # Regulator LQR
│   ├── control_lqr.py           # Obliczenia LQR
│   ├── control_matrices.py      # Linearyzacja
│   ├── control_rhs.py           # Model dynamiki
│   ├── control_rk45.py          # Integrator RK45
│   ├── control_limits.py        # Limity fizyczne
│   ├── trajectory.py            # Generator trajektorii
│   └── geo_utils.py             # Narzędzia geograficzne
├── data/                         # Dane map (tworzone automatycznie)
├── requirements.txt              # Zależności Python
└── README.md                     # Ten plik
```

## 🎮 Instrukcja użycia

### Pierwszy start - Tryb Interaktywny

1. Uruchom program:
   ```bash
   python main.py
   ```

2. **Sekcja 1 - Dane map:**
   - Program pobierze dane Warszawy z OpenStreetMap (~100MB)
   - Dane są zapisywane w `data/` i wykorzystywane ponownie

3. **Sekcja 2 - Wybór punktów:**
   - **Opcja 1 (zalecana):** Interaktywna mapa
     - Naciśnij SPACJĘ aby włączyć tryb zaznaczania
     - Kliknij punkt A (start), potem punkt B (cel)
     - Zamknij wykres
   - **Opcja 2:** Ręczne współrzędne
   - **Opcja 3:** Przykładowe punkty (szybki test)

4. **Sekcja 3 - Konfiguracja drona:**
   - Naciśnij ENTER = użyj domyślnych parametrów
   - Lub wpisz własne wartości

5. **Sekcja 4 - Planowanie trasy:**
   - Algorytm A* 3D znajdzie trasę omijającą budynki
   - Wyświetli wizualizację 3D

6. **Sekcja 5 - Symulacja:**
   - Symulacja lotu z wizualizacją w czasie rzeczywistym
   - 3 okna: trasa 3D, widok z góry, profil wysokości

### Tryb szybki (domyślne wartości)

Po pierwszym uruchomieniu, dane map są zapisane. Następne uruchomienia:

```bash
python main.py
# Naciśnij ENTER dla wszystkich pytań = użyj domyślnych wartości
```

## ⚙️ Parametry konfiguracyjne

### Fizyka drona (domyślne)
- Masa: 9 kg
- Masa rotora: 1 kg każdy
- Ramię: 0.7 m
- Wymiary korpusu: 0.2 × 0.2 × 0.2 m

### Parametry lotu
- Wysokość startowa: 0 m
- Wysokość przelotu: 1-3 m
- Prędkość: 5 m/s
- Margines bezpieczeństwa: 4 m

### Limity fizyczne
- Maks. prędkość pozioma: 23 m/s
- Maks. prędkość wznoszenia: 6 m/s
- Maks. kąty pitch/roll: 25°
- Maks. prędkość kątowa: 300°/s

## 🔧 Rozwiązywanie problemów

### ImportError: No module named 'osmnx'
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### Błąd geopandas na Windows
Zainstaluj GDAL używając pipwin (patrz sekcja Instalacja)

### Brak połączenia z internetem
Pierwsz uruchomienie wymaga internetu do pobrania map. Kolejne działają offline.

### Błąd "Nie znaleziono ścieżki"
- Wybierz punkty poza budynkami (ulice, place, parki)
- Zwiększ wysokość lotu w konfiguracji
- Wybierz bliższe punkty

### Wykres nie wyświetla się
```bash
# Linux - zainstaluj Tkinter
sudo apt-get install python3-tk

# macOS
brew install python-tk
```

## 📊 Przykładowe punkty testowe (Warszawa)

- **Plac Zamkowy → Katedra:**
  - A: (52.2248, 20.9891)
  - B: (52.2171, 20.9977)

- **Centrum → Stadion Narodowy:**
  - A: (52.2297, 21.0122)
  - B: (52.2396, 21.0464)

## 📝 Notatki

- Pierwsze uruchomienie trwa dłużej (pobieranie map)
- Dane map są cache'owane w `data/`
- Symulacja działa w czasie rzeczywistym (dt=0.05s)
- Wszystkie współrzędne w układzie NED (North-East-Down)

