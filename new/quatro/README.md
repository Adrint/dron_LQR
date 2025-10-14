# 🚁 Quadrocopter Simulation - Uporządkowana Struktura

## 📁 Struktura Projektu

```
src_new/
├── common/                      # Wspólne moduły
│   ├── constants.py            # Parametry fizyczne drona
│   ├── integrator.py           # Integrator RK45
│   ├── linearization.py        # Linearyzacja dla LQR
│   ├── lqr_controller.py       # Rozwiązywacz LQR
│   └── trajectory.py           # Generatory trajektorii (2D i 3D)
│
├── simulations_2D/             # Symulacje 2D (X-Z)
│   ├── dron_PID.py            # ⭐ Kontroler PID 2D
│   └── dron_LQR.py            # Kontroler LQR 2D
│
├── simulations_3D/             # Symulacje 3D (X-Y-Z)
│   ├── dron_PID.py            # ⭐ Kontroler PID 3D
│   └── dron_LQR.py            # Kontroler LQR 3D
│
├── docs/                       # Dokumentacja
│   └── (pliki README i MD)
│
├── backup/                     # Stare wersje
└── old_cascade_approach/       # Stare podejście kaskadowe
```

---

## 🚀 Szybki Start

### **Opcja 1: Symulacja 2D (Najprotsza)** ⭐ POLECANE NA START

```bash
cd simulations_2D
python dron_PID.py
```

**Co zobaczysz:**
- Lewy wykres: Profil lotu Z(X) z terenem
- Prawy wykres: Wysokość vs czas
- Prosty lot w płaszczyźnie X-Z

---

### **Opcja 2: Symulacja 3D (Zaawansowana)**

```bash
cd simulations_3D
python dron_PID.py
```

**Co zobaczysz:**
- Górny rząd:
  - 3D trajektoria (stały kąt widzenia)
  - Z(X) - profil wysokości z terenem
  - Y(X) - profil manewrów bocznych
- Dolny rząd:
  - Pozycje XYZ vs czas
  - Kąty roll/pitch/yaw
  - Siły silników

---

## 📊 Trajektoria 3D - Opis Manewrów

### **Uproszczona trajektoria:**

**0-10s**: Lot prosty
- Y = 0m (bez manewrów bocznych)
- Śledzenie terenu

**10-20s**: Slalom (fala sinusoidalna)
- Y oscyluje ±3m
- 2 pełne fale
- Testuje kontrolę boczną

**20-30s**: Krzywa S (tłumiona)
- Płynne przejście z fali do zera
- Oscylacja z tłumieniem wykładniczym

**30-50s**: Powrót i lot prosty
- Powrót do Y=0
- Stabilizacja na linii prostej

---

## 🎮 Który Kontroler Wybrać?

| Cecha | PID | LQR |
|-------|-----|-----|
| **Łatwość użycia** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Stabilność** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Wydajność** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Tuning** | Łatwy | Trudny |
| **Zalecenie** | **START TUTAJ** | Dla zaawansowanych |

---

## 🔧 Parametry Fizyczne

Zdefiniowane w `common/constants.py`:

```python
MASS = 2.0 kg           # Masa drona
g = 9.81 m/s²          # Przyspieszenie grawitacyjne
T_max = 400.0 N        # Max siła silnika
T_min = 0.0 N          # Min siła silnika
MOTOR_ARM_LENGTH = 0.25 m
IX, IY, IZ = 0.01, 0.01, 0.02 kg·m²  # Momenty bezwładności
```

---

## 📈 Interpretacja Wykresów

### **Wykres 3D (simulations_3D):**
- **Czerwona linia przerywana**: Trajektoria referencyjna
- **Niebieska linia ciągła**: Rzeczywista ścieżka drona
- **Niebieski punkt**: Aktualna pozycja

### **Wykres Z(X):**
- **Brązowe wypełnienie**: Teren
- **Czerwona linia**: Referencyjna wysokość
- **Niebieska linia**: Rzeczywista wysokość

### **Wykres Y(X):**
- **Czerwona linia**: Pożądane manewry boczne
- **Niebieska linia**: Rzeczywiste manewry
- **Linia przerywana Y=0**: Środek

---

## ⚙️ Dostrajanie Kontrolerów

### **PID (łatwiejsze):**

W pliku `dron_PID.py`, klasa kontrolera:

```python
# Wysokość (Z)
self.kp_z = 25.0  # ↑ = szybsza reakcja
self.kd_z = 12.0  # ↑ = większe tłumienie

# Pozycja pozioma (X, Y)
self.kp_x = 3.0   # ↑ = szybsze śledzenie X
self.kp_y = 3.0   # ↑ = szybsze śledzenie Y

# Kąty
self.kp_att = 6.0  # ↑ = szybsze pochylanie (UWAGA!)
self.kd_att = 3.0  # ↑ = większe tłumienie kątów
```

**Zasady dostrajania PID:**
1. Zacznij od małych wartości
2. Zwiększaj `kp` aż do oscylacji
3. Dodaj `kd` aby stłumić oscylacje
4. Zawsze `kd` < `kp`

---

### **LQR (trudniejsze):**

W pliku `dron_LQR.py`:

```python
R = np.eye(m) * 15.0  # Kara za sterowanie (↑ = łagodniejsze)

Q[0, 0] = 3.0    # waga dla vx
Q[1, 1] = 3.0    # waga dla vy
Q[8, 8] = 50.0   # waga dla Z (najważniejsze!)
Q[9, 9] = 25.0   # waga dla phi (roll)
Q[10, 10] = 25.0 # waga dla theta (pitch)
```

**Zasady LQR:**
- **Zwiększ R**: Bardziej łagodne sterowanie (bezpieczniejsze)
- **Zwiększ Q[i,i]**: Większy nacisk na stan `i`
- **Z (wysokość)**: Zawsze najwyższa waga!
- **Kąty**: Wysokie wagi = zapobiega przewracaniu

---

## 🐛 Rozwiązywanie Problemów

### **Problem: Dron spada**
✅ Zwiększ `kp_z` i `kd_z` (PID)
✅ Zwiększ `Q[8,8]` (LQR)

### **Problem: Dron się przewraca**
✅ Zmniejsz `kp_att` (PID)
✅ Zwiększ `Q[9,9]` i `Q[10,10]` (LQR)
✅ Zwiększ `R` (LQR)

### **Problem: Oscylacje**
✅ Zwiększ składowe `kd_*` (PID)
✅ Zwiększ `R` (LQR)

### **Problem: Zbyt wolne śledzenie**
✅ Zwiększ `kp_x`, `kp_y` (PID)
✅ Zwiększ `Q[6,6]`, `Q[7,7]` (LQR)

### **Problem: LQR zwraca NaN**
✅ Zwiększ `R` (prawdopodobnie za mała kara)
✅ Sprawdź czy stan nie jest ekstremalny

---

## 📚 Dla Pracy Magisterskiej

### **Eksperymenty do przeprowadzenia:**

1. **Porównanie kontrolerów**
   - Uruchom PID i LQR dla tej samej trajektorii
   - Porównaj błędy śledzenia
   - Oceń stabilność

2. **Analiza wpływu parametrów**
   - Zmień wagi Q i R dla LQR
   - Zapisz błędy i statystyki
   - Stwórz wykresy

3. **Testy trajektorii**
   - Modyfikuj `trajectory.py`
   - Testuj ostrzejsze manewry
   - Znajdź granice kontrolera

4. **Odporność na zakłócenia**
   - Dodaj szum do pomiarów
   - Symuluj wiatr
   - Oceń degradację wydajności

---

## 📝 Modyfikacja Trajektorii

Edytuj `common/trajectory.py`, funkcja `generate_3d_trajectory()`:

```python
# Przykład: Bardziej agresywny slalom
elif t < 20.0:
    phase = (t - 10.0) / 10.0
    Y_ref[i] = 5.0 * np.sin(phase * 4 * np.pi)  # Było: 3.0 i 2*pi
```

**Możliwe modyfikacje:**
- Amplituda manewrów (±3m → ±5m)
- Częstotliwość oscylacji (2 fale → 4 fale)
- Czas trwania segmentów
- Dodanie nowych manewrów (spirale, ósemki)

---

## 🎓 Struktura Kodu

### **Wspólne moduły (common/):**
- `constants.py`: Wszystkie stałe fizyczne
- `integrator.py`: RK45 do integracji stanu
- `linearization.py`: Linearyzacja dynamiki
- `lqr_controller.py`: Rozwiązywacz LQR (Riccati)
- `trajectory.py`: Generatory referencji 2D i 3D

### **Symulacje:**
Każdy plik symulacji ma strukturę:
1. Import modułów
2. Definicja kontrolera (klasa lub funkcja)
3. Funkcja `main()`:
   - Inicjalizacja
   - Generacja trajektorii
   - Pętla symulacyjna
   - Wizualizacja
   - Statystyki końcowe

---

## 🔬 Stan Systemu

Wektor stanu (12 elementów):
```
x = [vx, vy, vz,      # Prędkości liniowe [m/s]
     p, q, r,         # Prędkości kątowe [rad/s]
     X, Y, Z,         # Pozycja [m]
     phi, theta, psi] # Kąty Eulera [rad]
```

Sterowanie (4 silniki):
```
u = [T1, T2, T3, T4]  # Siły ciągu [N]
```

Konfiguracja X:
```
    2 (FL)  1 (FR)
       \    /
        \  /
         \/
         /\
        /  \
       /    \
    3 (BL)  4 (BR)
```

---

## 📞 Wsparcie

W razie problemów:
1. Sprawdź wydruki w konsoli
2. Obejrzyj wykresy w czasie rzeczywistym
3. Przeczytaj komentarze w kodzie
4. Zacznij od prostej symulacji 2D PID

---

**Powodzenia z symulacją! 🚁✨**
