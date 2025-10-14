# 🚁 Quadrocopter Simulation - Quick Start Guide

## 📦 NOWA UPORZĄDKOWANA STRUKTURA

```
src_new/
│
├── 📂 common/                    # Wspólne moduły
│   ├── constants.py             # Parametry fizyczne
│   ├── integrator.py            # RK45 integrator
│   ├── linearization.py         # Linearyzacja dla LQR
│   ├── lqr_controller.py        # Solver LQR
│   └── trajectory.py            # Generatory trajektorii
│
├── 📂 simulations_2D/           # ✅ SYMULACJE 2D (X-Z)
│   ├── dron_PID.py             # PID controller 2D
│   └── dron_LQR.py             # LQR controller 2D
│
├── 📂 simulations_3D/           # ✅ SYMULACJE 3D (X-Y-Z)
│   ├── dron_PID.py             # PID controller 3D ⭐
│   └── dron_LQR.py             # LQR controller 3D
│
├── 📂 docs/                     # Dokumentacja
│   ├── COMPARISON.md
│   ├── IMPLEMENTATION_SUMMARY.md
│   ├── QUICK_REFERENCE.md
│   ├── README_3D.md
│   └── README_SIMPLE_LQR.md
│
├── 📂 backup/                   # Stare wersje (archiwum)
├── 📂 old_cascade_approach/     # Stare podejście
│
├── 📄 README.md                 # ⭐ GŁÓWNA DOKUMENTACJA
└── 📄 run_simulation.py         # ⭐ LAUNCHER (menu)
```

---

## 🚀 SZYBKI START (3 sposoby)

### **Sposób 1: Launcher z menu** ⭐ NAJŁATWIEJSZY

```bash
python run_simulation.py
```

Pojawi się interaktywne menu:
```
🚁 QUADROCOPTER SIMULATION LAUNCHER
================================================================================

📊 Wybierz symulację:

  2D Simulations (prostsze):
    [1] 2D PID  - Kontroler PID dla lotu 2D (X-Z)
    [2] 2D LQR  - Kontroler LQR dla lotu 2D

  3D Simulations (zaawansowane):
    [3] 3D PID  - Kontroler PID dla lotu 3D (X-Y-Z) ⭐ POLECANE
    [4] 3D LQR  - Kontroler LQR dla lotu 3D

    [0] Wyjście
```

---

### **Sposób 2: Bezpośrednio** (dla zaawansowanych)

```bash
# 2D PID (najprostrza)
cd simulations_2D
python dron_PID.py

# 3D PID (polecana)
cd simulations_3D
python dron_PID.py
```

---

### **Sposób 3: Z głownego folderu**

```bash
# Windows
python simulations_3D\dron_PID.py

# Linux/Mac
python simulations_3D/dron_PID.py
```

---

## 📊 CO ZOBACZYSZ?

### **Symulacja 2D:**
- **Lewy wykres**: Profil lotu Z(X) z terenem (brązowe wypełnienie)
- **Prawy wykres**: Wysokość vs czas

### **Symulacja 3D:**
- **Rząd górny**:
  1. Trajektoria 3D (perspektywa, stały kąt)
  2. Profil wysokości Z(X) + teren
  3. Profil manewrów Y(X) (ruchy boczne)
  
- **Rząd dolny**:
  4. Pozycje X, Y, Z vs czas
  5. Kąty roll, pitch, yaw vs czas
  6. Siły silników T1-T4 vs czas

---

## 🎯 TRAJEKTORIA 3D (UPROSZCZONA)

**0-10s**: Lot prosty (bez manewrów)

**10-20s**: Slalom
- Fala sinusoidalna ±3m
- 2 pełne oscylacje
- Testuje kontrolę boczną

**20-30s**: Krzywa S
- Tłumiona oscylacja
- Powrót do linii prostej

**30-50s**: Finisz
- Stabilizacja na Y=0
- Lot prosty

---

## 🎮 KTÓRĄ SYMULACJĘ WYBRAĆ?

### **Dla początkujących:**
✅ `simulations_2D/dron_PID.py`
- Najprostsza
- Szybka (1-2 min)
- Dobrze pokazuje podstawy

### **Dla pracy magisterskiej:**
✅ `simulations_3D/dron_PID.py`
- Pełna funkcjonalność 3D
- Stabilna i niezawodna
- Dobre wyniki

### **Dla eksperymentów:**
⚠️ `simulations_3D/dron_LQR.py`
- Wymaga tuningu
- Może być niestabilna
- Dla zaawansowanych

---

## ⚙️ DOSTRAJANIE

### **PID (łatwe):**

Edytuj plik `dron_PID.py`, klasa kontrolera:

```python
# Wysokość
self.kp_z = 25.0  # ↑ szybciej reaguje
self.kd_z = 12.0  # ↑ większe tłumienie

# Pozycja X, Y
self.kp_x = 3.0   # ↑ szybsze śledzenie
self.kp_y = 3.0   # ↑ lepsze manewry boczne

# Kąty
self.kp_att = 6.0  # ↑ szybsze pochylanie
self.kd_att = 3.0  # ↑ stabilniejsze kąty
```

**Reguły:**
- Zaczynaj od małych wartości
- Zwiększaj `kp` aż do oscylacji
- Dodaj `kd` żeby stłumić
- Zawsze: `kd` < `kp`

---

### **LQR (trudne):**

Edytuj plik `dron_LQR.py`:

```python
R = np.eye(m) * 15.0  # Kara za sterowanie
                      # ↑ = łagodniejsze, ↓ = agresywniejsze

Q[8, 8] = 50.0   # Wysokość Z - NAJWAŻNIEJSZA!
Q[9, 9] = 25.0   # Roll phi
Q[10, 10] = 25.0 # Pitch theta
```

**Jeśli dron:**
- **Spada**: ↑ Q[8,8]
- **Przewraca się**: ↑ Q[9,9], Q[10,10], ↑ R
- **Oscyluje**: ↑ R
- **Wolno reaguje**: ↓ R

---

## 🐛 ROZWIĄZYWANIE PROBLEMÓW

### **Import Error: No module named 'common'**

Upewnij się że jesteś w odpowiednim folderze:

```bash
# Jeśli w simulations_3D/
cd ..

# Teraz uruchom launcher
python run_simulation.py
```

---

### **Dron spada / przewraca się**

**PID:**
```python
self.kp_z = 30.0  # było 25.0
self.kd_z = 15.0  # było 12.0
```

**LQR:**
```python
Q[8, 8] = 100.0   # było 50.0
Q[9, 9] = 50.0    # było 25.0
Q[10, 10] = 50.0  # było 25.0
R = np.eye(m) * 20.0  # było 15.0
```

---

### **Wykresy nie wyświetlają się**

Sprawdź czy masz zainstalowane matplotlib:
```bash
pip install matplotlib numpy scipy
```

---

## 📚 WIĘCEJ INFORMACJI

- **Główny README**: `README.md` (szczegółowa dokumentacja)
- **Dokumentacja**: folder `docs/`
- **Stare wersje**: folder `backup/` (do porównania)

---

## 🎓 DLA PRACY MAGISTERSKIEJ

### **Zalecana kolejność:**

1. **Uruchom 2D PID** - zrozum podstawy
2. **Uruchom 3D PID** - zobacz pełną funkcjonalność
3. **Eksperymentuj z LQR** - porównaj z PID
4. **Modyfikuj trajektorię** - testuj różne scenariusze
5. **Zbieraj statystyki** - analizuj błędy i wydajność

### **Pliki do edycji:**

- **Trajektoria**: `common/trajectory.py`
- **Parametry PID**: `simulations_3D/dron_PID.py`
- **Parametry LQR**: `simulations_3D/dron_LQR.py`
- **Fizyka drona**: `common/constants.py`

---

## ✅ CHECKLIST

- [ ] Zainstalowane Python 3.x
- [ ] Zainstalowane biblioteki (numpy, scipy, matplotlib)
- [ ] Struktura folderów OK (common/, simulations_2D/, simulations_3D/)
- [ ] Uruchomiony launcher: `python run_simulation.py`
- [ ] Przetestowana symulacja 2D PID
- [ ] Przetestowana symulacja 3D PID

---

**Wszystko gotowe! Powodzenia! 🚀**

*W razie problemów, sprawdź główny README.md lub komentarze w kodzie.*
