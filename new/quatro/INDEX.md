# 📁 Indeks Projektu - Quadrocopter Simulation

## ✅ AKTYWNE PLIKI (DO UŻYCIA)

### 🚀 **Uruchamianie**
- `run_simulation.py` - Główny launcher z menu interaktywnym
- `QUICKSTART.md` - Szybki start (czytaj jako pierwszy!)
- `README.md` - Pełna dokumentacja

### 📂 **Moduły wspólne** (`common/`)
- `constants.py` - Parametry fizyczne drona (masa, momenty, limity)
- `integrator.py` - Integrator RK45 dla równań ruchu
- `linearization.py` - Linearyzacja układu dla LQR
- `lqr_controller.py` - Solver LQR (Riccati equation)
- `trajectory.py` - Generatory trajektorii 2D i 3D

### 📂 **Symulacje 2D** (`simulations_2D/`)
- `dron_PID.py` - Kontroler PID dla lotu 2D (X-Z)
- `dron_LQR.py` - Kontroler LQR dla lotu 2D

### 📂 **Symulacje 3D** (`simulations_3D/`)
- `dron_PID.py` ⭐ - Kontroler PID dla lotu 3D (X-Y-Z) **POLECANE**
- `dron_LQR.py` - Kontroler LQR dla lotu 3D

### 📂 **Dokumentacja** (`docs/`)
- `COMPARISON.md` - Porównanie różnych podejść
- `IMPLEMENTATION_SUMMARY.md` - Podsumowanie implementacji
- `QUICK_REFERENCE.md` - Szybka referencja
- `README_3D.md` - Szczegóły trajektorii 3D
- `README_SIMPLE_LQR.md` - Informacje o LQR

---

## 📦 ARCHIWUM (NIE UŻYWAĆ)

### 📂 `backup/`
Stare wersje plików przed uporządkowaniem:
- `main.py` - Stary główny plik (niestabilny)
- `quadrocopter_working.py` - Stara wersja 2D PID
- `quadrocopter_simple_lqr_stable.py` - Stara wersja 2D LQR
- `quadrocopter_3d_trajectory.py` - Stara wersja 3D PID
- `quadrocopter_3d_lqr.py` - Stara wersja 3D LQR
- `compare_controllers.py` - Stary skrypt porównawczy
- `visualize_trajectory.py` - Stara wizualizacja
- Duplikaty plików wspólnych

### 📂 `old_cascade_approach/`
Stare podejście kaskadowe (nieukończone):
- Różne pliki testowe i eksperymentalne

### 📂 `__pycache__/`
Skompilowane pliki Pythona (ignorować)

---

## 🎯 WORKFLOW DLA UŻYTKOWNIKA

### **Poziom 1: Pierwszy raz** (15 min)
1. Przeczytaj `QUICKSTART.md`
2. Uruchom `python run_simulation.py`
3. Wybierz opcję [1] - 2D PID (najprościej)
4. Obejrzyj wykresy

### **Poziom 2: Zrozumienie** (30 min)
1. Uruchom opcję [3] - 3D PID
2. Przeczytaj `README.md` - sekcje o kontrolerze PID
3. Zmodyfikuj parametry PID w `simulations_3D/dron_PID.py`
4. Uruchom ponownie i porównaj

### **Poziom 3: Eksperymenty** (1-2 godz)
1. Przeczytaj `README.md` - pełna dokumentacja
2. Uruchom LQR i porównaj z PID
3. Modyfikuj trajektorię w `common/trajectory.py`
4. Zmień parametry fizyczne w `common/constants.py`
5. Zbieraj statystyki dla pracy

---

## 📊 MAPA ZALEŻNOŚCI

```
run_simulation.py
    │
    ├─→ simulations_2D/dron_PID.py
    │       └─→ common/ (constants, integrator, trajectory)
    │
    ├─→ simulations_2D/dron_LQR.py
    │       └─→ common/ (constants, integrator, trajectory, linearization, lqr_controller)
    │
    ├─→ simulations_3D/dron_PID.py
    │       └─→ common/ (constants, integrator, trajectory)
    │
    └─→ simulations_3D/dron_LQR.py
            └─→ common/ (constants, integrator, trajectory, linearization, lqr_controller)
```

---

## 🔑 KLUCZOWE PLIKI DO EDYCJI

| Chcę zmienić... | Edytuj plik... | Sekcja... |
|---|---|---|
| **Parametry PID** | `simulations_3D/dron_PID.py` | Klasa `QuadcopterPIDController.__init__()` |
| **Parametry LQR** | `simulations_3D/dron_LQR.py` | Macierze Q i R w funkcji `main()` |
| **Trajektorię 3D** | `common/trajectory.py` | Funkcja `generate_3d_trajectory()` |
| **Masę drona** | `common/constants.py` | Zmienna `MASS` |
| **Limity silników** | `common/constants.py` | Zmienne `T_max`, `T_min` |
| **Czas symulacji** | `dron_*.py` | Zmienna `total_time` w `main()` |

---

## ✅ STATUS PLIKÓW

| Plik | Status | Uwagi |
|---|---|---|
| `run_simulation.py` | ✅ GOTOWY | Uruchamia wszystko |
| `simulations_3D/dron_PID.py` | ✅ GOTOWY | Najlepsza opcja |
| `simulations_3D/dron_LQR.py` | ✅ GOTOWY | Wymaga tuningu |
| `simulations_2D/dron_PID.py` | ✅ GOTOWY | Do nauki |
| `simulations_2D/dron_LQR.py` | ✅ GOTOWY | Do porównania |
| `common/*.py` | ✅ GOTOWE | Stabilne moduły |
| `backup/*` | ⚠️ ARCHIWUM | Nie używać |

---

## 📞 POMOC

**Problem z importem?**
→ Sprawdź czy jesteś w `src_new/`, nie w `simulations_*/`

**Wykresy nie działają?**
→ `pip install matplotlib numpy scipy`

**Dron spada/przewraca się?**
→ Zobacz README.md sekcja "Rozwiązywanie Problemów"

**Chcę zmienić trajektorię?**
→ Edytuj `common/trajectory.py`, funkcja `generate_3d_trajectory()`

---

**Wszystko jasne? Zaczynaj od QUICKSTART.md! 🚀**
