# Quadrocopter 3D Trajectory Simulation

## Nowe pliki z trajektorią 3D

Dodano obsługę trajektorii 3D z manewrami poprzecznymi (os Y)!

### 📁 Nowe pliki:

1. **`trajectory.py`** (zaktualizowany)
   - Funkcja `generate_3d_trajectory()` - generuje ekscytującą trajektorię 3D
   
2. **`quadrocopter_3d_trajectory.py`** 
   - Kontroler PID dla trajektorii 3D
   - **ZALECANY** - stabilny i niezawodny
   
3. **`quadrocopter_3d_lqr.py`**
   - Kontroler LQR dla trajektorii 3D
   - Bardziej zaawansowany, wymaga tuningu

---

## 🎯 Trajektoria 3D - Co zawiera?

Trajektoria obejmuje różnorodne manewry:

### **0-5s**: Start
- Lot prosty
- Wznoszenie do wysokości operacyjnej

### **5-10s**: Manewer 1 - Łagodna krzywa S
- Delikatne przechylenie na bok (±3m)
- Testuje kontrolę boczną

### **10-15s**: Manewer 2 - Ostry skręt
- Szybka zmiana kierunku
- Pop-up (zwiększenie wysokości)

### **15-22s**: Manewer 3 - Slalom
- Falowanie sinusoidalne (±4m amplituda)
- Jednoczesne zmiany wysokości
- 3 pełne oscylacje

### **22-28s**: Manewer 4 - Łuk/Okrąg
- Łuk o promieniu 5m
- Testuje zdolność do zakrętów

### **28-35s**: Manewer 5 - Powrót do centrum
- Tłumione oscylacje
- Stabilizacja na osi Y=0
- Stopniowe opadanie

### **35-50s**: Finał
- Lot prosty
- Przygotowanie do lądowania

---

## 🚀 Jak uruchomić?

### Opcja 1: Kontroler PID (ZALECANA)
```bash
python quadrocopter_3d_trajectory.py
```
✅ Stabilny i niezawodny
✅ Dobre śledzenie trajektorii
✅ Łatwy w użyciu

### Opcja 2: Kontroler LQR
```bash
python quadrocopter_3d_lqr.py
```
⚠️ Wymaga tuningu parametrów Q i R
⚠️ Może być niestabilny przy agresywnych manewrach

---

## 📊 Wizualizacja

Oba programy pokazują:

1. **Wykres 3D** (lewy)
   - Trajektoria 3D w czasie rzeczywistym
   - Czerwona linia = referencja
   - Niebieska linia = rzeczywista ścieżka
   - Niebieski punkt = aktualna pozycja drona
   - Widok obraca się automatycznie!

2. **Wykresy czasowe** (prawe)
   - Górny: Pozycja X vs czas
   - Środkowy: Pozycja Y vs czas (manewry boczne!)
   - Dolny: Wysokość Z vs czas

---

## 🎮 Parametry kontrolera PID

W pliku `quadrocopter_3d_trajectory.py`:

```python
# Wysokość
self.kp_z = 25.0
self.kd_z = 12.0

# Pozycja pozioma (X i Y)
self.kp_x = 3.0
self.kd_x = 4.0
self.kp_y = 3.0
self.kd_y = 4.0

# Kąty (roll, pitch)
self.kp_att = 6.0
self.kd_att = 3.0

# Yaw
self.kp_yaw = 3.0
self.kd_yaw = 1.5
```

### Jak dostroić?

- **Zwiększ kp_x / kp_y** → szybsza reakcja na błędy pozycji
- **Zwiększ kd_x / kd_y** → większe tłumienie, mniej oscylacji
- **Zwiększ kp_att** → szybsze pochylanie (uwaga na stabilność!)
- **Zwiększ kd_att** → większe tłumienie kątów

---

## ⚙️ Parametry LQR

W pliku `quadrocopter_3d_lqr.py`:

```python
R = np.eye(m) * 15.0  # Kara za sterowanie (wyższe = łagodniejsze)

Q[0, 0] = 3.0    # vx
Q[1, 1] = 3.0    # vy - WAŻNE dla ruchu bocznego!
Q[2, 2] = 5.0    # vz

Q[6, 6] = 2.0    # X
Q[7, 7] = 2.0    # Y - WAŻNE!
Q[8, 8] = 50.0   # Z - najważniejsza!

Q[9, 9] = 25.0   # phi (roll)
Q[10, 10] = 25.0 # theta (pitch)
```

---

## 📈 Statystyki końcowe

Po zakończeniu symulacji zobaczysz:

```
Tracking Performance:
  X error: mean=0.234m, max=1.456m
  Y error: mean=0.312m, max=1.823m
  Z error: mean=0.145m, max=0.876m
  3D position error: mean=0.412m, max=2.134m

Attitude Statistics:
  Roll (phi): max=18.5°
  Pitch (theta): max=15.3°
  Yaw (psi): range=[-12.4°, 23.6°]
```

---

## 🔍 Porównanie: PID vs LQR

| Cecha | PID | LQR |
|-------|-----|-----|
| **Stabilność** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Prostota** | ⭐⭐⭐⭐⭐ | ⭐⭐ |
| **Optymalność** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Odporność** | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Łatwość tuningu** | ⭐⭐⭐⭐ | ⭐⭐ |

---

## 🛠️ Modyfikacja trajektorii

Aby zmienić trajektorię, edytuj `trajectory.py`, funkcję `generate_3d_trajectory()`:

```python
# Przykład: Zwiększenie amplitudy slalomu
if 15.0 < t < 22.0:
    phase = (t - 15.0) / 7.0
    Y_ref[i] = -3.0 + 8.0 * np.sin(phase * 3 * np.pi)  # Było 4.0, teraz 8.0!
```

Możesz dodać własne manewry:
- Spirale
- Ósemki
- Loty pionowe
- Beczki (roll maneuvers)

---

## ⚠️ Znane problemy

1. **LQR może być niestabilny** przy ostrych manewrach
   - Rozwiązanie: Zwiększ R (np. do 20.0)
   
2. **Drone odpływa na boki** przy PID
   - Rozwiązanie: Zwiększ kp_y i kd_y

3. **Oscylacje wysokości**
   - Rozwiązanie: Zwiększ kd_z

---

## 📚 Struktura projektu

```
src_new/
├── main.py                          # Prosty LQR (nie działa)
├── quadrocopter_simple_lqr_stable.py  # Stabilny LQR 2D
├── quadrocopter_working.py          # PID 2D (działa dobrze)
├── quadrocopter_3d_trajectory.py    # PID 3D ⭐ NOWY!
├── quadrocopter_3d_lqr.py          # LQR 3D ⭐ NOWY!
├── trajectory.py                    # Generatory trajektorii ⭐ ZAKTUALIZOWANY!
├── integrator.py                    # RK45
├── linearization.py                 # Linearyzacja dla LQR
├── lqr_controller.py               # Rozwiązywacz LQR
└── constants.py                     # Parametry fizyczne
```

---

## 🎓 Dla pracy magisterskiej

Sugerowane eksperymenty:

1. **Porównanie kontrolerów**
   - Uruchom oba programy dla tej samej trajektorii
   - Porównaj błędy śledzenia
   - Analiza zużycia energii (suma sił silników)

2. **Tuning LQR**
   - Eksperymentuj z różnymi Q i R
   - Zapisz wyniki dla różnych konfiguracji
   - Stwórz wykresy Pareto (stabilność vs dokładność)

3. **Modyfikacja trajektorii**
   - Testuj coraz ostrzejsze manewry
   - Znajdź granice kontrolera
   - Oceń odporność na zakłócenia

4. **Analiza kątów**
   - Monitoruj max roll/pitch podczas manewrów
   - Sprawdź czy są zgodne z ograniczeniami fizycznymi

---

## 🚁 Dalszy rozwój

Możliwe rozszerzenia:

1. **MPC (Model Predictive Control)**
2. **Adaptatywny LQR** z online tuningu
3. **Obsługa wiatru** i zakłóceń
4. **Śledzenie ruchomego celu**
5. **Kolizje z przeszkodami**

---

**Powodzenia z symulacją! 🎉**

Jeśli masz pytania, sprawdź:
- Komentarze w kodzie
- Wydruki diagnostyczne w konsoli
- Wykresy w czasie rzeczywistym
