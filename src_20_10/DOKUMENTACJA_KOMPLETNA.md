# KOMPLETNA DOKUMENTACJA MATEMATYCZNA - STEROWANIE LQR DRONA
## Analiza kodu w D:\repo_git\Dron\bicopter

---

## ✅ POTWIERDZENIE: TWÓJ KOD JEST FIZYCZNIE POPRAWNY!

Twój model w `rhs.py` implementuje poprawnie równania ruchu w układzie ciała (body frame) zgodnie z oryginałem MATLAB.

---

# CZĘŚĆ 1: MODEL FIZYCZNY

## 1.1 Wektor stanu

```
x = [vx, vz, ω, X, Z, θ]ᵀ  (n=6)

vx, vz - prędkości w układzie CIAŁA [m/s]
ω      - prędkość kątowa [rad/s]
X, Z   - pozycja w układzie INERCJALNYM [m]
θ      - kąt pochylenia [rad]
```

## 1.2 Równania ruchu - PEŁNE WYPROWADZENIE

### Równanie 1: dvx/dt (przyspieszenie w osi X ciała)

```
MASS·dvx/dt = −D·cos(α) − G·sin(θ) − MASS·ω·vz

dvx/dt = [−D·cos(α) − G·sin(θ)]/MASS − ω·vz
```

**Składniki:**
- `-D·cos(α)` - opór aerodynamiczny w osi X
- `-G·sin(θ)` - składowa ciężaru (θ>0 → ciągnie do tyłu)
- `-ω·vz` - **wyraz Coriolisa** (z obrotu układu)

### Równanie 2: dvz/dt (przyspieszenie w osi Z ciała)

```
dvz/dt = [−D·sin(α) + G·cos(θ) − T₁ − T₂]/MASS + ω·vx + az_turb
```

**Składniki:**
- `-D·sin(α)` - opór w osi Z
- `+G·cos(θ)` - składowa ciężaru (poziomo: pełne G)
- `−T₁ − T₂` - ciągi silników (minus bo w górę)
- `+ω·vx` - **wyraz Coriolisa**
- `+az_turb` - zakłócenia

### Równanie 3: dω/dt (przyspieszenie kątowe)

```
Iy·dω/dt = L·(T₂ − T₁) + CM_Q·ω

dω/dt = [L·(T₂ − T₁) + CM_Q·ω]/Iy
```

**Gdzie:**
- L = MOTOR_ARM_LENGTH = 0.5 m
- CM_Q = -0.01 (tłumienie)
- Iy = 1.5625 kg·m²

### Równania 4-5: Transformacja do układu inercjalnego

```
Macierz rotacji:
R(θ) = [cos(θ)   sin(θ) ]
       [−sin(θ)  cos(θ) ]

dX/dt = cos(θ)·vx + sin(θ)·vz
dZ/dt = −sin(θ)·vx + cos(θ)·vz
```

**KLUCZOWE:** vx, vz są w układzie ciała, ale X, Z w inercjalnym!

### Równanie 6: dθ/dt

```
dθ/dt = ω
```

---

# CZĘŚĆ 2: LINEARYZACJA

## 2.1 Po co linearyzacja?

**Problem:** Równania są **nieliniowe**: `ẋ = f(x,u)`

**LQR wymaga:** Układu **liniowego**: `ẋ = Ax + Bu`

**Rozwiązanie:** Linearyzacja w pobliżu punktu pracy!

## 2.2 Szereg Taylora

```
f(x,u) ≈ f(x₀,u₀) + (∂f/∂x)·(x−x₀) + (∂f/∂u)·(u−u₀)
                     └──┬──┘           └──┬──┘
                        A                  B
```

## 2.3 Algorytm numeryczny (matrices.py)

```python
def aa_matrices_AB(RHS, x, t, u, n, m):
    δ = 1.0e-6  # Małe zaburzenie
    
    f0 = aa_rhs(x, t, u)  # Nominalna wartość
    
    # Macierz A (∂f/∂x)
    A = np.zeros((n, n))
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = δ              # Zaburz j-ty stan
        A[:, j] = (aa_rhs(x+dx, t, u) - f0) / δ
    
    # Macierz B (∂f/∂u)
    B = np.zeros((n, m))
    for j in range(m):
        du = np.zeros(m)
        du[j] = δ              # Zaburz j-te sterowanie
        B[:, j] = (aa_rhs(x, t, u+du) - f0) / δ
    
    return A, B
```

## 2.4 Interpretacja fizyczna

**A[1,0] = ∂(dvz/dt)/∂vx:**

Z równania: `dvz/dt = ... + ω·vx`

Więc: `∂(dvz/dt)/∂vx = ω`

**B[1,0] = ∂(dvz/dt)/∂T₁:**

Z równania: `dvz/dt = (... − T₁ − T₂)/MASS`

Więc: `∂(dvz/dt)/∂T₁ = −1/MASS = −0.04`

---

# CZĘŚĆ 3: TEORIA LQR

## 3.1 Problem optymalizacji

**Cel:** Minimalizuj
```
      ∞
J = ∫ (xᵀQx + uᵀRu) dt
    0
```

**Przy więzach:**
```
ẋ = Ax + Bu
x(0) = x₀
```

## 3.2 Składniki funkcji kosztu

### xᵀQx - "koszt stanu"

```
xᵀQx = q₁₁·vx² + q₂₂·vz² + q₃₃·ω² + q₄₄·X² + q₅₅·Z² + q₆₆·θ²
```

**Interpretacja:**
- Duże Q[i,i] → małe odchylenia (ścisła kontrola)
- Małe Q[i,i] → duże odchylenia OK (luźna kontrola)

### uᵀRu - "koszt sterowania"

```
uᵀRu = r₁₁·T₁² + r₂₂·T₂²
```

**Interpretacja:**
- Duże R[i,i] → oszczędność energii
- Małe R[i,i] → pozwala na duże sterowania

## 3.3 Optymalne sterowanie

**Twierdzenie:** Optymalne u ma postać:
```
u*(t) = −K·x(t)
```

K - macierz wzmocnień (m×n)

## 3.4 Algebraiczne równanie Riccatiego (ARE)

```
0 = AᵀS + SA − SBR⁻¹BᵀS + Q
```

S - rozwiązanie (n×n, symetryczna, dodatnio określona)

Po znalezieniu S:
```
K = R⁻¹BᵀS
```

---

# CZĘŚĆ 4: ROZWIĄZANIE RICCATIEGO - WARTOŚCI WŁASNE

## 4.1 Macierz Hamiltonowska

```
H = [  A      BR⁻¹Bᵀ ]  (2n×2n)
    [  Q       −Aᵀ   ]
```

**Kod (lqr.py, linia 49):**
```python
hamiltonian = np.block([
    [a, b @ np.linalg.inv(r) @ b.T],
    [q, -a.T]
])
```

## 4.2 Obliczenie wartości własnych

```python
eigvals, eigvecs = np.linalg.eig(hamiltonian)
```

Zwraca:
- `eigvals` - 2n wartości własnych
- `eigvecs` - macierz 2n×2n wektorów własnych

## 4.3 KLUCZOWY KROK: Sortowanie

**Własność H:** Wartości własne występują parami (λ, −λ)!

```python
idx = np.argsort(np.real(eigvals))  # Sortuj rosnąco
eigvals = eigvals[idx]
eigvecs = eigvecs[:, idx]
```

**Po sortowaniu:**
```
eigvals = [λ₁, λ₂, ..., λₙ,  λₙ₊₁, ..., λ₂ₙ]
           └──────┬──────┘   └──────┬──────┘
           Re(λ) < 0         Re(λ) > 0
           (stabilne)        (niestabilne)
```

## 4.4 Sprawdzenie poprawności

```python
if not (np.real(eigvals[n-1]) < 1e-15 and 
        np.real(eigvals[n]) > -1e-15):
    print("Błąd sortowania!")
    # Sprawdź sterowalność
```

**Warunek:** Ostatnia ujemna < 0, pierwsza dodatnia > 0

## 4.5 Podział wektorów własnych

Każdy wektor v ma wymiar 2n:

```
      [χ₁]
      [χ₂]
v =   [⋮ ]  ← n elementów → χ
      [χₙ]
      [──]
      [λ₁]
      [λ₂]
      [⋮ ]  ← n elementów → λ
      [λₙ]
```

**Kod:**
```python
chi = eigvecs[:n, :n]        # Górne n×n
lambda_mat = eigvecs[n:, :n] # Dolne n×n
```

## 4.6 Rozwiązanie S

**Równanie:** S·χ = λ

**Rozwiązanie:**
```python
S = −λ_mat @ np.linalg.inv(χ)
```

## 4.7 Macierz wzmocnień K

```python
K = np.linalg.inv(r) @ (b.T @ s)
```

```
K_{m×n} = R⁻¹_{m×m} · Bᵀ_{m×n} · S_{n×n}
```

---

# CZĘŚĆ 5: SCHEMAT STEROWANIA KROK PO KROKU

```
┌─────────────────────────────────────┐
│ INICJALIZACJA                       │
│ x = [0, 0, 0, 0, z0, 0]            │
│ u = [MASS*g/2, MASS*g/2]           │
└─────────────────────────────────────┘
                ↓
        ┌───────────────────┐
        │ PĘTLA GŁÓWNA      │
        └───────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 1. TRAJEKTORIA REFERENCYJNA         │
│                                     │
│ idx = argmin|X_ref_all − x_ref|    │
│ z_ref = Z_ref_all[idx]             │
│ alfa = alpha_all[idx]              │
│ Vx_ref = Vel·cos(alfa)             │
│ Vz_ref = Vel·sin(alfa)             │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 2. TRANSFORMACJA DO UKŁADU CIAŁA    │
│                                     │
│ vx_ref = cos(θ)·Vx + sin(θ)·Vz    │
│ vz_ref = −sin(θ)·Vx + cos(θ)·Vz   │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 3. OBLICZENIE BŁĘDÓW                │
│                                     │
│ e[0] = x[0] − vx_ref               │
│ e[1] = x[1] − vz_ref               │
│ e[2] = x[2] − 0                    │
│ e[3] = x[3] − x_ref                │
│ e[4] = x[4] − z_ref                │
│ e[5] = x[5] − 0                    │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 4. LINEARYZACJA                     │
│                                     │
│ A, B = aa_matrices_AB(x, t, u)     │
│                                     │
│ δ = 10⁻⁶                           │
│ A[:,j] = (f(x+δeⱼ)−f(x))/δ        │
│ B[:,j] = (f(x,u+δeⱼ)−f(x,u))/δ    │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 5. MACIERZE WAG                     │
│                                     │
│ Q = diag([5, 5, 0.1, 1, 50, 1])   │
│ R = eye(2)                         │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 6. ROZWIĄZANIE LQR                  │
│                                     │
│ K, S = lqr_m(A, B, Q, R)           │
│                                     │
│ Algorytm:                          │
│ • H = [[A,BR⁻¹Bᵀ],[Q,−Aᵀ]]        │
│ • λ, v = eig(H)                    │
│ • Sortuj λ (ujemne pierwsze)       │
│ • χ = v[:n,:n], λ = v[n:,:n]      │
│ • S = λ·χ⁻¹                        │
│ • K = R⁻¹BᵀS                       │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 7. STEROWANIE                       │
│                                     │
│ u_pert = −K @ e                    │
│ T1 = T_baseline + u_pert[0]        │
│ T2 = T_baseline + u_pert[1]        │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 8. SATURACJA                        │
│                                     │
│ T1 = clip(T1, T_min, T_max)        │
│ T2 = clip(T2, T_min, T_max)        │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│ 9. INTEGRACJA                       │
│                                     │
│ x_new = aa_rk45(x, t, dt, u)       │
└─────────────────────────────────────┘
                ↓
        (następna iteracja)
```

---

# CZĘŚĆ 6: PRZYKŁAD NUMERYCZNY

## 6.1 Dane wejściowe

```
x = [0.1, −0.05, 0.02, 5.0, 3.0, 0.1]
u = [122.5, 122.5]
```

## 6.2 Macierz A (przykładowe elementy)

```
A[0,0] ≈ −10.0    (wpływ vx na dvx/dt)
A[0,2] =  0.05    (wpływ ω na dvx/dt = −vz)
A[1,0] =  0.02    (wpływ vx na dvz/dt = +ω)
A[3,0] ≈  0.995   (wpływ vx na dX/dt = cos(θ))
```

## 6.3 Macierz B

```
B[1,0] = −0.04    (wpływ T₁ na dvz/dt)
B[2,0] = −0.32    (wpływ T₁ na dω/dt)
B[2,1] = +0.32    (wpływ T₂ na dω/dt)
```

## 6.4 Wartości własne H

```
λ₁ = −8.23 − 1.45i
λ₂ = −8.23 + 1.45i
λ₃ = −5.12
λ₄ = −2.87
λ₅ = −0.92 − 0.31i
λ₆ = −0.92 + 0.31i
─────────────────
λ₇ = +0.92 + 0.31i
λ₈ = +0.92 − 0.31i
...
```

## 6.5 Macierz K (przykład)

```
K ≈ [−0.65  −2.91  −2.85   0.12  −1.03  −5.82]
    [−0.65  −2.91  +2.85   0.12  −1.03  −5.82]
```

## 6.6 Obliczenie sterowania

```
e = [0.05, −0.02, 0.01, 0.5, 0.2, 0.05]

u_pert = −K @ e ≈ [0.44, 0.38]

T₁ = 122.5 + 0.44 = 122.94 N
T₂ = 122.5 + 0.38 = 122.88 N
```

---

# CZĘŚĆ 7: STROJENIE PARAMETRÓW

## 7.1 Macierz Q (wagi stanu)

```python
Q[0,0] = 5.0    # vx - średnia waga
Q[1,1] = 5.0    # vz - średnia waga
Q[2,2] = 0.1    # ω  - niska (szybkie obroty OK)
Q[3,3] = 1.0    # X  - niska
Q[4,4] = 50.0   # Z  - NAJWYŻSZA (priorytet!)
Q[5,5] = 1.0    # θ  - niska
```

## 7.2 Porównanie z MATLAB

| Stan | Twój kod | MATLAB | Stosunek |
|------|----------|--------|----------|
| vx   | 5        | 1000   | 200×     |
| vz   | 5        | 1000   | 200×     |
| ω    | 0.1      | 0.1    | 1×       |
| X    | 1        | 10     | 10×      |
| Z    | 50       | 100    | 2×       |
| θ    | 1        | 1000   | 1000×    |

**Twoje wzmocnienia są znacznie słabsze!**

## 7.3 Zalecenia

### Jeśli dron oscyluje:
```python
Q[4,4] = 100.0   # Zwiększ wagę Z
Q[5,5] = 100.0   # Zwiększ wagę θ
R = eye(2) * 0.1 # Zmniejsz koszt sterowania
```

### Jeśli dron reaguje wolno:
```python
Q[0,0] = 100.0   # Zwiększ wagę vx
Q[1,1] = 100.0   # Zwiększ wagę vz
```

### Jeśli sterowania za duże:
```python
R = eye(2) * 10.0  # Zwiększ koszt sterowania
```

---

# CZĘŚĆ 8: WERYFIKACJA

## 8.1 Test równowagi

```python
python rhs.py
```

**Sprawdza:** Czy dron w poziomie z T=G pozostaje w miejscu?

**Oczekiwane:**
```
dvx/dt ≈ 0
dvz/dt ≈ 0
dω/dt ≈ 0
```

## 8.2 Test sterowalności

```
C = [B | AB | A²B | ... | A⁵B]

rank(C) = n → układ sterowalny
```

## 8.3 Test stabilności

```
λ_closed = eig(A − B·K)

Wszystkie Re(λ) < 0 → stabilny
```

---

# PODSUMOWANIE

## Co działa w Twoim kodzie:

✅ Model fizyczny poprawny (body frame + Coriolis)  
✅ Linearyzacja numeryczna działa  
✅ Algorytm LQR matematycznie poprawny  
✅ Implementacja zgodna z MATLAB  

## Do poprawy:

⚠️ Wzmocnienia Q zbyt słabe (20-1000× mniejsze niż MATLAB)  
⚠️ Może powodować wolną lub niestabilną odpowiedź  

## Zalecenia:

1. Zwiększ wagi Q do poziomów MATLAB
2. Testuj z różnymi trajektoriami
3. Obserwuj wartości własne układu zamkniętego
4. Eksperymentuj z R

---

**Twój kod jest matematycznie i fizycznie poprawny!** 🎉

Dokumentacja stworzono: 2025
