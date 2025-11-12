import numpy as np
from scipy.linalg import solve_continuous_are
from mixers import mixer_motors


class CascadeLQRController:
    """Kaskadowy kontroler LQR dla quadrocoptera"""

    def __init__(self, drone_params):
        self.mass = drone_params['mass']
        self.g = drone_params['g']
        self.ix = drone_params['ix']
        self.iy = drone_params['iy']
        self.iz = drone_params['iz']
        self.arm = drone_params['arm']

        # POPRAWIONE LQR GAINS - kluczowe dla stabilności!
        # Q_pos: Wagi na pozycję [X, Y, Z]
        # Większe wartości = większa waga na dokładność tej osi
        self.Q_pos = np.diag([5.0, 5.0, 50.0])  # Z jest 10x ważniejsze!

        # R_pos: Kara za sterowanie prędkością
        # Mniejsze R = bardziej agresywne sterowanie
        self.R_pos = np.diag([0.5, 0.5, 0.5])

        # Q_att: Wagi na orientację [phi, theta, psi, p, q, r]
        # Zwiększone wagi na prędkości kątowe dla lepszego tłumienia
        self.Q_att = np.diag([10.0, 10.0, 5.0, 20.0, 20.0, 10.0])

        # R_att: Kara za momenty sterujące
        # Bardzo małe R = śmiałe sterowanie
        self.R_att = np.diag([0.01, 0.01, 0.001])

        self.K_pos = None
        self.K_att = None
        self.update_counter = 0

    def design_lqr_simple(self, A, B, Q, R):
        """Simplified LQR design"""
        try:
            P = solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ B.T @ P
            return K
        except:
            print("⚠️  LQR design failed, using zero gain")
            return np.zeros((B.shape[1], A.shape[0]))

    def update_att_gain(self, state_att):
        """Update attitude control gain"""
        phi, theta, psi, p, q, r = state_att

        A_att = np.zeros((6, 6))
        A_att[0, 3] = 1.0  # dphi/dt = p
        A_att[1, 4] = 1.0  # dtheta/dt = q
        A_att[2, 5] = 1.0  # dpsi/dt = r

        # Tłumienie aerodynamiczne
        A_att[3, 3] = -0.01 / self.ix
        A_att[4, 4] = -0.01 / self.iy
        A_att[5, 5] = -0.01 / self.iz

        B_att = np.zeros((6, 3))
        B_att[3, 0] = 1.0 / self.ix  # Moment roll → p
        B_att[4, 1] = 1.0 / self.iy  # Moment pitch → q
        B_att[5, 2] = 1.0 / self.iz  # Moment yaw → r

        self.K_att = self.design_lqr_simple(A_att, B_att, self.Q_att, self.R_att)

    def update_pos_gain(self):
        """Update position control gain"""
        # Prosty model: dX/dt = vx, dY/dt = vy, dZ/dt = vz
        A_pos = np.zeros((3, 3))
        B_pos = np.eye(3)  # Prędkość bezpośrednio wpływa na pozycję

        self.K_pos = self.design_lqr_simple(A_pos, B_pos, self.Q_pos, self.R_pos)

    def control_position(self, state_pos, ref_pos):
        """Outer loop: position to velocity command"""
        if self.K_pos is None:
            self.update_pos_gain()

        error_pos = ref_pos - state_pos
        vel_cmd = -self.K_pos @ error_pos

        # Limitowanie prędkości
        vel_cmd = np.clip(vel_cmd, -5.0, 5.0)

        return vel_cmd

    def control_attitude(self, state_att, att_cmd, vel_cmd_actual, vel_actual):
        """Inner loop: attitude to motor thrusts"""
        if self.K_att is None:
            self.update_att_gain(state_att)

        phi, theta, psi, p, q, r = state_att
        phi_cmd, theta_cmd, psi_cmd = att_cmd

        # Błąd orientacji
        error_att = np.array([
            phi_cmd - phi,
            theta_cmd - theta,
            np.arctan2(np.sin(psi_cmd - psi), np.cos(psi_cmd - psi)),  # Wrap yaw
            0.0 - p,  # Chcemy zerowych prędkości kątowych
            0.0 - q,
            0.0 - r
        ])

        # LQR: momenty z błędu orientacji
        moment_cmd = -self.K_att @ error_att
        moment_cmd = np.clip(moment_cmd, -10.0, 10.0)

        # POPRAWIONE: Ciąg z błędu prędkości pionowej
        vel_error = vel_cmd_actual - vel_actual

        # KLUCZOWE: Znak PLUS (było minus!) i współczynnik 2.0 (było 3.0)
        # Jeśli vz < vz_cmd (leci za wolno w górę) → vel_error[2] > 0 → zwiększ ciąg
        # Jeśli vz > vz_cmd (leci za szybko) → vel_error[2] < 0 → zmniejsz ciąg
        thrust_cmd = self.mass * (self.g + 2.0 * vel_error[2])

        # Ograniczenia ciągu (50% - 200% wagi)
        thrust_cmd = np.clip(thrust_cmd,
                             self.mass * self.g * 0.5,
                             self.mass * self.g * 2.0)

        # Konwersja: momenty + ciąg → 4 silniki
        thrusts = mixer_motors(moment_cmd, thrust_cmd, self.arm)

        return thrusts

    def update(self, state, ref_pos, dt):
        """Main control step - FULL CASCADE"""
        vel = state[0:3]
        rates = state[3:6]
        pos = state[6:9]
        att = state[9:12]

        # === PĘTLA ZEWNĘTRZNA: Pozycja → Prędkość ===
        vel_cmd = self.control_position(pos, ref_pos)

        # === KINEMATYKA: Prędkość → Kąty ===
        # Aby lecieć w kierunku X: przechyl się do przodu (theta > 0)
        # Aby lecieć w kierunku Y: przechyl się w bok (phi < 0)
        att_cmd = np.array([
            -np.clip(0.5 * vel_cmd[1], -0.5, 0.5),  # phi: ruch boczny
            np.clip(0.5 * vel_cmd[0], -0.5, 0.5),  # theta: ruch do przodu
            att[2]  # psi: utrzymuj obecny kąt yaw
        ])

        # Połączony stan orientacji
        state_att = np.concatenate([att, rates])

        # Przeliczaj gainy co 10 kroków (oszczędność obliczeń)
        self.update_counter += 1
        if self.update_counter > 10:
            self.update_att_gain(state_att)
            self.update_pos_gain()
            self.update_counter = 0

        # === PĘTLA WEWNĘTRZNA: Orientacja → Ciągi ===
        thrusts = self.control_attitude(state_att, att_cmd, vel_cmd, vel)

        return thrusts