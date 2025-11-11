from config import config
import numpy as np
import matplotlib as plt
class DroneVisualizer:
    """
    Dwie figury:
      • fig12: ax1 (YX – widok z góry) + ax2 (ZX – profil po X), obie animowane jak w animate_combined_view
      • fig34: ax3 (trajektoria 3D) + ax4 (orientacja 3D)

    Interfejs update_plots pozostaje zgodny z control_dron_lqr.py
    """

    # ====== init ======
    def __init__(self, X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all):
        # dane referencyjne (NED)
        self.X_ref_all = X_ref_all
        self.Y_terr_all = Y_terr_all
        self.Z_terr_all = Z_terr_all
        self.Y_ref_all = Y_ref_all
        self.Z_ref_all = Z_ref_all

        # parametry drona do AX4 oraz konturów 2D
        self.body_length = config.body_length
        self.body_width  = config.body_width
        self.body_height = config.body_height
        self.motor_arm   = config.motor_arm

        # bufory wykresów / uchwyty artystów
        self._artists = {}

        # pomocnicze (do budynków/profilu jeśli używasz)
        self.s_build, self.h_bldg_up = compute_building_envelope_along_path()

        plt.ion()
        self._init_figures()      # tworzy fig12 i fig34
        self._init_ax1()
        self._init_ax2()
        self._init_ax3()
        self._init_ax4()
        plt.pause(0.001)

    # ====== figs ======
    def _init_figures(self):
        # figura z animacjami 2D
        self.fig12 = plt.figure(figsize=(16, 7))
        gs12 = self.fig12.add_gridspec(1, 2, wspace=0.25)
        self.ax1 = self.fig12.add_subplot(gs12[0, 0])      # YX (top view)
        self.ax2 = self.fig12.add_subplot(gs12[0, 1])      # ZX (profil)

        # figura z 3D
        self.fig34 = plt.figure(figsize=(16, 9))
        gs34 = self.fig34.add_gridspec(2, 2, height_ratios=[1, 1])
        self.ax3 = self.fig34.add_subplot(gs34[1, 0], projection="3d")
        self.ax4 = self.fig34.add_subplot(gs34[1, 1], projection="3d")
        # Miejsca na nagłówki u góry (opcjonalnie): gs34[0, :]

    # ====== ax1 (YX) ======
    def _init_ax1(self):
        ax = self.ax1
        ax.set_aspect("equal")
        ax.set_facecolor("#f0f0f0")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("Widok z góry (YX) — animacja", fontweight="bold")

        # cała referencja (jeśli chcesz – linia półprzezroczysta)
        ax.plot(self.X_ref_all, self.Y_ref_all, "r-", lw=1.6, alpha=0.35, label="Referencja")

        # artyści do animacji: przebyta, przyszła, punkt drona, kontur drona
        self._artists["ax1_traveled"], = ax.plot([], [], "g-", lw=3, alpha=0.8, label="Przebyta")
        self._artists["ax1_future"],   = ax.plot([], [], "b-", lw=2, alpha=0.55, label="Przód")
        self._artists["ax1_drone"],    = ax.plot([], [], "ro", ms=10, mec="black", mew=1.8, zorder=1000)
        self._artists["ax1_contour"],  = ax.plot([], [], "k-", lw=2)

        ax.legend(fontsize=9, loc="upper right")

    # ====== ax2 (ZX) ======
    def _init_ax2(self):
        ax = self.ax2
        ax.set_facecolor("#f9f9f9")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Z [m NED]")
        ax.set_title("Profil (ZX) — animacja", fontweight="bold")

        # referencja + teren
        ax.plot(self.X_ref_all, self.Z_ref_all, "r-", lw=2, label="Z_ref (NED)")
        ax.plot(self.X_ref_all, self.Z_terr_all, "g-", lw=1.8, alpha=0.7, label="Teren (NED)")

        # zabudowa w profilu (jeśli policzona – tutaj placeholder jako w wcześniejszej wersji)
        if self.s_build is not None and self.h_bldg_up is not None:
            z_bldg_ned = -self.h_bldg_up
            # self.s_build jest w metrach dystansu; nasz profil po X — zostawiamy jako info
            # jeśli chcesz rzeczywiste dopasowanie po s, przemapuj X->s w swoim kodzie.
            # Tu podaję delikatne tło, gdy z_bldg_ned zdefiniowane
            ax.fill_between(self.s_build, 0.0, z_bldg_ned, color="#ffbf80", alpha=0.35, label="Zabudowa (uog.)")

        # artyści do animacji
        self._artists["ax2_path"],   = ax.plot([], [], "b-", lw=2.2, alpha=0.9, label="Z lotu (NED)")
        self._artists["ax2_drone"],  = ax.plot([], [], "ro", ms=8, mec="black", mew=1.2)
        self._artists["ax2_contour"],= ax.plot([], [], "k-", lw=2)

        ax.invert_yaxis()
        ax.legend(fontsize=9, loc="upper right")

        # pasek info jak w Twojej animacji
        self._artists["info_text"] = self.fig12.text(
            0.5, 0.02, "", ha="center", fontsize=10, family="monospace",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.9)
        )

    # ====== ax3 (trajektoria 3D) ======
    def _init_ax3(self):
        ax = self.ax3
        ax.plot(self.X_ref_all, self.Y_ref_all, self.Z_ref_all, "r-", lw=2, label="Referencja")
        ax.plot(self.X_ref_all, self.Y_terr_all, self.Z_terr_all, "g-", lw=1.5, alpha=0.7, label="Teren")
        self._artists["ax3_path"], = ax.plot([], [], [], "b-", lw=2, alpha=0.9, label="Tor lotu")
        self._artists["ax3_pt"]    = ax.scatter([], [], [], color="k", s=40)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m NED]")
        ax.set_title("Trajektoria 3D")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.25)
        ax.invert_zaxis()

    # ====== ax4 (orientacja 3D) ======
    def _init_ax4(self):
        # ustawienia zakresów będą w rysowaniu, żeby stabilnie odświeżać
        self.ax4.set_title("Quadcopter 3D Orientation (Full Rotation)")
        self.ax4.invert_zaxis()

    # ====== helpers (kontury) ======
    def _contour_xy(self, X_pos, Y_pos, roll_phi):
        xsy, ys = aa_mdl(X_pos, Y_pos, -roll_phi, 5)
        return xsy[:5], ys[:5]

    def _contour_xz(self, X_pos, Z_pos, pitch_theta):
        xs, zs = aa_mdl(X_pos, Z_pos, -pitch_theta, 5)
        return xs[:5], zs[:5]

    # ====== update: wywoływane z pętli LQR ======
    def update_plots(self, x, t, i, yp_array, T1, T2, T3, T4, limits_exceeded):
        # Weź bieżące pozycje/orientacje
        X_pos, Y_pos, Z_pos = x[6], x[7], x[8]
        phi, theta          = x[9], x[10]

        # ---------- AX1 (YX) animacja ----------
        self._artists["ax1_traveled"].set_data(yp_array[:i+1, 6], yp_array[:i+1, 7])

        # "look-ahead" jak w animate_combined_view (80 m, heurystyka kroków po X)
        look_ahead_m = 80.0
        step_m = max(1.0, (self.X_ref_all[-1] - self.X_ref_all[0]) / max(1, len(self.X_ref_all)-1))
        n_look = int(look_ahead_m / step_m)
        j1 = max(0, i)
        j2 = min(len(self.X_ref_all)-1, i + n_look)
        self._artists["ax1_future"].set_data(self.X_ref_all[j1:j2], self.Y_ref_all[j1:j2])

        self._artists["ax1_drone"].set_data([X_pos], [Y_pos])

        cx, cy = self._contour_xy(X_pos, Y_pos, phi)
        self._artists["ax1_contour"].set_data(cx, cy)

        # okno podążające (100 m)
        view_range = 100.0
        self.ax1.set_xlim(X_pos - view_range, X_pos + view_range)
        self.ax1.set_ylim(Y_pos - view_range, Y_pos + view_range)

        # ---------- AX2 (ZX) animacja ----------
        self._artists["ax2_path"].set_data(yp_array[:i+1, 6], yp_array[:i+1, 8])
        self._artists["ax2_drone"].set_data([X_pos], [Z_pos])

        cx2, cz2 = self._contour_xz(X_pos, Z_pos, theta)
        self._artists["ax2_contour"].set_data(cx2, cz2)

        view_half_width = 80.0
        self.ax2.set_xlim(max(0, X_pos - view_half_width), min(self.X_ref_all[-1], X_pos + view_half_width))

        # pasek informacji (postęp, dystans po X, wysokość NED)
        progress = 100.0 * i / max(1, len(self.X_ref_all)-1)
        self._artists["info_text"].set_text(
            f"Postęp: {progress:5.1f}% | X: {X_pos:7.1f} m | Z(NED): {Z_pos:7.1f} m"
        )

        # ---------- AX3 (trajektoria 3D) ----------
        self._artists["ax3_path"].set_data_3d(yp_array[:i+1, 6], yp_array[:i+1, 7], yp_array[:i+1, 8])
        self._artists["ax3_pt"]._offsets3d = ([X_pos], [Y_pos], [Z_pos])

        # ---------- AX4 (orientacja 3D) ----------
        self._draw_drone_orientation(x, T1, T2, T3, T4)

        # odśwież obie figury
        self.fig12.canvas.draw_idle()
        self.fig34.canvas.draw_idle()
        plt.pause(0.001)

    # ====== rysowanie orientacji (jak u Ciebie) ======
    def _draw_drone_orientation(self, x, T1, T2, T3, T4):
        ax = self.ax4
        ax.clear()

        # zakresy i opisy
        xbm = np.floor(-self.motor_arm); xbp = np.ceil(self.motor_arm)
        ybm = np.floor(-self.motor_arm); ybp = np.ceil(self.motor_arm)
        zbm = np.floor(-self.motor_arm); zbp = np.ceil(self.motor_arm)
        ax.set_xlim(xbm, xbp); ax.set_ylim(ybm, ybp); ax.set_zlim(zbm, zbp)
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
        ax.set_title('Quadcopter 3D Orientation (Full Rotation)')
        ax.invert_zaxis()

        # rotacja ciała
        phi, theta, psi = x[9], x[10], x[11]
        R = rotation_matrix_zyx(phi, theta, psi)

        # sześcian kadłuba
        rx = self.body_length/2; ry = self.body_width/2; rz = self.body_height/2
        verts = [
            np.array([-rx,-ry,-rz]), np.array([ rx,-ry,-rz]),
            np.array([ rx, ry,-rz]), np.array([-rx, ry,-rz]),
            np.array([-rx,-ry, rz]), np.array([ rx,-ry, rz]),
            np.array([ rx, ry, rz]), np.array([-rx, ry, rz]),
        ]
        vertsW = [R @ v for v in verts]
        faces = [
            [vertsW[0], vertsW[1], vertsW[2], vertsW[3]],
            [vertsW[4], vertsW[5], vertsW[6], vertsW[7]],
            [vertsW[0], vertsW[1], vertsW[5], vertsW[4]],
            [vertsW[2], vertsW[3], vertsW[7], vertsW[6]],
            [vertsW[0], vertsW[3], vertsW[7], vertsW[4]],
            [vertsW[1], vertsW[2], vertsW[6], vertsW[5]],
        ]
        cube = Poly3DCollection(faces, alpha=0.8, facecolor='royalblue', edgecolor='navy', linewidth=2)
        ax.add_collection3d(cube)

        # ramiona X
        ang = np.pi/4
        motors_body = [
            np.array([ self.motor_arm*np.cos(ang),  self.motor_arm*np.sin(ang), 0]),
            np.array([ self.motor_arm*np.cos(ang), -self.motor_arm*np.sin(ang), 0]),
            np.array([-self.motor_arm*np.cos(ang),  self.motor_arm*np.sin(ang), 0]),
            np.array([-self.motor_arm*np.cos(ang), -self.motor_arm*np.sin(ang), 0]),
        ]
        motors_world = [R @ m for m in motors_body]
        ax.plot([motors_world[0][0], motors_world[3][0]],
                [motors_world[0][1], motors_world[3][1]],
                [motors_world[0][2], motors_world[3][2]], 'k-', lw=3)
        ax.plot([motors_world[1][0], motors_world[2][0]],
                [motors_world[1][1], motors_world[2][1]],
                [motors_world[1][2], motors_world[2][2]], 'k-', lw=3)

        colors = ['red','blue','green','orange']
        for j, m in enumerate(motors_world):
            ax.scatter([m[0]], [m[1]], [m[2]], color=colors[j], s=200,
                       edgecolors='black', linewidths=2, depthshade=True)

        # osie ciała
        axis_len = self.motor_arm*1.6
        xb = R @ np.array([axis_len,0,0]); yb = R @ np.array([0,axis_len,0]); zb = R @ np.array([0,0,axis_len])
        ax.quiver(0,0,0, xb[0],xb[1],xb[2], color='red',   linewidth=2)
        ax.quiver(0,0,0, yb[0],yb[1],yb[2], color='green', linewidth=2)
        ax.quiver(0,0,0, zb[0],zb[1],zb[2], color='blue',  linewidth=2)

    def close(self):
        plt.ioff()
        plt.show()
