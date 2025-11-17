import matplotlib.pyplot as plt
from map_4_animate_route import DroneLiveAnimator
from map_5_visualization import DroneVisualizer
from simulation_controller import SimulationController


class UnifiedVisualizer:
    """
    Łączy:
    - animację trasy (DroneLiveAnimator z map_4_animate_route)
    - wizualizację orientacji (DroneVisualizer z map_5_visualization)
    - okno sterujące (SimulationController z simulation_controller)

    API pozostaje kompatybilne z control_dron_lqr.dron_lqr:
    - initialize(path_result=None)
    - is_paused()
    - is_stopped()
    - update(x, t, i, u)
    - close()
    """

    def __init__(self):
        # Stan sterowania (proxy do SimulationController)
        self.controller = SimulationController()

        # Wizualizacje
        self.animator = DroneLiveAnimator()
        self.orientation = DroneVisualizer()

        # Czy animator trasy został zainicjalizowany
        self.route_enabled = False

    def initialize(self, path_result=None):
        """
        Uruchamia wszystkie wizualizacje + okno sterujące.

        Parameters
        ----------
        path_result : dict lub None
            Wynik planera trasy. Jeśli None – animacja trasy jest wyłączona
            (tylko orientacja 3D).
        """

        # 1) Okno sterujące w osobnym wątku (Tkinter)
        self.controller.start()

        # 2) Animacja trasy – tylko jeśli mamy path_result
        if path_result is not None:
            try:
                self.animator.initialize(path_result=path_result)
                self.route_enabled = True
            except Exception as e:
                print(f"[UnifiedVisualizer] Błąd inicjalizacji animacji trasy: {e}")
                self.route_enabled = False
        else:
            print("[UnifiedVisualizer] Brak path_result – animacja trasy wyłączona")
            self.route_enabled = False

        # 3) DroneVisualizer tworzy własną figurę już w __init__,
        #    więc tutaj nie trzeba nic więcej
        plt.pause(0.1)

    # ====== Metody sterujące – delegowane do SimulationController ======

    def is_paused(self):
        return self.controller.is_paused()

    def is_stopped(self):
        return self.controller.is_stopped()

    # ====== Główna aktualizacja z pętli symulacji ======

    def update(self, x, t, i, u):
        """
        Aktualizuje wszystkie wizualizacje.

        Parameters
        ----------
        x : ndarray
            Wektor stanu (jak w dron_lqr).
        t : float
            Czas.
        i : int
            Numer kroku.
        u : ndarray (4,)
            Ciągi silników [T1, T2, T3, T4].
        """
        # Jeśli użytkownik zatrzymał symulację – nie aktualizujemy nic
        if self.is_stopped():
            return

        # Jeśli pauza – również nie odświeżamy wykresów
        if self.is_paused():
            plt.pause(0.05)
            return

        # 1) Trasa (jeśli dostępna)
        if self.route_enabled:
            try:
                self.animator.update(x_state=x, t=t, step=i)
            except Exception as e:
                print(f"[UnifiedVisualizer] Błąd aktualizacji animacji trasy: {e}")
                # Jednorazowe wyłączenie animacji jeśli coś poszło nie tak
                self.route_enabled = False

        # 2) Orientacja + panel thrust/angles/rates
        try:
            T1, T2, T3, T4 = u
            self.orientation.update_plots(x, t, i, T1, T2, T3, T4)
        except Exception as e:
            print(f"[UnifiedVisualizer] Błąd aktualizacji wizualizacji orientacji: {e}")

        plt.pause(0.001)

    def close(self):
        """
        Zamyka wszystkie okna wizualizacji i panel sterowania.
        """
        try:
            if self.route_enabled:
                self.animator.close()
        except Exception as e:
            print(f"[UnifiedVisualizer] Błąd przy zamykaniu animatora: {e}")

        try:
            self.orientation.close()
        except Exception as e:
            print(f"[UnifiedVisualizer] Błąd przy zamykaniu orientacji: {e}")

        try:
            self.controller.close()
        except Exception as e:
            print(f"[UnifiedVisualizer] Błąd przy zamykaniu kontrolera: {e}")

        plt.ioff()
        plt.show()
