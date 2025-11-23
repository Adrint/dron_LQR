"""
control_simulation_controller.py

Kontroler stanu symulacji (pauza/stop) bez oddzielnego okna GUI.
Przyciski są teraz zintegrowane bezpośrednio w matplotlib (map_5_visualization.py).
"""


class SimulationController:
    """
    Zarządza stanem symulacji (paused/stopped) bez tworzenia osobnego okna GUI.
    Przyciski są umieszczone bezpośrednio w oknie matplotlib.
    """

    def __init__(self):
        self.paused = False
        self.stopped = False

    def toggle_pause(self):
        """Przełącza stan pauzy symulacji."""
        self.paused = not self.paused

    def stop_simulation(self):
        """Kończy symulację."""
        self.stopped = True
        self.paused = False

    def start(self):
        """
        Metoda pozostawiona dla kompatybilności wstecznej.
        """
        pass

    def is_paused(self):
        """Sprawdza czy symulacja jest zatrzymana."""
        return self.paused

    def is_stopped(self):
        """Sprawdza czy symulacja powinna się zakończyć."""
        return self.stopped

    def close(self):
        """
        Metoda pozostawiona dla kompatybilności wstecznej.
        """
        pass