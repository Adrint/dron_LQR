import tkinter as tk
from tkinter import ttk
import threading


class SimulationController:
    """Okno sterujące symulacją z przyciskami Stop/Wznów i Zakończ."""

    def __init__(self):
        self.paused = False
        self.stopped = False
        self.window = None
        self.pause_button = None
        self.gui_thread = None

    def create_window(self):
        """Tworzy okno GUI w osobnym wątku."""
        self.window = tk.Tk()
        self.window.title("Sterowanie Symulacją")
        self.window.geometry("300x200")
        self.window.resizable(False, False)

        # Ustawienie okna zawsze na wierzchu
        self.window.attributes('-topmost', True)

        # Ramka główna
        main_frame = ttk.Frame(self.window, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Etykieta statusu
        self.status_label = ttk.Label(
            main_frame,
            text="Symulacja w toku...",
            font=("Arial", 11, "bold")
        )
        self.status_label.pack(pady=(0, 20))

        # Przycisk Stop/Wznów
        self.pause_button = ttk.Button(
            main_frame,
            text="⏸ Stop",
            command=self.toggle_pause,
            width=20
        )
        self.pause_button.pack(pady=5)

        # Przycisk Zakończ
        self.stop_button = ttk.Button(
            main_frame,
            text="⏹ Zakończ",
            command=self.stop_simulation,
            width=20
        )
        self.stop_button.pack(pady=5)

        # Obsługa zamknięcia okna (X)
        self.window.protocol("WM_DELETE_WINDOW", self.stop_simulation)

        # Uruchomienie pętli GUI
        self.window.mainloop()

    def toggle_pause(self):
        """Przełącza stan pauzy symulacji."""
        self.paused = not self.paused

        if self.paused:
            self.pause_button.config(text="Wznów")
            self.status_label.config(text="Symulacja zatrzymana")
        else:
            self.pause_button.config(text="⏸ Stop")
            self.status_label.config(text="Symulacja w toku...")

    def stop_simulation(self):
        """Kończy symulację i zamyka okno."""
        self.stopped = True
        self.paused = False
        if self.window:
            self.window.quit()
            self.window.destroy()

    def start(self):
        """Uruchamia okno sterujące w osobnym wątku."""
        self.gui_thread = threading.Thread(target=self.create_window, daemon=True)
        self.gui_thread.start()

    def is_paused(self):
        """Sprawdza czy symulacja jest zatrzymana."""
        return self.paused

    def is_stopped(self):
        """Sprawdza czy symulacja powinna się zakończyć."""
        return self.stopped

    def close(self):
        """Zamyka okno sterujące."""
        if self.window:
            try:
                self.window.quit()
                self.window.destroy()
            except:
                pass