"""
Globalna konfiguracja drona - dane są przechowywane tutaj
i dostępne w każdym innym pliku bez przekazywania parametrów
"""


class DroneConfig:
    """Singleton - jedna instancja dla całego programu"""
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def set_params(self, **kwargs):
        """Zapisz wszystkie parametry naraz"""
        for key, value in kwargs.items():
            setattr(self, key, value)

        # Automatyczne obliczenia
        self.IX = 4 * self.mass_engine * self.motor_arm_y ** 2
        self.IY = 4 * self.mass_engine * self.motor_arm_x ** 2
        self.IZ = 4 * self.mass_engine * (self.motor_arm_y ** 2 + self.motor_arm_x ** 2)
        self.F_g = self.mass * self.g
        self.S = self.size
        self.nominal_thrust = self.mass_engine * self.g


# Globalna instancja
config = DroneConfig()