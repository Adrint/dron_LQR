
import numpy as np
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

        # ========================================================================
        # AUTOMATYCZNE OBLICZENIA MOMENTÓW BEZWŁADNOŚCI
        # ========================================================================

        # Masa korpusu (bez silników)
        mass_body = self.mass - 4 * self.mass_engine

        # Wymiary korpusu - użyj przekazanych wartości
        # (zostały już obliczone w config_parameters.py w trybie auto lub podane przez użytkownika)
        body_length = self.body_length
        body_width = self.body_width
        body_height = self.body_height

        # ========================================================================
        # MOMENTY BEZWŁADNOŚCI KORPUSU
        # Model: prostopadłościan o jednolitej gęstości
        # ========================================================================

        # Wzory na moment bezwładności prostopadłościanu:
        # I_x = (1/12) * m * (h² + d²)  - dla obrotu wokół osi X
        # I_y = (1/12) * m * (l² + h²)  - dla obrotu wokół osi Y
        # I_z = (1/12) * m * (l² + d²)  - dla obrotu wokół osi Z

        I_body_x = (1.0/12.0) * mass_body * (body_width**2 + body_height**2)
        I_body_y = (1.0/12.0) * mass_body * (body_length**2 + body_height**2)
        I_body_z = (1.0/12.0) * mass_body * (body_length**2 + body_width**2)

        # ========================================================================
        # MOMENTY BEZWŁADNOŚCI SILNIKÓW
        # Model: masy punktowe + własny moment bezwładności wirnika
        # ========================================================================

        # Moment własny pojedynczego silnika (wirnik + obudowa)
        # Typowa wartość dla silnika 2kg z wirnikiem Ø40cm
        I_motor_own = 0.0005  # [kg·m²]

        # Momenty od 4 silników jako mas punktowych (twierdzenie Steinera)
        # I_parallel = I_cm + m*d²
        # gdzie d to odległość od osi obrotu

        # Roll (obrót wokół X): silniki odległe o motor_arm_y od osi X
        I_engines_x = 4 * (self.mass_engine * self.motor_arm ** 2 + I_motor_own)

        # Pitch (obrót wokół Y): silniki odległe o motor_arm_x od osi Y
        I_engines_y = 4 * (self.mass_engine * self.motor_arm ** 2 + I_motor_own)

        # Yaw (obrót wokół Z): silniki odległe o sqrt(motor_arm_x² + motor_arm_y²) od osi Z
        I_engines_z = 4 * (self.mass_engine * (self.motor_arm ** 2 + self.motor_arm ** 2) + I_motor_own)

        # ========================================================================
        # CAŁKOWITE MOMENTY BEZWŁADNOŚCI
        # ========================================================================

        self.IX = I_body_x + I_engines_x
        self.IY = I_body_y + I_engines_y
        self.IZ = I_body_z + I_engines_z

        # Zapisz składowe dla analizy
        self.I_body_x = I_body_x
        self.I_body_y = I_body_y
        self.I_body_z = I_body_z
        self.I_engines_x = I_engines_x
        self.I_engines_y = I_engines_y
        self.I_engines_z = I_engines_z

        # ========================================================================
        # INNE OBLICZENIA AUTOMATYCZNE
        # ========================================================================

        self.F_g = self.mass * self.g
        self.g_ned = np.array([0.0, 0.0, self.F_g])
        self.S = self.size
        self.single_engine_thrust = self.mass_engine * self.g
        self.full_thrust = self.mass * self.g


    def print_inertia_breakdown(self):
        print("\n" + "=" * 80)
        print("SZCZEGÓŁOWA ANALIZA MOMENTÓW BEZWŁADNOŚCI")
        print("=" * 80)

        print(f"\n1. KORPUS (masa: {self.mass - 4*self.mass_engine:.2f} kg)")
        print(f"   Wymiary: {self.body_length:.3f}m × {self.body_width:.3f}m × {self.body_height:.3f}m")
        print(f"   I_body_x (roll):  {self.I_body_x:.6f} kg·m²")
        print(f"   I_body_y (pitch): {self.I_body_y:.6f} kg·m²")
        print(f"   I_body_z (yaw):   {self.I_body_z:.6f} kg·m²")

        print(f"\n2. SILNIKI (4 × {self.mass_engine:.2f} kg)")
        print(f"   Ramiona: X={self.motor_arm:.3f}m, Y={self.motor_arm:.3f}m")
        print(f"   I_engines_x (roll):  {self.I_engines_x:.6f} kg·m²")
        print(f"   I_engines_y (pitch): {self.I_engines_y:.6f} kg·m²")
        print(f"   I_engines_z (yaw):   {self.I_engines_z:.6f} kg·m²")

        print(f"\n3. CAŁKOWITE MOMENTY BEZWŁADNOŚCI")
        print(f"   IX (roll):  {self.IX:.6f} kg·m² ({100*self.I_body_x/self.IX:.1f}% korpus + {100*self.I_engines_x/self.IX:.1f}% silniki)")
        print(f"   IY (pitch): {self.IY:.6f} kg·m² ({100*self.I_body_y/self.IY:.1f}% korpus + {100*self.I_engines_y/self.IY:.1f}% silniki)")
        print(f"   IZ (yaw):   {self.IZ:.6f} kg·m² ({100*self.I_body_z/self.IZ:.1f}% korpus + {100*self.I_engines_z/self.IZ:.1f}% silniki)")
        print("=" * 80 + "\n")


# Globalna instancja
config = DroneConfig()