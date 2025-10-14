import numpy as np


def generate_reference_profile(Vel, dt, X_max=50):
    """Generate 2D reference trajectory including terrain and flight altitude"""
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    for X in X_ref:
        if X <= 10.0:
            Z = 1.0
        elif X < 20.0:
            Z = 1.0 + (X - 10.0) * (5.0 / 10.0)
        elif X <= 30.0:
            Z = 6.0
        elif X <= 40.0:
            Z = 6.0 - (X - 30.0) * (5.0 / 10.0)
        else:
            Z = 1.0
        Z_terr.append(Z)

    Z_terr = np.array(Z_terr)
    alpha = np.gradient(Z_terr, dx)
    alpha = np.arctan(alpha)

    Z_ref = Z_terr + 2.0  # 2m above terrain

    return X_ref, Z_terr, Z_ref, alpha


def generate_3d_trajectory(Vel, dt, total_time=50.0):
    """
    Generate simplified 3D trajectory with lateral maneuvers
    
    Maneuvers:
    - 0-10s: Straight flight
    - 10-20s: Gentle sine wave (slalom)
    - 20-30s: S-curve maneuver
    - 30-50s: Return to center and straight flight
    
    Returns:
        t_ref: time points
        X_ref: forward position reference
        Y_ref: lateral position reference  
        Z_ref: altitude reference
        Z_terr: terrain elevation
        Vx_ref: forward velocity reference
        Vy_ref: lateral velocity reference
        Vz_ref: vertical velocity reference
    """
    t_ref = np.arange(0, total_time + dt, dt)
    n_points = len(t_ref)
    
    X_ref = np.zeros(n_points)
    Y_ref = np.zeros(n_points)
    Z_ref = np.zeros(n_points)
    Z_terr = np.zeros(n_points)
    
    for i, t in enumerate(t_ref):
        # FORWARD MOTION (X) - constant velocity
        X_ref[i] = Vel * t
        X = X_ref[i]
        
        # LATERAL MOTION (Y) - simplified maneuvers
        if t < 10.0:
            # Segment 1: Straight flight
            Y_ref[i] = 0.0
            
        elif t < 20.0:
            # Segment 2: Gentle slalom (sine wave)
            phase = (t - 10.0) / 10.0  # 0 to 1
            Y_ref[i] = 3.0 * np.sin(phase * 2 * np.pi)  # 2 full waves, amplitude 3m
            
        elif t < 30.0:
            # Segment 3: S-curve (smooth transition)
            phase = (t - 20.0) / 10.0  # 0 to 1
            Y_ref[i] = 3.0 * np.cos(phase * np.pi) * np.exp(-phase)  # Damped oscillation
            
        else:
            # Segment 4: Return to center and straight
            if t < 35.0:
                phase = (t - 30.0) / 5.0
                Y_ref[i] = Y_ref[int((30.0 - dt) / dt)] * (1 - phase)  # Linear return to 0
            else:
                Y_ref[i] = 0.0
        
        # TERRAIN (same as 2D)
        if X <= 10.0:
            Z_terr[i] = 1.0
        elif X < 20.0:
            Z_terr[i] = 1.0 + (X - 10.0) * (5.0 / 10.0)
        elif X <= 30.0:
            Z_terr[i] = 6.0
        elif X <= 40.0:
            Z_terr[i] = 6.0 - (X - 30.0) * (5.0 / 10.0)
        else:
            Z_terr[i] = 1.0
        
        # ALTITUDE - terrain following + slight variations during maneuvers
        base_clearance = 2.5
        
        if 10.0 < t < 20.0:
            # Slight altitude variation during slalom
            phase = (t - 10.0) / 10.0
            clearance = base_clearance + 0.5 * np.sin(phase * 2 * np.pi)
        elif 20.0 < t < 30.0:
            # Slight climb during S-curve
            phase = (t - 20.0) / 10.0
            clearance = base_clearance + 0.5 * (1 - phase)
        else:
            clearance = base_clearance
        
        Z_ref[i] = Z_terr[i] + clearance
    
    # Compute velocity references by numerical differentiation
    Vx_ref = np.gradient(X_ref, dt)
    Vy_ref = np.gradient(Y_ref, dt)
    Vz_ref = np.gradient(Z_ref, dt)
    
    # Smooth velocities to avoid discontinuities
    window = 5
    Vx_ref = np.convolve(Vx_ref, np.ones(window)/window, mode='same')
    Vy_ref = np.convolve(Vy_ref, np.ones(window)/window, mode='same')
    Vz_ref = np.convolve(Vz_ref, np.ones(window)/window, mode='same')
    
    return t_ref, X_ref, Y_ref, Z_ref, Z_terr, Vx_ref, Vy_ref, Vz_ref
