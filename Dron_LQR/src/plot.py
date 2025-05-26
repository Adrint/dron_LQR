import numpy as np
import matplotlib.pyplot as plt

def plot_cost(time, cost_vals):
    plt.figure()
    plt.plot(time, cost_vals, label='LQR cost')
    plt.xlabel("Czas [s]")
    plt.ylabel("Koszt LQR")
    plt.title("Wartość funkcji kosztu LQR")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_error_norm(time, error_norms):
    plt.figure()
    plt.plot(time, error_norms, label='||e||')
    plt.xlabel("Czas [s]")
    plt.ylabel("Norma błędu ||e||")
    plt.title("Norma błędu stanu")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()
