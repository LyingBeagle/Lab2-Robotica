"""
Grafica los datos registrados durante la simulación.
Ejecutar desde terminal: python graficar_resultados_complejo.py
"""

import csv
import matplotlib.pyplot as plt
import os

# Cargar datos
ruta = os.path.join(os.path.dirname(__file__), 'datos_sensores_complejo.csv')

tiempos, dist_cruda, dist_kalman, delta_enc, ganancia_k = [], [], [], [], []
acciones = []

with open(ruta, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        tiempos.append(float(row['tiempo_s']))
        dist_cruda.append(float(row['front_metros_crudo']))
        dist_kalman.append(float(row['front_metros_kalman']))
        delta_enc.append(float(row['delta_d_encoder']))
        ganancia_k.append(float(row['ganancia_K']))
        acciones.append(row['accion'])

# ── Gráfico 1: Comparación señales ──
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axes[0].plot(tiempos, dist_cruda, color='tomato', alpha=0.7, label='Cruda')
axes[0].plot(tiempos, dist_kalman, color='green', linewidth=1.5, label='Kalman')
axes[0].axhline(y=0.15, color='black', linestyle='--', label='Umbral')
axes[0].set_ylabel('Distancia frontal (m)')
axes[0].set_title('Comparación: señal cruda vs Kalman')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].plot(tiempos, delta_enc, color='purple', label='Δd encoder (m)')
axes[1].set_ylabel('Avance estimado (m)')
axes[1].set_title('Avance estimado por encoders')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

axes[2].plot(tiempos, ganancia_k, color='darkorange', label='Ganancia K')
axes[2].set_ylabel('Ganancia Kalman')
axes[2].set_xlabel('Tiempo (s)')
axes[2].set_title('Evolución de la ganancia de Kalman')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), 'graficos_sensores_complejo.png'), dpi=150)
plt.close()

# ── Gráfico 2: Acciones del robot ──
accion_num = {'avanzar': 2, 'desacelerar': 1, 'girar_derecha': 0, 'girar_izquierda': -1}
acciones_n = [accion_num.get(a, 0) for a in acciones]

plt.figure(figsize=(12, 3))
plt.step(tiempos, acciones_n, color='teal', where='post')
plt.yticks([-1, 0, 1, 2], ['Girar izq.', 'Girar der.', 'Desacelerar', 'Avanzar'])
plt.xlabel('Tiempo (s)')
plt.title('Acciones del robot a lo largo del tiempo')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), 'acciones_robot_complejo.png'), dpi=150)
plt.close()

print("Graficos guardados (complejo).")