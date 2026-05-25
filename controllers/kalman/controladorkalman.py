"""
Laboratorio 2 - Navegación Reactiva con Kalman
Basado en el controlador oficial del e-puck (Corregido)
"""

from controller import Robot
import math
import csv
import os

# ─────────────────────────────────────────
# PARÁMETROS
# ─────────────────────────────────────────
TIME_STEP     = 64
MAX_SPEED     = 6.28
WHEEL_RADIUS  = 0.0205
AXLE_LENGTH   = 0.052

ALPHA         = 0.3      # filtro paso bajo
Q             = 0.001    # ruido del proceso Kalman
R_KALMAN      = 0.05     # ruido de medición Kalman

# ─────────────────────────────────────────
# CONVERSIÓN
# ─────────────────────────────────────────
def normalizar(raw_value):
    return raw_value / 4096.0

def normalizado_a_metros(norm):
    """
    Convierte el valor normalizado a metros.
    Rango real del sensor IR e-puck: ~0.002 m (pegado) a ~0.10 m (nada).
    raw ~67 sin obstáculo → ~0.054 m
    """
    raw_value = norm * 4096.0
    return 8.0 / (raw_value + 80.0)

# ─────────────────────────────────────────
# FILTRO PASO BAJO
# ─────────────────────────────────────────
def filtro_pb(nuevo, previo, alpha=ALPHA):
    return alpha * nuevo + (1 - alpha) * previo

# ─────────────────────────────────────────
# KALMAN ESCALAR
# ─────────────────────────────────────────
class KalmanFilter1D:
    def __init__(self, x0, P0=1.0, Q=0.001, R=0.05):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.R = R

    def predecir(self, delta):
        self.x = self.x - delta
        self.P = self.P + self.Q
        return self.x

    def corregir(self, z):
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * self.P
        return self.x, K

# ─────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────
def main():
    robot = Robot()

    # ── Motores ──
    motor_izq = robot.getDevice('left wheel motor')
    motor_der = robot.getDevice('right wheel motor')
    motor_izq.setPosition(float('inf'))
    motor_der.setPosition(float('inf'))
    motor_izq.setVelocity(0.0)
    motor_der.setVelocity(0.0)

    # ── Encoders ──
    enc_izq = robot.getDevice('left wheel sensor')
    enc_der = robot.getDevice('right wheel sensor')
    enc_izq.enable(TIME_STEP)
    enc_der.enable(TIME_STEP)

    # ── Sensores de distancia (ps0–ps7) ──
    ps = []
    for i in range(8):
        sensor = robot.getDevice(f'ps{i}')
        sensor.enable(TIME_STEP)
        ps.append(sensor)

    # ── Primer paso para inicializar ──
    robot.step(TIME_STEP)
    enc_izq_prev = enc_izq.getValue()
    enc_der_prev = enc_der.getValue()

    front_norm_init = (normalizar(ps[0].getValue()) + normalizar(ps[7].getValue())) / 2.0
    front_m_init    = normalizado_a_metros(front_norm_init)

    filtrado_prev = front_norm_init
    kf = KalmanFilter1D(x0=front_m_init, P0=1.0, Q=Q, R=R_KALMAN)

    # ── CSV (ruta relativa — se guarda junto al controlador) ──
    log_path = 'datos_sensores.csv'
    log_file = open(log_path, 'w', newline='')
    writer   = csv.writer(log_file)
    writer.writerow([
        'paso', 'tiempo_s',
        'ps0_raw', 'ps7_raw',
        'ps0_norm', 'ps7_norm',
        'front_norm_crudo',
        'front_norm_filtrado',
        'front_metros_crudo',
        'front_metros_kalman',
        'ganancia_K',
        'delta_d_encoder',
        'ps1_norm', 'ps6_norm',
        'ps2_norm', 'ps5_norm',
        'accion'
    ])

    # ─────────────────────────────────────
    # UMBRALES — ajustados al rango real del sensor IR
    # Rango: 0.002 m (pegado) – 0.054 m (sin obstáculo)
    # ─────────────────────────────────────
    UMBRAL_GIRO      = 0.04   # metros — obstáculo cerca, girar
    UMBRAL_DECEL     = 0.05   # metros — obstáculo medio, desacelerar
    PASOS_GIRO_MIN   = 10     # mantener giro mínimo N pasos

    paso = 0
    giro_contador = 0
    giro_dir = None

    while robot.step(TIME_STEP) != -1:
        t = paso * (TIME_STEP / 1000.0)

        # ── 1. Leer sensores ──
        raw  = [ps[i].getValue() for i in range(8)]
        norm = [normalizar(v) for v in raw]

        front_norm = (norm[0] + norm[7]) / 2.0
        front_m    = normalizado_a_metros(front_norm)

        lateral_der_norm = norm[2]
        lateral_izq_norm = norm[5]

        # ── 2. Filtro paso bajo ──
        front_filtrado = filtro_pb(front_norm, filtrado_prev)
        filtrado_prev  = front_filtrado

        # ── 3. Encoders → avance lineal ──
        enc_izq_act = enc_izq.getValue()
        enc_der_act = enc_der.getValue()

        delta_izq = WHEEL_RADIUS * (enc_izq_act - enc_izq_prev)
        delta_der = WHEEL_RADIUS * (enc_der_act - enc_der_prev)
        delta_d   = (delta_izq + delta_der) / 2.0

        enc_izq_prev = enc_izq_act
        enc_der_prev = enc_der_act

        # ── 4. Kalman ──
        kf.predecir(delta_d)
        dist_kalman, K = kf.corregir(front_m)

        # ── 5. Navegación reactiva ──
        if giro_contador > 0:
            giro_contador -= 1
            if giro_dir == 'derecha':
                vel_izq =  MAX_SPEED * 0.4
                vel_der = -MAX_SPEED * 0.4
                accion  = 'girar_derecha'
            else:
                vel_izq = -MAX_SPEED * 0.4
                vel_der =  MAX_SPEED * 0.4
                accion  = 'girar_izquierda'

        elif dist_kalman < UMBRAL_GIRO:
            giro_contador = PASOS_GIRO_MIN
            if lateral_izq_norm > lateral_der_norm:
                giro_dir = 'derecha'
                vel_izq =  MAX_SPEED * 0.4
                vel_der = -MAX_SPEED * 0.4
                accion  = 'girar_derecha'
            else:
                giro_dir = 'izquierda'
                vel_izq = -MAX_SPEED * 0.4
                vel_der =  MAX_SPEED * 0.4
                accion  = 'girar_izquierda'

        elif dist_kalman < UMBRAL_DECEL:
            factor = (dist_kalman - UMBRAL_GIRO) / (UMBRAL_DECEL - UMBRAL_GIRO)
            factor = max(0.2, min(factor, 1.0))
            vel_izq = MAX_SPEED * 0.5 * factor
            vel_der = MAX_SPEED * 0.5 * factor
            accion  = 'desacelerar'

        else:
            vel_izq = MAX_SPEED * 0.6
            vel_der = MAX_SPEED * 0.6
            accion  = 'avanzar'

        motor_izq.setVelocity(vel_izq)
        motor_der.setVelocity(vel_der)

        # ── 6. Log ──
        writer.writerow([
            paso, round(t, 3),
            round(raw[0], 1),  round(raw[7], 1),
            round(norm[0], 4), round(norm[7], 4),
            round(front_norm, 4),
            round(front_filtrado, 4),
            round(front_m, 5),
            round(dist_kalman, 5),
            round(K, 5),
            round(delta_d, 6),
            round(norm[1], 4), round(norm[6], 4),
            round(norm[2], 4), round(norm[5], 4),
            accion
        ])

        paso += 1

    log_file.close()

if __name__ == '__main__':
    main()
