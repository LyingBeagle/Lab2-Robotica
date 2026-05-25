# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores

Curso: Robótica Laboratorio: Laboratorio 1 – Control de Movimiento en Robot Diferencial Plataforma: Webots

Integrantes:

Patricio Henriquez
Naomi Nuñez
Alonso Bravo
Bastian Trejo
Guillermo Castillo
Fecha: 24 Mayo 2026
---

## 1. Objetivo

Implementar un sistema de navegación reactiva en Webots para un robot móvil diferencial, aplicando filtrado sobre las mediciones de sensores y un filtro de Kalman para estimar la distancia frontal a obstáculos.

---

## 2. Robot y sensores utilizados

Se utilizó el robot **e-puck** de Webots, un robot móvil diferencial con las siguientes características:

| Parámetro | Valor |
|-----------|-------|
| Radio de rueda | 0.0205 m |
| Distancia entre ruedas (axle) | 0.052 m |
| Velocidad máxima angular | 6.28 rad/s |

**Sensores empleados:**

| Sensor | Identificador | Ubicación |
|--------|--------------|-----------|
| Frontal derecho | ps0 | ~10° derecha |
| Diagonal frontal derecho | ps1 | ~45° derecha |
| Lateral derecho | ps2 | ~90° derecha |
| Lateral izquierdo | ps5 | ~90° izquierda |
| Diagonal frontal izquierdo | ps6 | ~45° izquierda |
| Frontal izquierdo | ps7 | ~10° izquierda |
| Encoder izquierdo | left wheel sensor | Rueda izquierda |
| Encoder derecho | right wheel sensor | Rueda derecha |

Los sensores de distancia del e-puck son infrarrojos con un rango efectivo de aproximadamente 0.002 m (objeto pegado) a 0.10 m (sin obstáculo). Entregan valores crudos entre 0 (nada detectado) y 4095 (objeto muy cercano).

---

## 3. Frecuencia de muestreo

| Parámetro | Valor |
|-----------|-------|
| Tiempo de muestreo Ts | 0.064 s (TIME_STEP = 64 ms) |
| Frecuencia de muestreo fs | 15.625 Hz |
| Muestras registradas (escenario simple) | 241 (15.36 s) |
| Muestras registradas (escenario complejo) | 202 (12.86 s) |

---

## 4. Conversión de señales

Los sensores IR del e-puck entregan un valor crudo (0–4095). Se aplicaron dos transformaciones:

**Normalización:**  
Se divide el valor crudo por 4096 para obtener un valor entre 0 y 1:

```
norm = raw / 4096
```

**Conversión a metros:**  
Se utiliza una aproximación hiperbólica que modela la curva no lineal del sensor:

```
distancia_m = 8.0 / (raw + 80)
```

Esto produce: raw = 0 → 0.10 m (lejos), raw = 4095 → 0.002 m (pegado). En la práctica, sin obstáculo el valor típico es raw ≈ 67, lo que da ~0.054 m. Este valor representa el "techo" del rango útil del sensor.

---

## 5. Análisis de señales registradas

### 5.1 Señales crudas de sensores frontales

La distancia frontal cruda se calcula como el promedio de ps0 y ps7 convertido a metros. En los gráficos se observa ruido de alta frecuencia con variaciones de ±0.002 m alrededor de la media, producto de la naturaleza ruidosa de los sensores infrarrojos.

### 5.2 Señal filtrada (filtro paso bajo)

Se aplicó un filtro de media exponencial (EMA) con α = 0.3:

```
filtrado[k] = α · nuevo[k] + (1 - α) · filtrado[k-1]
```

Con α = 0.3, el filtro da más peso al historial (70%) que a la medición actual (30%), suavizando las fluctuaciones rápidas. El resultado es una señal con menor varianza que sigue la tendencia general sin retraso excesivo.

### 5.3 Estimación del avance mediante encoders

Los encoders entregan posición angular en radianes. El avance lineal por paso se calcula como:

```
Δd_izq = r · (θ_izq[k] - θ_izq[k-1])
Δd_der = r · (θ_der[k] - θ_der[k-1])
Δd = (Δd_izq + Δd_der) / 2
```

Donde r = 0.0205 m es el radio de la rueda. El avance típico por paso es de ~0.0008 m cuando avanza y ~0 m durante los giros (las ruedas giran en sentidos opuestos, cancelándose).

---

## 6. Filtro de Kalman

### 6.1 Variable estimada

La variable principal estimada es la **distancia frontal al obstáculo más cercano** (en metros).

### 6.2 Etapa de predicción

Se predice la nueva distancia restando el avance del robot:

```
d⁻[k] = d[k-1] - Δd
P⁻[k] = P[k-1] + Q
```

Donde Q = 0.001 es la varianza del ruido del proceso (incertidumbre del modelo de movimiento).

### 6.3 Etapa de corrección

Se corrige la predicción usando la medición del sensor frontal:

```
K[k] = P⁻[k] / (P⁻[k] + R)
d[k] = d⁻[k] + K[k] · (z[k] - d⁻[k])
P[k] = (1 - K[k]) · P⁻[k]
```

Donde R = 0.05 es la varianza del ruido de medición y z[k] es la distancia medida por los sensores frontales.

### 6.4 Evolución de la ganancia de Kalman

La ganancia K inicia alta (~0.95) porque la incertidumbre inicial P₀ = 1.0 es mucho mayor que R. Esto hace que el filtro confíe casi totalmente en la medición al inicio. Tras ~10 pasos, K converge a ~0.13, lo que indica que el filtro pondera ~13% la medición y ~87% la predicción, produciendo una estimación estable.

---

## 7. Lógica de navegación reactiva

La decisión de movimiento se basa en la distancia frontal estimada por Kalman y los sensores laterales:

| Condición | Acción | Velocidades |
|-----------|--------|-------------|
| `dist_kalman ≥ 0.05 m` | Avanzar | ambas ruedas a 60% de MAX_SPEED |
| `0.04 ≤ dist_kalman < 0.05 m` | Desacelerar | velocidad proporcional al margen |
| `dist_kalman < 0.04 m` | Girar | ruedas opuestas a 40% de MAX_SPEED |

Los umbrales (0.04 y 0.05 m) se ajustaron al rango real del sensor IR, donde ~0.054 m representa "sin obstáculo".

**Dirección del giro:** se comparan los sensores laterales ps2 (derecho) y ps5 (izquierdo). Si el obstáculo está más cerca por la izquierda (ps5 > ps2), se gira a la derecha, y viceversa.

**Mecanismo anti-oscilación:** al iniciar un giro, el robot se compromete a mantener la misma dirección por un mínimo de 10 pasos (0.64 s). Esto evita que el robot oscile entre girar izquierda y derecha en cada timestep cuando los sensores laterales son similares.

---

## 8. Escenarios de prueba

### 8.1 Escenario simple

Arena rectangular (1×1 m) sin obstáculos internos. El robot navega dentro del rectángulo, girando al llegar a las paredes.

**Resultados:**

| Métrica | Valor |
|---------|-------|
| Duración | 15.36 s |
| Muestras | 241 |
| Acciones: desacelerar | 133 (55.2%) |
| Acciones: girar izquierda | 63 (26.1%) |
| Acciones: girar derecha | 45 (18.7%) |
| Acciones: avanzar | 0 (0%) |

**Gráficos:**

![Sensores escenario simple](graficos_sensores_simple.png)

![Acciones escenario simple](acciones_robot_simple.png)

### 8.2 Escenario complejo

Arena rectangular con dos cajas de cartón (0.3×0.3×0.3 m) colocadas como obstáculos adicionales, creando pasillos estrechos.

**Resultados:**

| Métrica | Valor |
|---------|-------|
| Duración | 12.86 s |
| Muestras | 202 |
| Acciones: desacelerar | 112 (55.4%) |
| Acciones: girar izquierda | 63 (31.2%) |
| Acciones: girar derecha | 27 (13.4%) |
| Acciones: avanzar | 0 (0%) |

**Gráficos:**

![Sensores escenario complejo](graficos_sensores_complejo.png)

![Acciones escenario complejo](acciones_robot_complejo.png)

### 8.3 Análisis comparativo

En ambos escenarios la distancia Kalman se mantiene entre 0.049 y 0.054 m. Dado que el rango máximo del sensor IR es ~0.054 m, el robot opera siempre cerca del límite del sensor. La señal Kalman es consistentemente más suave que la cruda, con menor varianza (~0.001 m vs ~0.005 m).

En el escenario complejo se observa mayor proporción de giros a la izquierda (31.2% vs 26.1%), indicando que los obstáculos están más concentrados a la derecha del recorrido.

La ganancia de Kalman converge rápidamente en ambos escenarios (K final ≈ 0.132), demostrando estabilidad del filtro independiente del entorno.

---

## 9. Comparación: señal cruda vs filtrada vs Kalman

| Aspecto | Señal cruda | Filtro paso bajo | Kalman |
|---------|-------------|-----------------|--------|
| Ruido | Alto (~±0.003 m) | Medio (~±0.001 m) | Bajo (~±0.0005 m) |
| Retardo | Ninguno | Bajo (por α=0.3) | Mínimo |
| Usa encoders | No | No | Sí (predicción) |
| Reacciona a cambios bruscos | Inmediato | Con retardo | Ponderado por K |

El filtro de Kalman produce la estimación más estable porque combina dos fuentes de información: la predicción por movimiento (encoders) y la medición directa (sensores IR). Esto lo hace más robusto ante picos de ruido puntuales comparado con el filtro paso bajo que solo suaviza la señal.

---

## 10. Conclusiones

El filtro de Kalman cumple su objetivo de producir una estimación más estable de la distancia frontal. La ganancia converge rápidamente, indicando que el balance entre predicción y medición se estabiliza en pocas iteraciones.

El mecanismo de giro comprometido (mínimo 10 pasos) elimina las oscilaciones que se producían cuando el robot re-evaluaba la dirección de giro en cada timestep.

El principal desafío fue ajustar los umbrales de decisión al rango real del sensor IR del e-puck (~0.002–0.054 m), que es significativamente menor a lo que se podría asumir si se trabajara con valores en unidades arbitrarias.

La navegación reactiva resultante permite al robot recorrer ambos escenarios sin colisiones, alternando entre desaceleración y giros según la proximidad de obstáculos.

---

## 11. Instrucciones para ejecutar la simulación

1. Abrir Webots y cargar el mundo deseado:
   - `worldlaboratorio2_Simple_.wbt` para el escenario simple
   - `worldlaboratorio2_Complejo_.wbt` para el escenario complejo

2. Asegurarse de que el controlador del e-puck esté configurado como `my_controller` (o el nombre asignado en el mundo) y apunte al archivo `controladorkalman.py`.

3. Ejecutar la simulación. El archivo `datos_sensores.csv` se generará en el directorio del controlador.

4. Para graficar los resultados:
   ```bash
   python graficar_resultados_simple.py
   python graficar_resultados_complejo.py
   ```

5. Los gráficos se guardan como `graficos_sensores_*.png` y `acciones_robot_*.png`.

---

## 12. Estructura del repositorio

```
├── controladorkalman.py              # Controlador principal del robot
├── graficar_resultados.py            # Script de graficación general
├── graficar_resultados_simple.py     # Graficación escenario simple
├── graficar_resultados_complejo.py   # Graficación escenario complejo
├── datos_sensores.csv                # Datos registrados (general)
├── datos_sensores_simple.csv         # Datos escenario simple
├── datos_sensores_complejo.csv       # Datos escenario complejo
├── graficos_sensores_simple.png      # Gráficos escenario simple
├── graficos_sensores_complejo.png    # Gráficos escenario complejo
├── acciones_robot_simple.png         # Acciones escenario simple
├── acciones_robot_complejo.png       # Acciones escenario complejo
├── worldlaboratorio2_Simple_.wbt     # Mundo Webots simple
├── worldlaboratorio2_Complejo_.wbt   # Mundo Webots complejo
└── README.md                         # Este archivo
```
