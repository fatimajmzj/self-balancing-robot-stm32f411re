/*
 * kalman.h
 *
 *  Created on: 14 abr 2026
 *      Author: FatimaJJ
 */
/*
 * kalman.h — Filtro de Kalman 1D para ángulo de pitch
 *
 * Estado estimado: [angulo, bias_gyro]
 *   - angulo:    inclinación real en grados
 *   - bias_gyro: deriva del giroscopio (se estima y se corrige sola)
 *
 * Entrada:  velocidad angular Gy (°/s) + ángulo del acelerómetro (°)
 * Salida:   ángulo filtrado suave y preciso
 *
 * Optimizado para STM32F411RE con FPU hardware
 *  Created on: 12 abr 2026
 *      Author: FatimaJJ
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>

/* ---------------------------------------------------------------
 * Parámetros de ruido — AQUÍ se afina el filtro
 *
 * Q_angle:  ruido del proceso en el ángulo
 *           BAJO  → confía más en el giroscopio (más suave, más lento)
 *           ALTO  → confía más en el acelerómetro (más rápido, más ruidoso)
 *
 * Q_bias:   ruido del proceso en el bias del giroscopio
 *           BAJO  → el bias cambia poco (giroscopio estable)
 *           ALTO  → el bias cambia rápido (giroscopio muy inestable)
 *
 * R_measure: ruido de medición del acelerómetro
 *           BAJO  → acelerómetro muy confiable
 *           ALTO  → acelerómetro muy ruidoso
 *
 * Valores iniciales para MPU9250 ±4g / ±500°/s:
 * --------------------------------------------------------------- */
#define KALMAN_Q_ANGLE    0.001f   /* ruido proceso — ángulo        */
#define KALMAN_Q_BIAS     0.003f   /* ruido proceso — bias gyro     */
#define KALMAN_R_MEASURE  0.03f    /* ruido medición — acelerómetro */

/* Estructura del filtro — una instancia por eje a filtrar */
typedef struct {
    /* Estado estimado */
    float angle;      /* ángulo filtrado (grados)           */
    float bias;       /* bias del giroscopio estimado (°/s) */

    /* Matriz de covarianza del error P (2x2) */
    float P[2][2];

    /* Parámetros (se pueden cambiar en runtime) */
    float Q_angle;
    float Q_bias;
    float R_measure;
} KalmanFilter;

/* Funciones públicas */
void  kalman_init(KalmanFilter *kf, float angle_inicial);
float kalman_update(KalmanFilter *kf, float accel_angle, float gyro_rate, float dt);

#endif /* INC_KALMAN_H_ */
