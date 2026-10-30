/*
 * kalman.c
 *
 *  Created on: 14 abr 2026
 *      Author: FatimaJJ
 */

/*
 * kalman.c — Implementación del Filtro de Kalman para STM32F411RE
 *
 * Modelo del sistema:
 *   angulo_k = angulo_{k-1} + (gyro_rate - bias) * dt
 *   bias_k   = bias_{k-1}
 *
 * Matrices:
 *   F = [[1, -dt],   (transición de estado)
 *        [0,   1]]
 *
 *   H = [1, 0]       (solo medimos el ángulo, no el bias)
 *
 * NOTA: La FPU del Cortex-M4 hace estas operaciones en hardware,
 *       no hay penalización por usar float en lugar de int.
 *
 *  Created on: 12 abr 2026
 *      Author: FatimaJJ
 */

#include "kalman.h"

/*
 * kalman_init — inicializa el filtro con el ángulo de arranque
 *
 * Llama esto UNA VEZ con la primera lectura del acelerómetro
 * para que el filtro no parta de 0° y tenga que converger.
 */
void kalman_init(KalmanFilter *kf, float angle_inicial)
{
    kf->angle   = angle_inicial;
    kf->bias    = 0.0f;           /* asumimos bias=0 al inicio */

    /* Covarianza inicial — incertidumbre alta al arrancar */
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    /* Parámetros de ruido */
    kf->Q_angle   = KALMAN_Q_ANGLE;
    kf->Q_bias    = KALMAN_Q_BIAS;
    kf->R_measure = KALMAN_R_MEASURE;
}

/*
 * kalman_update — corre UN paso del filtro
 *
 * Parámetros:
 *   kf          — puntero a la instancia del filtro
 *   accel_angle — ángulo calculado del acelerómetro (grados)
 *   gyro_rate   — velocidad angular del giroscopio Gy (°/s)
 *   dt          — tiempo desde la última llamada (segundos)
 *
 * Retorna: ángulo filtrado (grados)
 *
 * PASOS DEL FILTRO DE KALMAN:
 *   1. PREDICT  — propaga el estado con el giroscopio
 *   2. UPDATE   — corrige con la medición del acelerómetro
 */
float kalman_update(KalmanFilter *kf, float accel_angle, float gyro_rate, float dt)
{
    /* ============================================================
     * PREDICCIÓN
     * Usamos el giroscopio (rápido, sin ruido a corto plazo)
     * para predecir el nuevo ángulo
     * ============================================================ */

    /* Tasa de cambio del ángulo corregida por el bias estimado */
    float rate = gyro_rate - kf->bias;

    /* Predicción del estado:
     * angle = angle + (gyro - bias) * dt  */
    kf->angle += dt * rate;

    /* Predicción de la covarianza del error:
     * P = F*P*F' + Q
     * Expandido (matrices 2x2 con F=[[1,-dt],[0,1]]):  */
    kf->P[0][0] += dt * (dt * kf->P[1][1]
                       - kf->P[0][1]
                       - kf->P[1][0]
                       + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* ============================================================
     * ACTUALIZACIÓN (CORRECCIÓN)
     * Usamos el acelerómetro para corregir la deriva
     * ============================================================ */

    /* Innovación: diferencia entre medición y predicción */
    float innovation = accel_angle - kf->angle;

    /* Covarianza de la innovación: S = H*P*H' + R */
    float S = kf->P[0][0] + kf->R_measure;

    /* Ganancia de Kalman: K = P*H' / S
     * H = [1, 0] → K es un vector de 2 elementos */
    float K[2];
    K[0] = kf->P[0][0] / S;   /* ganancia para el ángulo */
    K[1] = kf->P[1][0] / S;   /* ganancia para el bias   */

    /* Corrección del estado:
     * x = x + K * innovación  */
    kf->angle += K[0] * innovation;
    kf->bias  += K[1] * innovation;

    /* Corrección de la covarianza:
     * P = (I - K*H) * P  */
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    /* Retornar el ángulo filtrado */
    return kf->angle;
}
