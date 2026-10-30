/*
 * mpu9250.h — Driver MPU9250 con lectura DMA no-bloqueante
 *  STM32F411RE — Balancín
 */
#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include <stdio.h>
#include <stdint.h>

/* Dirección I2C — AD0 a GND = 0x68 */
#define MPU9250_ADDR        (0x68 << 1)

/* Registros principales */
#define SMPLRT_DIV_REG      0x19
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_CONFIG2_REG   0x1D
#define PWR_MGMT_1_REG      0x6B
#define WHO_AM_I_REG        0x75

/* Primer registro del bloque de 14 bytes:
 * ACCEL_XOUT_H(0x3B)..GYRO_ZOUT_L(0x48)
 * El MPU9250 autoincrementa la dirección en modo burst */
#define ACCEL_XOUT_H        0x3B

/* Número de bytes a leer de una sola vez (accel XYZ + temp + gyro XYZ = 14) */
#define MPU_BURST_LEN       14

/* Filtro complementario */
#define ALPHA               0.98f
#define DT                  0.01f   /* 10 ms — periodo del TIM5 */

/* Rangos y sensibilidades */
#define GYRO_RANGE_500      0x08
#define ACCEL_RANGE_4G      0x08
#define ACCEL_SENSITIVITY   8192.0f
#define GYRO_SENSITIVITY    65.5f

/* -----------------------------------------------------------------------
 * Flag de transferencia DMA completada.
 * Lo pone a 1 la callback HAL_I2C_MemRxCpltCallback().
 * Lo lee y limpia el while() de main antes de procesar.
 * volatile porque es compartido entre contextos (ISR ↔ main).
 * ----------------------------------------------------------------------- */
extern volatile uint8_t mpu_dma_ready;

/* Buffer interno: 14 bytes = accel(6) + temp(2) + gyro(6)
 * Declarado en mpu9250.c. Acceso de solo lectura para el main. */
extern uint8_t mpu_rx_buf[MPU_BURST_LEN];

/* Estructuras de datos procesados */
typedef struct {
    float x_val;
    float y_val;
    float z_val;

    float pitch_angle;   /* ángulo crudo acelerómetro */
    float roll_angle;
    float pitch_filtered;/* filtro complementario     */
} imu_data;

typedef imu_data gyro_data;
typedef imu_data accel_data;

/* ------------------------------------------------------------------
 * API pública
 * ------------------------------------------------------------------ */

/* Inicialización bloqueante (solo en arranque, antes de TIM5) */
void mpu9250_init(void);

/* Dispara lectura DMA burst de 14 bytes.
 * Llama una sola vez por ciclo; la CPU queda libre inmediatamente.
 * Cuando termine, HAL llamará HAL_I2C_MemRxCpltCallback → mpu_dma_ready=1 */
void mpu9250_request_dma(void);

/* Convierte mpu_rx_buf → accel_data y gyro_data.
 * Llamar SOLO cuando mpu_dma_ready == 1 */
void mpu9250_parse(accel_data *accel, gyro_data *gyro);

/* Filtro complementario — llamar después de mpu9250_parse */
void mpu9250_complementary(accel_data *accel, gyro_data *gyro);

#endif /* INC_MPU9250_H_ */
