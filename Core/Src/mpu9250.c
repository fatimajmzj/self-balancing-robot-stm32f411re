/*
 * mpu9250.c — Driver MPU9250 con lectura DMA no-bloqueante (burst 14 bytes)
 *  STM32F411RE — Balancín
 *
 */
#include "mpu9250.h"
#include "main.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/* Flag: 1 = DMA terminó, hay datos nuevos en mpu_rx_buf */
volatile uint8_t mpu_dma_ready = 0;

/* Buffer de recepción DMA — 14 bytes raw del MPU */
uint8_t mpu_rx_buf[MPU_BURST_LEN];

/* Estado interno del filtro complementario */
static float pitch_comp = 0.0f;

/* ------------------------------------------------------------------ */
void mpu9250_init(void)
{
    uint8_t who_am_i = 0;
    uint8_t data;

    /* WHO_AM_I — bloqueante, solo al arranque */
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, WHO_AM_I_REG, 1, &who_am_i, 1, 100);

    if (who_am_i != 0x71 && who_am_i != 0x70) {
        printf("MPU9250 no encontrado! WHO_AM_I=0x%02X\r\n", who_am_i);
    } else {
        printf("MPU9250 OK. WHO_AM_I=0x%02X\r\n", who_am_i);
    }

    /* Despertar — clock automático */
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
    HAL_Delay(100);

    /* Tasa de muestreo 125 Hz */
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

    /* Giroscopio ±500°/s */
    data = GYRO_RANGE_500;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);

    /* Acelerómetro ±4g */
    data = ACCEL_RANGE_4G;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);

    /* Primera lectura bloqueante para inicializar pitch_comp */
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H, 1,
                     mpu_rx_buf, MPU_BURST_LEN, 200);

    accel_data tmp_a;
    gyro_data  tmp_g;
    mpu9250_parse(&tmp_a, &tmp_g);
    pitch_comp = tmp_a.pitch_angle;

    printf("MPU9250 configurado. Pitch inicial: %.2f\r\n", pitch_comp);
}

/* ------------------------------------------------------------------ */
/*  Dispara lectura DMA — no bloquea                                   */
/* ------------------------------------------------------------------ */
void mpu9250_request_dma(void)
{
    /* Si el periférico aún está ocupado (transferencia anterior sin terminar)
     * no hacemos nada: esperamos al siguiente ciclo del while().
     * En condiciones normales esto no ocurre porque el while() solo llama
     * esta función DESPUÉS de haber procesado el dato anterior. */
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        return;
    }

    mpu_dma_ready = 0;

    /* Burst de 14 bytes a partir de ACCEL_XOUT_H (0x3B).
     * El MPU9250 autoincrementa el registro internamente.
     * La CPU regresa de esta llamada en microsegundos. */
    HAL_I2C_Mem_Read_DMA(&hi2c1,
                          MPU9250_ADDR,
                          ACCEL_XOUT_H,
                          I2C_MEMADD_SIZE_8BIT,
                          mpu_rx_buf,
                          MPU_BURST_LEN);
}

/* ------------------------------------------------------------------ */
/*  Convierte buffer raw → accel_data + gyro_data                     */
/*  Llamar SOLO con mpu_dma_ready == 1                                 */
/* ------------------------------------------------------------------ */
void mpu9250_parse(accel_data *accel, gyro_data *gyro)
{

    int16_t raw_ax = (int16_t)((mpu_rx_buf[0]  << 8) | mpu_rx_buf[1]);
    int16_t raw_ay = (int16_t)((mpu_rx_buf[2]  << 8) | mpu_rx_buf[3]);
    int16_t raw_az = (int16_t)((mpu_rx_buf[4]  << 8) | mpu_rx_buf[5]);

    int16_t raw_gx = (int16_t)((mpu_rx_buf[8]  << 8) | mpu_rx_buf[9]);
    int16_t raw_gy = (int16_t)((mpu_rx_buf[10] << 8) | mpu_rx_buf[11]);
    int16_t raw_gz = (int16_t)((mpu_rx_buf[12] << 8) | mpu_rx_buf[13]);

    /* Acelerómetro — unidades: g (×10 para compatibilidad con tu código) */
    accel->x_val = ((float)raw_ax * 10.0f) / ACCEL_SENSITIVITY;
    accel->y_val = ((float)raw_ay * 10.0f) / ACCEL_SENSITIVITY;
    accel->z_val = ((float)raw_az * 10.0f) / ACCEL_SENSITIVITY;

    /* Ángulos desde acelerómetro */
    accel->roll_angle  =  atan2f(accel->y_val,
                           sqrtf(accel->x_val*accel->x_val +
                                 accel->z_val*accel->z_val))
                           * (180.0f / 3.14159f);

    accel->pitch_angle = (-atan2f(accel->x_val,
                           sqrtf(accel->y_val*accel->y_val +
                                 accel->z_val*accel->z_val))
                           * (180.0f / 3.14159f)) ;

    /* Giroscopio — unidades: °/s */
    gyro->x_val = (float)raw_gx / GYRO_SENSITIVITY;
    gyro->y_val = (float)raw_gy / GYRO_SENSITIVITY;
    gyro->z_val = (float)raw_gz / GYRO_SENSITIVITY;
}

/* ------------------------------------------------------------------ */
/*  Filtro complementario — no se usa en el codigo           */
/* ------------------------------------------------------------------ */
void mpu9250_complementary(accel_data *accel, gyro_data *gyro)
{
    pitch_comp = ALPHA * (pitch_comp + gyro->y_val * DT)
               + (1.0f - ALPHA) * accel->pitch_angle;

    accel->pitch_filtered = pitch_comp;
}

/* ------------------------------------------------------------------ */
/*  Callback del HAL — llamada por DMA cuando termina la transferencia */
/*  IMPORTANTE: esto corre en contexto de interrupción (DMA ISR).     */
/* ------------------------------------------------------------------ */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        mpu_dma_ready = 1;
    }
}
