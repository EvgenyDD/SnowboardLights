#include "mpu6050.h"
#include "adc.h"
#include "debug.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

float accel_filt[3], gyro_filt[3];
int16_t accel_raw[3], gyro_raw[3];

static uint32_t fail_read_ms = 0;

static void write(uint8_t reg, uint8_t data)
{
    uint8_t d[2] = {reg, data};
    HAL_StatusTypeDef sts = HAL_I2C_Master_Transmit(&hi2c1, 0x68 << 1, d, 2, 100);
    if(sts != 0)
    {
        debug("[E] write FAIL: %d\n", sts);
    }
    // while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    // I2C_GenerateSTART(I2C1, ENABLE);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    // I2C_Send7bitAddress(I2C1, (0x68 << 1), I2C_Direction_Transmitter);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    // I2C_SendData(I2C1, reg); // Передаём адрес регистра
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    // I2C_SendData(I2C1, data); // Передаём данные
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    // I2C_GenerateSTOP(I2C1, ENABLE);
}

static uint8_t MPU6050_Read(uint8_t reg)
{
    HAL_StatusTypeDef sts = HAL_I2C_Master_Transmit(&hi2c1, 0x68 << 1, &reg, 1, 100);
    if(sts != 0)
    {
        // debug("FAIL: %d\n", sts);
    }
    static uint8_t data;
    sts = HAL_I2C_Master_Receive(&hi2c1, 0x68 << 1, &data, 1, 100);
    if(sts != 0)
    {
        // debug("FAIL: %d\n", sts);
    }
    return data;
    // while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    // I2C_GenerateSTART(I2C1, ENABLE);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    // I2C_Send7bitAddress(I2C1, (0x68 << 1), I2C_Direction_Transmitter);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    // I2C_Cmd(I2C1, ENABLE);
    // I2C_SendData(I2C1, reg); // Передаём адрес регистра
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    // I2C_GenerateSTART(I2C1, ENABLE);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    // I2C_Send7bitAddress(I2C1, (0x68 << 1), I2C_Direction_Receiver);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    // I2C_AcknowledgeConfig(I2C1, DISABLE);
    // I2C_GenerateSTOP(I2C1, ENABLE);
    // while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    // data = I2C_ReceiveData(I2C1); // Принимаем данные
    // I2C_AcknowledgeConfig(I2C1, ENABLE);
    // return data;
}

void mpu6050_init(void)
{
    //Датчик тактируется от встроенного 8Мгц осциллятора
    //write(0x6B, 0x41); // Register_PWR_M1 = 0, Disable sleep mode
    write(0x6B, 0x00);

    //Выполнить очистку встроенных регистров датчика
    write(0x1b, 0x08); // Register_UsCtrl = 1

    write(0x1C, 0x10); // Register_UsCtrl = 1
}

int mpu6050_read_acc_gyro(uint32_t diff_ms)
{
    const uint8_t dat = MPU6050_Read(0x3a);

    if(dat & 1)
    {
        fail_read_ms = 0;

        accel_raw[0] = MPU6050_Read(0x3B) << 8;
        accel_raw[0] |= MPU6050_Read(0x3C);
        accel_raw[1] = MPU6050_Read(0x3D) << 8;
        accel_raw[1] |= MPU6050_Read(0x3E);
        accel_raw[2] = MPU6050_Read(0x3F) << 8;
        accel_raw[2] |= MPU6050_Read(0x40);
        gyro_raw[0] = MPU6050_Read(0x43) << 8;
        gyro_raw[0] |= MPU6050_Read(0x44);
        gyro_raw[1] = MPU6050_Read(0x45) << 8;
        gyro_raw[1] |= MPU6050_Read(0x46);
        gyro_raw[2] = MPU6050_Read(0x47) << 8;
        gyro_raw[2] |= MPU6050_Read(0x48);

        accel_raw[1] *= -1;
        accel_raw[2] *= -1;
        gyro_raw[1] *= -1;
        gyro_raw[2] *= -1;

        for(int i = 0; i < 3; i++)
        {
            UTILS_LP_FAST(accel_filt[i], (float)accel_raw[i], 0.2);
            UTILS_LP_FAST(gyro_filt[i], (float)gyro_raw[i], 0.2);
        }

        return 1;
    }
    else
    {
        fail_read_ms += diff_ms;
        if(fail_read_ms > 200)
        {
            for(int i = 0; i < 3; i++)
            {
                accel_filt[i] = gyro_filt[i] = 0;
            }
        }
    }

    return 0;
}