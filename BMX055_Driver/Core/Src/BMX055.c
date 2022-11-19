#include "BMX055.h"

int accl_addr;
int gyro_addr;
int mag_addr;

/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
*/
bool BMX055_init(BMX055_Handle* bmx055) {
  // Set CS pins HIGH
  HAL_GPIO_WritePin(bmx055->acc_CS_port, bmx055->acc_CS_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(bmx055->gyro_CS_port, bmx055->gyro_CS_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(bmx055->mag_CS_port, bmx055->mag_CS_pin, GPIO_PIN_SET);

  if ( BMX055_searchDevice(bmx055) ) {
    BMX055_configuration(bmx055);
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Search bmx055
 * @retval true  Found Device
 * @retval false : Not Found Device
 */
bool BMX055_searchDevice(BMX055_Handle* bmx055)
{
  uint8_t device = 0x00;
  BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_WHO_AM_I_REG, &device, 1);

  if(device == BMX055_ACC_DEVICE){
    return true;
  } else{
    return false;
  }
}

/**
 * @brief Set Config
 */
void BMX055_configuration(BMX055_Handle* bmx055)
{
  /* SoftReset */
  uint8_t data = BMX055_INITIATED_SOFT_RESET;
  // Accel SoftReset
  BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_RESET_REG, &data, 1);
  osDelay(2);  // wait 1.8ms
  // Gyro SoftReset
  BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_RESET_REG, &data, 1);
  osDelay(2);  // wait 1.8ms
  // Mag SoftReset
  BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_RESET_REG, &data, 1);
  osDelay(2);

  /* Accel Setting */
  // Select Accel PMU Range
  data = bmx055->acc_range;
  BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_RANGE_REG, &data, 1);
  // Select Accel PMU_BW   
  data = bmx055->acc_range;
  BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_BW_REG, &data, 1);
  // Select Accel PMU_LPW  (NomalMode, SleepDuration 0.5ms)
  data = BMX055_ACC_PMU_LPW_MODE_NOMAL|BMX055_ACC_PMU_LPW_SLEEP_DUR_0_5MS;
  BMX055_writeSPI(bmx055 ,bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_LPW_REG, &data, 1);

  /* Gyro Setting */
  // Select Gyro Range(262.4 LSB/°/s)
  data = bmx055->gyro_range;
  BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_RANGE_REG, &data, 1);
  // Select Gyro BW   (32Hz)
  data = bmx055->gyro_bandwidth;
  BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_BW_REG, &data, 1);
  // Select Gyro LPM1 (NomalMode, SleepDuration 2ms)
  data = BMX055_GYRO_LPM1_MODE_NOMAL|BMX055_GYRO_LPM1_SLEEP_DUR_2MS;
  BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_LPM1_REG, &data, 1);

  /* Mag Setting */
  // set sleep mode
  data = BMX055_MAG_POW_CTL_SLEEP_MODE;
  BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_POW_CTL_REG, &data, 1);
  osDelay(100);
  // adv.st, DataRate, OperationMode, SelfTest (NomalMode)
  data = bmx055->mag_data_rate;
  BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);
  // Repetitions for X-Y Axis  0x04 -> 0b00000100 -> (1+2(2^2)) = 9
  data = 0x04;
  BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_REP_XY_REG, &data, 1);
  // Repetitions for Z-Axis  0x0F-> 0b00001111-> (1 +(2^0 + 2^1 + 2^2 + 2^3) = 15
  data = 0x0F;
  BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_REP_Z_REG, &data, 1);

  osDelay(200);
}

/**
 * @brief Read Accel
 * @param [out] *accl : accel value  (X-accel : accl[0], Y-accel : accl[1], Z-accel : accl[2])
 */
 void BMX055_readAccel(BMX055_Handle* bmx055, int *accl)
{
  uint8_t accl_data[6];

  // read accel value
  for (int i = 0; i < 6; i++)
  {
    BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_DATA_START_REG+i, &accl_data[i], 1);
  }

  // conv data  accel:12bit
  accl[0] = ((accl_data[1]<<4) + (accl_data[0]>>4));

  if (accl[0] > 2047)
  {
    accl[0] -= 4096;
  }

  accl[1] = ((accl_data[3]<<4) + (accl_data[2]>>4));
  if (accl[1]> 2047)
  {
    accl[1] -= 4096;
  }

  accl[2] = ((accl_data[5]<<4) + (accl_data[4]>>4));
  if (accl[2] > 2047)
  {
    accl[2] -= 4096;
  }
}

/**
 * @brief Read Gyro
 * @param [out] *gyro gyro value (X-gyro: gyro[0], Y-gyro: gyro[1], Z-gyro: gyro[2])
 */
void BMX055_readGyro(BMX055_Handle* bmx055, int *gyro)
{
  uint8_t gyro_data[6];

  // read gyro value
  for (int i = 0; i < 6; i++)
  {
    BMX055_readSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_DATA_START_REG+i, &gyro_data[i], 1);
  }

  // conv data  gyro:16bit
  gyro[0] = ((gyro_data[1]<<8) + gyro_data[0]);
  if (gyro[0] > 32767)
  {
    gyro[0] -= 65536;
  }

  gyro[1] = ((gyro_data[3]<<8) + gyro_data[2]);
  if (gyro[1] > 32767)
  {
    gyro[1] -= 65536;
  }

  gyro[2] = ((gyro_data[5]<<8) + gyro_data[4]);
  if (gyro[2] > 32767)
  {
    gyro[2] -= 65536;
  }
}

/**
 * @brief Read Mag
 * @param [out] *mag mag value (X-mag: mag[0], Y-mag: mag[1], Z-mag: mag[2])
 */
void BMX055_readMag(BMX055_Handle* bmx055, int *mag)
{
  uint8_t mag_data[6];

  // read mag value
  for (int i = 0; i < 6; i++)
  {
    BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_DATA_START_REG+i, &mag_data[i], 1);
  }

  // conv data  mag x:12bit
  mag[0] = ((mag_data[1]<<5) + (mag_data[0]>>3));
  if (mag[0] > 4095)
  {
    mag[0] -= 8192;
  }

  // conv data  mag y:12bit
  mag[1] = ((mag_data[3]<<5) + (mag_data[2]>>3));
  if (mag[1] > 4095)
  {
    mag[1] -= 8192;
  }

  // conv data  mag z:15bit
  mag[2] = ((mag_data[3]<<7) + (mag_data[2]>>1));
  if (mag[2] > 16383)
  {
    mag[2] -= 32768;
  }
}

/**
 * @brief Write SPI Data
 * @param [in] CS_Port
 * @param [in] CS_Pin
 * @param [in] register_addr
 * @param [in] data
 * @param [in] len
 */
void BMX055_writeSPI(BMX055_Handle* bmx055, GPIO_TypeDef* CS_Port, uint16_t CS_Pin, uint8_t register_addr, uint8_t* data, size_t len) {
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(bmx055->hspi, register_addr, len, 1000);
    HAL_SPI_Transmit(bmx055->hspi, data, len, 1000);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Read SPI Data
 * @param [in] device Device type (gyro, accel or mag)
 * @param [in] register_addr Register Address
 * @param [in] num Data Length
 * @param [out] *buf Read Data
 */
void BMX055_readSPI(BMX055_Handle* bmx055, GPIO_TypeDef* CS_Port, uint16_t CS_Pin, uint8_t register_addr, uint8_t* data, size_t len) {
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(bmx055->hspi, register_addr, data, len, 1000);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
}
