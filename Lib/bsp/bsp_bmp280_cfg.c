/*!
 * @file bsp_bmp280_cfg.c
 */
#include <bsp_bmp280_cfg.h>

/*! @brief is used in BMP280_Setup() function when BMP280_SPI is defined */
SPI_HandleTypeDef *bmp280_spi_handler;
/*! @brief is used in BMP280_Setup() function when BMP280_I2C is defined */
I2C_HandleTypeDef *bmp280_i2c_handler;

/*!  @brief store all info of BMP280 (device addr, chip id, calib parameters, function pointers, etc.) */
struct bmp280_dev bmp;
/*!  @brief store bmp280 configuration */
struct bmp280_config conf;
/*!  @brief store raw (uncomputed) data */
struct bmp280_uncomp_data ucomp_data;

/*!  @brief store 32bit temperature */
int32_t temp32;
/*!  @brief store floating point temperature */
double temp;
unsigned char msg1[51];
/*!  @brief store 32bit pressure */
uint32_t pres32;
/*!  @brief store floating point pressure */
double pres;
/*!
 *  @brief Function that reads and computes data (temp and pressure)
 *
 *  @return void.
 *
 */
void BMP280_Read(int* temperature, int* pressure)
{
    bmp280_get_uncomp_data(&ucomp_data, &bmp);

    bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);
    bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);

    bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
    bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);

    *temperature = (int)temp;
    *pressure = (int)pres;

    // sprintf(MSG,
    //         "\rTemperature = %d Celsius | Pressure = %d Pa\n\r", (int)temp,
    //         (int)pres);
}

/*!
 *  @brief Function that initializes and configurates BMP280.
 *			Call BMP280_Setup() before BMP280_Read()
 *
 *  @return void.
 *
 */
void BMP280_Setup(void)
{
    /* Assign address*/
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Map functions */
    bmp.delay_ms = Delay_ms;
#ifdef BMP280_SPI
    bmp280_spi_handler = &hspi1;
    bmp.read = SPI_Reg_Read;
    bmp.write = SPI_Reg_Write;
    bmp.intf = BMP280_SPI_INTF;
#endif
#ifdef BMP280_I2C
    bmp280_i2c_handler = &hi2c1;
    bmp.read = I2C_Reg_Read;
    bmp.write = I2C_Reg_Write;
    bmp.intf = BMP280_I2C_INTF;
#endif

    conf.filter = BMP280_FILTER_COEFF_2;
    conf.os_temp = BMP280_OS_4X;
    conf.os_pres = BMP280_OS_NONE;
    conf.odr = BMP280_ODR_1000_MS;
    bmp280_init(&bmp);
    bmp280_set_config(&conf, &bmp);
    bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
}

/*!
 * @brief Function that print temperature and pressure to the given UART
 *
 * @param[in] uart: UART Handler address (&huart1, &huart2, etc.)
 * @return void
 */
void BMP280_Print(UART_HandleTypeDef *uart)
{
    OS_ERR os_err;
    // sprintf((char *)msg,
    //         "\rTemperature = %d Celsius | Pressure = %d Pa\n\r", (int)temp,
    //         (int)pres);
    
    // Nucleo32 (slave) connect with sensor (usart2 to communicate with PC then receive with usart3 then transmit with usart3)
    
    sprintf((char *)msg1,
            "\rTemperature = %d Celsius | Pressure = %d Pa\n\r", (int)temp,
            (int)pres);
    HAL_UART_Transmit(uart, msg1, sizeof(msg1), 100);
    //HAL_UART_Transmit(&huart3, msg1, sizeof(msg1), 100);
    // Nucleo32 (master) without sensor (usart3 to communicate with nucleo32 (slave) and usart2 communiate with PC)
    // HAL_UART_Receive(&huart3,msg1,sizeof(msg1),100); 
    // HAL_UART_Transmit(&huart2, msg1, sizeof(msg1), 100); 
    // OSTimeDlyHMSM(0, 0, 0, 1, OS_OPT_TIME_HMSM_STRICT, &os_err);
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs
 * such as "bmg250_soft_reset", "bmg250_set_foc", "bmg250_perform_self_test" and
 * so on.
 *
 *  @param[in] period_ms  : delay period in milliseconds
 *  @return void.
 *
 */
void Delay_ms(uint32_t period_ms)
{
    OS_ERR os_err;
    /* Non-blocking delay */
	OSTimeDlyHMSM(period_ms/3600000, (period_ms/60000)%60, (period_ms/1000)%60, period_ms%1000, OS_OPT_TIME_HMSM_STRICT, &os_err);
    
	/* Blocking delay */
    /*
    OS_TICK ticks     = ((OSCfg_TickRate_Hz * ((OS_TICK)period_ms + ((OS_TICK)500u / OSCfg_TickRate_Hz))) / (OS_TICK)1000u);
    OS_TICK start_ticks = OSTimeGet(&os_err);
	OS_TICK current_ticks = start_ticks;

	while(current_ticks - start_ticks <= ticks)
	{
		current_ticks = OSTimeGet(&os_err);
		if(current_ticks < start_ticks)
		{
			ticks = (start_ticks + ticks) - UINT32_MAX;
			start_ticks = 0;
		}
	}
    */
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be
 * written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failed
 *
 */
int8_t I2C_Reg_Write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                     uint16_t length)
{

    uint16_t bmp280_addr = (bmp.dev_id << 1);
    if (HAL_I2C_Mem_Write(bmp280_i2c_handler, bmp280_addr, reg_addr, 1,
                          reg_data, length, 500) == HAL_OK)
        return 0;
    else
        return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failed
 *
 */
int8_t I2C_Reg_Read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                    uint16_t length)
{

    uint16_t bmp280_addr;
    bmp280_addr = (bmp.dev_id << 1);

    if (HAL_I2C_Mem_Read(bmp280_i2c_handler, bmp280_addr, reg_addr, 1, reg_data,
                         length, 500) == HAL_OK)
    {
        return 0;
    }
    else
        return -1;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : NOT USE
 *  @param[in] reg_addr     : NOT USE
 *  @param[in] reg_data 	: Pointer to the data buffer whose data has to be
 * written.
 *  @param[in] length       : Number of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failed
 *
 */
int8_t SPI_Reg_Write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
                     uint16_t length)
{
    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(bmp280_spi_handler, reg_data, length, 100) == HAL_OK)
    {
        HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_SET);
        return 0;
    }
    else
        return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       	: NOT USE
 *  @param[in] reg_addr 	: Register command
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   	: Number of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval 1 -> Failed
 *
 */
int8_t SPI_Reg_Read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
                    uint16_t length)
{

    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(bmp280_spi_handler, &reg_addr, 1, 100) == HAL_OK &&
        HAL_SPI_Receive(bmp280_spi_handler, reg_data, length, 100) == HAL_OK)
    {
        HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_SET);
        return 0;
    }
    else
        return -1;
}
