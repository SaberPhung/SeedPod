#include "ldr.h"
#include "main.h"
#include "adc.h"
unsigned char MSG[20];
void read_ldr_sensor(void)
{
    uint16_t raw_value;
    double temp,lux;
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

    raw_value = HAL_ADC_GetValue(&hadc);
    temp = ((double)raw_value);
    lux = 3.0 * 1000000.0 * pow(temp, -1.367);
    int l_decimal=(int)(round(lux));
    sprintf(MSG, "Lux: %d\r\n", l_decimal);
    HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
}
void ldr_print(UART_HandleTypeDef *uart)
{
    // Nucleo32 (slave) connect with sensor (usart2 to communicate with PC then receive with usart3 then transmit with usart3)
    HAL_UART_Transmit(uart, MSG, sizeof(MSG), 100);
    //HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
}