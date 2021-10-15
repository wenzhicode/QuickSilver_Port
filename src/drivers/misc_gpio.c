#include "misc_gpio.h"
#include "targets.h"
#include "targets.h"


void misc_gpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //ENABLE AFIO Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.GPIO_Pin = GYRO_CS_PIN;
    GPIO_Init(GYRO_CS_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BARO_CS_PIN;
    GPIO_Init(BARO_CS_GPIO, &GPIO_InitStructure);

    GPIO_SetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
    GPIO_SetBits(BARO_CS_GPIO, BARO_CS_PIN);
    
    GPIO_InitTypeDef ExtiState;

    ExtiState.GPIO_Pin=CC2500Exit_Pin;
    ExtiState.GPIO_Mode=GPIO_Mode_IPU;
    ExtiState.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CC2500Exit_Port,&ExtiState);
}









