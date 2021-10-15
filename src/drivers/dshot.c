#include "dshot.h"
#include "control.h"
  
volatile uint16_t dshot_portA[1] = { 0 };       // sum of all motor pins at portA
volatile uint16_t dshot_portB[1] = { 0 };       // sum of all motor pins at portB

volatile uint16_t motor_data_portA[ 16 ] = { 0 };   // DMA buffer: reset output when bit data=0 at TOH timing
volatile uint16_t motor_data_portB[ 16 ] = { 0 };   //

volatile int dshot_dma_phase = 0;                                   // 1:portA  2:portB  0:idle
volatile uint16_t dshot_packet[4];                              // 16bits dshot data for 4 motors

int16_t motor_value[4];

uint8_t disFlip=0;



void dshotInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = PWM_PIN_0 ;
    GPIO_Init(PWM_PORT_0, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PWM_PIN_1 ;
    GPIO_Init(PWM_PORT_1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PWM_PIN_2 ;
    GPIO_Init(PWM_PORT_2, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PWM_PIN_3 ;
    GPIO_Init(PWM_PORT_3, &GPIO_InitStructure);


    *dshot_portB |= PWM_PIN_0;

    *dshot_portB |= PWM_PIN_1;

    *dshot_portB |= PWM_PIN_2;

    *dshot_portB |= PWM_PIN_3;

    // DShot timer/DMA init
    // TIM1_UP  DMA_CH5: set all output to HIGH     at TIM1 update
    // TIM1_CH1 DMA_CH2: reset output if data=0     at T0H timing
    // TIM1_CH4 DMA_CH4: reset all output           at T1H timing


    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    // TIM1 Periph clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period =              DSHOT_BIT_TIME;
    TIM_TimeBaseStructure.TIM_Prescaler =           0;
    TIM_TimeBaseStructure.TIM_ClockDivision =       0;
    TIM_TimeBaseStructure.TIM_CounterMode =         TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM1, DISABLE);

    /* Timing Mode configuration: Channel 1 */
    TIM_OCInitStructure.TIM_OCMode =                TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState =           TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse =                 DSHOT_T0H_TIME;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

    /* Timing Mode configuration: Channel 4 */
    TIM_OCInitStructure.TIM_OCMode =                TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState =           TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse =                 DSHOT_T1H_TIME;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);


    DMA_StructInit(&DMA_InitStructure);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channe5 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOB->BSRR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dshot_portB;
    DMA_InitStructure.DMA_DIR =                                     DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize =                      16;
    DMA_InitStructure.DMA_PeripheralInc =               DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc =                       DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize =      DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize =              DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode =                                    DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority =                            DMA_Priority_High;
    DMA_InitStructure.DMA_M2M =                                     DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    /* DMA1 Channel2 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOB->BRR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)motor_data_portB;
    DMA_InitStructure.DMA_DIR =                                     DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize =                      16;
    DMA_InitStructure.DMA_PeripheralInc =               DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc =                       DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize =      DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize =              DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode =                                    DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority =                            DMA_Priority_High;
    DMA_InitStructure.DMA_M2M =                                     DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    /* DMA1 Channel4 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOB->BRR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dshot_portB;
    DMA_InitStructure.DMA_DIR =                                     DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize =                      16;
    DMA_InitStructure.DMA_PeripheralInc =               DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc =                       DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize =      DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize =              DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode =                                    DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority =                            DMA_Priority_High;
    DMA_InitStructure.DMA_M2M =                                     DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);


    /* configure DMA1 Channel4 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =    DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)DMA_Priority_High;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)DMA_Priority_High;
    NVIC_Init(&NVIC_InitStructure);
    /* enable DMA1 Channel4 transfer complete interrupt */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}


void dshot_dma_portB()
{
    DMA1_Channel5->CPAR = (uint32_t)&GPIOB->BSRR;
    DMA1_Channel5->CMAR = (uint32_t)dshot_portB;
    DMA1_Channel2->CPAR = (uint32_t)&GPIOB->BRR;
    DMA1_Channel2->CMAR = (uint32_t)motor_data_portB;
    DMA1_Channel4->CPAR = (uint32_t)&GPIOB->BRR;
    DMA1_Channel4->CMAR = (uint32_t)dshot_portB;

    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5);

    DMA1_Channel5->CNDTR = 16;
    DMA1_Channel2->CNDTR = 16;
    DMA1_Channel4->CNDTR = 16;

    TIM1->SR = 0;
    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);


    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);
    TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
    TIM_Cmd(TIM1, ENABLE);
}


void makePacket(uint8_t number, uint16_t value, bool telemetry)
{
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);     // Here goes telemetry bit
    // compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;

    for (uint8_t i = 0; i < 3; ++i)
    {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }

    csum &= 0xf;
    // append checksum
    dshot_packet[ number ] = (packet << 4) | csum;
}

void dshotDmaStart()
{
    if (dshot_dma_phase != 0) return;

    extern int rgb_dma_phase;
    if(rgb_dma_phase == 1)
    {
        return;
    }

    // generate dshot dma packet
    for (uint8_t i = 0; i < 16; i++)
    {
        motor_data_portB[ i ] = 0;

        if (!(dshot_packet[0] & 0x8000))
        {
            motor_data_portB[ i ] |= PWM_PIN_0;
        }
        if (!(dshot_packet[1] & 0x8000))
        {
            motor_data_portB[ i ] |= PWM_PIN_1;
        }
        if (!(dshot_packet[2] & 0x8000))
        {
            motor_data_portB[ i ] |= PWM_PIN_2;
        }
        if (!(dshot_packet[3] & 0x8000))
        {
            motor_data_portB[ i ] |= PWM_PIN_3;
        }

        dshot_packet[0] <<= 1;
        dshot_packet[1] <<= 1;
        dshot_packet[2] <<= 1;
        dshot_packet[3] <<= 1;
    }

    dshot_dma_phase = DSHOT_DMA_PHASE;

    TIM1->ARR   = DSHOT_BIT_TIME;
    TIM1->CCR1  = DSHOT_T0H_TIME;
    TIM1->CCR4  = DSHOT_T1H_TIME;

    DMA1_Channel2->CCR |= DMA_MemoryDataSize_HalfWord | DMA_PeripheralDataSize_HalfWord; // switch from byte to halfword

    dshot_dma_portB();
}


void dshotSet(uint8_t number, float pwm)
{
    uint16_t value = 0;

    if (number > 3) return;

    if (pwm < 0.0f)
    {
        pwm = 0.0;
    }
    if (pwm > 0.999f)
    {
        pwm = 0.999f;
    }

    // maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
    value = 230 + (uint16_t)(pwm * 1800 );

    if (flags.on_ground) {
        value = 0; // stop the motors
    }
    
    if(rx_aux_on(AUX_TURTLE))
    {
        if ((state.rx.axis[0] > 0.3f))
        {
            if (number > 1)
            {
                value = 0 + state.rx.axis[0] * 1000  + 1000;
            }
            else
            {
                value = 0;
            }
        }
        if ((state.rx.axis[0] < -0.3f))
        {
            if (number < 2)
            {
                value = 0 + state.rx.axis[0] * (-1000)  + 1000;
            }
            else
            {
                value = 0;
            }
        }
        if ((state.rx.axis[1] > 0.3f))
        {
            if (number == 1 || number == 3)
            {
                value = 0 + state.rx.axis[1] * 1000  + 1000;
            }
            else
            {
                value = 0;
            }
        }
        if ((state.rx.axis[1] < -0.3f))
        {
            if (number == 0 || number == 2)
            {
                value = 0 + state.rx.axis[1] * (-1000)  + 1000;
            }
            else
            {
                value = 0;
            }
        }   
    }
    
    makePacket(number, value, false);

    if (number == 3 )
    {
        dshotDmaStart();
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_Cmd(DMA1_Channel2, DISABLE);


    TIM_DMACmd(TIM1, TIM_DMA_Update, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_CC1, DISABLE);

    DMA_ClearITPendingBit(DMA1_IT_TC4);

    TIM_Cmd(TIM1, DISABLE);

    RGB_PORT->BRR     = RGB_PIN;

    extern void rgbBLDmaTrigger();
    extern int rgb_dma_phase;
    if(rgb_dma_phase == 1)
    {
        rgb_dma_phase = 0;
    }
    switch (dshot_dma_phase)
    {
    case 1:
        dshot_dma_phase = 0;

        if(rgb_dma_phase == 2)
        {
            rgbBLDmaTrigger();
            rgb_dma_phase =1;
        }
        return;

    default :
        dshot_dma_phase = 0;
        break;
    }
}



void motorDir(unsigned char  number, unsigned char value)
{
    makePacket(number, value, true);
    if (number == 3)
    {
        dshotDmaStart();
    }
}


void flipDetect(void)
{
    static char dir[4] = {1, 1, 1, 1};
    static int initDir=0;
    static int flipInitDir=0;
    static int flipCmdNum=2000;
    
    if(flags.arm_state)
        return;

    if(rx_aux_on(AUX_TURTLE))
    {
        initDir = 0;
        disFlip = 0;
        flipCmdNum = 60;
        if(flipInitDir<200)
        {
            for (int i = 20; i > 0; i--)
            {
                motorDir(0, (dir[0] ? DSHOT_CMD_ROTATE_NORMAL : DSHOT_CMD_ROTATE_REVERSE));
                motorDir(1, (dir[1] ? DSHOT_CMD_ROTATE_NORMAL : DSHOT_CMD_ROTATE_REVERSE));
                motorDir(2, (dir[2] ? DSHOT_CMD_ROTATE_NORMAL : DSHOT_CMD_ROTATE_REVERSE));
                motorDir(3, (dir[3] ? DSHOT_CMD_ROTATE_NORMAL : DSHOT_CMD_ROTATE_REVERSE));
            }
            flipInitDir ++;
        }
    }
    else
    {
        flipInitDir = 0;
        if(initDir<flipCmdNum)
        {
            for (int i = 20; i > 0; i--)
            {
                motorDir(0, (dir[0] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
                motorDir(1, (dir[1] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
                motorDir(2, (dir[2] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
                motorDir(3, (dir[3] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
            }
            initDir ++;
        }
        else
        {
            disFlip = 1;
        }
    }
}

