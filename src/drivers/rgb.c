#include "rgb.h"
#include "targets.h"
#include "time.h"
#include "util.h"


/*
 3:rgb data ready
 2:rgb dma buffer ready
     wait to be cascaded after dshot, or fired at next frame if no dshot activity
 1:rgb dma busy
 0:idle
*/
int	rgb_dma_phase = 0;

uint32_t	rgb_data_portA[ RGB_LED_NUMBER*24/4 ] = { 0 };
uint16_t	rgb_portX[1] = { RGB_PIN };
uint32_t RGB_DATA16[16];
int rgb_led_value[RGB_LED_NUMBER];
float r_filt, g_filt, b_filt;
const int offset = RGB_PIN > GPIO_Pin_7;


// sets all leds to a brightness
void rgb_led_set_all( int rgb )
{
// deconstruct the colour into components
    int g = rgb>>16;
    int r = (rgb&0x0000FF00)>>8;
    int b = rgb & 0xff;

// filter individual colors
    lpf( &r_filt, r, RGB_FILTER_TIME);
    lpf( &g_filt, g, RGB_FILTER_TIME);
    lpf( &b_filt, b, RGB_FILTER_TIME);

    int temp = RGB( r_filt, g_filt, b_filt );

    for ( int i = 0 ; i < RGB_LED_NUMBER ; i++)
        rgb_led_value[i] = temp;
}

void rgb_ledflash( int color1, int color2, uint32_t period, int duty )
{
    if ( gettime() % period > (period*duty)>>4 )
    {
        rgb_led_set_all( color1 );
    }
    else
    {
        rgb_led_set_all( color2 );
    }

}

void rgbInit(uint8_t deveice)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RGB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RGB_PORT, &GPIO_InitStructure);
    GPIO_SetBits(RGB_PORT,RGB_PIN);
    
    for (int i=0; i<RGB_LED_NUMBER; i++) {
        rgb_led_value[i]=0;
    }
    if( !rgb_dma_phase )	rgb_dma_phase = 2;

    int pin = rgb_portX[0];

    if ( offset )
    {
        pin >>=8;
    }

    for( int i=0; i<16; i++ ) {
        RGB_DATA16[ i ]  = (i & 0x01) ? 0:pin;
        RGB_DATA16[ i ]<<= 8;
        RGB_DATA16[ i ] |= (i & 0x02) ? 0:pin;
        RGB_DATA16[ i ]<<= 8;
        RGB_DATA16[ i ] |= (i & 0x04) ? 0:pin;
        RGB_DATA16[ i ]<<= 8;
        RGB_DATA16[ i ] |= (i & 0x08) ? 0:pin;
    }
}



void rgbDmaBufferMaking()
{
    // generate rgb dma packet
    int j=0;
    for( int n=0; n<RGB_LED_NUMBER; n++ )
    {
        rgb_data_portA[ j++ ] = RGB_DATA16[ (rgb_led_value[ n ] >> 20) & 0x0f ];
        rgb_data_portA[ j++ ] = RGB_DATA16[ (rgb_led_value[ n ] >> 16) & 0x0f ];
        rgb_data_portA[ j++ ] = RGB_DATA16[ (rgb_led_value[ n ] >> 12) & 0x0f ];
        rgb_data_portA[ j++ ] = RGB_DATA16[ (rgb_led_value[ n ] >>  8) & 0x0f ];
        rgb_data_portA[ j++ ] = RGB_DATA16[ (rgb_led_value[ n ] >>  4) & 0x0f ];
        rgb_data_portA[ j++ ] = RGB_DATA16[ (rgb_led_value[ n ]      ) & 0x0f ];
    }
}


void rgbDmaTrigger()
{
    TIM1->ARR 	= RGB_BIT_TIME;
    TIM1->CCR1 	= RGB_T0H_TIME;
    TIM1->CCR2 	= RGB_T1H_TIME;

    RGB_PORT->BRR     = RGB_PIN;
    DMA1_Channel5->CPAR = (uint32_t)&RGB_PORT->BSRR;
    DMA1_Channel5->CMAR = (uint32_t)rgb_portX;
    DMA1_Channel2->CPAR = (uint32_t)&RGB_PORT->BRR;
    DMA1_Channel2->CMAR = (uint32_t)rgb_data_portA;
    DMA1_Channel3->CPAR = (uint32_t)&RGB_PORT->BRR;
    DMA1_Channel3->CMAR = (uint32_t)rgb_portX;

    DMA1_Channel2->CCR &= ~(0x00000400 | 0x00000800
                            | 0x00000100 | 0x00000200);		// switch from halfword to byte

    DMA_ClearFlag( DMA1_FLAG_GL2 | DMA1_FLAG_GL3 | DMA1_FLAG_GL5 );
    DMA1_Channel3->CNDTR = RGB_LED_NUMBER*24;

    DMA1_Channel5->CNDTR = RGB_LED_NUMBER*24;
    DMA1_Channel2->CNDTR = RGB_LED_NUMBER*24;


    TIM1->SR = 0;

    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    DMA_Cmd(DMA1_Channel3, ENABLE);
    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC2 | TIM_DMA_CC1, ENABLE);

    TIM_SetCounter( TIM1, RGB_BIT_TIME );
    TIM_Cmd( TIM1, ENABLE );
}

void rgbBLDmaTrigger()
{
    TIM1->ARR 	= RGB_BIT_TIME;
    TIM1->CCR1 	= RGB_T0H_TIME;
    TIM1->CCR4 	= RGB_T1H_TIME;

    RGB_PORT->BRR     = RGB_PIN;
    DMA1_Channel5->CPAR = (uint32_t)&GPIOA->BSRR;
    DMA1_Channel5->CMAR = (uint32_t)rgb_portX;
    DMA1_Channel2->CPAR = (uint32_t)&GPIOA->BRR;
    DMA1_Channel2->CMAR = (uint32_t)rgb_data_portA;
    DMA1_Channel4->CPAR = (uint32_t)&GPIOA->BRR;
    DMA1_Channel4->CMAR = (uint32_t)rgb_portX;

    DMA1_Channel2->CCR &= ~(0x00000400 | 0x00000800
                            | 0x00000100 | 0x00000200);

//    DMA1_Channel2->CCR |= DMA_MemoryDataSize_Byte | DMA_PeripheralDataSize_Byte;

    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5);

    DMA1_Channel5->CNDTR = RGB_LED_NUMBER*24;
    DMA1_Channel2->CNDTR = RGB_LED_NUMBER*24;
    DMA1_Channel4->CNDTR = RGB_LED_NUMBER*24;

    TIM1->SR = 0;

    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);


    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);
    TIM_SetCounter(TIM1, RGB_BIT_TIME);
    TIM_Cmd(TIM1, ENABLE);
}

void DMA1_Channel3_IRQHandler(void)
{
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_Cmd(DMA1_Channel3, DISABLE);

    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC2 | TIM_DMA_CC1, DISABLE);
    DMA_ClearITPendingBit(DMA1_IT_TC3);
    TIM_Cmd( TIM1, DISABLE );
    RGB_PORT->BRR     = RGB_PIN;
    rgb_dma_phase = 0;
}


void rgbUpdate(uint8_t device)
{
    static uint16_t timecount = 0;
    timecount ++;

    if(flags.failsafe)
    {
        if(timecount%300<100)
            rgb_led_set_all(RGB( 165,0,0));
        else if(timecount%300<200)
            rgb_led_set_all(RGB( 0,165,0));
        else if(timecount%300<300)
            rgb_led_set_all(RGB( 0,0,165));
        else
            rgb_led_set_all(RGB( 165,165,165));
    }
    else if(rx_aux_on(AUX_LEVELMODE))
    {
        rgb_led_set_all(RGB( 0,0,165));
    }
    else{
        rgb_led_set_all(RGB( 0,165,0));
    }

    rgb_dma_phase =2;
    rgbDmaBufferMaking();
}







