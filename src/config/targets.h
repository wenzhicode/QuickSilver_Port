
#include "config.h"


#define PWM_PIN_0           GPIO_Pin_5
#define PWM_PORT_0          GPIOB

#define PWM_PIN_1           GPIO_Pin_4
#define PWM_PORT_1          GPIOB

#define PWM_PIN_2           GPIO_Pin_1
#define PWM_PORT_2          GPIOB

#define PWM_PIN_3           GPIO_Pin_0
#define PWM_PORT_3          GPIOB

#define SYS_CLOCK_FREQ_HZ   96000000

#define DSHOT_BIT_TIME      ((SYS_CLOCK_FREQ_HZ/1000/600)-1)
#define DSHOT_T0H_TIME      (DSHOT_BIT_TIME*0.30 + 0.05 )
#define DSHOT_T1H_TIME      (DSHOT_BIT_TIME*0.60 + 0.05 )



#define RGB_PIN             GPIO_Pin_8
#define RGB_PORT            GPIOA

#define RGB_BIT_TIME 		((SYS_CLOCK_FREQ_HZ/1000/800)-1)
#define RGB_T0H_TIME 		(RGB_BIT_TIME*0.30 + 0.05 )
#define RGB_T1H_TIME 		(RGB_BIT_TIME*0.60 + 0.05 )

#define RGB_LED_NUMBER 2


#define GYRO_SPI             1              //陀螺仪SPI配置
#define GYRO_CS_GPIO         GPIOB
#define GYRO_CS_PIN          GPIO_Pin_7

#define CC2500_SPI            1              //陀螺仪2SPI配置
#define CC2500_CS_GPIO        GPIOA
#define CC2500_CS_PIN         GPIO_Pin_4

#define BARO_SPI             1              //气压计SPI配置
#define BARO_CS_GPIO         GPIOA
#define BARO_CS_PIN          GPIO_Pin_1


#define CC2500Exit_Pin         GPIO_Pin_2
#define CC2500Exit_Port        GPIOB

