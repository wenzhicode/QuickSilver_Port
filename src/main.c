#include "MM32F103.h"

#include "time.h"
#include "misc_gpio.h"
#include "flash.h"
#include "adc.h"
#include "rgb.h"
#include "dshot.h"
#include "vbat.h"
#include "spi.h"
#include "mpu6000.h"
#include "rx.h"
#include "rx_spi.h"
#include "config.h"
#include "defines.h"
#include "imu.h"
#include "control.h"
#include "pid.h"

void failloop(int val);

uint32_t lastlooptime;




int main(void)
{
    SCB->VTOR = FLASH_BASE | 0x3000;
    
    state.looptime_autodetect = LOOPTIME;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay(1000);

    sysTick_init();
    delay_ms(10);

    misc_gpioInit();

    // setup filters early
    filter_global_init();
    pid_init();

    flash_load();

    adc_init();
  

    dshotInit();
    for (u8 i = 0; i < 4; i++)
    {
        dshotSet(i, 0);
    }

    rgbInit(0);
    
    vbat_init();

    spi_init();

    mpu6000_init();

    rx_init();
    
    delay(1000);
    gyro_cal();
    
    acc_cal();
    
    imu_init();
    
    
    lastlooptime = gettime();
    
     while (1) {
        uint32_t time = gettime();
        state.looptime = ((uint32_t)(time - lastlooptime));
        lastlooptime = time;

        if (state.looptime <= 0)
          state.looptime = 1;
        state.looptime = state.looptime * 1e-6f;
        if (state.looptime > 0.02f) { // max loop 20ms
//          failloop(6);
          //endless loop
        }
    
        
        // read gyro and accelerometer data
        mpu6000_readGyro();

        // all flight calculations and motors
        control();

        // attitude calculations for level mode
        imu_calc();

        // battery low logic
        vbat_calc();
        
        
        // RGB led control
        rgbUpdate(0);
        
        frskyUpdate();
        
        flags.usb_active = 0;
        
        flipDetect();
        
        while ((gettime() - time) < LOOPTIME)
            __NOP();
    }
}


void failloop(int val)
{
    while (1)
    {

    }
}



