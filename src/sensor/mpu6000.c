

#include "mpu6000.h"
#include "math.h"
#include "util.h"



float accel[3];
float accel_one[3];
float gyro[3];
float gyronew[3];
float accelraw[3];
int16 accEf[3];
float accelcal[3];
float gyrocal[3];

u8 mpu6000_id;

int8_t temper=0;

#define spl06_CSH  GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define spl06_CSL  GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define mpu6000_CSH  GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define mpu6000_CSL  GPIO_ResetBits(GPIOB, GPIO_Pin_7)

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];


profile_filter_t profile_filter = {
     .gyro = {
#ifdef GYRO_FILTER_PASS1
            {
                .type = FILTER_LP_PT1,
                .cutoff_freq = GYRO_FILTER_PASS1,
            },
#else
            {
                .type = FILTER_NONE,
            },
#endif
#ifdef GYRO_FILTER_PASS2
            {
                .type = FILTER_LP_PT1,
                .cutoff_freq = GYRO_FILTER_PASS2,
            }
#else
            {
                .type = FILTER_NONE,
            }
#endif
        },

        .dterm = {
#ifdef DTERM_LPF_1ST_HZ
            {
                .type = FILTER_LP_PT1,
                .cutoff_freq = DTERM_LPF_1ST_HZ,
            },
#else
            {
                .type = FILTER_NONE,
            },
#endif
#ifdef DTERM_LPF_2ND_HZ
            {
                .type = FILTER_LP2_PT1,
                .cutoff_freq = DTERM_LPF_2ND_HZ,
            }
#else
            {
                .type = FILTER_NONE,
            }
#endif
        },

#ifdef DTERM_DYNAMIC_LPF
        .dterm_dynamic_enable = 1,
#else
        .dterm_dynamic_enable = 0,
#endif
#ifdef DYNAMIC_FREQ_MIN
        .dterm_dynamic_min = DYNAMIC_FREQ_MIN,
#endif
#ifdef DYNAMIC_FREQ_MAX
        .dterm_dynamic_max = DYNAMIC_FREQ_MAX,
#endif
};

void mpu6000_readRegs(u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
    SPIM_CSHigh();
    spl06_CSH;
    mpu6000_CSL;
    SPI_RW(reg | 0x80);
    for (count = 0; count < length; count++)
    {
        data[count] = SPI_RW(0xff);
    }
    mpu6000_CSH;
}


static u8 mpu6000_readReg(u8 reg)
{
    u8 data;
    SPIM_CSHigh();
    spl06_CSH;
    mpu6000_CSL;//CS拉低则为SPI模式
    SPI_RW(reg | 0x80); //发送寄存器地址+读命令
    data = SPI_RW(0xff);
    mpu6000_CSH;
    return data;
}

static void mpu6000_writeReg(u8 REG, u8 DATA)
{
    SPIM_CSHigh();
    spl06_CSH;
    mpu6000_CSL;//CS拉低则为SPI模式
    SPI_RW(REG & 0x7f); //发送寄存器地址+写命令
    SPI_RW(DATA);
    mpu6000_CSH;
}

u8 mpu6000_readID(void)
{
    return mpu6000_readReg(MPU_RA_WHO_AM_I);
}


void mpu6000_init(void)
{
    uint32_t tmpreg = 0;
    tmpreg = SPI1->SPBRG;
    /* Clear spbrg bits */
    tmpreg &= (uint16_t)0x0000;
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    tmpreg |= (uint16_t) 96;
    /* Write to SPIx SPBRG */
    SPI1->SPBRG = tmpreg;
    delay_ms(100);

    mpu6000_writeReg(MPU_RA_PWR_MGMT_1, BIT_H_RESET); //Reset ing
    delay_ms(150);

    mpu6000_writeReg(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay_ms(150);

    mpu6000_writeReg(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ); //Internal 20MHz oscillator
    delay_ms(1);

    mpu6000_writeReg(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delay_ms(1);

    mpu6000_writeReg(MPU_RA_PWR_MGMT_2, 0x00);
    delay_ms(1);
    
//    mpu6000_writeReg(MPU_RA_CONFIG, 3);   //低通滤波 //26
//    delay_ms(1);

    mpu6000_writeReg(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);//加速度度最大量程 +-2G
    delay_ms(1);

    mpu6000_writeReg(MPU_RA_SMPLRT_DIV, 0);   //采样率 1khz
    delay_ms(1);

    mpu6000_writeReg(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3); //陀螺仪最大量程 +-2000度每秒
    delay_ms(1);

    mpu6000_writeReg(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);
    delay_ms(1);

    mpu6000_id = mpu6000_readID();

    tmpreg = SPI1->SPBRG;
    /* Clear spbrg bits */
    tmpreg &= (uint16_t)0x0000;
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    tmpreg |= (uint16_t) 8;
    /* Write to SPIx SPBRG */
    SPI1->SPBRG = tmpreg;
    delay_ms(100);

    for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
        filter_init(profile_filter.gyro[i].type, &filter[i], filter_state[i], 3, profile_filter.gyro[i].cutoff_freq);
    }
      
}

void mpu6000_readGyro(void)
{
    uint8_t buf[16];

    mpu6000_readRegs(MPU_RA_ACCEL_XOUT_H, 14, buf);

    accelraw[1] = (int16_t)((buf[0] << 8) | buf[1]);
    accelraw[0] = (int16_t)((buf[2] << 8) | buf[3]);
    accelraw[2] = -(int16_t)((buf[4] << 8) | buf[5]);

    accelraw[0] =accelraw[0] - accelcal[0];
    accelraw[1] =accelraw[1] - accelcal[1];
    accelraw[2] =accelraw[2] - accelcal[2];

    temper=((int16_t)(buf[6]<<8)+buf[7])/326.8f + 25.0f;

    //gyro
    gyronew[1] = (int16_t)((buf[8] << 8) | buf[9]);
    gyronew[0] = (int16_t)((buf[10] << 8) | buf[11]);
    gyronew[2] = -(int16_t)((buf[12] << 8) | buf[13]);


    gyronew[0] = gyronew[0] - gyrocal[0];
    gyronew[1] = gyronew[1] - gyrocal[1];
    gyronew[2] = gyronew[2] - gyrocal[2];

    for (int i = 0; i < 3; i++)
    {
        state.accel_raw.axis[i] = accelraw[i] * (1 / 2048.0f);

        state.gyro_raw.axis[i] = gyronew[i] * 0.061035156f * 0.017453292f;
    }
    
  filter_coeff(profile_filter.gyro[0].type, &filter[0], profile_filter.gyro[0].cutoff_freq);
  filter_coeff(profile_filter.gyro[1].type, &filter[1], profile_filter.gyro[1].cutoff_freq);

  for (int i = 0; i < 3; i++) {
    state.gyro.axis[i] = state.gyro_raw.axis[i];

    state.gyro.axis[i] = filter_step(profile_filter.gyro[0].type, &filter[0], &filter_state[0][i], state.gyro.axis[i]);
    state.gyro.axis[i] = filter_step(profile_filter.gyro[1].type, &filter[1], &filter_state[1][i], state.gyro.axis[i]);
  }
}



void mpu6000_readAcc(void)
{
    uint8_t buf[6];
    float accelraw[3];

    mpu6000_readRegs(MPU_RA_ACCEL_XOUT_H, 6, buf);
    accelraw[0] = (int16_t)((buf[0] << 8) + buf[1]);
    accelraw[1] = -(int16_t)((buf[2] << 8) + buf[3]);
    accelraw[2] = -(int16_t)((buf[4] << 8) + buf[5]);

    accelraw[0] -= accelcal[0];
    accelraw[1] -= accelcal[1];
    accelraw[2] -= accelcal[2];

    for (int i = 0; i < 3; i++)
    {
        accel[i] = accelraw[i] * (1 / 2048.0f);
    }
}


#define CAL_TIME 2e6

void gyro_cal(void)
{
    u8 data[6];
    float limit[3];
    unsigned long time = gettime();
    unsigned long timestart = time;
    unsigned long timemax = time;
    unsigned long lastlooptime = time;

    float gyro[3];

    for (int i = 0 ; i < 3 ; i++)
    {
        limit[i] = gyrocal[i];
    }
    while (time - timestart < CAL_TIME  &&  time - timemax < 15e6)
    {
        unsigned long looptime;
        looptime = time - lastlooptime;
        lastlooptime = time;
        if (looptime == 0) looptime = 1;

        mpu6000_readRegs(MPU_RA_GYRO_XOUT_H, 6, data);

        gyro[1] = (int16_t)((data[0] << 8) + data[1]);
        gyro[0] = (int16_t)((data[2] << 8) + data[3]);
        gyro[2] = (int16_t)((data[4] << 8) + data[5]);

        for (int i = 0 ; i < 3 ; i++)
        {
            if (gyro[i] > limit[i])  limit[i] += 0.1f;   // 100 gyro bias / second change
            if (gyro[i] < limit[i])  limit[i] -= 0.1f;

            limitf(&limit[i], 800);

            if (fabsf(gyro[i]) > 100 + fabsf(limit[i]))
            {
                timestart = gettime();
            }
            else
            {
                lpf(&gyrocal[i], gyro[i], lpfcalc((float) looptime, 0.5 * 1e6));
            }
        }
        while ((gettime() - time) < 1000) delay(10);
        time = gettime();
    }

    if (time - timestart < CAL_TIME)
    {
        for (int i = 0 ; i < 3; i++)
        {
            gyrocal[i] = 0;
        }
    }
}


void acc_cal(void)
{
    float accelraw[3];
    accelcal[2] = 2048;

    for (int y = 0; y < 500; y++)
    {
        static u8 buf[6];

        mpu6000_readRegs(MPU_RA_ACCEL_XOUT_H, 6, buf);


        accelraw[0] = (int16_t)((buf[0] << 8) + buf[1]);
        accelraw[1] = -(int16_t)((buf[2] << 8) + buf[3]);
        accelraw[2] = -(int16_t)((buf[4] << 8) + buf[5]);


        for (int x = 0; x < 3; x++)
        {
            lpf(&accelcal[x], accelraw[x], 0.92);
        }
        gettime();
    }
    accelcal[2] -= 2048;

    for (int x = 0; x < 3; x++)
    {
        limitf(&accelcal[x], 500);
    }
}




