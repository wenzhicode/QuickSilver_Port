#include "MM32F103.h"
#include "stdbool.h"
#include "targets.h"


#define DSHOT_DMA_PHASE 1           // motor pins at both portA and portB


#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21


void dshotInit(void);

void dshotPWMSet(uint8_t number, uint16_t pwm);

void motorDir(unsigned char  number, unsigned char value);

void makePacket(uint8_t number, uint16_t value, bool telemetry);

void dshotDmaStart(void);


void dshotSet(uint8_t number, float pwm);

void flipDetect(void);

