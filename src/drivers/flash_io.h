#include "hardware.h"



#define FLASH_ADDR 0x0801B000


float fmc_read_float(unsigned long address);

void fmc_write_float(unsigned long address, float float_to_write);

int fmc_erase(void);

unsigned long fmc_read(unsigned long address);

void writeword(unsigned long address, unsigned long value);




