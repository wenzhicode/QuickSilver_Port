#include "flash_io.h"


extern void failloop(int);


void writeword(unsigned long address, unsigned long value)
{
    int test = FLASH_ProgramWord(FLASH_ADDR + (address << 2), value);
    if (test != FLASH_COMPLETE)
    {
        FLASH_Lock();
        failloop(5);
    }
}

unsigned long fmc_read(unsigned long address)
{
    address = address * 4 + FLASH_ADDR;
    unsigned int *addressptr = (unsigned int *)address;
    return (*addressptr);
}


int fmc_erase(void)
{
    int test = FLASH_ErasePage(FLASH_ADDR);
    if (test != FLASH_COMPLETE) FLASH_Lock();
    else return 0;
    return 1;
}


void fmc_write_float(unsigned long address, float float_to_write)
{
    writeword(address, *(unsigned long *) &float_to_write);
}

float fmc_read_float(unsigned long address)
{
    unsigned long result = fmc_read(address);
    return *(float *)&result;
}



