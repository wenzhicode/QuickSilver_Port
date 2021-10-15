
#include "flash.h"
#include "flash_io.h"
#include "stdlib.h"
#include "string.h"
#include "cc2500.h"



extern float hardcoded_pid_identifier;
float initial_pid_identifier = -10;
float saved_pid_identifier;



float flash_get_hard_coded_pid_identifier(void)
{
    float result = 0;

    for (int i = 0;  i < 3 ; i++)
    {
        for (int j = 0; j < 3 ; j++)
        {
//            result += pids_array[i][j] * (i + 1) * (j + 1) * 0.932f;
        }
    }
    return result;
}


void flash_hard_coded_pid_identifier(void)
{
    initial_pid_identifier = flash_get_hard_coded_pid_identifier();
}


/**************************************************************************
**函数信息 ：flash_save()
**功能描述 ：存储变量
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void flash_save()
{
    FLASH_Unlock();
    fmc_erase();

    unsigned long addresscount = 0;
    writeword(addresscount++, FMC_HEADER);
    
    unsigned long *p = (unsigned long *)&rxCc2500SpiConfigMutable;
    for (int i = 0; i < 15; i++)
    {
        writeword(addresscount++, (unsigned long)(*p++));

    }
    
    writeword(255, FMC_HEADER);

    FLASH_Lock();
}



/**************************************************************************
**函数信息 ：flash_load()
**功能描述 ：加载变量
**输入参数 ：无
**输出参数 ：无

头尾有一个简单的校验

**************************************************************************/
void flash_load()
{
    unsigned long addresscount = 0;

    flash_hard_coded_pid_identifier();

    if (FMC_HEADER == fmc_read(addresscount++) && FMC_HEADER == fmc_read(255))
    {
        unsigned long temp;
        unsigned long *p = (unsigned long *)&rxCc2500SpiConfigMutable;
        for (int i = 0; i < 15; i++)
        {
            temp = fmc_read(addresscount++);
            memcpy(p++, &temp, 4);
        }
    }
}




