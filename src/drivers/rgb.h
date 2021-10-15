#ifndef __RGB_H
#define	__RGB_H

#include "MM32F103.h"

#include "defines.h"
// fade from one color to another when changed
#define RGB_FILTER_ENABLE
#define RGB_FILTER_TIME_MICROSECONDS 50e3

// runs the update once every 16 loop times ( 16 mS )
#define DOWNSAMPLE 16

#define RGB_FILTER_TIME FILTERCALC( 1000*DOWNSAMPLE , RGB_FILTER_TIME_MICROSECONDS)
#define RGB( r , g , b ) ( ( ((int)g&0xff)<<16)|( ((int)r&0xff)<<8)|( (int)b&0xff ))


void rgbInit(uint8_t deveice);

void rgbUpdate(uint8_t devic);

#endif

