#ifndef SOURCES_VARIO_H_
#define SOURCES_VARIO_H_
#include <stdint.h>
#include "er9x.h"

typedef struct Vario_SilenceZone_t{
int16_t Lo;     /*Vspeed*100*/
int16_t Hi;     /*Vspeed*100*/
} Vario_SilenceZone;

typedef struct Vario_data_t{
int16_t MS;        /* Vspeed*100*/
uint16_t Hertz;    /* tone frequency*/ 
uint16_t Cycle;    /*ms*/
uint8_t Duty;      /*%*/
} Vario_data;

typedef struct Vario_table_t{
Vario_SilenceZone_t  SilenceZone;
Vario_data_t Data[12];
} __attribute__((packed)) Vario_table;

void Vario_play(int32_t Vspeed, bool play);
void Vario_driver(void);

#endif //SOURCES_VARIO_H_