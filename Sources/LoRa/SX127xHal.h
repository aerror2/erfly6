#pragma once

#include "targets.h"
#include "SX127xRegs.h"


class SX127xHal
{

public:
    static SX127xHal *instance;

    SX127xHal();

    void init();
    void end();

    static void ICACHE_RAM_ATTR dioISR();
    void (*IsrCallback)(); //function pointer for callback

    void ICACHE_RAM_ATTR TXenable();
    void ICACHE_RAM_ATTR RXenable();
    void ICACHE_RAM_ATTR TXRXdisable();

    uint8_t ICACHE_RAM_ATTR getRegValue(uint8_t reg, uint8_t msb = 7, uint8_t lsb = 0);
    uint8_t ICACHE_RAM_ATTR readRegister(uint8_t reg);
    void ICACHE_RAM_ATTR readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes);

    uint8_t ICACHE_RAM_ATTR setRegValue(uint8_t reg, uint8_t value, uint8_t msb = 7, uint8_t lsb = 0);

    void ICACHE_RAM_ATTR writeRegister(uint8_t reg, uint8_t data);
    void ICACHE_RAM_ATTR writeRegisterFIFO(volatile uint8_t *data, uint8_t numBytes);
    void ICACHE_RAM_ATTR readRegisterFIFO(volatile uint8_t *data, uint8_t numBytes);
    void ICACHE_RAM_ATTR writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes);
};