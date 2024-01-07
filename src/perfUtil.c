
#include "stm32h7xx_hal.h"

#include "perfUtil.h"

void initPerfUtil()
{
    DWT->LAR = 0xC5ACCE55;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t getCycles()
{
    uint32_t cycles = DWT->CYCCNT;

    return cycles;
}

/**
 * return the difference between two calls to getCycles() as microseconds
*/
uint32_t getDeltaUs(uint32_t start, uint32_t end)
{
    uint32_t result;

    if (end > start)
    {
        result = (end - start) / 550;
    }
    else
    {
        // counter wrapped so need to compensate
        uint32_t deltaT = (0xFFFFFFFF - start) + end;
        result = deltaT / 550;
    }

    return result;
}