#include <stm32f103xxGPIO.h>

uint32_t high_low_pin(uint32_t pinNumber){
    if(pinNumber > 7)
    {
        return 1;
    }
    return 0;
}
