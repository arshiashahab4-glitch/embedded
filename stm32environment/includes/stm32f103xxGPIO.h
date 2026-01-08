#ifndef STM32F103XXGPIO
#define STM32F103XXGPIO 

/*
All definitions pertain to stm32103xx family, based on reference manual
https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
*/


#include <stdint.h>

uint32_t high_low_pin(uint32_t); //GPIO configuration registers are split based on pin number [0-7] low, [8-15] high.

#define PERIPHERAL_BASE (0x40000000U) //Base for address space pertaining to peripheral devices
#define RCC_BASE (PERIPHERAL_BASE + 0x21000U) //Base for address space pertaining to 

#define APB2_BASE (PERIPHERAL_BASE + 0x10000U)

#define GPIOA_BASE (APB2_BASE + 0x800U)
#define GPIOB_BASE (APB2_BASE + 0xC00U)
#define GPIOC_BASE (APB2_BASE + 0x1000U)
#define GPIOD_BASE (APB2_BASE + 0x1400U)
#define GPIOE_BASE (APB2_BASE + 0x1800U)
#define GPIOF_BASE (APB2_BASE + 0x1C00U)
#define GPIOG_BASE (APB2_BASE + 0x2000U)

#define GPIO_A (GPIOA_BASE)
#define GPIO_B (GPIOB_BASE)
#define GPIO_C (GPIOC_BASE)
#define GPIO_D (GPIOD_BASE)
#define GPIO_E (GPIOE_BASE)
#define GPIO_F (GPIOF_BASE)
#define GPIO_G (GPIOG_BASE)

//other GPIO base memory offsets (however port C is used for this application)

#define RCC_APB2ENR_OFFSET (0x18U) //register offset
#define RCC_APB2ENR ((volatile uint32_t*) (RCC_BASE + RCC_APB2ENR_OFFSET))
#define RCC_APB2ENR_GPIOCEN (0x04U) //bit offset

#define GPIO_LMODER_OFFSET (0x00U) //Port config register for 'low pins [0-7]'
#define GPIO_HMODER_OFFSET (0x04U) //Port config register for 'high pins [8-15]'
#define GPIOC_HMODER ((volatile uint32_t*) (GPIOC_BASE + GPIO_HMODER_OFFSET))
#define GPIO_HMODER_HMODER13 (20U)
#define GPIO_ODR_OFFSET (0x0CU)
#define GPIOC_ODR ((volatile uint32_t*) (GPIOC_BASE + GPIO_ODR_OFFSET))

#define GPIOX_CONFIGR(GPIO_PORT, PIN_NUMBER) ((volatile uint32_t*) (GPIO_PORT + ((high_low_pin(PIN_NUMBER)) ? GPIO_HMODER_OFFSET: GPIO_LMODER_OFFSET)))


#define LED_PIN 13


#endif
