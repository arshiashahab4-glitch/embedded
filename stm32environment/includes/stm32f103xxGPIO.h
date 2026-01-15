#ifndef STM32F103XXGPIO
#define STM32F103XXGPIO 

/*
All definitions pertain to stm32103xx family, based on reference manual
https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
*/


#include <stdint.h>

/*
FUNCTION PROTOTYPES
*/

uint32_t high_low_pin(uint32_t); //GPIO configuration registers are split based on pin number [0-7] low, [8-15] high.

/*
BASE ADDRESSES TO MAJOR PERIPHERAL ADDRESS SPACES 
*/

#define PERIPHERAL_BASE (0x40000000U) //Base for address space pertaining to peripheral devices
#define APB2_BASE (PERIPHERAL_BASE + 0x10000U) //Base address for advanced peripheral buffer 2 space


/*
RESET CLOCK CONTROL
*/

#define RCC_BASE (PERIPHERAL_BASE + 0x21000U) //Base address for RCC registers
#define RCC_APB2ENR_OFFSET (0x18U) //RCC Advanced Peripheral Buffer 2 Enable Register
#define RCC_APB2ENR ((volatile uint32_t*) (RCC_BASE + RCC_APB2ENR_OFFSET))

#define RCC_APB2ENR_GPIOA_ENABLEBIT (0x02U)
#define RCC_APB2ENR_GPIOB_ENABLEBIT (0x03U)
#define RCC_APB2ENR_GPIOC_ENABLEBIT (0x04U)
#define RCC_APB2ENR_GPIOD_ENABLEBIT (0x05U)
#define RCC_APB2ENR_GPIOE_ENABLEBIT (0x06U)
#define RCC_APB2ENR_GPIOF_ENABLEBIT (0x07U)
#define RCC_APB2ENR_GPIOG_ENABLEBIT (0x08U)

//GENERAL 

#define RCC_APB2ENR_ENABLE(ENABLE_BIT) (*RCC_APB2ENR |= (1 << ENABLE_BIT))
#define RCC_APB2ENR_DISABLE(DISABLE_BIT) (*RCC_APB2ENR &= (0 << DISABLE_BIT))

#define GPIO_RCC_ENABLE(GPIO_RCC_OFFSET) (RCC_APB2ENR_ENABLE(GPIO_RCC_OFFSET))
#define GPIO_RCC_DISABLE(GPIO_RCC_OFFSET) (RCC_APB2ENR_DISABLE(GPIO_RCC_OFFSET))


/*
GPIO PORTS -- BASE ADDRESSES TO PORT SPECIFIC REGISTER SPACES
*/

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

/*
GPIO PIN CONFIGURATION REGISTERS
*/

#define GPIO_LCR_OFFSET (0x00U) //Base address for GPIO config register for 'low pins [0-7].'
#define GPIO_HCR_OFFSET (0x04U) //Base address for GPIO config register for 'high pins [8-15].'
#define GPIO_CR_PINOFFSET(PIN_NUMBER) (4 * (PIN_NUMBER - ((high_low_pin(PIN_NUMBER)) ? 8 : 0)))

#define GPIOX_CR(GPIO_PORT, PIN_NUMBER) ((volatile uint32_t*) (GPIO_PORT + ((high_low_pin(PIN_NUMBER)) ? GPIO_HCR_OFFSET: GPIO_LCR_OFFSET))) //Pointer to GPIOx (x = A,B,C,...) Config Register base address.
#define CONFIGURE_GPIO_PIN(GPIO_PORT, PIN_NUMBER, PIN_CONFIG)                              \
    do {                                                                                   \
        *GPIOX_CR(GPIO_PORT, PIN_NUMBER) &= ~(0xF << GPIO_CR_PINOFFSET(PIN_NUMBER));       \
        *GPIOX_CR(GPIO_PORT, PIN_NUMBER) |= (PIN_CONFIG << GPIO_CR_PINOFFSET(PIN_NUMBER)); \
    } while (0)

//INPUT CONFIGS

#define ANALOG_INPUT (0b0000U)
#define FLOATING_INPUT (0b0100U) //RESET STATE
#define PULLPUSH_INPUT (0b1000U)
#define RESERVED (0b1100U)

//OUTPUT CONFIGS

#define OUTPUT_10MHz (0b01U)
#define OUTPUT_2MHz (0b10U)
#define OUTPUT_50MHz (0b11U)

#define GP_OUTPUT_PUSHPULL(OUTPUT_FREQUENCY) (0b0000U + OUTPUT_FREQUENCY)
#define GP_OUTPUT_DRAIN(OUTPUT_FREQUENCY) (0b0100U + OUTPUT_FREQUENCY)
#define ALTERNATE_OUTPUT_PUSHPULL(OUTPUT_FREQUENCY) (0b1000U + OUTPUT_FREQUENCY)
#define ALTERNATE_OUTPUT_DRAIN(OUTPUT_FREQUENCY) (0b1100U + OUTPUT_FREQUENCY)


/*
GPIO PIN INPUT/OUTPUT DATA REGISTERS
*/

#define GPIO_IDR_OFFSET (0x08U)
#define GPIOX_IDR(GPIO_PORT) ((volatile uint32_t*) (GPIO_PORT + GPIO_IDR_OFFSET))

#define GPIO_ODR_OFFSET (0x0CU)
#define GPIOX_ODR(GPIO_PORT) ((volatile uint32_t*) (GPIO_PORT + GPIO_ODR_OFFSET))

#define HIGH 1U
#define LOW 0U

//GPIO PIN FUNCTIONS
#define GPIO_PIN_INPUT(GPIO_PORT, PIN_NUMBER) (((0 << PIN_NUMBER) |= *GPIOX_IDR(GPIO_PORT)) >> PIN_NUMBER) //Reads pin input in GPIOX_IDR.
#define GPIO_PIN_OUTPUT(GPIO_PORT,PIN_NUMBER,STATE) ((STATE) ? (*GPIOX_ODR(GPIO_PORT) |= (1 << PIN_NUMBER)):(*GPIOX_ODR(GPIO_PORT) &= ~(1 << PIN_NUMBER))) //Sets output state in GPIOX_ODR.
#define TOGGLE_OUTPUT_PIN(GPIO_PORT, PIN_NUMBER) (*GPIOX_ODR(GPIO_PORT) ^= (1 << PIN_NUMBER)) //Toggles state of GPIO output pin.

/*
GPIO BSRR REGISTER
*/

#define GPIO_BSRR_OFFSET (0x10U)
#define GPIOX_BSRR(GPIO_PORT) ((volatile uint32_t *) (GPIO_PORT + GPIO_BSRR_OFFSET))

#define PIN_SET (0x00000001)
#define PIN_RESET (0x00010000)

#define GPIO_SET_PIN(GPIO_PORT, PIN_NUMBER, SET_STATE) (*GPIOX_BSRR(GPIO_PORT) |= (SET_STATE << PIN_NUMBER))



//GPIO PIN FUNCTIONS





#endif
