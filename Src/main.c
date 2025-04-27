#include <stdint.h>

// ==============================
// Base Address Definitions
// ==============================

#define PERIPH_BASE             (0x40000000UL)                      // Base address for all peripherals
#define AHB1PERIPH_OFFSET       (0x00020000UL)
#define AHB1PERIPH_BASE         (PERIPH_BASE + AHB1PERIPH_OFFSET)
#define SCS_BASE                (0xE000E000UL)                      // System Control Space base address (for SysTick)

#define GPIOD_OFFSET            (0x0C00UL)
#define RCC_OFFSET              (0x3800UL)
#define SysTick_OFFSET          (0x0010UL)

#define GPIOD_BASE              (AHB1PERIPH_BASE + GPIOD_OFFSET)
#define RCC_BASE                (AHB1PERIPH_BASE + RCC_OFFSET)
#define SysTick_BASE            (SCS_BASE + SysTick_OFFSET)

// Clock enable bits
#define GPIODEN                 (1U << 3)                           // Enable GPIOD clock (bit 3 of RCC AHB1ENR)

// ==============================
// Typedef Struct Definitions
// ==============================

#define __IO volatile

// GPIO register structure
typedef struct {
    __IO uint32_t MODER;         // Mode register
    __IO uint32_t OTYPER;        // Output type register
    __IO uint32_t OSPEEDR;       // Output speed register
    __IO uint32_t PUPDR;         // Pull-up/pull-down register
    __IO uint32_t IDR;           // Input data register
    __IO uint32_t ODR;           // Output data register
    __IO uint32_t BSRR;          // Bit set/reset register
    __IO uint32_t LCKR;          // Configuration lock register
    __IO uint32_t AFRL;          // Alternate function low register
    __IO uint32_t AFRH;          // Alternate function high register
} GPIO_TypeDef;

// RCC register structure
typedef struct {
    __IO uint32_t CR;
    __IO uint32_t PLLCFGR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t AHB1RSTR;
    __IO uint32_t AHB2RSTR;
    __IO uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    __IO uint32_t APB1RSTR;
    __IO uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    __IO uint32_t AHB1ENR;
    __IO uint32_t AHB2ENR;
    __IO uint32_t AHB3ENR;
    uint32_t RESERVED2;
    __IO uint32_t APB1ENR;
    __IO uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    __IO uint32_t AHB1LPENR;
    __IO uint32_t AHB2LPENR;
    __IO uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    __IO uint32_t APB1LPENR;
    __IO uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    __IO uint32_t BDCR;
    __IO uint32_t CSR;
    uint32_t RESERVED6[2];
    __IO uint32_t SSCGR;
    __IO uint32_t PLLI2SCFGR;
    __IO uint32_t PLLSAICFGR;
    __IO uint32_t DCKCFGR;
} RCC_TypeDef;

// SysTick register structure
typedef struct {
    __IO uint32_t CSR;           // Control and status register
    __IO uint32_t RVR;           // Reload value register
    __IO uint32_t CVR;           // Current value register
    __IO const uint32_t CALIB;   // Calibration value register
} SysTick_Type;

// Peripheral base pointers
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define GPIOD   ((GPIO_TypeDef *) GPIOD_BASE)
#define SysTick ((SysTick_Type *) SysTick_BASE)

// SysTick configuration
#define SYSTICK_RVR_VAL          (16000)      // 1ms delay with 16MHz clock
#define CSR_ENABLE               (1U << 0)    // Enable SysTick
#define CSR_CLKSRC               (1U << 2)    // Select processor clock (AHB)
#define CSR_COUNTFLAG            (1U << 16)   // Timer counted to 0 since last read

// GPIO Pin Definitions
#define GPIO_PIN_15  ((uint16_t)0x8000)        // LD6 (Blue LED on STM32F4 Discovery)

// Simple enum for digital pin states
typedef enum {
    RESET,
    SET
} pin_status;

// ==============================
// Function Prototypes
// ==============================

void GPIO_init(void);                          // Configure GPIOD pin for output
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, pin_status status);  // Control output pin
void systickDelayMs(int delay);                 // Delay function using SysTick timer

// ==============================
// Main Function
// ==============================

uint32_t adc_value;

int main(void) {

    GPIO_init();  // Initialize GPIOD pin 15 as output (LED)

    while (1) {

        GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);   // Turn LED on
        systickDelayMs(1000);                     // 1 second delay
        GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET); // Turn LED off
        systickDelayMs(1000);                     // 1 second delay
    }
}

// ==============================
// GPIO Initialization
// ==============================

void GPIO_init(void){

    // Enable clock access to GPIOD
    RCC->AHB1ENR |= GPIODEN;

    // Configure PD15 as general-purpose output (MODER15 = 01)
    GPIOD->MODER |= (1U << 30);
    GPIOD->MODER &= ~(1U << 31);
}

// ==============================
// GPIO Write Pin
// ==============================

void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, pin_status status) {
    if (status == SET) {
        GPIOx->BSRR = GPIO_Pin;             // Set the pin
    } else {
        GPIOx->BSRR = (GPIO_Pin << 16);      // Reset the pin
    }
}

// ==============================
// SysTick-based Delay
// ==============================

void systickDelayMs(int delay){

    /* Load the number of ticks for 1ms delay into the reload register */
    SysTick->RVR = SYSTICK_RVR_VAL;

    /* Clear the current value register */
    SysTick->CVR = 0;

    /* Enable SysTick timer and select processor clock (AHB) */
    SysTick->CSR = CSR_ENABLE | CSR_CLKSRC;

    /* Perform the required number of delays */
    for(int i = 0; i < delay; i++) {
        /* Wait until COUNTFLAG is set */
        while((SysTick->CSR & CSR_COUNTFLAG) == 0) {
            __asm__("nop"); // Prevent optimization
        }
    }

    /* Disable SysTick after delay is complete */
    SysTick->CSR = 0;
}
