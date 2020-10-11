// MCU-specific header file for STM32F446XX

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>

#define __vo volatile

// Base addresses of Flash and SRAM memories
#define FLASH_BASE_ADDR (uint32_t*) 0x08000000
#define SRAM1_BASE_ADDR (uint32_t*) 0x20000000
#define SRAM2_BASE_ADDR (uint32_t*) 0x2001C000
#define ROM_BASE_ADDR (uint32_t*) 0x1FFF0000

// Only using SRAM1
#define SRAM SRAM1_BASE_ADDR

// AHBx and APHx bus peripheral base addresses
#define PERIPH_BASE (uint32_t*) 0x40000000
#define APB1_PERIPH_BASE PERIPH_BASE
#define APB2_PERIPH_BASE (uint32_t*) 0x40010000
#define AHB1_PERIPH_BASE (uint32_t*) 0x40020000
#define AHB2_PERIPH_BASE (uint32_t*) 0x50000000

// AHB1 bus peripherals
#define GPIOA_BASE_ADDR (AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDR (AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASE_ADDR (AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASE_ADDR (AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASE_ADDR (AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASE_ADDR (AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASE_ADDR (AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASE_ADDR (AHB1_PERIPH_BASE + 0x1C00)
#define RCC_BASE_ADDR (AHB1_PERIPH_BASE + 0x3800)


// APB1 bus peripherals
#define SPI2_BASE_ADDR (APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASE_ADDR (APB1_PERIPH_BASE + 0x3C00)
#define USART2_BASE_ADDR (APB1_PERIPH_BASE + 0x4400)
#define USART3_BASE_ADDR (APB1_PERIPH_BASE + 0x4800)
#define UART4_BASE_ADDR (APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASE_ADDR (APB1_PERIPH_BASE + 0x5000)
#define I2C1_BASE_ADDR (APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASE_ADDR (APB1_PERIPH_BASE + 0x5800)
#define I2C3_BASE_ADDR (APB1_PERIPH_BASE + 0x5C00)

// APB2 bus peripherals
#define SPI1_BASE_ADDR (APB2_PERIPH_BASE + 0x3000)
#define USART1_BASE_ADDR (APB2_PERIPH_BASE + 0x1000)
#define USART6_BASE_ADDR (APB2_PERIPH_BASE + 0x1400)
#define EXTI_BASE_ADDR (APB2_PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE_ADDR (APB2_PERIPH_BASE + 0x3800)

// Peripheral register definitions

// GPIO register map
typedef struct {
	__vo uint32_t MODER; // Port mode
	__vo uint32_t OTYPER; // Port output type
	__vo uint32_t OSPEEDR; // Port output speed
	__vo uint32_t PUPDR; // Port pull-up/pull-down
	__vo uint32_t IDR; // Port input data
	__vo uint32_t ODR; // Port output data
	__vo uint32_t BSRR; // Port bit set/reset
	__vo uint32_t LCKR; // Port config lock
	__vo uint32_t AFR[2]; // AFR[0] Altfn low, AFR[1] Altfn high
} GPIO_reg_def_t;

// RCC register map
typedef struct {
	__vo uint32_t CR; // Clock control
	__vo uint32_t PLL_CFGR; // PLL config
	__vo uint32_t CFGR; // Clock config
	__vo uint32_t CIR; // Clock interrupt
	__vo uint32_t AHB1RSTR; // AHB1 peripheral reset
	__vo uint32_t AHB2RSTR; // AHB2 peripheral reset
	__vo uint32_t AHB3RSTR; // AHB3 peripheral reset
	uint32_t RESERVED_0;
	__vo uint32_t APB1RSTR; // APB1 peripheral reset
	__vo uint32_t APB2RSTR; // APB2 peripheral reset
	uint32_t RESERVED_1[2];
	__vo uint32_t AHB1ENR; // AHB1 peripheral clock enable
	__vo uint32_t AHB2ENR; // AHB2 peripheral clock enable
	__vo uint32_t AHB3ENR; // AHB3 peripheral clock enable
	uint32_t RESERVED_2;
	__vo uint32_t APB1ENR; // APB1 peripheral clock enable
	__vo uint32_t APB2ENR; // APB2 peripheral clock enable
	uint32_t RESERVED_3[2];
	__vo uint32_t AHB1LPENR; // AHB1 peripheral clock enable (low-power)
	__vo uint32_t AHB2LPENR; // AHB2 peripheral clock enable (low-power)
	__vo uint32_t AHB3LPENR; // AHB3 peripheral clock enable (low-power)
	uint32_t RESERVED_4;
	__vo uint32_t APB1LPENR; // APB1 peripheral clock enable (low-power)
	__vo uint32_t APB2LPENR; // APB2 peripheral clock enable (low-power)
	uint32_t RESERVED_5[2];
	__vo uint32_t BDCR; // RCC backup domain control
	__vo uint32_t CSR; // RCC clock control & status
	uint32_t RESERVED_6[2];
	__vo uint32_t SSCGR; // RCC spread spectrum clock generation
	__vo uint32_t PLLI2SCFGR; // RCC PLLI2S configuration
	__vo uint32_t PLLSAICFGR; // RCC PLL configuration
	__vo uint32_t DCKCFGR; // RCC dedicated clock config
	__vo uint32_t CKGATENR; // RCC clocks gated enable
	__vo uint32_t DCKCFGR2; // RCC dedicated clock config 2
} RCC_reg_def_t;


// CLOCK ENABLE MACROS

// Clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))

// Clock enable macros for I2C peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

// Clock enable macros for SPI peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB2ENR |= (1 << 15))

// Clock enable macros for USART peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))

// Clock enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


// CLOCK DISABLE MACROS

// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))

// Clock disable macros for I2C peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

// Clock disable macros for SPI peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB2ENR &= ~(1 << 15))

// Clock disable macros for USART peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))

// Clock disable macros for SYSCFG peripherals
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))


// Register-specific addresses
#define GPIOA ((GPIO_reg_def_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_reg_def_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_reg_def_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_reg_def_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_reg_def_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_reg_def_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_reg_def_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_reg_def_t*)GPIOH_BASE_ADDR)

#define RCC ((RCC_reg_def_t*)RCC_BASE_ADDR)

// Macros to reset GPIO peripherals
#define GPIOA_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

// Generic macros

#define ENABLE 1
#define DISABLE 0

#define SET ENABLE
#define RESET DISABLE

#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#endif /* INC_STM32F446XX_H_ */
