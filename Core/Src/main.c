/**
 *******************************************************************************
 * @file  main.c
 * @brief Main program template.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f460.h"
#include "hc32_ll.h"
#include "hc32_ll_gpio.h"

#include "arm_math.h"
/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/**
 * @brief ICG parameters configuration
 */
/* The ICG area filled with default value, Please modify this value required */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
const uint32_t u32ICG[] __attribute__((section(".ARM.__at_0x400"))) =
#elif defined(__GNUC__) && !defined(__CC_ARM)
const uint32_t u32ICG[] __attribute__((section(".icg_sec"))) =
#elif defined(__CC_ARM)
const uint32_t u32ICG[] __attribute__((at(0x400))) =
#elif defined(__ICCARM__)
#pragma location = 0x400
__root static const uint32_t u32ICG[] =
#else
#error "unsupported compiler!!"
#endif
    {
        0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
        0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,

    };
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/


extern void BSP_CLK_Init(void);

volatile float hello2 = 10.0f;
/**
 * @brief  Main function of template project
 * @param  None
 * @retval int32_t return value, if needed
 */
int main(void) {
  /* Register write unprotected for some required peripherals. */
  LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO |
               LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);
  
  BSP_CLK_Init();

  stc_gpio_init_t stcGpioInit;
  GPIO_StructInit(&stcGpioInit);
  stcGpioInit.u16PinState = PIN_STAT_RST;
  stcGpioInit.u16PinDir = PIN_DIR_OUT;
  GPIO_Init(GPIO_PORT_A, GPIO_PIN_00, &stcGpioInit);
  GPIO_Init(GPIO_PORT_A, GPIO_PIN_01, &stcGpioInit);
  GPIO_Init(GPIO_PORT_A, GPIO_PIN_02, &stcGpioInit);


  GPIO_Init(GPIO_PORT_B, GPIO_PIN_15, &stcGpioInit);

  
  /* Register write protected for some required peripherals. */
  LL_PERIPH_WP(LL_PERIPH_EFM | LL_PERIPH_GPIO | LL_PERIPH_SRAM);
  volatile float hello = 0.0f;
  /* Add your code here */
  while (1) {
    // hello += 0.1f;
    // hello2 *= hello;
    // hello += arm_sin_f32(hello2);
    // GPIO_TogglePins(GPIO_PORT_A, GPIO_PIN_00);
    // DDL_DelayMS(1000);
    // GPIO_TogglePins(GPIO_PORT_A, GPIO_PIN_01);
    // DDL_DelayMS(1000);
    // GPIO_TogglePins(GPIO_PORT_A, GPIO_PIN_02);
    // DDL_DelayMS(1000);
    GPIO_TogglePins(GPIO_PORT_B, GPIO_PIN_15);
  }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
