/**
 ******************************************************************************
 * @file     main.c
 * @author   正点原子团队(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    新建工程实验-HAL库版本 实验
 * @license  Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ******************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 STM32F103 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 ******************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

#include "./bsp/bsp_led.h" 
#include "./bsp/bsp_beep.h"   
#include "./bsp/bsp_key.h"
#include "./bsp/bsp_key_exti.h"
#include "./bsp/bsp_i2c_ee.h"
#include "./bsp/bsp_spi_flash.h"
#include "./FatFs/ff.h"
#include "./bsp/bsp_FatFs_test.h"
#include "./bsp/bsp_sram.h"  
#include "./bsp/bsp_ili9341_lcd.h"
#include "./bsp/bsp_xpt2046_lcd.h"
#include "./bsp/bsp_adc.h"
#include "./bsp/bsp_touchpad.h"
#include "./bsp/bsp_iwdg.h"  
#include "./bsp/bsp_wwdg.h"   
#include "./bsp/bsp_rtc.h"
#include "./bsp/bsp_internal_flash.h"  
#include "./bsp/sdio_test.h"


int main(void)
{
    /*系统初始化*/
    HAL_Init();                              /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9);      /* 设置时钟, 72Mhz */
    delay_init(72);                          /* 延时初始化 */
    usart_init(115200);                      /* 串口初始化为115200 */
    
    /*bsp初始化*/
    LED_GPIO_Config();
    BEEP_GPIO_Config();
    //Key_GPIO_Config();     
    EXTI_Key_Config();
    I2C_EE_Init();           /* I2C 外设初(AT24C02)始化 */
    SPI_FLASH_Init();        /* 8M串行flash W25Q64初始化 */
    FSMC_SRAM_Init();        /* 初始化外部SRAM */     
    ILI9341_Init ();         /* LCD 初始化 */
    XPT2046_Init();          /* 触摸屏初始化 */
    ADCx_Init();             /* adc初始化 */
    //TPAD_Init();             /* 初始化电容按键获取未被触摸时参数 */
    //IWDG_Config(IWDG_PRESCALER_64 ,625);    /* IWDG 1s 超时溢出 */
    //WWDG_Config(0x7F,0X5F,WWDG_PRESCALER_8);   /* 初始化WWDG：配置计数器初始值，配置上窗口值，启动WWDG，使能提前唤醒中断 */
    RTC_CLK_Config();        /* RTC配置：选择时钟源，设置RTC_CLK的分频系数 */
    
    
    /*测试函数*/
    //I2C_Test();
    //SPI_Test();
    //FatFs_Test();
    //my_SRAM_Test();
    //LCD_Test();         //LCD测试      放到while(1)中测试 需禁用看门狗
    //Touch_Lcd_Test();   //触摸屏测试   放到while(1)中测试 需禁用看门狗
    //ADC_Test();         //adc测试      放到while(1)中测试 需禁用看门狗
    //Touch_key_Test();   //触摸按键测试 放到while(1)中测试 需禁用看门狗  
    //Rtc_Test();
    //Internal_Flash_Test();
    //SD_Test();
    
    
    printf("\r\nmain while(1) start\r\n");
    while(1)
    { 
        
        LED_RED_TOGGLE();
        delay_ms(500); 
        
        
//        HAL_IWDG_Refresh(&IWDG_Handle);                              //独立看门狗喂狗
//        uint16_t wwdg_count = get_wwdg_cout();                          
//        if(wwdg_count <= 0X5F) //需要在计数值为0x5F到0x40之间时喂狗  
//        {                                                           
//            WWDG_Feed();                                             //窗口看门狗喂狗
//        }
    }
}


