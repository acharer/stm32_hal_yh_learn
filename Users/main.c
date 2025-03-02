/**
 ******************************************************************************
 * @file     main.c
 * @author   ����ԭ���Ŷ�(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    �½�����ʵ��-HAL��汾 ʵ��
 * @license  Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ******************************************************************************
 * @attention
 * 
 * ʵ��ƽ̨:����ԭ�� STM32F103 ������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
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
    /*ϵͳ��ʼ��*/
    HAL_Init();                              /* ��ʼ��HAL�� */
    sys_stm32_clock_init(RCC_PLL_MUL9);      /* ����ʱ��, 72Mhz */
    delay_init(72);                          /* ��ʱ��ʼ�� */
    usart_init(115200);                      /* ���ڳ�ʼ��Ϊ115200 */
    
    /*bsp��ʼ��*/
    LED_GPIO_Config();
    BEEP_GPIO_Config();
    //Key_GPIO_Config();     
    EXTI_Key_Config();
    I2C_EE_Init();           /* I2C �����(AT24C02)ʼ�� */
    SPI_FLASH_Init();        /* 8M����flash W25Q64��ʼ�� */
    FSMC_SRAM_Init();        /* ��ʼ���ⲿSRAM */     
    ILI9341_Init ();         /* LCD ��ʼ�� */
    XPT2046_Init();          /* ��������ʼ�� */
    ADCx_Init();             /* adc��ʼ�� */
    //TPAD_Init();             /* ��ʼ�����ݰ�����ȡδ������ʱ���� */
    //IWDG_Config(IWDG_PRESCALER_64 ,625);    /* IWDG 1s ��ʱ��� */
    //WWDG_Config(0x7F,0X5F,WWDG_PRESCALER_8);   /* ��ʼ��WWDG�����ü�������ʼֵ�������ϴ���ֵ������WWDG��ʹ����ǰ�����ж� */
    RTC_CLK_Config();        /* RTC���ã�ѡ��ʱ��Դ������RTC_CLK�ķ�Ƶϵ�� */
    
    
    /*���Ժ���*/
    //I2C_Test();
    //SPI_Test();
    //FatFs_Test();
    //my_SRAM_Test();
    //LCD_Test();         //LCD����      �ŵ�while(1)�в��� ����ÿ��Ź�
    //Touch_Lcd_Test();   //����������   �ŵ�while(1)�в��� ����ÿ��Ź�
    //ADC_Test();         //adc����      �ŵ�while(1)�в��� ����ÿ��Ź�
    //Touch_key_Test();   //������������ �ŵ�while(1)�в��� ����ÿ��Ź�  
    //Rtc_Test();
    //Internal_Flash_Test();
    //SD_Test();
    
    
    printf("\r\nmain while(1) start\r\n");
    while(1)
    { 
        
        LED_RED_TOGGLE();
        delay_ms(500); 
        
        
//        HAL_IWDG_Refresh(&IWDG_Handle);                              //�������Ź�ι��
//        uint16_t wwdg_count = get_wwdg_cout();                          
//        if(wwdg_count <= 0X5F) //��Ҫ�ڼ���ֵΪ0x5F��0x40֮��ʱι��  
//        {                                                           
//            WWDG_Feed();                                             //���ڿ��Ź�ι��
//        }
    }
}


