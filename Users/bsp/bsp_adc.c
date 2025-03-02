
/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ADC
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-�Ե� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "./bsp/bsp_adc.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

ADC_HandleTypeDef hadcx;
DMA_HandleTypeDef hdma_adcx;

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
   /* ����ʱ��ʹ�� */
    ADCx_RCC_CLK_ENABLE();
    
    /* DMAʱ��ʹ�� */
    DMAx_RCC_CLK_ENABLE();

    /* DMA�����ʼ������ */  
    hdma_adcx.Instance = ADC_DMAx_CHANNELn;
    hdma_adcx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adcx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adcx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adcx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adcx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adcx.Init.Mode = DMA_CIRCULAR;
    hdma_adcx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adcx);
    /* ����DMA */
    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adcx);
}

void ADCx_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  // ADC��������
  hadcx.Instance = ADCx;
  hadcx.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadcx.Init.ContinuousConvMode = ENABLE;
  hadcx.Init.DiscontinuousConvMode = DISABLE;
  hadcx.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcx.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadcx);

  // ���ò���ͨ��
  sConfig.Channel = ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
}



#define V25  0x6EE                  //����12λ��ADC��3.3V��ADCֵΪ0xfff,�¶�Ϊ25��ʱ��Ӧ�ĵ�ѹֵΪ1.43V��0x6EE
#define AVG_SLOPE 0x05              //б�� ÿ���϶�4.3mV ��Ӧÿ���϶�0x05
__IO uint16_t Current_Temperature;  // ���ڱ���ת�������ĵ�ѹֵ  
uint16_t ADC_ConvertedValue;        // ADת�����ֵ
void ADC_Test(void)
{
    uint8_t print_flag = 0;
    if(print_flag == 0)
    {
        printf("\r\n ����һ���ڲ��¶ȴ�����ʵ�� \r\n");
        printf( "\r\n Print current Temperature  \r\n");	
        HAL_ADCEx_Calibration_Start(&hadcx);
        HAL_ADC_Start_DMA(&hadcx,(uint32_t *)&ADC_ConvertedValue,sizeof(ADC_ConvertedValue)); 
	}
    /* ����ѭ�� */
    while (1)
    {
        delay_ms(5);

        Current_Temperature = (V25-ADC_ConvertedValue)/AVG_SLOPE+25;	
        printf("The IC current temp = %3d ��\n",Current_Temperature);
    }
}

