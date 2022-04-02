/**
 * @file Untitled-1
 * @brief 
 * @author yangskyhigh (yangskyhigh@163.com)
 * @version 1.0
 * @date 2020-07-14
 * @copyright Copyright (c) 2020  CAS 
 ***************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2019-11-17 <td>1.0     <td>wangh     <td>内容
 * </table>
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "UserApp.h"

/* Private define ------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
//extern ADC_HandleTypeDef hadc4;

extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

//extern DAC_HandleTypeDef hdac4;

// stack size must be multiple of 8 Bytes
#define USER_APP_Init_STK_SZ (2048U)
uint64_t user_app_init_stk[USER_APP_Init_STK_SZ / 8];
const osThreadAttr_t user_app_init_attr = {
    .stack_mem = &user_app_init_stk[0],
    .stack_size = sizeof(user_app_init_stk)};

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
int16_t ADC2_RawResult[3][2];
int16_t ADC2_CH1RawResult[20];
int16_t ADC2_CH2RawResult[20];
int16_t ADC2_CH3RawResult[20];



/**
 * @brief  ADC缓冲区存满后处理,双缓冲区中的 M0 存满
 * @param  hadc  hdc指针
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        EventRecord2(0x03, 0x01, 0x01); // id, val1 ,val2
    }
    else if (hadc == &hadc2)
    {
       // Process_Mag((uint16_t *)(SRAM_ADC2_RawBuf_Start_A));
        EventRecord2(0x03, 0x01, 0x02); // id, val1 ,val2
    }
   
}

/**
 * @brief  ADC缓冲区存满后处理,双缓冲区中的 M1存满
 * @param  hadc    hdc指针
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        EventRecord2(0x04, 0x01, 0x01); // id, val1 ,val2
    }
    else if (hadc == &hadc2)
    {
     //   Process_Mag((uint16_t *)(SRAM_ADC2_RawBuf_Start_A + ADC2_Chs * ADC2_Cycles));
        EventRecord2(0x04, 0x01, 0x02); // id, val1 ,val2
    }
   
}

/* Public functions ---------------------------------------------------------*/
__NO_RETURN void user_app_init(void *arg)
{
    char *sp, *cp, *next;
    uint32_t i;

    unsigned char bytes[20];
    unsigned short Addr;

    (void)arg;

    // 		Storage_Init("M0:");

    // 	  Get_ParaInf();

    osDelay(100);

    //	Init_RTC();
    //
    //    Storage_Init("N0");

    //    init_DigSensor_ringbuffer();
    //    User_UART_Start(Huart3_Index);

    //osDelay(500);

 

    //    if (Get_Reset_Cfg() == Reset_Type_HighSpeed)
    //    {
    //        USBD_Initialize(0U); // MSC
    //        USBD_Connect(0U);

    //        Set_Reset_Cfg(Reset_Type_Normal);
    //    }

    //   Creat_user_app_CMD_task();
    //     HAL_ADC_Start_DMA(&hadc1, (uint32_t *)SRAM_ADC1_RawBuf_Start_A,  SRAM_ADC1_RawBuf_Size);
    //    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)SRAM_ADC2_RawBuf_Start_A,  SRAM_ADC2_RawBuf_Size); // 第3个参数为记录条数，不是数据字节数
    //   HAL_ADC_Start_DMA(&hadc4, (uint32_t *)SRAM_ADC4_RawBuf_Start_A,  SRAM_ADC4_RawBuf_Size);

   

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)SRAM_ADC2_RawBuf_Start_A, ADC2_Chs * ADC2_Cycles); // 第3个参数为记录条数，不是数据字节数

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim3);
    //HAL_TIM_Base_Start(&htim2);
   

    while (1)
    {
       
        LED_G_FLASH;
        osDelay(500);

        //EventStartA(0);
        LED_R_FLASH;
        osDelay(500);

        //EventStopA(0);
        //        Get_RTC_DDHHMMSS(rtc_val);
        //        // Uart3_Write(rtc_val, 6);

        //        Storage_SetfsDateTime(rtc_val);
    }
}

void Creat_user_app_init_task(void)
{
    osThreadNew(user_app_init, NULL, &user_app_init_attr);
}

/*********** (C) COPYRIGHT 2020  skyhigh  *****END OF FILE****/
