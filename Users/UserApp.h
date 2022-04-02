/**
 * @file Untitled-1
 * @brief 
 * @author yangskyhigh (yangskyhigh@163.com)
 * @version 1.0
 * @date 2020-07-14
 * @copyright Copyright (c) 2020  CAS 
 */
#ifndef __UserAPP_H_
#define __UserAPP_H_


/* Includes ------------------------------------------------------------------*/



/* Public define --------------------------------------------------------—----*/
#define SRAM_User_Start   0x2001B100

// ADC1 ，4 通道 ，采样率160 Hz
// 每16Hz处理一次
#define ADC1_Chs 4
#define ADC1_Cycles 8
#define SRAM_ADC1_RawBuf_Size     (2*ADC1_Chs*ADC1_Cycles)     // 2bytes * 4chs * 10
#define SRAM_ADC1_RawBuf_Start_A  SRAM_User_Start
#define SRAM_ADC1_RawBuf_Start_B  (SRAM_ADC1_RawBuf_Start_A+SRAM_ADC1_RawBuf_Size)

// ADC2 ，3 通道 ，采样率160 Hz
// 每16Hz处理一次
#define ADC2_Chs 3
#define ADC2_Cycles 8            //
#define SRAM_ADC2_RawBuf_Size     (2*ADC2_Chs*ADC2_Cycles)     // 2bytes * 3chs * 10
#define SRAM_ADC2_RawBuf_Start_A  (SRAM_ADC1_RawBuf_Start_B+SRAM_ADC1_RawBuf_Size)
#define SRAM_ADC2_RawBuf_Start_B  (SRAM_ADC2_RawBuf_Start_A+SRAM_ADC2_RawBuf_Size)

// ADC4 ，2 通道 ，采样率4 Hz
// 每1Hz处理一次
#define ADC4_Chs 2
#define ADC4_Cycles 4
#define SRAM_ADC4_RawBuf_Size     (2*ADC4_Chs*ADC4_Cycles)     // 2bytes * 2chs * 4
#define SRAM_ADC4_RawBuf_Start_A  (SRAM_ADC2_RawBuf_Start_B+SRAM_ADC2_RawBuf_Size)
#define SRAM_ADC4_RawBuf_Start_B  (SRAM_ADC4_RawBuf_Start_A+SRAM_ADC4_RawBuf_Size)


/* Typedef ----------------------------------------------------------————————-*/


/* Public macro -------------------------------------------------------------*/


/* Public variables ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

extern void Creat_user_app_init_task(void);
           


#endif	 // __UserAPP_H_ 

/*********** (C) COPYRIGHT 2020 skyhigh *****END OF FILE****/
