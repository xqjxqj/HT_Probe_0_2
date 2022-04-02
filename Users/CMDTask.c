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

#include "ringbuffer.h"

/* Private define ------------------------------------------------------------*/

// stack size must be multiple of 8 Bytes
#define USER_APP_CMD_STK_SZ (2048U)
uint64_t user_app_CMD_stk[USER_APP_CMD_STK_SZ / 8];
const osThreadAttr_t user_app_CMD_attr = {
    .stack_mem = &user_app_CMD_stk[0],
    .stack_size = sizeof(user_app_CMD_stk)};

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t cmd_bytes[256];
struct rt_ringbuffer rt_cmd;

/* Public variables ---------------------------------------------------------*/

uint8_t NewCMDPack = 0; // 收到总线上新的数据包



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


// 串口发送数据
void Com_Send_Data(unsigned char *pSrc, unsigned short Size)
{
#if OnebusCOM2
   Uart2_Write(pSrc, Size);
#else
   Uart4_Write(pSrc, Size);
#endif
}

void init_cmd_ringbuffer(void)
{
   rt_ringbuffer_init(&rt_cmd, cmd_bytes, sizeof(cmd_bytes));
}


static uint8_t rt_cmd_buf_remain_bytes=0; // 0=empty,1=not empty

// 处理总线包数据
uint8_t Recvive_Cmd_Bus(void)
{
	unsigned short i;
   unsigned char tmpC;
   unsigned short size;

   size = rt_ringbuffer_data_len(&rt_cmd);

	if(size>0)
	{
	   for (i = 0; i < size; i++)
	   {
		  if (rt_ringbuffer_getchar(&rt_cmd, &tmpC) > 0)
		  {
			 // ComPackState = PacketAna_P(tmpC, &ComPacket);
			 if (ComPackState.PacketCpltFlag == FLAG_TRUE) // 收到了一包
			 {
				NewCMDPack = 1;
				 
				 goto judge_rt_cmd_buf_empty;
			 }
		  }
	   }
	   
	    
	}
	 
}


/**
 * @brief  总线通信主循环
 * @param  arg              My Param doc
 * @return __NO_RETURN 
 */
__NO_RETURN void user_app_CMD(void *arg)
{
    
	unsigned char tmpC;

	
   (void)arg;

   init_cmd_ringbuffer();

#if OnebusCOM2
   User_UART_Start(Huart2_Index);
#else
   User_UART_Start(Huart4_Index);
#endif

   while (1)
   {

#if 1
		   
	   
      if (NewCMDPack)
      {
         NewCMDPack = 0;

         //-----  处理收到了一个数据包要干的事情
      }

       
#endif

	 
	 
      osDelay(10);
   }
}

/* Public functions ---------------------------------------------------------*/

/**
 * @brief  收到总线数据后处理 
 * @param  pSrc             My Param doc
 * @param  size             My Param doc
 */
void User_UART_RxCpltCallback_2(uint8_t *pSrc, uint16_t size)
{
   rt_ringbuffer_put(&rt_cmd, pSrc, size);
}

void User_UART_IDLECallback_2(void)
{
 
 
	Recvive_Cmd_Bus();
 
}

/**
 * @brief  收到总线数据后处理 
 * @param  pSrc             My Param doc
 * @param  size             My Param doc
 */
void User_UART_RxCpltCallback_4(uint8_t *pSrc, uint16_t size)
{
   rt_ringbuffer_put(&rt_cmd, pSrc, size);
}

void User_UART_IDLECallback_4(void)
{
	 
	Recvive_Cmd_Bus();
	 
}

void Creat_user_app_CMD_task(void)
{
   osThreadNew(user_app_CMD, NULL, &user_app_CMD_attr);
}

/*********** (C) COPYRIGHT 2020  skyhigh  *****END OF FILE****/
