#include"ec20.h"

//雷达模块 PB10、PB11
//Radar Mod
void u3Conf(void)
{
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
  HAL_UART_Receive_DMA(&huart3, u3rx, u3mx);  
}
void u3rxProcess(void)
{
  unsigned short i=0;
  unsigned char flag=0;
  static unsigned char sflagCnt=0;
  if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET))//空闲中断
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UART_DMAStop(&huart3);
    flag=0;
    RxLenU3=u3mx-huart3.hdmarx->Instance->CNDTR;//接收到的字节长度
    for(i=0;i<RxLenU3;i++)
    {
      if(u3rx[i]<='9'&&u3rx[i]>='0')
      {
        flag=1;
      }
    }
     for(i=0;i<RxLenU3;i++)
    {
      u3rx[i]=0x00;
    }
    
    if(flag==1)
    {
        sflagCnt++;
    }else
    {
        sflagCnt=0;
        
    }
    if(sflagCnt>3)
    {
      sflagCnt=0;
      ready_2_warning = 1; 
      warning_ID = 0;
      warmSend[0]=1;
    }
    
    HAL_UART_Receive_DMA(&huart3, u3rx, u3mx); //再次开启DMA接收
  }
}