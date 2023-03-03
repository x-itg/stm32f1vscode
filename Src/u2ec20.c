#include"ec20.h"

//串口初始化EC20连着uart2  PA2、PA3
void u2Conf(void)
{
  HAL_UART_Receive_DMA(&huart2, RxBufferDMA, DMARXBUFFERSIZE);  
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); 
}

//这个函数放串口接收空闲中断中
void u2rxitProcess(void)
{
  if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!=RESET))//空闲中断
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    HAL_UART_DMAStop(&huart2);
    DMARxLenU2=DMARXBUFFERSIZE-huart2.hdmarx->Instance->CNDTR;//NDTR;//接收到的字节长度
    HAL_UART_DMAStop(&huart2);
    rsPackFlag=1;//正在接收
    rsRxTime=0;
    unsigned short i=rsRxIndexLen;//继续收集
    for(i=rsRxIndexLen;i<rsRxIndexLen+DMARxLenU2;i++)
    {
      rsRxBuf[i]=RxBufferDMA[i-rsRxIndexLen];
    }
    rsRxIndexLen=rsRxIndexLen+DMARxLenU2;//长度
  }
  
}


//EC20字符串发送
void SendTxBuf( unsigned char *p)
{
  unsigned short len=strlen((char const *)p);
  HAL_UART_Transmit(&huart2,p,len,100);
}