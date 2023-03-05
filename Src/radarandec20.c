#include "EC20.h"
#include "oled.h"
#pragma pack(1)

unsigned int CpuID[3];
unsigned int SendID;
unsigned char GpsRecFlag=0;
GPS_DATA gspdata;
GSM_init GSMinit;
unsigned char CloudtriggerFlag=0;
unsigned int  CloudtriggerTime=0;
//EC20
unsigned short PollTimeCnt=0;                  //在毫秒中断中加加
unsigned char RxBufferDMA[DMARXBUFFERSIZE];    // DMA接收到的缓存
unsigned short DMARxLenU2=0;                   //DMA缓存接收长度
unsigned char rsRxBuf[LASTSZIE];              //最后的数据
unsigned int rsRxIndexLen=0;                 //这个长度会在DMA空闲中断中更新
unsigned int rsRxLen=0;                      //最终长度
unsigned int rsRxTime=0;                     //超时时间计数
unsigned char rsPackFlag=0;                   //正在接收的标记
unsigned char rsRecFlag=0;                    //接收标志
unsigned char ProSta=0;
unsigned char linkflagSta=0;
unsigned char CurStaSendCnt=0;
unsigned int SendCntRecClear=0; 
unsigned int SendGPSCnt=0;
unsigned char GPSStatFlag=0;
Ec20SendData ec20send,*pec20;
Ec20HEART ec20heart,*pech20;
//radar
unsigned char u3rx[u3mx];
unsigned short RxLenU3;

unsigned char last_Res_6=0;
unsigned char radar_ID =0;
unsigned char frame_num = 0;
unsigned char warning_cfg = 0; 
unsigned int msint=0;
char trigger_order = 0x01;
unsigned char lifetime_count[4];
unsigned char danger_keep = 0;
unsigned char ready_2_warning=0; 
unsigned char waringtimeflag=0;
unsigned char waringspeedflag=0;
unsigned int waringtime=0;
unsigned char warning_ID=0;
unsigned short ms10=0;            

unsigned char warmSend[4]={0,0,0,0};
unsigned char sendbuf[255];
unsigned char warmnobeep=0;
unsigned char linkflag=0;
unsigned char sendingflag=0;
unsigned int simCnt=0;
const unsigned char *psend;
GPIO_PinState readsxtioflag=GPIO_PIN_RESET;//摄像头触发状态
unsigned int POLLTime=NORMARLTIME;

#define InitEC2Index                            0       //从头开始初始化
#define InitTCPIndex                            6       //从连接TCP开始初始化
#define ATCodeSnd                               11      //发送TCP内容的那条AT指令序号
#define ATTcpReadIndex                          12      //TCP读取的AT指令序号序号
#define ATCodeLastNum                           12      //最后一条读取GPS数据的AT指令序号  15
#define ATCodeNum                               12      //整个流程AT指令条数 16
#define SendButNoRecCnt                         50      //发送但是EC20没有响应
#define SendRecButNotExpCnt                     10      //发送EC20有响应但是接收到数据跟预期内容不符合
#define SendRecButGpsNotExp                     3       //gps数据不在预期范围的计数
//发送列表
const unsigned char   *SendList[] = {
  "AT\r\n\0",                                           //0       
  "ATE0\r\n\0",                                         //1  回显关闭
  "AT+CPIN?\r\n\0",                                     //2 读卡测试
  "AT+CSQ\r\n\0",                                       //3 信号质量查询
  "AT+CREG?\r\n\0",                                     //4 网络注册查询
  "AT+CGATT?\r\n\0",                                    //5 附着网络查询
  "AT+QIACT=1\r\n",                                     //6 激活移动场景 激活 context profile
  "AT+QIACT?\r\n\0",                                    //7 IP地址查询   QIACT: 1,1,1,"10.23.163.28"  第一个1context号（1-16）  0-1连接号 1
  "AT+QIDEACT=1\r\n\0",                                 //8  Deactivate a PDP Context
  "AT+QICLOSE=0,2000\r\n\0",                            //9 关闭connect 0 不管怎样先关闭一下
#if FacIP==0
  "AT+QIOPEN=1,0,\"TCP\",\"139.198.162.95\",999\r\n\0",    //10 连接context 1号   connect0号 TCP ip及端口47.103.104.119   47.97.25.70
#else
  "AT+QIOPEN=1,0,\"TCP\",\"47.103.104.120\",999\r\n\0", //10 连接context 1号   connect0号 TCP ip及端口47.103.104.119   47.97.25.70
#endif
  "AT+QISENDEX=0,\"313233343536373839\"\r\n\0",         //11 发送数据
  "AT+QIRD=0,1500\r\n\0",                               //12  接收
  "AT+QGPSCFG=\"outport\",\"uartdebug\"\r\n\0",         //13  设置gnss的端口----------------------------------------
  "AT+QGPS=1\r\n\0",                                    //14开启GNNS功能   使用串口发送 
  "AT+QGPSLOC=0\r\n\0",                                 //15读取位置信息发送
  
};


//接收期待列表
const unsigned char   *RecExpected [] = {
  "OK",         //0     OK
  "OK",         //1     OK
  "READY",      //2     +CPIN: READY   如果两次ERROR
  "CSQ",        //3     +CSQ: 28,0   
  "0,1",         //4     +CREG: 0,1   0,5   1,1  1,5 这些都可以
  "1",          //5     +CGATT: 1
  "O",          //6     OK 或者 ERROR 重复激活为ERROR
  "OK",         //7     OK
  "OK",         //8     OK
  "\r",         //9     OK
  "\r",         //10    
  "OK",         //11    发送
  "+QIRD",      //12    接收
  "\r",         //13     
  "\r",         //14
  "QGPSLOC",        //15   
  "ERROR"
};

//清除缓存
void Clear_Buffer(void)//清空缓存
{
  int i;
  for(i=0;i<LASTSZIE;i++)
  {
    rsRxBuf[i]=0;//缓存
  }
  rsRxLen=0;
  
}
//全部相同返回1
unsigned char ucharCmp(unsigned char *p1,unsigned char *p2,unsigned char len)
{
  unsigned char i;
  for(i=0;i<len;i++)
  {
    if((*p1++)!=(*p2++))
    {
      return 0;
    }
  }
  return 1;//全部相等
}

//这部分完全包含在整个中就返回1 不然返回零
unsigned char findStrCmp(unsigned char *allPstr,unsigned char alllen,unsigned char *partPstr, unsigned char partlen)
{
  unsigned char i=0;
  for(i=0;i<alllen-partlen+1;i++)
  {
    if(ucharCmp(allPstr+i,partPstr,partlen)==1)
    {
      return 1;//找到完全相同
    }
  }
  return 0;
}
               
//AT发送TCP数据时候需要编码一下
void HexStrConvert(unsigned char *Inp,unsigned char len,unsigned char *Outp)
{
  unsigned char hexH=0;
  unsigned char hexL=0;
  unsigned char i=0;
  
  for(i=0;i<len;i++)
  {
    hexH=(*Inp)/16;
    hexL=(*Inp)%16;
    Inp++;
    if(hexH>=10)hexH=hexH-10+'A';
    else hexH=hexH+'0';
    if(hexL>=10)hexL=hexL-10+'A';
    else hexL=hexL+'0';
    *Outp++=hexH;
    *Outp++=hexL;
  }
}
//延时 这个函数需要把编译优化改成NONE才行
void delay_ms(unsigned int n)
{
  msint=0;
  while(n>msint);
}

//开关EC20的电源连着三极管控制开关电源 
void ec20power(unsigned char onoffflag)
{
  rstHig;
  powerHig;
  
  if(onoffflag)//开
  {
    powerLow;
    delay_oledms(500);
    rstLow;
    
  }else
  {
    rstHig;
    powerHig;
  }
}
  char txb[20];

//获取唯一ID
unsigned int GetIDCode(void)
{
  //获取CPU唯一ID
#if 0//stm32f407
  CpuID[0]=*(unsigned int *)(0x1fff7a10);
  CpuID[1]=*(unsigned int *)(0x1fff7a14);
  CpuID[2]=*(unsigned int *)(0x1fff7a18);
#endif
#if 1//stm32f103
  CpuID[0]=*(unsigned int *)(0x1FFFF7E8);
  CpuID[1]=*(unsigned int *)(0x1FFFF7EC);
  CpuID[2]=*(unsigned int *)(0x1FFFF7F0);
#endif
  SendID=(CpuID[0]>>1)+(CpuID[1]>>2)+(CpuID[2]>>3);
//  SendID=0x15A0;
 SendID=0x15B1;
// SendID=0x15B2;
//  SendID=0x15C0;
//  SendID=0x15D1;
//  SendID=0x15D2;
  sprintf(txb,"id:%x\r\n\0",SendID);
  SendTxDebug((unsigned char *)txb);

  return SendID;
}
void DataSendPre(void)
{
  
  unsigned short i=0;
  unsigned char  cnt=0;
  static unsigned int  indx=0;
  if(waringtime>0)
  {
    pec20=&ec20send;
    ec20send.Len=sizeof(Ec20SendData);
    ec20send.ID=GetIDCode();
    ec20send.Send_Index=indx++;
    ec20send.Radar1_Cnt=lifetime_count[0];
    ec20send.Radar2_Cnt=lifetime_count[1];
    ec20send.Radar3_Cnt=lifetime_count[2];
    ec20send.Radar4_Cnt=lifetime_count[3];
    ec20send.Radar1_Warm=warmSend[0];//报警
    ec20send.Radar2_Warm=warmSend[1];
    ec20send.Radar3_Warm=warmSend[2];
    ec20send.Radar4_Warm=warmSend[3];
    if(ec20send.Radar1_Warm==1)cnt++;
    if(ec20send.Radar2_Warm==1)cnt++;
    if(ec20send.Radar3_Warm==1)cnt++;
    if(ec20send.Radar4_Warm==1)cnt++;
    ec20send.Alarm_Cnt=cnt; 
    ec20send.Alarm_Enable=warmnobeep;
    ec20send.GPS_State=GPSStatFlag;
    if(GpsRecFlag==0)//未曾接受到gps数据
    {
      memcpy((char *)&(pec20->Time[0]),"000000.000",10);
      memcpy((char *)&(pec20->Latitude[0]),"0000.0000N",10);
      memcpy((char *)&(pec20->longitude[0]),"00000.0000E",11);
    }else
    {
      GpsRecFlag=0;
    }
    sprintf((char *)&(pec20->Latitude[0]),"2517.2831N");
    sprintf((char *)&(pec20->longitude[0]),"117.242314E");
    ec20send.End=0xffff0000;
    i=0;
    sprintf((char *)&sendbuf[i],"AT+QISENDEX=0,\"\0");//AT+QISENDEX=0,"
    i=i+strlen("AT+QISENDEX=0,\"\0");
    HexStrConvert((unsigned char *)&ec20send,sizeof(Ec20SendData),&sendbuf[i]);
    i=i+sizeof(Ec20SendData)*2;
    sprintf((char *)&sendbuf[i],"\"\r\n\0");//3字节
    HAL_UART_Transmit(&huart2,sendbuf,2*sizeof(Ec20SendData)+18,800);//69*2+18=156一共发送156字节
          
  }else
  {
    ec20heart.Len=sizeof(Ec20HEART);
    ec20heart.ID=GetIDCode();
    i=0;
    sprintf((char *)&sendbuf[i],"AT+QISENDEX=0,\"\0");//AT+QISENDEX=0,"
    i=i+strlen("AT+QISENDEX=0,\"\0");
    HexStrConvert((unsigned char *)&ec20heart,sizeof(Ec20HEART),&sendbuf[i]);
    i=i+sizeof(Ec20HEART)*2;
    sprintf((char *)&sendbuf[i],"\"\r\n\0");//3字节
    HAL_UART_Transmit(&huart2,sendbuf,2*sizeof(Ec20HEART)+18,800);//69*2+18=156一共发送156字节
  }
  
}
unsigned char sendedflag=0;
unsigned char warmonceflag=0;
//发送进程sendingflag=0;//发送好了
//radar
void ec20ProcessFun(void)
{
 
  char numstr[3];
  unsigned short i=0;
  //接收
  if(rsPackFlag==1&&rsRxTime>50)//断包 防止有些上位机软件发送数据不连续
  {
    rsPackFlag=0;
    rsRxTime=0;
    rsRxLen=rsRxIndexLen;
    rsRxBuf[rsRxLen]=0;//为了打印时候保险起见手动在末尾追加一个零
    rsRxIndexLen=0;
    rsRecFlag=1;
    
    //COM_Get_Callback(GSMinit.status,rsRxBuf,rsRxLen);//状态、接收数据、接收数据的长度
    HAL_UART_Receive_DMA(&huart2, RxBufferDMA, DMARXBUFFERSIZE); //再次开启DMA接收
  }
  
  if(ProSta<InitTCPIndex)
  {
    POLLTime=ALARMTIME;//未连接TCP前可以以报警速度进行
  }else
  {
    
    if(warmonceflag==0)//从未报警
    {
       POLLTime=NORMARLTIME;
    }
    if(waringspeedflag==1&&sendedflag==1)//有过报警 且已经发送恢复慢速
    {
      sendedflag=0;
      waringspeedflag=0;
      POLLTime=NORMARLTIME;
    }
    
  }
  if(PollTimeCnt>POLLTime)//0.5秒执行一次
  {
    PollTimeCnt=0;
    //应答计数  识别断线
    if(SendCntRecClear>SendButNoRecCnt)//发送计数  收到任何东西都清除
    {
      ProSta=InitEC2Index;
      SendCntRecClear=0;
      GPSStatFlag=0;
      SendGPSCnt=0;
      CurStaSendCnt=0;
      ec20power(1);//复位一下
      linkflag=0;
#if DEBUGMODE==0      
      OLED_ShowString(0,0,"                  ",16);
      PrintHZ(0,0,"RST ERROR             ",0,1,0);
#else
      SendTxDebug((unsigned char *)"EC20非预期错误\0");
#endif
    }
    //发送 有接收 但非预期的计数 识别断线
    if(CurStaSendCnt>SendRecButNotExpCnt&&ProSta!=ATCodeLastNum)
    {
      ProSta=InitEC2Index;
      CurStaSendCnt=0;
      SendCntRecClear=0;
      linkflag=0;
#if DEBUGMODE==0   
      OLED_ShowString(0,0,"                  ",16);
      PrintHZ(0,0,"EXP ERROR             ",0,1,0);
#else
      SendTxDebug((unsigned char *)"EC20非预期错误\0");
#endif
    }
    
    //最后一条发送 有接收 但非预期的计数据
    if(SendGPSCnt>SendRecButGpsNotExp&&ProSta==ATCodeLastNum)
    {
      ProSta= InitTCPIndex;//重新开始
      CurStaSendCnt=0;
      SendGPSCnt=0;
      GPSStatFlag=0;//GPS未正常工作
    }
    
    if(ProSta<=ATCodeNum)
    {
      if(rsRecFlag==0)//发送
      {

        if(ProSta==ATCodeSnd)//发送的那条信息
        {
          
          DataSendPre();
          
        }else
        {
          SendTxBuf((unsigned char *)SendList[ProSta]);
          psend=SendList[ProSta];
        }
        
        if(ProSta==ATCodeLastNum)
        {
          SendGPSCnt++;//读取GPS计数
        }
        SendCntRecClear++;
        sprintf((char *)numstr,"%02d\0",ProSta);
#if DEBUGMODE==0           
        OLED_ShowString(0,0,( char *)numstr,2);
         
#else
        SendTxDebug((unsigned char *)numstr);
        if(ProSta==ATCodeSnd)//发送的那条信息
        {
          SendTxDebug(sendbuf);
        }else
        {
          SendTxDebug((unsigned char *)SendList[ProSta]);
        }
        
#endif 
        
        if(ProSta==6)
        {
#if DEBUGMODE==0
          OLED_ShowString(0,0,"                  ",16);
          PrintHZ(0,0,"正在连接               ",0,1,0);
#else
          SendTxDebug((unsigned char *)"正在连接\0");
#endif
        } 
        
        //togLEDstatus;
        CurStaSendCnt++;//收到预期会清零
        
        
      }else//接收内容的识别处理
      {
        rsRecFlag=0;
        SendCntRecClear=0;
        sprintf((char *)numstr,"%02d\0",ProSta);
#if DEBUGMODE==0        
        OLED_ShowString(0,0,"                   \0",8);
        OLED_ShowString(0,2,"                   \0",8);
        OLED_ShowString(0,0,(char *)numstr,2);
        OLED_ShowString(24,0,(char *)rsRxBuf,8);
#else
        SendTxDebug((unsigned char *)numstr);
        SendTxDebug((unsigned char *)rsRxBuf);
#endif
        
        if(ProSta==2&&strstr((const char*)rsRxBuf,(const char*)RecExpected[16])!=NULL)
        {
#if DEBUGMODE==0    
          OLED_ShowString(0,0,"                  ",16);
          PrintHZ(0,0,"SIM ERROR             ",0,1,0);
#else
          SendTxDebug((unsigned char *)"电话卡有问题\0");
#endif
          linkflag=0;
        }
        
        if(ProSta==12)//TCP数据返回
        {
          linkflag=1;
          if(waringtimeflag)
          {
              sendedflag=1;//报警已发送
          }
#if DEBUGMODE==0
          OLED_ShowString(0,0,"                  ",16);
          PrintHZ(0,0,"连接成功              ",0,1,0);
#else
          SendTxDebug((unsigned char *)"连接成功\0");
#endif
        }
        
        if(strstr((const char*)rsRxBuf,(const char*)RecExpected[ProSta])!=NULL||
           (ProSta==2&&strstr((const char*)rsRxBuf,(const char*)"1,1")!=NULL)||
           (ProSta==2&&strstr((const char*)rsRxBuf,(const char*)"1,5")!=NULL)||
           (ProSta==2&&strstr((const char*)rsRxBuf,(const char*)"0,5")!=NULL)||
           (ProSta==11&&strstr((const char*)rsRxBuf,(const char*)"recv")!=NULL)||
           (ProSta==11&&strstr((const char*)rsRxBuf,(const char*)",0")!=NULL)
           )//符合预期  再接受到的字节中找到
        {
          
          if(ProSta==ATTcpReadIndex)//读取到了TCP返回值
          {
            
            if(findStrCmp((unsigned char*)&rsRxBuf[20],20,(unsigned char*)"ALARM",5)!=0)
            {
              WarmHig;
              CloudtriggerFlag=1;
            }
            if(findStrCmp((unsigned char*)&rsRxBuf[20],20,(unsigned char*)"NONEA",5)!=0)
            {
              WarmLow; 
            }
            for(i=0;i<rsRxLen;i++)
            {
              if(rsRxBuf[i]<'9'&&rsRxBuf[i]>'0')//查第一个数字
              {
                break;
              }
            }
            if(rsRxLen>=19)
            {
              
              if(rsRxBuf[i+9]==1)
              {
                warmSend[0]=0;
                warmSend[1]=0;
                warmSend[2]=0;
                warmSend[3]=0;
              }
              if(rsRxBuf[i+10]==0)warmnobeep=0;
              if(rsRxBuf[i+10]==1)warmnobeep=1;
              if(rsRxBuf[i+11]==1)lifetime_count[0]=0;
              if(rsRxBuf[i+12]==1)lifetime_count[1]=0;
              if(rsRxBuf[i+13]==1)lifetime_count[2]=0;
              if(rsRxBuf[i+14]==1)lifetime_count[3]=0;
            }
          }
#if 0
          if(ProSta==ATCodeLastNum)//gps的读取  并且 QGPSLOC读取正确
          {
            SendGPSCnt=0;//收到gps预期清零
            GPSStatFlag=1;//gps运行状态正常
            pec20=&ec20send;
            for(i=0;i<rsRxLen;i++)
            {
              if(rsRxBuf[i]<'9'&&rsRxBuf[i]>'0')//查第一个数字
              {
                break;
              }
            }
            memcpy((char *)&(pec20->Time[0]),&rsRxBuf[i],10);
            for(;i<rsRxLen;i++)
            {
              if(rsRxBuf[i]==',')//查第一个数字
              {
                break;
              }
            }
            memcpy((char *)&(pec20->Latitude[0]),&rsRxBuf[i+1],10);
            i++;//跳过上一个逗号
            for(;i<rsRxLen;i++)
            {
              if(rsRxBuf[i]==',')//查第一个数字
              {
                break;
              }
            }
            memcpy((char *)&(pec20->longitude[0]),&rsRxBuf[i+1],11);
            GpsRecFlag=1;
          }
#endif
          GpsRecFlag=1;//放弃经纬度的识别
          if(ProSta==11)//发送成功则
          {
            linkflagSta=1;
            SendGPSCnt=0;
          }
          
          ProSta++;//符合预期往下走
          CurStaSendCnt=0;//收到预期清零
        }else
        {
          if(ProSta==11)//发送不成功
          {
            linkflagSta=0;
            ProSta=6;//发都没发成功就别接收了 直接重新开始
          }
        }
        Clear_Buffer();
      }
    }else//正常走完整个流程 从第九步[关闭TCP连接这步重新执行]开始
    {
      
      
      if(linkflagSta)
      {
        ProSta= 11;//重新发送
      }else
      {
        ProSta=6;
      }
      sendingflag=0;//未发送
    }
  }
}






//放在while(1)总的函数=EC20+RADAR
void PollFun(void)
{
  
  static unsigned char initflag=0;
  
  if(initflag==0)
  {
    initflag=1;//这里放一些初始化函数
    ec20power(1);
    u1Conf();//RS232
    u2Conf();//EC20  AT指令
    u3Conf();//Radar
#if DEBUGMODE==0
    OLED_Init();
    OLED_CLS();
    OLED_ShowString(0,1,"Setup Complete",16);
    delay_ms(1000);
    OLED_CLS();
#else
    SendTxDebug((unsigned char *)"初始化完成\0");
#endif
    lifetime_count[0] = 0;
    lifetime_count[1] = 0;
    lifetime_count[2] = 0;
    lifetime_count[3] = 0;
    warmSend[0]=0;
    warmSend[1]=0;
    warmSend[2]=0;
    warmSend[3]=0;
    
  }
  
  ec20ProcessFun();
  
#if FacSend==1  //这里测试
  if(simCnt>10000)
  {
    ready_2_warning = 1; 
    warning_ID = 1;
    warmSend[0]=1;
    warmSend[1]=1;
    warmSend[2]=1;
    warmSend[3]=1;
    if(linkflag==1&&sendingflag==0)
    {
      sendingflag=1;
      ProSta=11;//立马准备发送
    }
  }
  
  if(simCnt>20000)
  {
    simCnt=0;
    sendingflag=0;
    ready_2_warning = 0; 
    warning_ID = 0;
    warmSend[0]=0;
    warmSend[1]=0;
    warmSend[2]=0;
    warmSend[3]=0;
  }
  
#endif
  
  //上次1这次0
  if(readsxtioflag==GPIO_PIN_SET&&readSXTwarm==GPIO_PIN_RESET)
  {
    ready_2_warning=1;
    warning_ID = 0;
    warmSend[0]=1;
  }
  
  readsxtioflag=readSXTwarm;
  
  if(ready_2_warning == 1)
  {
    danger_keep = 10;
    ready_2_warning = 0;
    
    if(linkflag==1&&sendingflag==0)//已连接 未发送
    {
      sendingflag=1;
      ProSta=11;//立马准备发送
      
      PollTimeCnt=POLLTime+1;

    }
    if(warning_ID == 1)
    {
#if DEBUGMODE==0
      OLED_ShowString(0,0,"RADAR[1]DANGER!!",16);
#else
      SendTxDebug((unsigned char *)"雷达1报警\0");
#endif
    }
    else if(warning_ID == 2)
    {
#if DEBUGMODE==0
      OLED_ShowString(0,0,"RADAR[2]DANGER!!",16);
#else
      SendTxDebug((unsigned char *)"雷达2报警\0");
#endif
    }
    else if(warning_ID == 3)
    {
#if DEBUGMODE==0
      OLED_ShowString(0,0,"RADAR[3]DANGER!!",16);
#else
      SendTxDebug((unsigned char *)"雷达3报警\0");
#endif
    }
    else if(warning_ID == 4)
    {;
#if DEBUGMODE==0
    OLED_ShowString(0,0,"RADAR[4]DANGER!!",16);
#else
    SendTxDebug((unsigned char *)"雷达4报警\0");
#endif
    }
    
    //if(warmnobeep==0)//
    {
      if(waringtimeflag==0)
      {
        waringtimeflag=1;
        waringspeedflag=1;
        warmonceflag=1;
        POLLTime=ALARMTIME;
        WarmHig;//报警  报警没有被禁用
      }
    }
  }
  if(waringtime>10000)
  {
    WarmLow;
#if DEBUGMODE==0
    SendTxDebug((unsigned char *)"已报警\0");
#else
    SendTxDebug((unsigned char *)"已报警\0");
#endif
    waringtimeflag=0;
    sendingflag=0;//未发送 20220527
    waringtime=0;
    warmSend[0]=0;
    warmSend[1]=0;
    warmSend[2]=0;
    warmSend[3]=0;
  }
  
  if(ms10 >= 6000)//6秒
  {
    ms10 = 0;
    for(int i = 0;i<4;i++)
    {
      lifetime_count[i] ++;
      if(lifetime_count[i] > 10)//1分钟
      {
        //warmSend[i]=0;
        //printf("radar[%d] offline",i+1);
      }
    }
    
    danger_keep -- ;
    if(danger_keep == 0)
    {
#if DEBUGMODE==0
      OLED_Clear();
#else
      SendTxDebug((unsigned char *)"清除屏幕\0");
#endif
      WarmLow;
    }
  }
}




