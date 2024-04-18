/* ============================================================================
* 文件名:main.c
* 摘要:	主程序
* 版本V1.00
* 修正日期：2018.05.06
* 组织/作者:
历史记录：
* 版本V1.00
* 修正日期：2018.05.06
* 组织/作者:
==============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "DoorCtl_Globals.h"
#include "24cxx.h" 
#include "display_key.h"
#include "BstDoor_CAN_Configure.h"

#define BufferSize  50
#define SPI2_DR_Addr  0x4000380c
#define SPI1_DR_Addr  0x4001300c 
void TerminalInit1(void);
void PWR_PVDConfig(void);
void CAN_TX_Analy(void);
extern Uint16 p_charPWMEn;
extern CanTxMsg TMessage;  
extern int ADCL_adFlag;

extern Uint16 Doorstu_currmax;

u32 CpuID[3];



uint8_t Idex=0;
typedef enum {FAILED=0,PASSED=!FAILED} TextStatus;
Uint16 SPI1_Buffer_Tx[BufferSize]={0xFD3f,0x0203,0x0304,0x0405,0x0506,0x0607,0x0708,0x0809,0x090a,0x0a0b,
									 0x0102,0x0203,0x0304,0x0405,0x0506,0x0607,0x0708,0x0809,0x090a,0x0a0b,
									 0x0102,0x0203,0x0304,0x0405,0x0506,0x0607,0x0708,0x0809,0x090a,0x0a0b,
									 0x0102,0x0203,0x0304,0x0405,0x0506,0x0607,0x0708,0x0809,0x090a,0x0a0b,
									 0x0102,0x0203,0x0304,0x0405,0x0506,0x0607,0x0708,0x0809,0x090a,0x0a0b
                                     };
Uint16 SPI2_Buffer_Rx[BufferSize];

//角度校验
struct
{
	int32 AnglePWMSave;
	Uint16 CheckTime;
}AngleCheckVar;


__IO uint8_t TxIdx = 0;
//volatile TestStatus TransferStatus = FAILED;

extern Uint16 g_intQepInitCnt;

static __IO ErrorStatus HSEStartUpStatus = SUCCESS;
static __IO uint32_t TimingDelay = 0;
Uint16 CtlCurrentBase = 110;//控制主板系统检测电流，这个值与控制板检测电流电路最大值有关。我们生产中PM与异步是不一样的，注意在更正硬件电路

_iq Tbase;
float TbaseFloat;
Uint16 g_charEn = 0;
Uint8 I2C_Erorr =0;
_iq g_q24MotorIq = _IQ(0.1);
unsigned char led_status;
unsigned char aaaa;
int i;
/* Init Structure definition */
DAC_InitTypeDef            DAC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
//Uint32 buffer[4];
//E2PROM读写缓冲
Uint16 ReadBuff[8],WriteBuff[8];
#define NUM 16
u8  writeBuf[NUM]={0,1,2,3,4,5,6,7};
u8  readBuf[NUM]={0,0,0,0,0,0,0,0};
int16 g_intKEYAD = 0,g_intKEYADCnt = 0;
Uint16 templl;
Uint16 g_intWacthDogCnt = 0;
//static u8 tmp=0;
ECAN_STRUCT Ecan_state = ECAN_STRUCT_DEFAULTS;  //2013.3.20
ECAN_STRUCT Ecan_stateSave = ECAN_STRUCT_DEFAULTS;  //2013.3.20
struct ECANSendBufStruct ECANSendStack;// = ECAN_SENDSTACK_DEFAULTS;


static Uint32 g_u32StateSave = 0;

Uint16 tempfj = 0;

extern Uint16 Speed_cnt;


extern int p_intKey1;
extern int p_intKey2;

int main(void)
{
    //上电延时，防止电压的波动
    Uint32 progdelay = 60000;//60000-8ms
    while(progdelay--)
    {
        asm("nop");
    }
    //while(1);
    for(progdelay=0;progdelay<ECANBUFLENMAX;progdelay++)
    {
        ECANSendStack.ECANSendData[progdelay].BufDataVaildFlag = 0;
    }

    RCC_Configuration();
    
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | 
    RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,  ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭SWJ，使用PB3用作普通I/O
    DBGMCU_Config(TRCE_ON_STOP,DISABLE);

    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x08007000);

    __enable_irq();
    //PWR_PVDConfig();
    //SVPWM_3ShuntInit();
    /* GPIOA Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
    
    
     
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOA,GPIO_Pin_12);
    
    

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIOC->BRR = GPIO_Pin_5;
  
  
   RCC_Configuration();
   
   //SCI_Init(&Sci_Strut);
   //调用一下信息区，防止信息被优化
   
   readBuf[0]=AppKeyWords[1];
   readBuf[1]=DataKeyWords[0];

 
    //定时器处理---注意是否为1ms中断   
    TaskBaseTime();

  //DAC初始化
#ifdef DACEN
    DACInit();
#endif
    //初始化E2PROM--必须有1ms中断
    //gE2rom.init(&gE2rom);
    AT24CXX_Init();
    

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5|GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_3|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
   
    
    GPIOB->BRR = GPIO_Pin_4;
    GPIOB->BSRR = GPIO_Pin_10;
    GPIOB->BRR = GPIO_Pin_3;
    asm("nop");
    GPIOB->BSRR = GPIO_Pin_3;
    asm("nop");
    asm("nop");
    GPIOB->BRR = GPIO_Pin_10;
    GPIOB->BRR = GPIO_Pin_3;
    asm("nop");  
    GPIOB->BSRR = GPIO_Pin_3;
    GPIOB->BSRR = GPIO_Pin_4;
    asm("nop");
   GPIOB->BRR = GPIO_Pin_3;
    asm("nop");
    
    /*
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOA,GPIO_Pin_12);*/
    
    //初始化端子
    //TerminalVar.init = TerminalInit1;
    TerminalVar.init(&TerminalVar);
#ifdef SPIDIS
    //初始化按键与显示
    Display_init();
#endif
    //E2PROM与参数初始化
    gE2rom.E2promDataSturct.ReadDataPtr = ReadBuff;//连接缓存地址
    gE2rom.E2promDataSturct.WriteDataPtr = WriteBuff;
    gE2rom.E2promDataSturct.PararPointStep = PARA_ATTR_LEN;

  
  //初始化上电标志，包括编码器重新初始化，门宽重新寻找
    QeqResumeVar.ResumeInitTime =  QERINTIETIME;
    gCap.InitTime = 100;
    gCap.CheckSignalTime = 3000;
    PowerFstVar.ProInit = 1;
    PowerFstVar.PowerFst = 1;
    QeqResumeVar.CmdState.ResumeEn = 1;
    QeqResumeVar.CmdState.ResumeFlag = 1;
    gCap.InitMechThetaFlag = 0;
    FaultVar.fault.FautFlag0.bit.QEPErrorFlag = 0;
    FaultVar.fault.FautFlag0.bit.StallFlag = 0;
    FaultVar.fault.FautFlag0.bit.ForceStall = 0;
    TorqueProtecVar.StateCmd.TorqueOverFlag  = 0;
    TorqueProtecVar.StateCmd.ClrEn = 1;
    StallCheckStru.StateCmd.StallFlag = 0;
    StallCheckStru.StateCmd.ClrStall = 1;
    qep1.QepABError = ABSIGNALNONE;
    IWDG_ReloadCounter();//在读写EEPROM前，需要初始化看门狗，防止读写时间超过0.8s
    //而造成，CPU不能运行
    //初始化参数
    ParInit();

   /// CTLcmdVar.CMDSel = 0;//直接恢复到0
    //判断PWM开关频率是否有问题，有问题，则调用默认值
    if( (CarrFreq<SysParaAttrChar[5].LowLmt)||(CarrFreq>SysParaAttrChar[5].UpLmt) )
    {
        CarrFreq = SysParaAttrChar[5].OrgVal0;
    }
    else
    {
    }
    //上电次数--P07.13
    if(g_uSysDelayParSet>800)
    {
        g_uSysDelay = g_uSysDelayParSet-800;
    }
    else
    {
        g_uSysDelay = 0;
    }
    PowerUpCnt++;
    ProInit();
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    SCI_Init(&Sci_Strut);

    //正常运行时，把看门狗放小到8ms
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);     //①使能对寄存器 I 写操作
    IWDG_SetReload(10); //10
    IWDG_ReloadCounter();
    gCap.CapOver = 0;
    //CAN配置
  
    CAN_Configuration();
    
     /* GPIOA Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
     //输出PA11 故障报警喇叭
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    qep1.CalibratedAngle = _IQdiv( Magzerostudy.data.MagZeroPosition,3600);
    
    BreakLowSideTime = 200;
    
        

 while(1)
 {
        PosStateVar.State.bit.FstFlag =  PowerFstVar.PowerFst;

        EcanRxData(); 
        
        //状态是否改变进行判断 以及定时解发  
        CAN_TX_Analy();
        //CAN数据发送
        EcanTxData();
        
 
    GPIO_SetBits(GPIOB, GPIO_Pin_2);
    //通讯检测
    SciToOPRT_Updata(&Sci_Strut);
    GPIO_ResetBits(GPIOB, GPIO_Pin_2 );
    
    //FMSTR_Poll();
        //1ms任务----保证主程序完成两次
        if(TaskTime1ms==0)
        {
            if(templl!=0) templl--;
            Task1msFun();
            TaskTime1ms = 1;
            
            g_hBuzzerVar.update(&g_hBuzzerVar);
            //LSI的变化为30~60KHz，因此复位时间为5~10ms，复位时间应该为3,
            //允许10%误差，取一定的安全余量，这个主要用来防止时钟变缓慢的问题
            if(g_intWacthDogCnt!=0)
            {
                g_intWacthDogCnt--;
            }
            else
            {
                g_intWacthDogCnt =1;//1ms喂狗一次
                IWDG_ReloadCounter();
                GPIOA->ODR ^= GPIO_Pin_1;   ///硬件看门狗
            }
            //能过指令复位MCU，复位延时100ms
            if(Sci_Strut.ResetCMDTime!=0) Sci_Strut.ResetCMDTime--;
            else
            {
                if(Sci_Strut.ResetCMD==1)
                {
                    NVIC_SystemReset();
                }
                else
                {
                }
            }
        }
        
        
        
    if(VolProtectVar.state.PowerFst==0)
    {
      //故障分析
      FaultAnaly();
    }
    else
    {
    }
    //参数更新,进行内部参数计算
    if( Sci_Strut.ParUpdata==1 )
    {
        if( Sci_Strut.ProRest==0 )
        {
            (*(SysParaAttrChar[Sci_Strut.ParaIndex].Updata))();
        }
        else
        {
            ParSettoCTLAnaly();
        }
        Sci_Strut.ParUpdata = 0;
    }
    //E2PROM manage---E2PROM的任务管理
    E2prom_TaskManage();
    //E2prom deal
    gE2rom.updata(&gE2rom);
    //初始化时，要进行E2PROM读取或写入，如果不正确则读入默认参数；这些保护编码器初始化完
    //成时，才清程序化完成标志
    ProInitFun();
 
    /*
     if( (gCap.InitTime==0)&&(QeqResumeVar.ResumeInitTime==0) )
     {
        //一键学习控制
        if(g_uStudyEn==1)
        {
            StudyType = 2;
        }
        else if(g_uStudyEn==2)
        {
            StudyType = 1;
        }
        else
        {
        }
     }
         */


    //功能选择
    if( StudyType==0 )
    {
        if( (gCap.InitTime==0)&&(QeqResumeVar.ResumeInitTime==0) )//参数初始化完毕且角度校正正常时才进入正常电机控制
        //if(QeqResumeVar.ResumeInitTime==0)//参数初始化完毕且角度校正正常时才进入正常电机控制
        {
           //位置区间解析
            PosStateVar.DoorPosInput = qep1.OutputRawTheta>>2;
            PosStateVar.DoorWidth = DoorStudyVar.Width;
            PosStateVar.State.bit.FstFlag = PowerFstVar.PowerFst;
            if( MotorStru.MotorDirSet==1 )
            {
                PosStateVar.State.bit.DoorDirectionCMD = 1;
            }
            else
            {
                PosStateVar.State.bit.DoorDirectionCMD = 0;
            }

            PosStateVar.updata(&PosStateVar);
            //命令解析
            if(ForcStop==0)
            {
                CommandAnaly();
            }
            else
            {
                CTLcmdVar.FinalCmd.all = 0;
                CTLcmdVar.FinalCmdSave.all = 0;
            }
    
            //动作解析
            if( g_uSysDelay==0 )
            {
                CMDToActionAnaly();	
            }
            else
            {
            }	
            //位置校正
            PosCorrect();
            //开关堵转力矩保护，以防止电机过热损坏
            ODMaxTorqueLimitUp();   

        }
        else
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmdSave.all = 0;
            //门宽自校正初始化
            PosStateVar.ODDoorWidth = DoorStudyVar.Width+1;
            PosStateVar.CDDoorWidth = DoorStudyVar.Width;
            PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;
        }
        OutDeal();
    }
    else if( StudyType==1 )//门宽自学习
    {
       TorqueProtecVar.StateCmd.ClrEn = 1;
        if( (FaultVar.fault.FautFlag0.all&0xc9af)==0 )
        {
            DoorStudyVar.StateCmd.StudyEn = 1;
        }
        else
        {
            DoorStudyVar.StateCmd.StudyEn = 0;
            DoorStudyVar.StateCmd.Step = 0;
            StudyType = 0;
            g_uStudyEn = 0;
        }
        g_hPosLoop.StateCmd.PosLoop = 0;
    }
    else if( StudyType==4 )//磁极自学习
    {
        if( (FaultVar.fault.FautFlag0.all&0xc9af)==0 )
        {
            MageStudy.StateCmd.MageticStudyEn = 1;
        }
        else
        {
            MageStudy.StateCmd.MageticStudyEn = 0;
            MageStudy.StateCmd.Step = 0;
            StudyType = 0;
        }
        g_hPosLoop.StateCmd.PosLoop = 0;

    }
    else if( StudyType==3 )//电机电阻自学习
    {
            
    }
    else if( StudyType==2 )//编码器手动寻找零位值
    {
        if( (FaultVar.fault.FautFlag0.all&0xc1af)==0 )
        {
            MageStudy.StateCmd.MageticStudyEn = 1;
        }
        else
        {
            MageStudy.StateCmd.MageticStudyEn = 0;
            MageStudy.StateCmd.Step = 0;
            StudyType = 0;
            g_uStudyEn = 0;
        }
        g_hPosLoop.StateCmd.PosLoop = 0;
    }	
    //按键处理
    #ifdef SPIDIS
    KeyDeal();
    #endif
   //界面层次与显示分析
    //KeyProcess();
 }
}


void StatusLedInit(void)
{
 GPIO_InitTypeDef GPIO_InitStructure;

 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 GPIO_Init(GPIOE, &GPIO_InitStructure);
}
//=================================================================================================
// 初始化串口1，115200 8-N-1
//-------------------------------------------------------------------------------------------------
// coding
//=================================================================================================
void USART3_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStructure;
 USART_InitTypeDef USART_InitStructure; 

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

 USART_DeInit(USART3);
 
 GPIO_PinRemapConfig(GPIO_Remap_USART1,DISABLE);

 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

 USART_Cmd(USART3, DISABLE);

 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 USART_InitStructure.USART_BaudRate = 57600;
 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 USART_InitStructure.USART_Parity = USART_Parity_No;
 USART_InitStructure.USART_StopBits = USART_StopBits_1;
 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
 USART_Init(USART3, &USART_InitStructure);

 // 清除可能存在的中断标志
 USART_ClearFlag(USART3, USART_FLAG_RXNE);

 // 开接收中断
 //USART_ITConfig( USART1, USART_IT_RXNE | USART_IT_TXE, ENABLE);

 // 重新启用USART3
 USART_Cmd(USART3, ENABLE);
}
/*******************************************************************************
* Function Name  : TaskBaseTime
* Description    : 用作任务管理或其它定时的时间基准。基准为1ms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void TaskBaseTime(void)
{
    /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))//5000
    {
        /* Capture error */
        while (1);
    }
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);  
}

/*******************************************************************************
* Function Name  : DACInit
* Description    : DAC初始化,PA.04用做ADC输出，以观察程序的变量
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DACInit(void)
{

  /* GPIOA Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4 输出高


  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is 
  automatically connected to the DAC converter. */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
}
/*******************************************************************************
* 函数名称  : Task1msFun
* 描述      : S曲线有关参数计算
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Task1msFun(void)
{
  static Uint16 Time_1s = 1000,Time_100ms=100;
  static int16 IsrTicker = 0,p_intFlashLedFlag = 0;
  
  	OD_TorqueUPTime++;
	CD_TorqueUPTime++;
	if(OD_TorqueUPTime >= 60000) OD_TorqueUPTime = 60000;
	if(CD_TorqueUPTime >= 60000) CD_TorqueUPTime = 60000;

	OD_TorqueDWNTime--;
	CD_TorqueDWNTime--;
	if(OD_TorqueDWNTime <= 10) OD_TorqueDWNTime = 10;
	if(CD_TorqueDWNTime <= 10) CD_TorqueDWNTime = 10;
        if(g_uUartSendDelayTime!=0) g_uUartSendDelayTime--;

  if( TaskTimeVar.DisParTime!=0 ) TaskTimeVar.DisParTime--;
  if( TaskTimeVar.MainCalTime!=0 ) TaskTimeVar.MainCalTime--;
  if( OutDelay.ODOutDelayCnt<65535)	OutDelay.ODOutDelayCnt++;
  if( OutDelay.CDOutDelayCnt<65535)	OutDelay.CDOutDelayCnt++;
  if( AngleCheckVar.CheckTime!=0 )	AngleCheckVar.CheckTime--;
  if( OutDelay.OutDelayStall!=0 ) OutDelay.OutDelayStall--;
  if( g_uSysDelay!=0 ) g_uSysDelay--;
  if( QeqResumeVar.ResumeInitTime!=0 ) QeqResumeVar.ResumeInitTime--;
 /// if(PassWordRevTime!=0) PassWordRevTime--;
  if( Ecan_state.OnLineTime != 0 )   Ecan_state.OnLineTime--;
  if( Ecan_state.EcanTxTime != 0 )   Ecan_state.EcanTxTime--;
  if( Ecan_state.EcanRxTime != 0 )   Ecan_state.EcanRxTime--;
  if(Ecan_state.Txtimecounti != 0)  Ecan_state.Txtimecounti--;
  if(Ecan_state.Txtimecountj != 0)  Ecan_state.Txtimecountj--;
  if(( Ecan_state.RxdataFirstEn != 0)&&(Ecan_state.TxdataDealyTime != 0))  Ecan_state.TxdataDealyTime--;
  if(Camstateaction.data.CamStateActionTime != 0) Camstateaction.data.CamStateActionTime--;
  if(Speed_cnt != 0) Speed_cnt--;
  
  if(Ecan_state.InitOver)//初始化结束
  {
      if(Ecan_state.PluseErrRxcount!=0) Ecan_state.PluseErrRxcount--;
      if(Ecan_state.PluseErrTxcount!=0) Ecan_state.PluseErrTxcount--;
  }

  if( QeqResumeVar.CmdState.ResumeFlag==1 )
  {
    if( (MTCVar.StateCmd.RunEn!=0)&&(PowerFstVar.ProInit==0) )
    {
      if(QeqResumeVar.RemsumTime>QERRESUMETIME)
      {
        QeqResumeVar.CmdState.ResumeFlag = 0;
        QeqResumeVar.CmdState.ResumeEn = 0;
        QeqResumeVar.RemsumCnt = 0;
      }
      else
      {
        QeqResumeVar.RemsumTime++;
      }
    }
    else
    {
    }
  }
  else
  {
    QeqResumeVar.RemsumTime = 0;
  }
  if( (FaultVar.fault.FautFlag0.bit.QEPErrorFlag==1)&&(StudyType!=2) )
  {
    MTCVar.StateCmd.RunEn = 0;
  }
  else
  {
  }
  //100ms计数
  if( Time_100ms!=0 )
  {
    Time_100ms--;
  }
  else
  {
    if(Camstateaction.data.CamStateActionTime100ms != 0)  Camstateaction.data.CamStateActionTime100ms--;
    if(ProgDelay!=0)
    {
      ProgDelay--;
    }
    if( OCDOverVar.state.CDCntEn==1 )
    {
      OCDOverVar.ODTimeCnt = 0;
      if( OCDOverVar.CDTimeCnt<OCDOverVar.CDTime )
      {
        OCDOverVar.CDTimeCnt++;
      }
      else
      {
        OCDOverVar.state.CDOverFlag=1;
      }

    }
    else if( OCDOverVar.state.ODCntEn==1 )
    {
      OCDOverVar.CDTimeCnt = 0;
      if( OCDOverVar.ODTimeCnt<OCDOverVar.ODTime )
      {
        OCDOverVar.ODTimeCnt++;
      }
      else
      {
        OCDOverVar.state.ODOverFlag=1;
      }
    }
    else
    {
      OCDOverVar.CDTimeCnt = 0;
      OCDOverVar.ODTimeCnt = 0;
    }
    Time_100ms=99;
  }
  if( DoorFianlVar.StateCmd.ODSwithEn==1 )//开门力矩切换到最终保持允许
  {
    if( DoorFianlVar.ParSet.ODswithTime>(PosStateVar.ParSet.OD_TorqueSwTime*100) )
    {
      DoorFianlVar.StateCmd.ODSwithFlag = 1;
    }
    else
    {
      DoorFianlVar.ParSet.ODswithTime++;
      DoorFianlVar.StateCmd.ODSwithFlag = 0;
    }
  }
  else
  {
    DoorFianlVar.StateCmd.ODSwithFlag = 0;
    DoorFianlVar.ParSet.ODswithTime = 0;
  }
  if( DoorFianlVar.StateCmd.CDSwithEn==1 )//关门力矩切换到最终保持力记时允?
  {
    if( DoorFianlVar.ParSet.CDswithTime>(PosStateVar.ParSet.CD_TorqueSwTime*100) )
    {
      DoorFianlVar.StateCmd.CDSwithFlag = 1;
    }
    else
    {
      DoorFianlVar.ParSet.CDswithTime++;
      DoorFianlVar.StateCmd.CDSwithFlag = 0;
    }
  }
  else
  {
    DoorFianlVar.StateCmd.CDSwithFlag = 0;
    DoorFianlVar.ParSet.CDswithTime = 0;
  }

// ------------------------------------------------------------------------------
//   控制器端子处理
//	 三线端口滤波
// ------------------------------------------------------------------------------
    TerminalVar.updata(&TerminalVar);
  
   
// ------------------------------------------------------------------------------
//   过力矩检测
// ------------------------------------------------------------------------------
  TorqueProtecVar.Torqueinput = park1.Qs;
  TorqueProtecVar.StateCmd.RunEn = MTCVar.StateCmd.RunEn;
  if( g_hCurveGenerate.RealCurveCalc.CmdState.ODCD==0 )//关门命令
  {
    TorqueProtecVar.TorqueRefer = PosStateVar.TorqueInter.stallCDTorque*MotorStru.DoorDirect;
  }
  else
  {
    TorqueProtecVar.TorqueRefer = MTCVar.CurrentMax*MotorStru.DoorDirect;
  }
  
  Doorstu_currmax = (Uint16)((TorqueProtecVar.TorqueRefer * 100) >> 24);
  
  if( g_hPosLoop.StateCmd.StallPosLoop==0 )
  {
    TorqueProtecVar.calc(&TorqueProtecVar);
  }
  else
  {
  }
// ------------------------------------------------------------------------------
//   堵转检测
// ------------------------------------------------------------------------------
  StallCheckStru.StateCmd.RunState = MTCVar.StateCmd.RunEn;
  StallCheckStru.SpeedRefer = _IQtoIQ15(pid1_spd.Ref);
  StallCheckStru.SpeedInput = _IQtoIQ15(speed2.Speed);
  if( g_hPosLoop.StateCmd.StallPosLoop==0 )
  {
    StallCheckStru.calc(&StallCheckStru);
  }
  else
  {
  }
  if(VolProtectVar.state.PowerFst==0)
  {
    //温度检测与保护
    TemperatureVar.TemperatureInput = ilg2_vdc1.HeatAlarm;
    TemperatureVar.calc(&TemperatureVar);
    
    TemperaMtureVar.TemperatureMInput = (ilg2_vdc1.MotortpAlarm);
    TemperaMtureVar.calc(&TemperaMtureVar);
  }
  else
  {
  }
// ------------------------------------------------------------------------------
//   母线电压检测
// ------------------------------------------------------------------------------
  //VolProtectVar.state.PowerFst = PowerFstVar.ProInit;
  VolProtectVar.VolInput = ilg2_vdc1.VdcMeas;
  VolProtectVar.calc(&VolProtectVar);
  //演示功能
  if( CTLcmdVar.CMDSel==4 )//DemoVar
  {
   if( OutDelay.StateCmd.bit.CDEndOut==1 )
   {
      if( DemoVar.SwTimeCnt>DemoVar.CDSwithTime*100 )
      {
        CTLcmdVar.DemoCmd.bit.CD = 0;
        CTLcmdVar.DemoCmd.bit.OD = 1;
      }
      else
      {
        DemoVar.SwTimeCnt++;
      }
    }
    else if( OutDelay.StateCmd.bit.ODEndOut==1 )
    {
      if( DemoVar.SwTimeCnt>DemoVar.ODSwithTime*100 )
      {
        CTLcmdVar.DemoCmd.bit.CD = 1;
        CTLcmdVar.DemoCmd.bit.OD = 0;
      }
      else
      {
        DemoVar.SwTimeCnt++;
      }
    }
    else
    {
      DemoVar.SwTimeCnt = 0;
    }
  }
// ------------------------------------------------------------------------------
//    失速保护 
// ------------------------------------------------------------------------------ 
  if(MTCVar.StateCmd.RunEn==0)
  {
    g_sLossSpdVar.SpeedInput = _IQabs(0);
  }
  else
  {
    g_sLossSpdVar.SpeedInput = _IQabs(speed2.Speed);
  }
  g_sLossSpdVar.calc(&g_sLossSpdVar);
  
// ------------------------------------------------------------------------------
//    输入电源丢失检测
// ------------------------------------------------------------------------------    
  g_hPowerInLoss.lossPhaseFlag = ADCL_adFlag;
  g_hPowerInLoss.calc(&g_hPowerInLoss);
// ------------------------------------------------------------------------------
//    电机驱动有关参数显示 
// ------------------------------------------------------------------------------  
  MTCVar.Dis.SpeedRef = ((_IQ28abs(g_hCurveGenerate.CurveSpd.SetpointValue)>>14)*MotorStru.MotorRatedFreq)>>14;
  MTCVar.Dis.SpeedRefMil = (int32)( (int32)MTCVar.Dis.SpeedRef*(int32)g_hUnitTransform.SpeedHztoLinearVelCoeff )>>15;
  MTCVar.Dis.SpeedFdb = ((_IQabs(speed2.Speed)>>10)*MotorStru.MotorRatedFreq)>>14;
  MTCVar.Dis.SpeedFdbMil = (int32)((int32)MTCVar.Dis.SpeedFdb*(int32)g_hUnitTransform.SpeedHztoLinearVelCoeff)>>15;
  MTCVar.Dis.SpeedWarp = ((_IQabs(speed2.Speed-_IQ28toIQ(g_hCurveGenerate.CurveSpd.SetpointValue))>>10)*MotorStru.MotorRatedFreq)>>14;
  MTCVar.Dis.SpeedWarpMil = (int32)((int32)MTCVar.Dis.SpeedWarp*(int32)g_hUnitTransform.SpeedHztoLinearVelCoeff)>>15;
  MTCVar.Dis.CurrentFdb = (int32)(( _IQabs(park1.Qs)>>10)*CtlCurrentBase)>>14; //pid1_iq.Fdb//_IQmpyI32int( _IQabs(_IQmag(park1.Qs,park1.Ds)),CtlCurrentBase);

  if(Time_1s==0)
  {
    if( CTLcmdVar.HoldTimeCnt<=HOLDMAX ) CTLcmdVar.HoldTimeCnt++;
    Time_1s = 999;
  }
  else
  {
    Time_1s--;
  }
  IsrTicker++;
  if(IsrTicker>100)
  {
    if(p_intFlashLedFlag==1)
    {
      p_intFlashLedFlag = 0;
      GPIO_SetBits(GPIOA,GPIO_Pin_12);
    }
    else
    {
      p_intFlashLedFlag = 1;
      GPIO_ResetBits(GPIOA,GPIO_Pin_12);
    }
    IsrTicker = 0;
  }
  else
  {
  }
  
    if(CTLcmdVar.CMDSel==3)//CAN模式时
    {
        if( (Ecan_state.InitOver==0)||(Error_bak!=0) )
        {
            if( g_hLedFlash.FlashInterTime==0 )
            {
                //GPIOB->ODR ^= GPIO_Pin_10;
                g_hLedFlash.FlashInterTime = 500;
            }
            else
            {
                g_hLedFlash.FlashInterTime--;
            }
        }
        else
        {
           // GPIOB->BRR = GPIO_Pin_10;
        }

    }
    else
    {
        if( (Error_bak!=0) )
        {
            if( g_hLedFlash.FlashInterTime==0 )
            {
                //GPIOB->ODR ^= GPIO_Pin_10;
                g_hLedFlash.FlashInterTime = 500;
            }
            else
            {
                g_hLedFlash.FlashInterTime--;
            }
        }
        else
        {
            //GPIOB->BRR = GPIO_Pin_10;
        }
    }
  

}
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
     falling edges */
  EXTI_ClearITPendingBit(EXTI_Line16);
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}
/*******************************************************************************
* 函数名称  : PWR_PVDConfig
* 描述      : 配置PVD,欠压时，显示关闭
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void PWR_PVDConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable PWR and BKP clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR| RCC_APB1Periph_BKP, ENABLE);
    
        /* Configure the PVD Level to 2.9V */
    PWR_PVDLevelConfig(PWR_PVDLevel_2V9);

    /* Enable the PVD Output */
    PWR_PVDCmd(ENABLE);
    
    
    EXTI_Configuration();
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the PVD Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
/*******************************************************************************
* 函数名称  : RCC_Configuration
* 描述      : 配置RCC为外部晶振，可以运行到72MHz;如果失效切换到内部时钟，最大为
              64MHz。
              使用内部时钟时，在P0.00的界面显示P0.00.
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void RCC_Configuration(void)
{  
    ErrorStatus HSEStartUpStatus;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    SystemInit();
    RCC_DeInit();         
    RCC_HSEConfig(RCC_HSE_ON);  
    ///RCC_HSEConfig(RCC_HSE_OFF);  
    /* Disenable LSE */
    RCC_LSEConfig(RCC_LSE_OFF);
    
    /* Enable Clock Security System(CSS): this will generate an NMI exception
    when HSE clock fails 
    如果HSE时钟发生故障，HSE振荡器被自动关闭，时钟失效事件将被送到高级定时器TIM1
    的刹    车输入端，并产生时钟安全中断CSSI，允许软件完成营救操作。此CSSI中断连
    接到Cortex-M3 的NMI中断。*/

    RCC_ClockSecuritySystemCmd(ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = RCC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
     /* Flash 0 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
    //-------------------------------------------------------------------------
    //产生72M的系统时钟, 8M*9
    //-------------------------------------------------------------------------
    if(HSEStartUpStatus == SUCCESS)     ///     gw74   外部晶振为SUCCESS  
    {
        //g_eRccState = EXIT_RCC;

        RCC_HCLKConfig(RCC_SYSCLK_Div1);   //AHB时钟为72M（系统时钟）   
        RCC_PCLK2Config(RCC_HCLK_Div1);    //APB2时钟为72M  
        RCC_PCLK1Config(RCC_HCLK_Div2);    //APB1时钟为36M (对timer3来说还是36M*2=72M)

        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);    
        RCC_PLLCmd(ENABLE);             
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);   

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);       
        while(RCC_GetSYSCLKSource() != 0x08);   
        SystemCoreClock = 72000000;//时钟频率
    }
    else///内部时钟
    {
        //g_eRccState = INTER_RCC;
        /* RCC system reset(for debug purpose) */
        SystemInit();
        RCC_DeInit();
        SystemInit();
        RCC_HSEConfig(RCC_HSE_OFF);
        RCC_LSEConfig(RCC_LSE_OFF);
       

        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1); //64MHz

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1); //64MHz

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);//32MHz

        /* PLLCLK = 8MHz/2 * 16 = 64 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
        RCC_ClockSecuritySystemCmd(ENABLE);       
        /* Enable PLL */ 
        RCC_PLLCmd(ENABLE);
        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
        SystemCoreClock = 64000000;//时钟频率
    }
}




/**
  * @brief  Enables or disables EXTI for the menu navigation keys :
  *         EXTI lines 3, 7 and 15 which correpond respectively
  *         to "DOWN", "SEL" and "UP".
  * @param  NewState: New state of the navigation keys. This parameter
  *                   can be: ENABLE or DISABLE.
  * @retval None
  */
void IntExtOnOffConfig(FunctionalState NewState)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line 3, 7 and 15 on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line7 | EXTI_Line15;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
	
  /* Enable the EXTI line 3, 7 and 15 on falling edge */
  else
  {
    /* Clear the the EXTI line 3, 7 and 15 interrupt pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3 | EXTI_Line7 | EXTI_Line15);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line7 | EXTI_Line15;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}

/**
  * @brief  Configures the different GPIO ports pins.
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure PG.07, PG.08, PG.13, PG.14 and PG.15 as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* Configure PD.03 as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* RIGHT Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource13);

  /* LEFT Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource14);

  /* DOWN Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);

  /* UP Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource15);

  /* SEL Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7);

  /* KEY Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource8);
}
/*******************************************************************************
* 函数名称  : IWDG_Init
* 描述      : 看门狗初始化
              通讯口2为RS485端口。
* 输入      : None
* 输出      : None
* 返回      : None
*******************************************************************************/
void IWDG_Init(void)
{
  /* IWDG timeout equal to 3.276 s (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
  IWDG_SetPrescaler(IWDG_Prescaler_32);

  /* Set counter reload value to 80ms */
  IWDG_SetReload(100);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}

/*******************************************************************************
* 函数名称  : CurveParaCalc
* 描述      : S曲线有关参数计算
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void CurveParaCalc(void)
{
    g_hCurveGenerate.RealCurveCalc.CalcPara.SpdToPosCoeff = (int32)MotorStru.MotorRatedFreq*(int32)qep1.LineEncoder*(int32)32/((int32)MotorStru.Motor_Poles*(int32)125);//注意此值误差可能影响曲线
    

    g_hCurveGenerate.CD_RefCurve.HighSpd = -1*_IQ28div(g_hCurveGenerate.PolyLineVar.RefHighSpd,MotorStru.MotorRatedFreq);
    g_hCurveGenerate.CD_RefCurve.OriginSpd = -1*_IQ28div(g_hCurveGenerate.PolyLineVar.CDCreepSpd,MotorStru.MotorRatedFreq);

    g_hCurveGenerate.OD_RefCurve.HighSpd = _IQ28div(g_hCurveGenerate.PolyLineVar.RefHighSpd,MotorStru.MotorRatedFreq);
    g_hCurveGenerate.OD_RefCurve.OriginSpd = _IQ28div(g_hCurveGenerate.PolyLineVar.ODCreepSpd,MotorStru.MotorRatedFreq);

    

    g_hCurveGenerate.CD_HighSpdSave = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.CDActHighSpd_per,100),g_hCurveGenerate.CD_RefCurve.HighSpd);
    g_hCurveGenerate.CD_ActCurve.HighSpd = g_hCurveGenerate.CD_HighSpdSave ;

    g_hCurveGenerate.NomalrLCD_HighSpdSave = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.NomalLCDActHighSpd_per,100),g_hCurveGenerate.CD_RefCurve.HighSpd);

    g_hCurveGenerate.OD_ActCurve.HighSpd = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.ODActHighSpd_per,100),g_hCurveGenerate.OD_RefCurve.HighSpd);

    if( _IQ28abs(g_hCurveGenerate.CD_ActCurve.HighSpd)>_IQ28abs(g_hCurveGenerate.OD_ActCurve.HighSpd) )
    {
            g_sLossSpdVar.SpeedRefer =	_IQ28toIQ(_IQ28abs(g_hCurveGenerate.CD_ActCurve.HighSpd));
    }
    else
    {
            g_sLossSpdVar.SpeedRefer =	_IQ28toIQ(_IQ28abs(g_hCurveGenerate.OD_ActCurve.HighSpd));
    }
    g_hCurveGenerate.PolyLineVar.PowerFstLowSpdQ24 = _IQ28div(g_hCurveGenerate.PolyLineVar.PowerFstLowSpd,MotorStru.MotorRatedFreq);
    g_hCurveGenerate.PolyLineVar.StudySpdQ24 = _IQ28div(DoorStudyVar.SpeedStudey,MotorStru.MotorRatedFreq);
    g_hCurveGenerate.RealCurveCalc.CmdState.StudyMode = 0;//清零自学习标志

    g_hCurveGenerate.PolyLineVar.CurveComperQ24 = _IQ28(1.0)+_IQ28div(g_hCurveGenerate.PolyLineVar.CurveCompenPer,1000);
    g_hCurveGenerate.PolyLineVar.CDRetiringCamSpdQ24 = _IQ28div(g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd,MotorStru.MotorRatedFreq);//必须为正
    g_hCurveGenerate.PolyLineVar.ODRetiringCamSpdQ24 = _IQ28div(g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd,MotorStru.MotorRatedFreq);//必须为正
}

void Decrement_TimingDelay(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
/**
  * @brief  Configure the leds pins as output pushpull: LED1, LED2, LED3 and LED4
  * @param  None
  * @retval None
  */
void LedShow_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);

  /* Configure LEDs GPIO: as output push-pull */
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 719;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 0x270F;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1 Configuration in Timing mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0x0;
  
  TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
}
ErrorStatus Get_HSEStartUpStatus(void)
{
  return (HSEStartUpStatus);
}
/*******************************************************************************
* 函数名称  : ProInit
* 描述      : 初始化程序。
* 输入      : None
* 输出      : None
* 返回      : 无.
*******************************************************************************/
void ProInit(void)
{
    p_charPWMEn = 0;


    if( gE2rom.CmdState.WtireReadError==1 )//参数出错时，使用默认值
    {
        ParDefault(0,160);//调用默认参数
        ParDefault(160,176);//调用默认参数
    }
    else
    {
        /*SysTick_Handler();
        NMI_Handler();
        PVD_IRQHandler();
        SPI2_IRQHandler();
        ADC1_2_IRQHandler();*/
    }
    ParSettoCTLAnaly();//参数更新

    CtlCurrentBase = 328;//电流基准值  100mΩ---168   50mΩ-- 328
    SpdVariableI.DnValue = _IQ(0.2);
    SpdVariableI.UpValue = _IQ(0.8);

    ParSettoCTLAnaly();//参数更新
    FluxCurrentQ24 = _IQdiv(MotorStru.ExcitationCurrent,CtlCurrentBase);
    CltCurrentBaseQ15 =_IQ15div(CtlCurrentBase,100);
    TbaseFloat = 0.001/CarrFreq;
    Tbase = _IQ(TbaseFloat); 
    //Initialize PWM module
    pwm1.CarrFreq = CarrFreq;
    pwm1.init(&pwm1);
    // Initialize QEP module
    qep1.LineEncoder = 1024;
    qep1.PolePairs = (MotorStru.Motor_Poles>>1);
    qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
    qep1.PreScaler = 0x03fffffff/qep1.LineEncoder;
    qep1.OutputMechScaler = _IQ30((1.0*0.25)/(qep1.PreScaler*qep1.LineEncoder));
    qep1.init(&qep1);
    // Initialize RAMPGEN module
    rg1.StepAngleMax = _IQ(MotorStru.MotorRatedFreq*TbaseFloat/100);
    // Initialize the Speed module for QEP based speed calculation
    ///speed1.K1 = _IQ20(1/(MotorStru.MotorRatedFreq*TbaseFloat*5/100));  
    ///speed1.K2 = _IQ(1/(1+TbaseFloat*5*2*PI*300));  //截止频率100Hz
    
    speed1.K1 = _IQ20(1/(MotorStru.MotorRatedFreq*TbaseFloat/100));
    speed1.K2 = _IQ(1/(1+TbaseFloat*2*PI*100));  //截止频率100Hz 
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 12*(MotorStru.MotorRatedFreq/MotorStru.Motor_Poles)/10;

    speedPWM.K1 = _IQ20(1/(MotorStru.MotorRatedFreq*TbaseFloat/100));
    speedPWM.K2 = _IQ(1/(1+TbaseFloat*2*PI*30));  // 截止频率30Hz
    speedPWM.K3 = _IQ(1)-speedPWM.K2;
    speedPWM.BaseRpm = 12*(MotorStru.MotorRatedFreq/MotorStru.Motor_Poles)/10;
    //SpdMesureSw	 = (Uint32)500*(Uint32)MotorStru.MotorRatedFreq/1300;
    //初始化捕获模块
    gCap.PolePairs = (MotorStru.Motor_Poles>>1);
    gCap.LineEncoder = qep1.LineEncoder*4;
    gCap.init(&gCap);
    // Initialize ADC module
    //这在PWM中已进行，注意与原DSP程序的不同。
    //Intiliaze the Current filter factor 
    ///Spd_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*200));  // Low-pass cut-off frequency//
    
    Spd_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*60));  // Low-pass cut-off frequency//
    Spd_Fir.K2 = _IQ(1) - Spd_Fir.K1;//1/(1+T*2*PI*1000));//SpdCouple_Fir
    
    angle_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*100));
    angle_Fir.K2 = _IQ(1) - angle_Fir.K1;
    

    SpdCouple_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*10));  // Low-pass cut-off frequency//
    SpdCouple_Fir.K2 = _IQ(1) - SpdCouple_Fir.K1;//1/(1+T*2*PI*1000));//

    SpdOut_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*100));  // Low-pass cut-off frequency//
    SpdOut_Fir.K2 = _IQ(1) - SpdOut_Fir.K1;//


    IqRef_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*2000));  // Low-pass cut-off frequency//
    IqRef_Fir.K2 = _IQ(1) - IqRef_Fir.K1;//


    Iq_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*2000));  // Low-pass cut-off frequency
    Iq_Fir.K2 =  _IQ(1) - Iq_Fir.K1;//1/(1+T*2*PI*1000));
    Id_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*2000));  // Low-pass cut-off frequency
    Id_Fir.K2 =  _IQ(1) - Id_Fir.K1;//1/(1+T*2*PI*1000));

    /*IU_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*2000));  // Low-pass cut-off frequency
    IU_Fir.K2 =  _IQ(1) - IU_Fir.K1;//1/(1+T*2*PI*1000));
    IV_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*2000));  // Low-pass cut-off frequency
    IV_Fir.K2 =  _IQ(1) - IV_Fir.K1;//1/(1+T*2*PI*1000));
*/

    Pos_Fir.K1 =  _IQ(1/(1+TbaseFloat*2*PI*10));  // Low-pass cut-off frequency
    Pos_Fir.K2 = _IQ(1) - Pos_Fir.K1;
    // Initialize the PID_REG3 module for Id	pid1_id.Kp = PiAdjVar.ProPar.CurrentProportionGain0;// _IQ(0.5);//0.5  0.1
    // Initialize the PID_REG3 module for Id
    pid1_id.Kp = PiAdjVar.ProPar.CurrentProportionGain0;//_IQ(0.5);//0.5  0.1
    pid1_id.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);	//0.1
    pid1_id.Kd = _IQ(0);///TbaseFloat); 						
    pid1_id.Kc = _IQ(0.9);//0.2
    pid1_id.OutMax = _IQ(1.0);// _IQ(0.85);
    pid1_id.OutMin = _IQ(-1.0);// _IQ(-0.85);    
    // Initialize the PID_REG3 module for position control
    pid1_pos.Kp = _IQ(1.0);
    pid1_pos.Ki = _IQ(0.00);        // Integral term is not used 
    pid1_pos.Kd = _IQ(0.0);        // Derivative term is not used
    pid1_pos.Kc = _IQ(0.0);
    pid1_pos.OutMax = _IQ(1);
    pid1_pos.OutMin = _IQ(-1);
    // Initialize the PID_REG3 module for Iq
    pid1_iq.Kp = PiAdjVar.ProPar.CurrentProportionGain0;//_IQ(0.5);//0.5  0.1
    pid1_iq.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);//0.2		
    pid1_iq.Kd = _IQ(0); 				
    pid1_iq.Kc = _IQ(0.9);//0.2
    pid1_iq.OutMax = _IQ(1.0);// _IQ(0.85);
    pid1_iq.OutMin = _IQ(-1.0);// _IQ(-0.85);    

    // Initialize the PID_REG3 module for speed
    pid1_spd.Kp = PiAdjVar.ProPar.SpeedProportionGain0;//_IQ(0.50);         //0.5        
    pid1_spd.Ki = _IQdiv(_IQmpyI32(Tbase,SpeedLoopPrescaler),PiAdjVar.ProPar.SpeedIntegralGain0);//2.0			
    pid1_spd.Kd = _IQ(0.0);///(TbaseFloat*SpeedLoopPrescaler));					
    pid1_spd.Kc = _IQ(0.9);//0.2
    pid1_spd.OutMax = _IQ(0.5);
    pid1_spd.OutMin = _IQ(-0.5);
    //解耦结构体初始化
    CoupleStruVar.QCoupleQ = _IQ15(1.732*2*3.1415926*MotorStru.MotorRatedFreq/100*CtlCurrentBase/100*MotorStru.PM_DL/1000);//1.414*
    CoupleStruVar.QCoupleD = _IQ15(1.732*2*3.1415926*MotorStru.MotorRatedFreq/100*CtlCurrentBase/100*MotorStru.PM_QL/1000);//1.414*
    CoupleStruVar.RsK = _IQ15(1.414*1.732*CtlCurrentBase/100*MotorStru.PM_RS);

    //EQep1Regs.QPOSCNT = gCap.RawThetaOut + 0x7fffffff;
    qep1.PosInit(0x7fffffff,&qep1);
    qep1.RawThetaABLast = gCap.RawThetaOut;
    qep1.RawThetaPWMlast = gCap.RawThetaOut;
    qep1.RawThetaPWMInput = gCap.RawThetaOut;
    qep1.RawTheta = gCap.RawThetaOut;
    //曲线生成
    CurveParaCalc();
    CurveInitFun(&g_hCurveGenerate);

    ParSettoCTLAnaly();//参数更新

    Sci_Strut.CharInterTim = 120;//120;
    Sci_Strut.FrameTim = 270;//270

    VolProtectVar.VdcMin = 400;//最低线线电压
    //看门狗尚未处理
    PosStateVar.DoorPosOrigin = 0x20000200;
    PosStateVar.DoorPosOriginSave = PosStateVar.DoorPosOrigin;
    //测试用数据
    g_intQepInitCnt = 15000;
    //qep1.CalibratedAngle = _IQ(0.46);

    //看门狗初始化
    IWDG_Init();

    //USART3_Init();
    //Timer3_Init();
    //TIM_CtrlPWMOutputs(TIM1, ENABLE);
    //FMSTR_Init();

    //FMSTR_Init();
    //printf("hello, world");
    //DIS_ON//打开显示
}

/*******************************************************************************
* 函数名称  : KeyDeal
* 描述      : 自带按键处理，对IO数据进行数字滤波(20ms)，并根据滤波后的数据判断按键类型,
*             每2ms处理一次，并刷新数据码管显示。
* 输入      : None
* 输出      : None
* 返回      : 按键类型(长按，短按)、是否有效(yes NO)，按键值（ENT ,左键，右键，向上，向下，ESC）.
*******************************************************************************/
 void KeyDeal(void)
{
    //static u16 ReFlashCnt=0;
    if( ((CTLcmdVar.ExThreeProtocolCmd.bit.OH)||(CTLcmdVar.StandThreeProtocolCmd.bit.OH))||(CTLcmdVar.CMDSel==5)||(CTLcmdVar.CMDSel==1)||(CTLcmdVar.TerminalCmd.bit.OH==1)||(CTLcmdVar.CanCmd.bit.OH==1) )
    {
        OHFlash = 1;
    }
    else   
    {
        OHFlash = 0;
    }
    if(IOFir_stru.ScanTime==0 )
    {
        /*if(DisStru.MenuFunIndex==0)
        {
            ReFlashCnt = 0;
        }
        else
        {
            ReFlashCnt++;
        }*/
        IOFir_stru.ScanTime = 3;  ///20200309  解决LED闪烁问题 
        IOFir_stru.updata(&IOFir_stru);
        //key.InData = ADC_GetKey2Value()|ADC_GetKey1Value();
        key.InData  = IOFir_stru.IOData;
        KeyScan(&key);
        LedFlash();
        //if(ReFlashCnt>150)
        {
            ReFlash = 1; 
            //ReFlashCnt = 0;
        }
        KeyProcess();
        MenuKeySpdAdj();///门机运行速度三档调节
    }
}

/*******************************************************************************
* 函数名称  : TerminalInit1
* 描述      : 输入输出端口初始化。
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void TerminalInit1(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD |  RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC ,ENABLE);
    
    GPIO_StructInit(&GPIO_InitStructure);
    //输入端子，，上接输入
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | 
         GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOB,GPIO_Pin_3);
    GPIO_SetBits(GPIOB,GPIO_Pin_4);
    GPIO_SetBits(GPIOD,GPIO_Pin_2);
    GPIO_SetBits(GPIOC,GPIO_Pin_12);
    
    GPIO_StructInit(&GPIO_InitStructure);
    //输出，开漏输出
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************************************************************
* 函数名称  : CAN_TX_Analy
* 描述      : 状态发生变化或故障发生变化时或定时解发时发送数据
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void CAN_TX_Analy(void)
{
  
  if((Ecan_state.TxdataDealyTime == 0))///( Ecan_state.EcanTxTime == 0 )&&
  {
          Ecan_state.Ecan_Tx.Byte.byte3.bit.BLK = OutDelay.StateCmd.bit.StallOut;
          Ecan_state.Ecan_Tx.Byte.byte0.bit.CDL = OutDelay.StateCmd.bit.CDEndOut;
          Ecan_state.Ecan_Tx.Byte.byte0.bit.ODL = OutDelay.StateCmd.bit.ODEndOut;
          Ecan_state.Ecan_Tx.Byte.byte1.DoorIDCode = 162;

          if((Ecan_state.Ecan_RxD0.Byte.byte1.CMDID == 160) && (OutDelay.DoorLocation == 0)) //标识符、前后门判断
          {
                  Ecan_state.Ecan_Tx.Byte.byte2.all = Ecan_state.Ecan_RxD0.Byte.byte2.all;///命令字回传
          }


          ///if(Ecan_state.Ecan_RxD1.Byte.By2_CRtime < 35)///3.5s
          if(1)///3.5s
          {
                  if(((CTLcmdVar.CanCmd.bit.CD == 1))&&(MTCVar.StateCmd.RunEn != 0)&&(PowerFstVar.PowerFst == 0)&&(OutDelay.StateCmd.bit.StallOut==0))///关门或慢关门||(Ecan_state.Ecan_RxD0.bit.LCD == 1) Ecan_state.Ecan_RxD0.bit.CD
                  {

                          ///tempfj = (Uint16)((Uint32)DoorStudyVar.DecPos.DoorMil*Ecan_state.Ecan_RxD1.Byte.By2_CRtime/35);

                          tempfj = (Uint16)((Uint32)DoorStudyVar.DecPos.DoorMil*TerminalVar.FilterTime/100);
                          if(PosStateVar.DoorPosActualDec.PosMil < tempfj)///以3.5s作为实际关门参考时间
                          {
                                  Ecan_state.Ecan_Tx.Byte.byte3.bit.CRready = 1;///预备允许    暂不启动提前走车，先完成第一步，通讯先打通   20201209
                                  Ecan_state.Ecan_Tx.Byte.byte3.bit.CRrun = 1;///启动允许       先完成第二步，提前开门提前走梯逻辑测试   20201217  20201225  打开功能
                          }
                          else
                          {
                                Ecan_state.Ecan_Tx.Byte.byte3.bit.CRready = 0;///预备允许    
                                Ecan_state.Ecan_Tx.Byte.byte3.bit.CRrun = 0;///启动允许 
                          }
                  }
                  else
                  {
                          Ecan_state.Ecan_Tx.Byte.byte3.bit.CRready = 0;///预备允许    
                          Ecan_state.Ecan_Tx.Byte.byte3.bit.CRrun = 0;///启动允许 
                  }
          }
          else
          {
                  Ecan_state.Ecan_Tx.Byte.byte3.bit.CRready = 0;///预备允许    
                  Ecan_state.Ecan_Tx.Byte.byte3.bit.CRrun = 0;///启动允许 
          }

          Ecan_state.Ecan_Tx.Byte.byte3.bit.MtempOver = FaultVar.fault.FautFlag1.bit.TemperatureMOve;///电机过热


          tempfj = (Uint16)((Uint32)PosStateVar.DoorPosActualDec.PosMil*100/DoorStudyVar.DecPos.DoorMil);///门位置百分比
       /*   Ecan_state.Ecan_Tx.Byte.byte4.DoorAction = 0;///门位置百分比    暂不启动提前走车，先完成第一步，通讯先打通   20201209   tempfj
          Ecan_state.Ecan_Tx.Byte.byte6.Doorstate = 5;///门机状态 正常
          Ecan_state.Ecan_Tx.Byte.byte7.ERRcode = 0x0;///故障码清零
          Ecan_state.Ecan_Tx.Byte.byte7.ERRcode = (Uint16) (FaultVar.FaultPre & 0x000000ff);///当前故障码
         */ 
          
          
          Ecan_state.Ecan_Tx.Byte.byte4.all = (p_intKey1>>8)&0x00FF;///门位置百分比    暂不启动提前走车，先完成第一步，通讯先打通   20201209   tempfj
          Ecan_state.Ecan_Tx.Byte.byte5.all = p_intKey1&0x00FF;;
          Ecan_state.Ecan_Tx.Byte.byte6.all = (p_intKey2>>8)&0x00FF;
          Ecan_state.Ecan_Tx.Byte.byte7.all = p_intKey2&0x00FF;

          
          
          
          
          
          
  }
/*
    
    if( g_u32StateSave!=(Ecan_state.Ecan_Tx.Byte.byte1.all+(Ecan_state.Ecan_Tx.Byte.byte0.all<<8)) )
    {
        g_u32StateSave = (Ecan_state.Ecan_Tx.Byte.byte1.all+(Ecan_state.Ecan_Tx.Byte.byte0.all<<8));
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.byte0.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = Ecan_state.Ecan_Tx.Byte.byte1.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = Ecan_state.Ecan_Tx.Byte.byte2.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.Ecan_Tx.Byte.byte3.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx.Byte.byte4.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = Ecan_state.Ecan_Tx.Byte.byte5.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = Ecan_state.Ecan_Tx.Byte.byte6.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = Ecan_state.Ecan_Tx.Byte.byte7.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //帧长
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
        ECANSendStack.BufHead ++;
        if(ECANSendStack.BufHead >= ECANBUFLENMAX)
        {
            ECANSendStack.BufHead =0;
        }   
        else{} 
    }*/
    //定时触发发送
    if( Ecan_state.EcanTxTime==0 )
    {
      
          
          Ecan_state.Ecan_Tx.Byte.byte4.all = (p_intKey1>>8)&0x00FF;///门位置百分比    暂不启动提前走车，先完成第一步，通讯先打通   20201209   tempfj
          Ecan_state.Ecan_Tx.Byte.byte5.all = p_intKey1&0x00FF;;
          Ecan_state.Ecan_Tx.Byte.byte6.all = (p_intKey2>>8)&0x00FF;
          Ecan_state.Ecan_Tx.Byte.byte7.all = p_intKey2&0x00FF;

      
      
      
        Ecan_state.EcanTxTime = CAN_SEND_TIME;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.byte0.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = Ecan_state.Ecan_Tx.Byte.byte1.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = Ecan_state.Ecan_Tx.Byte.byte2.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.Ecan_Tx.Byte.byte3.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx.Byte.byte4.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = Ecan_state.Ecan_Tx.Byte.byte5.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = Ecan_state.Ecan_Tx.Byte.byte6.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = Ecan_state.Ecan_Tx.Byte.byte7.all;
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //帧长
        ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
        ECANSendStack.BufHead ++;
        if(ECANSendStack.BufHead >= ECANBUFLENMAX)
        {
            ECANSendStack.BufHead =0;
        }   
        else{} 
    }
 }

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 

  
  /* Infinite loop */
  while (1)
  {
  }
}



#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
