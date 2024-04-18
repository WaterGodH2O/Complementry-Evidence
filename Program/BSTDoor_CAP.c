/* ==============================================================================
�ļ�����	BSTDoor_Cap.c

��  ����	����PWM�ͱ������ź�

��  ����	�Ϻ�JMJ R&D

Ӳ  ����	400W�Ż������� R0909005 DSP: TMS320F280x
=====================================================================================
�޸ļ�¼��
=================================================================================  */ 


#include "main.h"
#include "IQmathLib.h"



//����������λ��PWM������ʼ��
CAPGEN gCap = CAPGEN_DEFAULTS;
static Uint32 g_lCapBuffer[4];


void  F280X_CAP_Init(CAPGEN *p)
{
    
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  DMA_InitTypeDef           DMA_InitStructure;
  
   
 /* DMA1 Channel5 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (Uint32)TIM2_CCR1_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (Uint32)g_lCapBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  /* Enable DMA1 Channel5 */
  DMA_Cmd(DMA1_Channel5, ENABLE);

    
#if defined(TIMER2_HANDLES_PWM)
    TIM_ICInitTypeDef  TIM2_ICInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);     //Open TIM4 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //open gpioB clock
 
    
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;                               //GPIO??
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    TIM_TimeBaseStructure.TIM_Period = 5000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ  
    TIM_TimeBaseStructure.TIM_Prescaler =71; //1MHZ���� 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
    /*�����ж����ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                     
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;                   
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
    TIM2_ICInitStructure.TIM_ICFilter = 0x0; 
 
    TIM_PWMIConfig(TIM2, &TIM2_ICInitStructure);     //PWM��������           
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);     //ѡ����Ч�����        
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //����Ϊ���Ӹ�λģʽ
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);                                       
    TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);          //�ж�����TIM_IT_Update
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //����жϱ�־λ TIM_IT_Update
    TIM_Cmd(TIM2, ENABLE); 
    
    // TIM3_OC4 DMA enable
    TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_CC2, ENABLE);
#endif
    
    
                                     			

}

void F280X_CAP_Update(CAPGEN *p)
{
  static Uint16 FstCap = 0,CAPErrorCnt = 0,PWM_ResumCnt = 50;
  static 	int32 thetasave = 0;
  int16 temp;
  //return;

  p->PWMCpaEn = 0;
  if (p->NoSignalTime >= p->CheckSignalTime)//3000
  {
    if( p->NoSignalFlag==0 )
    {
      p->NoSignalFlag = 1;
      PWM_ResumCnt = 0;
    }
    p->NoSignalTime = p->CheckSignalTime;//3000
  }
  else
  {
    p->NoSignalTime ++;
  }
	
  if( p->PowerOnDelay==0 )//�������ϵ���ʱ����
  {
  }
  else
  {
    p->PowerOnDelay--;
    p->InitMechthetaTicker = 0;
    p->InitMechthetaTicker = 0;
  }
  if( (TIMER_PWM_HAND->SR&TIM_IT_CC1)==TIM_IT_CC1 )//����1���������¼�
  {
    TIMER_PWM_HAND->SR= (uint16_t)~TIM_IT_CC1;//����жϱ�־λ 
    if(PWM_ResumCnt<30)
    {
      PWM_ResumCnt++;
    }
    if(FstCap<=10)
    {
      FstCap++;
      return;
    }

  }
  else
  {
    if( (TIMER_PWM_HAND->SR&TIM_IT_Update)==TIM_IT_Update )
    {
      TIMER_PWM_HAND->SR= (uint16_t)~TIM_IT_Update;//��������־
    }
    else
    {
    }
    return;
  }
  if(PWM_ResumCnt>=30)
  {
    p->NoSignalFlag = 0;
    p->NoSignalTime = 0;
  }
  p->PWMCpaEn = 1;
	
  p->Period1 = TIMER_PWM_HAND->CCR2;
  p->CapPeriod = TIMER_PWM_HAND->CCR1;
  p->Period2 = p->CapPeriod-p->Period1;
  

//	p->CAPType = 1;
  switch(p->CAPType)
  {
    case 0://�µ���
     /// if (p->CapPeriod < 3276 || p->CapPeriod > 4900)//����Χ80%~120%    AS5045 gw
      if ((p->CapPeriod < 1163) || (p->CapPeriod > 2295))//����Χ80%~120%
      {
          p->CapPeriod = p->Period1 + p->Period2;

          if(CAPErrorCnt<200)
          {
            CAPErrorCnt++;
          }
          else
          {
            p->NoSignalFlag = 1;
          }
          return;
      }
      // 	�����е�Ƕ� Q24
      p->MechThetaBuf = _IQdiv(p->Period2, p->CapPeriod);	
    break;
    case 1://������
      if (p->CapPeriod < 30720 || p->CapPeriod > 92160)//����Χ80%~120%
      {
      p->CapPeriod = p->Period1 + p->Period2;

      if(CAPErrorCnt<200)
      {
      CAPErrorCnt++;
      }
      else
      {
      p->NoSignalFlag = 1;
      }
      return;
      }
      // 	�����е�Ƕ� Q24
      p->MechThetaBuf = _IQdiv(p->Period2, p->CapPeriod);
    break;
    default:break;
  }

  CAPErrorCnt = 0;

	    

  if (p->InitMechThetaFlag == 0)
  {
    //�ж����ζ����ĽǶ��Ƿ���<1��(��е�Ƕ�)�ķ�Χ��,������������ʼ����λ
    if( _IQabs(thetasave-p->MechThetaBuf) <_IQ(0.002778) )
    {
      p->InitMechthetaTicker ++;
      p->InitMechTheta += p->MechThetaBuf;
    }
    else
    {
      p->InitMechthetaTicker = 0;	
      p->InitMechTheta = 0;
    }
    thetasave = p->MechThetaBuf;

    temp = ((p->CapPeriod * 18) >> 3);
    temp = labs(temp - p->LineEncoder);
    if( temp<(p->LineEncoder>>3) )//�������12.5%
    {
      p->ParSetErrorFlag = 0;
    }
    else
    {
      p->ParSetErrorFlag = 1;
    }

    if( p->InitMechthetaTicker >= 16)
    {
      p->InitMechThetaFlag = 1;
      p->CheckSignalTime = 300;

      p->InitMechTheta = p->InitMechTheta >> 4;
      p->RawThetaOut = _IQmpyI32int(p->MechThetaBuf,p->LineEncoder);		
      p->ElecTheta = p->PolePairs * p->MechThetaBuf; 
    }
    else
    {
    }
  }
  else if (p->InitMechThetaFlag == 2)
  {		
    p->PowerOnDelay = 1500;
    p->MechThetaDelta = p->MechThetaBuf - p->MechThetaOld;
    //���0.5��ʱ��ȷ�� 0.5/360 = 0.0013889
    if (p->MechThetaDelta >= _IQ(0.0013889) || p->MechThetaDelta <= _IQ(-0.0013889))
    {
      if (p->MechThetaDelta > 0)
      {
        if (p->MechThetaDelta < _IQ(0.5))//����180��˵��û�з��������С�Ƕ��л�
        {
          p->DirectionHad = 1;
        }
        else
        {
          p->DirectionHad = 0;
          p->MechThetaDelta -= _IQ(1.0);//ȡ����ֵ
          p->IndexSyncFlag = 2;
        }
      }
      else if (p->MechThetaDelta < 0)
      {
        if (p->MechThetaDelta < _IQ(-0.5))//����180��˵��û�з��������С�Ƕ��л�
        {
          p->DirectionHad = 1;
          p->MechThetaDelta += _IQ(1.0);//ȡ����ֵ
          p->IndexSyncFlag = 1;
        }
        else
        {
          p->DirectionHad = 0;
        }
      }
      else
      {
      }
      p->MechThetaOld = p->MechThetaBuf;
    }
    else
    {
    }
    p->MechTheta = p->MechThetaBuf;
    p->RawThetaOut = _IQmpyI32int(p->MechTheta,p->LineEncoder);		

    p->MechTheta &= 0x00FFFFFF; 	

    //�����Ƕ� Q24
    p->ElecTheta = p->PolePairs * p->MechTheta;         // 	Q24 = Q0*Q24 
    p->ElecTheta &= 0x00FFFFFF;  
  }
  else
  {
  }
}

