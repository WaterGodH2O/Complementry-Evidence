 #include "main.h"
#include "IQmathLib.h"
#include "SciToOprt.h" 
#include "BSTDoor_I2C.h"
#include "stm32_eval_i2c_ee_cpal.h"
//#include "parameter.h"
#include "BstDoorFun.h"
#include "stm32f10x_can.h"
//#include <math.h>	  
#include "i2c_ee_dma.h"
#include "BstDoor_Curve.h"
#include "display_key.h"

#include "DoorCtl_Globals.h"
extern Uint16 TestAdd2;
extern int ADCL_adFlag;

Uint16 DoorWidthAutoStudy_en = 1;///�ſ���ѧϰʹ��λ��Ĭ��Ϊ1
Uint16 Doorwidthsave = 0;///���籣���ſ��־λ
Uint16 DoorFirstLod_Flag = 1;///�����ϵ�Ϊ�����ű�־

Uint16 AutoWidthStudyRunCls_initOK = 0;///�ſ���ʶ��������������Ž�����־
Uint16 AutoWidthStudyRunOpen_initOK = 0;///�ſ���ʶ��������������Ž�����־
Uint16 AutoWidthStudyRun_initOver = 0;///�ſ���ʶ��������н�����־
Uint16 LCD_DoorMil = 0;
Uint16 LOD_DoorMil = 0;


Uint16 OD_TorqueUPTime = 0;
Uint16 OD_TorqueDWNTime = 100;
Uint16 CD_TorqueUPTime = 0;
Uint16 CD_TorqueDWNTime = 100;
Uint16 *pdatatemp;
DCC_ANALY_TYPE DccAnalyVar = DCC_ANALY_TYPE_DEFAULTS;

UNIT_TRANSFORM_STRUT g_hUnitTransform =  UNIT_TRANSFORM_STRUT_DEFAULTS;

_iq MotorFreMaxQ = 0;
_iq g_lRefCurrentQ = _IQ(0.1);

const _iq VdcSub[9] = {_IQ(0.190333),_IQ(0.285333),_IQ(0.379667),_IQ(0.472),_IQ(0.555333),_IQ(0.641333),_IQ(0.71),_IQ(0.788677),_IQ(0.842333)};
const _iq VdcSubK[10] = {_IQ(0.872929),_IQ(0.918253),_IQ(0.908954),_IQ(0.933251),_IQ(0.970213),_IQ(1.03909),_IQ(1.177443),_IQ(1.163),_IQ(1.2979143),_IQ(1.0104718),};
const _iq VdcSubOffset[10] = {_IQ(0.00657091870),_IQ(0.00173439148940),_IQ(-0.001243049310),_IQ(0.00425531914890),_IQ(0.01856628297870),_IQ(0.05136034468080),_IQ(0.135377446808510),_IQ(0.1227021276595744680),_IQ(0.222780878190),_IQ(0.0222766)};


extern struct Display DisStru;
extern Uint16 KeyAvailable;
extern Uint16 Speed_llkey;
extern Uint16 Speed_KeyCountOver;  ///�������
Uint16 Speed_cnt=800;
Uint16 Speed_step = 2;
Uint16 Speed_dis = 0;

Uint16 FstNum = 0x55AA;
const Uint16 *FstParAdd = &FstNum; //����������ַ


int32 RSmidval = 0;
Uint16 DoorRunOver = 0;
Uint16 DoorRunStart = 0;
Uint16 DoorInitEn = 0;
Uint16 DoorIniti = 0;
int32 MageStudy_Mech = 0;

extern ECAN_STRUCT Ecan_state;
extern struct ECANSendBufStruct ECANSendStack;
//extern CanTxMsg TMessage;  
//extern CanTxMsg RMessage;  
extern u32 CpuID[3];

Uint8 ARM_ID[24] = {0,0,0,0};
Uint8 ARM_IDflag=0;

Uint8 Key_down_last = 1;
Uint8 Key_down = 1;

Uint16 CanCommAdd = 0;

void MagZeroStudyMain(void)
{
    
        Magzerostudy.StateCmd.MagZeroFlag = gCap.CapOver;
        gCap.CapOver = 0;
    //gCap.CapOver = 0;
    
    
    if(	(Magzerostudy.StateCmd.MagZeroPosEn==1))
    {
        
        Magzerostudy.MagZeroCurrentRating =_IQmpy( _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase),_IQ(1.0));// _IQ(0.4025);
        MTCVar.StateCmd.CTLType = 0;
        Magzerostudy.cal(&Magzerostudy);
        if( (Magzerostudy.StateCmd.MagZeroPosOver==1)||(Magzerostudy.StateCmd.MagZeroPosError==1))
        {
            Magzerostudy.StateCmd.MagZeroPosEn = 0;
            StudyType = 0;
            Magzerostudy.StateCmd.MagZeroPosRun = 0;
            Magzerostudy.StateCmd.MagZeroPosOver = 0;
            MTCVar.StateCmd.CTLType = 1;
            
            if(Magzerostudy.StateCmd.MagZeroPosError==0)
            {
                E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[47].ParaPro.E2ROMAdd;
                E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[47].ParaPro.E2ROMBlock;
                E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
                E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
                E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[47].ParAdd);
                E2PROM_TaskSet(&E2promTask);
                
                Sci_Strut.ProRest = 1;
            }
          
        }
        if( Magzerostudy.StateCmd.MagZeroPosRun==1 )
        {
            MTCVar.StateCmd.RunEn = 1;

        }
        else
        {
            MTCVar.StateCmd.RunEn = 0;
        }
    }
    else
    {
        Magzerostudy.StateCmd.Step = 0;
    }
}




/*******************************************************************************
* ��������  : MagStudyMain
* ����      : �ż�ѧϰ��ѧϰ�ɹ��󱣴�Ƕȣ����ѧϰ���ɹ����Զ��˳�
* ����      : None
* ���      : None
* ����      : None
*******************************************************************************/ 

void MagStudyMain(void)
{
	if(MageStudy.StateCmd.MageticStudyEn==1)
	{
		MageStudy.MageticCurrentRating =_IQmpy( _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase),_IQ(0.9));// _IQ(0.4025);
		MageStudy.MageticAngleInput = gCap.ElecTheta;
		MTCVar.StateCmd.CTLType = 0;
		MageStudy.cal(&MageStudy);
		if( (FaultVar.fault.FautFlag0.bit.QEPErrorFlag==1)||(MageStudy.StateCmd.MageticStudyOver==1)||(MageStudy.StateCmd.MageticStudyError==1))
		{
			MageStudy.StateCmd.MageticStudyEn = 0;
			StudyType = 0;
			MageStudy.StateCmd.MageticStudyRun = 0;
			MageStudy.StateCmd.MageticStudyOver = 0;
			MTCVar.StateCmd.CTLType = 1;
			if( (MageStudy.StateCmd.MageticStudyError==0)&&(FaultVar.fault.FautFlag0.bit.QEPErrorFlag==0) )
			{
                                E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[47].ParaPro.E2ROMAdd;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[47].ParaPro.E2ROMBlock;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[47].ParAdd);
				E2PROM_TaskSet(&E2promTask);
                                
                                Sci_Strut.ProRest = 1;
				///gCap.InitTime = 100;
				///PowerFstVar.ProInit = 1;
				///PowerFstVar.PowerFst = 1;
				///gCap.InitMechThetaFlag = 0;
			}
		}
		if( MageStudy.StateCmd.MageticStudyRun==1 )
		{
			MTCVar.StateCmd.RunEn = 1;
		
		}
		else
		{
			MTCVar.StateCmd.RunEn = 0;
		}
	}
	else
	{
		MageStudy.StateCmd.Step = 0;
		MTCVar.StateCmd.CTLType = 1;
	}
}


void DoorStudyMain(void)
{
	static Uint16  p_uOD_TorqueSwTime = 0;
	if( DoorStudyVar.StateCmd.StudyEn==1 )
	{
		MTCVar.StateCmd.CTLType = 1;
		StallCheckStru.CheckTime = 2000;//ѧϰʱ��ת���ʱ��Ϊ2S
		DoorStudyVar.StateCmd.StallFlag = StallCheckStru.StateCmd.StallFlag;
		DoorStudyVar.PosInput = qep1.OutputRawTheta;
		DoorStudyVar.calc(&DoorStudyVar);

		if( (FaultVar.fault.FautFlag0.bit.QEPErrorFlag==1)||(DoorStudyVar.StateCmd.StudyError==1)||(DoorStudyVar.StateCmd.StudyOver==1) )
		{
			DoorStudyVar.StateCmd.StudyEn = 0;
			DoorStudyVar.StateCmd.Run = 0;
			StudyType = 0;
			p_uOD_TorqueSwTime = 0;
			DoorFianlVar.StateCmd.ODHDToHFst = 0;

			//�ж��ſ������о����ϵ
			if( (DoorStudyVar.Width<(g_hCurveGenerate.PolyLineVar.ODCreepDis+g_hCurveGenerate.PolyLineVar.ODStartDis))||(DoorStudyVar.Width<(g_hCurveGenerate.PolyLineVar.CDCreepDis+g_hCurveGenerate.PolyLineVar.CDRetiringCamDis)) )
			{
				DoorStudyVar.StateCmd.StudyError = 1;
			}
			if( (DoorStudyVar.StateCmd.StudyError==0)&&(FaultVar.fault.FautFlag0.bit.QEPErrorFlag==0) )
			{
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[35].ParaPro.E2ROMAdd;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[35].ParaPro.E2ROMBlock;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[35].ParAdd);
				E2PROM_TaskSet(&E2promTask);
                                
                                DoorWidthAutoStudy_en = 0;///ѧϰ��ɺ���ѧϰ��־����
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[8].ParaPro.E2ROMAdd;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[8].ParaPro.E2ROMBlock;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
				E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[8].ParAdd);
				E2PROM_TaskSet(&E2promTask);
				
                                Sci_Strut.ProRest = 1;
                               
                                /* QeqResumeVar.ResumeInitTime =  QERINTIETIME;
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
                                qep1.QepABError = ABSIGNALNONE;*/
			}
			else
			{
				DoorStudyVar.Width = DoorStudyVar.WidthSave;
			}
                        g_uStudyEn = 0;
                        g_uOneKeyStudy = 0;
		}
		//ѧϰ��ɺ����ת������ع���
		StallCheckStru.StateCmd.ClrStall = 1;
		TorqueProtecVar.StateCmd.ClrEn = 1;
	
		g_hCurveGenerate.RealCurveCalc.CmdState.StudyMode  = DoorStudyVar.StateCmd.StudyEn;

		MTCVar.StateCmd.RunEn = DoorStudyVar.StateCmd.Run;

		/*if( DoorStudyVar.StateCmd.RunDir==0)
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 1;//��
			if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque	)
			{
				
				if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
				{
					p_uOD_TorqueSwTime++;
					//���ű�����
					//MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
				}
				if(p_uOD_TorqueSwTime>100)
				{
					DoorFianlVar.StateCmd.ODHDToHFst = 1;
					MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
				}
			}
			else
			{
				if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
				{
					//���������
					MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
				}
			}
		}
		else
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 0;//����
		
		}*////�Ľ���ѧϰ�ſ��߼�
                
                if( DoorStudyVar.StateCmd.RunDir==0)
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 1;//����
		}
		else
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 0;//����
		
		}
		MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
		MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;

		StallCheckStru.StateCmd.ClrStall = DoorStudyVar.StateCmd.ClrStall;
		DoorStudyVar.StateCmd.ClrStall = 0;
	}									   								   
}
//-------------------------------------------------------------------------------------
 // Function:        E2prom_TaskManage
//  Description:     Manage the task of e2prom with read or write
//  Calls:           None
//  Called By:       Main
//  Input:           None
//  Output:          None
//  Return:          None
//  Others:          None
//         
//-------------------------------------------------------------------------------------
void E2prom_TaskManage(void)
{	
  //Uint16 temp;
 // Uint16 addr;
 // static Uint8 p_charEEPORMStep = EEPROM_IDLE,p_charWriteCnt = 0;//typedef enum {EEPROM_IDLE = 0, EEPROM_BUSY = 1,EEPROM_OS_END = 2} EEPROM_State;

  if( gE2rom.CmdState.OSBusy==FALSE )
  {
    if( gE2rom.CmdState.ReadOverFlag==TRUE )
    {
      /*for(temp=0;temp<gE2rom.E2promDataSturct.nrData;)
      {
        if(E2promTask.TaskReadSP!=0)
        {	
          *((Uint16 *)(*(E2promTask.E2PROM_Distribute[(E2promTask.TaskReadSP)-1].parameterlink+ParaLen*temp))) = ReadBuff[temp];
        }
        else
        {
          *((Uint16 *)(*(E2promTask.E2PROM_Distribute[E2PROM_MAXNUM-1].parameterlink+ParaLen*temp))) = ReadBuff[temp];
        }
        temp += 1;
      }*/
      gE2rom.CmdState.ReadOverFlag = FALSE;
    }
    else if( gE2rom.CmdState.WriteOverFlag==TRUE )
    {
      gE2rom.CmdState.WriteOverFlag = FALSE;
    }
    else 
    {
      if( E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].ResponseFlag==1 )
      {

        if( E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].ControlCMD==WRITECMD )
        {
          gE2rom.E2promDataSturct.E2ROMAdd = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMAdd;
          gE2rom.E2promDataSturct.E2ROMBlock = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMBlock;
          gE2rom.E2promDataSturct.nrData = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum;
          gE2rom.E2promDataSturct.parameterlink = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].parameterlink;
          /*pdatatemp = WriteBuff;
          for(temp=0;temp<gE2rom.E2promDataSturct.nrData;)
          {
          WriteBuff[temp] =  *((Uint16 *)(*(E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].parameterlink+ParaLen*temp)));
          temp++;	
          }*/
          E2PROM_TaskScheduler(&E2promTask);
          gE2rom.Command = WRITECMD;
        }
        else if( E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].ControlCMD==READCMD )
        {
          gE2rom.E2promDataSturct.E2ROMAdd = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMAdd;
          gE2rom.E2promDataSturct.E2ROMBlock = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMBlock;
          gE2rom.E2promDataSturct.nrData = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum;
          gE2rom.E2promDataSturct.parameterlink = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].parameterlink;
          E2PROM_TaskScheduler(&E2promTask);
          gE2rom.Command = READCMD;
        }
        else
        {
        }
      }
    }
  }
        
   /*if( E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].ResponseFlag==1 )
    {    
        if( E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].ControlCMD==1 )
        {
            switch(p_charEEPORMStep)
            {
              case EEPROM_IDLE:
                for(temp=0;temp< E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum;temp++)
                {
                    EEpromWBuffer[temp*2] = (u8)( (*((u16 *)(*(E2promTask.E2PROM_Distribute[(E2promTask.TaskReadSP)].parameterlink+3*temp))))>>8);
                    EEpromWBuffer[temp*2+1] =(u8)( (*((u16 *)(*(E2promTask.E2PROM_Distribute[(E2promTask.TaskReadSP)].parameterlink+3*temp))))&0xff );       
                }
                addr = E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMBlock*8+E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMAdd;
                //I2C_EE_BufferWrite(EEpromRWBuffer,addr,E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2);
                
                 // Write Data in EEPROM 
                sEE_WriteBuffer(&sEE_DevStructure, EEpromWBuffer, addr, E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2);
                p_charEEPORMStep = WRITE_EEPROM_BUSY;
              break;
              
              case WRITE_EEPROM_BUSY:
                // Wail until communication is complete 
                if((sEE_GetEepromState(&sEE_DevStructure) != sEE_STATE_IDLE) && 
                      (sEE_GetEepromState(&sEE_DevStructure) != sEE_STATE_ERROR))
                { 
                  // Application may perform other tasks while CPAL write operation is ongoing 
                }
                else
                {
                  addr = (E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMBlock)*8+E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMAdd;
                  //Read Data from EEPROM 
                  sEE_ReadBuffer(&sEE_DevStructure, EEpromRBuffer, addr, E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2);
                  p_charEEPORMStep = READ_EEPROM_BUSY;
                }
              break;
              case READ_EEPROM_BUSY:
              // Wail until communication is complete 
              if((sEE_GetEepromState(&sEE_DevStructure) != sEE_STATE_IDLE) && 
                    (sEE_GetEepromState(&sEE_DevStructure) != sEE_STATE_ERROR))
              {
                // Application may perform other tasks while CPAL read operation is ongoing 
              }
              else
              {
                if(sEE_GetEepromState(&sEE_DevStructure) == sEE_STATE_ERROR)
                {
                  gE2rom.CmdState.WtireReadError = 1;
                }
                else
                {
                  for(temp=0;temp< E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2;temp++)
                  {	
                    if( (EEpromRBuffer[temp]==EEpromWBuffer[temp]))
                    {
                    }
                    else
                    {
                      break;
                    }
                  }
                  //�ж��Ƿ�һ��
                  if(temp==E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2)
                  {
                    p_charEEPORMStep = EEPROM_OS_END;//�ɹ�
                  }
                  else
                  {
                      if(p_charWriteCnt<3)
                      {
                        p_charWriteCnt++;
                        p_charEEPORMStep = EEPROM_IDLE;
                      }
                      else
                      {
                        //EE2PROM����
                        gE2rom.CmdState.WtireReadError = 1;
                        p_charWriteCnt = 0;
                        p_charEEPORMStep = EEPROM_OS_END;//�ɹ�
                      }
                  }
                }
                
              }
            break;
              
              case EEPROM_OS_END:
                p_charWriteCnt = 0;
                E2PROM_TaskScheduler(&E2promTask);
                p_charEEPORMStep = EEPROM_IDLE;
              break;
              
              default:
                break;
            }
        }
        else if( E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].ControlCMD==0 )
        {
          switch(p_charEEPORMStep)
          {
            case EEPROM_IDLE:
              addr = (E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMBlock)*8+E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].E2ROMAdd;
              //Read Data from EEPROM 
              sEE_ReadBuffer(&sEE_DevStructure, EEpromRBuffer, addr, E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2);
              p_charEEPORMStep = READ_EEPROM_BUSY;
            break;
            
            case READ_EEPROM_BUSY:
              // Wail until communication is complete 
              if((sEE_GetEepromState(&sEE_DevStructure) != sEE_STATE_IDLE) && 
                    (sEE_GetEepromState(&sEE_DevStructure) != sEE_STATE_ERROR))
              {
                // Application may perform other tasks while CPAL read operation is ongoing 
              }
              else
              {
                if(sEE_GetEepromState(&sEE_DevStructure) == sEE_STATE_ERROR)
                {
                  gE2rom.CmdState.WtireReadError = 1;
                }
                else
                {
                  for(temp=0;temp< E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum;temp++)
                  {	
                    *((u16 *)(*(E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].parameterlink+3*temp))) = (EEpromRBuffer[temp*2]<<8)+EEpromRBuffer[temp*2+1];
                  }
                }
                p_charEEPORMStep = EEPROM_OS_END;
                
              }
            break;
            
            case EEPROM_OS_END:
              E2PROM_TaskScheduler(&E2promTask);
            break;
            
            default:
              break;
          }
          
            
            
            
        }
        else
        {
          p_charEEPORMStep = 0;
        }
    }
*/
}
/*******************************************************************************
* ��������  : SpeedCal
* ����      : ��ɶԴű������ٶȼ���Ĵ�����Ϊ�ű������ź����ٶ�ƽ������£�Ҳ
              ���нϴ󲨶������Ҫ������ж����ƽ��ֵ���Լ����ٶ�ƽ�ȣ������?
			  Ϊ��Ҫ�����ԣ���˻�Ҫ���п��ٴ���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void SpeedCal(void)
{
	static _iq speedbuff[64],speedsum,speedmax=0,speedmin=0;
	static Uint16 puslecntsave=0;
	Uint16 tempcnt;
	_iq temp;


	if( puslecntsave!=qep1.PusleCnt )
	{

		speedbuff[qep1.PusleCnt] = speed2.Speed;
		temp =_IQabs(speed2.Speed);
		if(temp>speedmax)
		{
			speedmax = temp;
		}
		else if(temp<speedmin)
		{
			speedmin = temp;
		}
		if(speedmax> _IQmpy(speedmin,_IQ(2.0)) )
		{
			for(tempcnt=0;tempcnt<64;tempcnt++)
			{
			//	speedbuff[tempcnt] = speed2.Speed;
			}
		//	speedmin = temp;
		//	speedmax = temp;
		}
		speedsum = 0;
		for(tempcnt=0;tempcnt<8;tempcnt++)
		{
			speedsum += speedbuff[tempcnt];
		}

	//	speed2.Speed = speedsum>>5;
		puslecntsave = qep1.PusleCnt;
	}
	else
	{
	}
	
	if(qep1.PusleCnt>=7)
	{
		qep1.PusleCnt = 0;
	}
	speed2.Speed = speedsum>>3;

}


/*******************************************************************************
* ��������  : K400EcanCommAnaly
* ����      : K400CANbusָ�����ȼ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/

void K400EcanCommAnaly(void)
{
    //static u16 p_uintCMD = 0;
    switch(g_uODCDSignalEn)
    {
            case 0:
              
            if( (Ecan_state.Ecan_InitSave.bit.CD==1)&&(Ecan_state.Ecan_InitSave.bit.OD==1) )//11X
            {
                    CTLcmdVar.CanCmd.bit.CD = 0;
                    CTLcmdVar.CanCmd.bit.OD = 0;
                    CTLcmdVar.CanCmd.bit.LCD = 0;
            }
            else
            {
                    if( (Ecan_state.Ecan_InitSave.bit.LCD==1)&&(Ecan_state.Ecan_InitSave.bit.OD==1) )//101
                    {
                            CTLcmdVar.CanCmd.bit.CD = 0;
                            CTLcmdVar.CanCmd.bit.OD = 0;
                            CTLcmdVar.CanCmd.bit.LCD = 0;
                    }
                    else
                    {
                            if( (Ecan_state.Ecan_InitSave.bit.OD==1)&&(Ecan_state.Ecan_InitSave.bit.CD==0)&&(Ecan_state.Ecan_InitSave.bit.LCD==0) )//100
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 1;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                            else if( (Ecan_state.Ecan_InitSave.bit.OD==0)&&(Ecan_state.Ecan_InitSave.bit.CD==1)&&(Ecan_state.Ecan_InitSave.bit.LCD==0) )//010
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 1;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                            else if( (Ecan_state.Ecan_InitSave.bit.OD==0)&&(Ecan_state.Ecan_InitSave.bit.LCD==1) )//010
                            {
                                    CTLcmdVar.CanCmd.bit.LCD = 1;
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    
                            }
                            else
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                    }
            }

            break;
            case 1:
    
            break;
            case 2:
    
            if( (Ecan_state.Ecan_InitSave.bit.CD==1)&&(Ecan_state.Ecan_InitSave.bit.OD==1) )//11X
            {
                    CTLcmdVar.CanCmd.bit.CD = 0;
                    CTLcmdVar.CanCmd.bit.OD = 0;
                    CTLcmdVar.CanCmd.bit.LCD = 0;
            }
            else
            {
                    if( (Ecan_state.Ecan_InitSave.bit.LCD==1)&&(Ecan_state.Ecan_InitSave.bit.OD==1) )//101
                    {
                            CTLcmdVar.CanCmd.bit.CD = 0;
                            CTLcmdVar.CanCmd.bit.OD = 1;
                            CTLcmdVar.CanCmd.bit.LCD = 0;
                    }
                    else
                    {
                            if( (Ecan_state.Ecan_InitSave.bit.OD==1)&&(Ecan_state.Ecan_InitSave.bit.CD==0)&&(Ecan_state.Ecan_InitSave.bit.LCD==0) )//100
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 1;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                            else if( (Ecan_state.Ecan_InitSave.bit.OD==0)&&(Ecan_state.Ecan_InitSave.bit.CD==1)&&(Ecan_state.Ecan_InitSave.bit.LCD==0) )//010
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 1;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                            else if( (Ecan_state.Ecan_InitSave.bit.OD==0)&&(Ecan_state.Ecan_InitSave.bit.LCD==1) )//010
                            {
                                    CTLcmdVar.CanCmd.bit.LCD = 1;
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                            }
                            else
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                    }
            }
            break;
            case 3:
    
            if( (Ecan_state.Ecan_InitSave.bit.CD==1)&&(Ecan_state.Ecan_InitSave.bit.OD==1) )//11X
            {
                    CTLcmdVar.CanCmd.bit.CD = 0;
                    CTLcmdVar.CanCmd.bit.OD = 0;
                    CTLcmdVar.CanCmd.bit.LCD = 0;
            }
            else
            {
                    if( (Ecan_state.Ecan_InitSave.bit.LCD==1)&&(Ecan_state.Ecan_InitSave.bit.OD==1) )//101
                    {
                            CTLcmdVar.CanCmd.bit.CD = 0;
                            CTLcmdVar.CanCmd.bit.OD = 0;
                            CTLcmdVar.CanCmd.bit.LCD = 1;
                    }
                    else
                    {
                            if( (Ecan_state.Ecan_InitSave.bit.OD==1)&&(Ecan_state.Ecan_InitSave.bit.CD==0)&&(Ecan_state.Ecan_InitSave.bit.LCD==0) )//100
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 1;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                            else if( (Ecan_state.Ecan_InitSave.bit.OD==0)&&(Ecan_state.Ecan_InitSave.bit.CD==1)&&(Ecan_state.Ecan_InitSave.bit.LCD==0) )//010
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 1;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                            else if( (Ecan_state.Ecan_InitSave.bit.OD==0)&&(Ecan_state.Ecan_InitSave.bit.LCD==1) )//010
                            {
                                    CTLcmdVar.CanCmd.bit.LCD = 1;
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                            }
                            else
                            {
                                    CTLcmdVar.CanCmd.bit.CD = 0;
                                    CTLcmdVar.CanCmd.bit.OD = 0;
                                    CTLcmdVar.CanCmd.bit.LCD = 0;
                            }
                    }
            }
            break;
            default:
            break;
    }

}

/*******************************************************************************
* ��������  : K400EcanCommErrCkeck
* ����      : CANBUS�������������ж�
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void K400EcanCommErrCkeck(void)
{
    /*if(((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
      (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0))&& (Ecan_state.InitOver))//�ж��ݺż�ǰ����	��ʼ�����̽���
      {
          if((((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
          ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))) ///�ж�ǰ����һ����
        {  
            if ((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x50) && (Ecan_state.PluseRxEn == 1))	///ʶ�������������
                {
                   if((Ecan_state.Ecan_CommSave.Byte.Mask != Ecan_state.Ecan_RxD0.Byte.By4_Mask) ||///.byte.mask
                      (Ecan_state.Ecan_CommSave.Byte.Data != Ecan_state.Ecan_RxD1.Byte.By1_Data) )///.byte.data
                   {
                          Ecan_state.PluseErrCounti++;
                          if(Ecan_state.PluseErrCounti >= 2)  
                          {
                              Ecan_state.PluseErrCounti = 2;

                              //ÿһ������д����ջ   
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low; 
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xfa;
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x07; 
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 4;         //֡��
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                              ECANSendStack.BufHead ++;
                              if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                              {
                                  ECANSendStack.BufHead =0;
                              }   else{}      
                              
                              if(OutDelay.DoorLocation == 0)//ǰ��
                              {
                                  if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//����
                                  if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//����
                                  if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //����
                              }
                              else
                              {
                                  if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.RearOD;//����
                                  if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//����
                                  if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.RearZTZD; //����							 
                              }

                                 K400EcanCommAnaly();//ָ�����
                                 Ecan_state.Ecan_CommSave.Byte.Mask = Ecan_state.Ecan_RxD0.Byte.By4_Mask;///maskָ���
                                 Ecan_state.Ecan_CommSave.Byte.Data = Ecan_state.Ecan_RxD1.Byte.By1_Data;///dataָ���
                          }
                   }
                   else
                   {
                          Ecan_state.PluseErrCounti = 0;
                   }
                   
                   Ecan_state.Ecan_RxD0.Byte.By3_Indix = 0x0;		  
              }
         }
    }*/
}

/*******************************************************************************
* ��������  : K400EcanCommStateCkeck
* ����      : CANBUS״̬�仯���ſ���������
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void K400EcanCommStateCkeck(void)
{
///״̬�仯����
  /*if(((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum) || 
     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) && (Ecan_state.InitOver != 0))//�ж��ݺż�ǰ����
     {
      if(((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
         ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))
              ///�ж�ǰ����һ����
        {
          if((Ecan_state.Ecan_Tx_Rear.Byte.By1_Data) != Ecan_state.stateLast)///״̬�����仯 
          {	
              if(OutDelay.DoorLocation == 0)//ǰ��
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x53;  
              }
              else
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0xAC;  
              } 
            ///  Ecan_state.Ecan_Tx.Byte.By4_Mask = Ecan_state.stateLast^Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
            /// Ecan_state.PluseSaveMask = Ecan_state.Ecan_Tx.Byte.By4_Mask; 
              
              
              //ÿһ������д����ջ   
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0x10;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.Ecan_Tx.Byte.By4_Mask; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 5;         //֡��
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
              ECANSendStack.BufHead ++;
              if(ECANSendStack.BufHead >= ECANBUFLENMAX)
              {
                  ECANSendStack.BufHead =0;
              }   else{} 
              
              Ecan_state.PluseTxEn = 1;//ʹ������	
              Ecan_state.PluseErrTxcount = 5000;
          }
          else if((Ecan_state.PluseErrTxcount == 0) && (Ecan_state.PluseTxEn == 1))///5s
          {
             if(OutDelay.DoorLocation == 0)//ǰ��
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x53;  
              }
              else
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0xAC;  
              } 
            ///  Ecan_state.Ecan_Tx.Byte.By4_Mask = Ecan_state.stateLast^Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
              Ecan_state.PluseSaveMask = Ecan_state.Ecan_Tx.Byte.By4_Mask;   
           
            //ÿһ������д����ջ   
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0x50;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.PluseSaveMask; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 5;         //֡��
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
              ECANSendStack.BufHead ++;
              if(ECANSendStack.BufHead >= ECANBUFLENMAX)
              {
                  ECANSendStack.BufHead =0;
              }   else{} 

              Ecan_state.PluseErrTxcount = 5000;
          }
          
          
          
          Ecan_state.stateLast = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
       }
      else
      {
      
      }
   }*/
}

/*******************************************************************************
* ��������  : K400EcanRradeId
* ����      : K400������汾�����ӱ�ǩ
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void K400EcanRradeId(void)//ASCII���ʾ
{
  /*  int16 temp1,txi,txj;

     if(((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum) || 
     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)))//�ж��ݺż�ǰ����
     {
      if(((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
         ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))
      {
          if( Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xe4 )//������汾�����ӱ�ǩ
          {
          //��ȡCPUΨһID
          CpuID[0]=*(vu32*)(0x1ffff7e8);
          CpuID[1]=*(vu32*)(0x1ffff7ec);
          CpuID[2]=*(vu32*)(0x1ffff7f0);

          for(txi=0;txi<3;txi++)
          {
              for(txj=0;txj<8;txj++)
              {
                  temp1 = (CpuID[txi] >> (txj*4)) & 0x000f;
                  if((temp1 >= 0) && (temp1 <= 9))
                  {
                      temp1 = temp1+0x30;
                  }
                  else if((temp1 >= 0x0a) && (temp1 <= 0x0f))
                  {
                      temp1 = temp1+0x37;
                  }
                  ARM_ID[ARM_IDflag] = temp1;
                  ARM_IDflag++;
                  if(ARM_IDflag > 23)     ARM_IDflag = 0;
              }
          }
          
          if(OutDelay.DoorLocation == 0)//ǰ��
          {
              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
          }
          else
          {
              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
          }

          switch(Ecan_state.IDStep)///�����ӱ�ǩ����
          {
            case 0://K400Ӳ���汾��  C005  0x43, 0x30, 0x30, 0x35
                   
                  //ÿһ������д����ջ   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x0; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x43;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x30;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = 0x30;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = 0x35; 
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //֡��
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                      ECANSendStack.BufHead ++;
                      if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                      {
                          ECANSendStack.BufHead =0;
                      }   else{} 
                     
                     Ecan_state.Txtimecountj = 100;
                     Ecan_state.IDStep = 1;
             break;
             case 1://K400����汾��  ��A01.0�� 0x41, 0x30, 0x31, 0x30
                 if(Ecan_state.Txtimecountj == 0)
                 {
                       //ÿһ������д����ջ   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x08; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x31;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x30;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = 0x31;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = 0x30; 
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //֡��
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                      ECANSendStack.BufHead ++;
                      if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                      {
                          ECANSendStack.BufHead =0;
                      }   else{} 
                     
                     Ecan_state.Txtimecountj = 100;
                     Ecan_state.IDStep = 2;
                 }
                 else
                 {}
               
             break;
             
             case 2://K400���ذ�IDֵ---E_ID���ݳ���  
                 if(Ecan_state.Txtimecountj == 0)
                 {
                      //ÿһ������д����ջ   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x10; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x18;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x00;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = 0x00;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = 0x00; 
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //֡��
                      ECANSendStack.BufHead ++;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                      if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                      {
                          ECANSendStack.BufHead =0;
                      }   else{} 
                     
                     Ecan_state.Txtimecountj = 100;
                     Ecan_state.IDStep = 3;
                 }
                 else
                 {}
               
             break;
             case 3://K400���ذ�IDֵ---E_ID����ֵ 

               if(Ecan_state.Txtimecountj == 0)
                 {  
                        //ÿһ������д����ջ   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x11 + Ecan_state.IDsendcounti; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = ARM_ID[Ecan_state.IDsendcountj];
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = ARM_ID[Ecan_state.IDsendcountj + 1];
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = ARM_ID[Ecan_state.IDsendcountj + 2];
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = ARM_ID[Ecan_state.IDsendcountj + 3];
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //֡��
                      ECANSendStack.BufHead ++;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                      if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                      {
                          ECANSendStack.BufHead =0;
                      }   else{} 
                       
                       Ecan_state.Txtimecountj = 100;
                       Ecan_state.IDStep = 3;  
                       
                       Ecan_state.IDsendcounti++;
                       Ecan_state.IDsendcountj += 4;
                       if(Ecan_state.IDsendcounti > 5)
                       {
                         Ecan_state.IDsendcounti = 0;
                         Ecan_state.IDsendcountj = 0;
                         Ecan_state.Ecan_RxD0.Byte.By3_Indix = 0x00;
                         Ecan_state.IDStep = 4;  
                       }
                 }
                 else
                 {}
               
             break;
             
             case 4://����
                 
                Ecan_state.IDStep = 0;  
                Ecan_state.IDsendcounti = 0;
               
             break;
             default:break; 
          }
        }
      }
    }*/
}


/*******************************************************************************
* ��������  : K400EcanAdvanInit
* ����      : K400�߼���ʼ��
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/

void K400EcanAdvanInit(void)
{
  //if(Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x04)//�߼���ʼ����ʶ��
  {
    //if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1)//���ѱ�ʶλ
    {
       ;/* Ecan_state.Ecan_InitSave.Byte.Low = Ecan_state.Ecan_RxD0.Byte.By1_LSP;//�����ݺš�ǰ������Ϣ
        Ecan_state.InitOver = 1; ///��ʼ�����������Խ�����������

        Ecan_state.PluseRxEn = 0;
        Ecan_state.PluseTxEn = 0;

        Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x0;//maskλ����
        Ecan_state.Ecan_Tx_Rear.Byte.By1_Data = 0x0;//dataλ����
        Ecan_state.stateLast = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;//���浱ǰ״̬

        Ecan_state.Ecan_RxD0.Byte.By3_Indix = 0x0;*/
    }
  }
}


/*******************************************************************************
* ��������  : K400EcanInit
* ����      : K400��ʼ���������ź�
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/

void K400EcanInit(void)
{
   /*if(((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
      ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))///�ж�ǰ����һ����
      { 
        if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xf8)) 
        {
            //��λ����
            PosStateVar.State.all = 0;
            
            Ecan_state.Ecan_RxD0.all = 0;
            Ecan_state.Ecan_RxD1.all = 0;
            CTLcmdVar.FinalCmd.all = 0;
            MTCVar.StateCmd.RunEn = 0;//�������ʹ��
            PowerFstVar.ProInit = 1;
            PowerFstVar.PowerFst = 1;
            Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x0;
            Ecan_state.Ecan_Tx_Rear.all = 0x0;
            Ecan_state.Ecan_InitSave.bit.OD = 0;
            Ecan_state.Ecan_InitSave.bit.CD = 0;
            Ecan_state.Ecan_InitSave.bit.LCD = 0;
            CTLcmdVar.CanCmd.bit.CD = 0;
            CTLcmdVar.CanCmd.bit.OD = 0;
            CTLcmdVar.CanCmd.bit.LCD = 0;
            Ecan_state.PluseRxEn = 0;
            Ecan_state.PluseTxEn = 0;
          ///  K400EcanCommAnaly();//ָ�����
            
            if(OutDelay.DoorLocation == 0)//ǰ��
            {
                Ecan_state.Ecan_Tx.bit.LifeNum = 0x0;
                Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
            }
            else
            {
                Ecan_state.Ecan_Tx.bit.LifeNum = 0x0;
                Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
            }

        //ÿһ������д����ջ
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xf9;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x25; 
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x00;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x0;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 6;         //֡��
            ECANSendStack.BufHead ++;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
            if(ECANSendStack.BufHead >= ECANBUFLENMAX)
            {
                ECANSendStack.BufHead =0;
            }   else{} 

            Ecan_state.Ecan_RxD0.all = 0x0;
            Ecan_state.InitOver = 0;//��ʼ״̬λ
            Ecan_state.Txtimecounti = 2;
            Ecan_state.Step = 1;
        }
        else
        {
                
        }

        switch(Ecan_state.Step)///��ʼ������
        {
           case 0: ///��λ
          
          break;
          case 1:	  ///����

              if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xe8) )//&&(Ecan_state.sendSuccess==1)&& (Ecan_state.Txtimecounti == 0)) 
              {      
                    if(OutDelay.DoorLocation == 0)//ǰ��
                    {
                        Ecan_state.Ecan_Tx.bit.LifeNum = 0x0;
                        Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                        Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                    }
                    else
                    {
                        Ecan_state.Ecan_Tx.bit.LifeNum = 0x0;
                        Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                        Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                    }
                    Ecan_state.Ecan_InitSave.Byte.Low = Ecan_state.Ecan_RxD0.Byte.By1_LSP;//�����ݺš�ǰ������Ϣ

                //ÿһ������д����ջ
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe9;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 3;         //֡��
                    ECANSendStack.BufHead ++;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                    if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                    {
                        ECANSendStack.BufHead =0;
                    }   else{} 
                          
                    Ecan_state.Txtimecounti = 100;
                    Ecan_state.Step = 3;
                    Ecan_state.sendSuccess = 0;
                  
                }
                else
                {
                
                }
              
               if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))// && (Ecan_state.Txtimecounti == 0)) 
               {
                  if(OutDelay.DoorLocation == 0)//ǰ��
                   {
                      if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//����
                      if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//����
                      if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //����
                    }
                   else
                   {
                      if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//����
                      if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//����
                      if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //����
                   }
                  Ecan_state.Txtimecounti = 100;
               }  

            break;
            case 2:///�Ϲ��������԰� ������������ 20160229
                  if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //�ж��ݺŻ�㲥
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///��ʼ������1
                    {
                       if(OutDelay.DoorLocation == 0)//ǰ��
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //����
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //����
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
                if(Ecan_state.sendSuccess==1)
                {
                    Ecan_state.Txtimecounti = 100;
                    Ecan_state.Step = 3;
                }
                else
                {
                }
          break;
          case 3: ///��ʼ������1
                  if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //�ж��ݺŻ�㲥
                  {
                        if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xea)&&(Ecan_state.sendSuccess==1) )//&& (Ecan_state.Txtimecounti == 0)) ///��ʼ������2
                        {
                          //ÿһ������д����ջ
                           if(OutDelay.DoorLocation == 0)//ǰ��
                          {
                              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
                              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                              Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x53;  
                          }
                          else
                          {
                              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//�ݺ�
                              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                              Ecan_state.Ecan_Tx.Byte.By4_Mask = 0xAC;  
                          } 

                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0x10;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.Ecan_Tx.Byte.By4_Mask;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 5;         //֡��
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                          ECANSendStack.BufHead ++;
                          if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                          {
                              ECANSendStack.BufHead =0;
                          }   else{} 
                      
                          Ecan_state.Txtimecounti = 100;
                          Ecan_state.Step = 4;
                          Ecan_state.sendSuccess = 0;
                      }
                      else
                      {
                      
                      }
                  }
                  
                   if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //�ж��ݺŻ�㲥
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///��ʼ������1
                    {
                       if(OutDelay.DoorLocation == 0)//ǰ��
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //����
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //����
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
                  
          break;
          case 4:///��ʼ������2
                  //if(Ecan_state.Txtimecounti == 0) ///�ʼ�����?
                  if(Ecan_state.sendSuccess==1)
                  {
                   //ÿһ������д����ջ
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xeb;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//������Ч
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 3;         //֡��
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                      ECANSendStack.BufHead ++;
                      if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                      {
                          ECANSendStack.BufHead =0;
                      }   else{} 

                      Ecan_state.Txtimecounti = 100;
                      Ecan_state.Step = 5;
                      Ecan_state.sendSuccess = 0;
                  }
                 // else
                  {
                  
                  }
                  
                   if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //�ж��ݺŻ�㲥
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///��ʼ������1
                    {
                       if(OutDelay.DoorLocation == 0)//ǰ��
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //����
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //����
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
          break;
          case 5:
                  if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //�ж��ݺŻ�㲥
                  {
                      if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xec)&&(Ecan_state.sendSuccess==1))// && (Ecan_state.Txtimecounti == 0)) ///��ʼ������2
                      {
                          Ecan_state.Txtimecounti = 100;
                          Ecan_state.Step = 6;
                          Ecan_state.sendSuccess = 0;
                      }
                      else
                      {
                      
                      }
                     
                  }
                  
                   if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //�ж��ݺŻ�㲥
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///��ʼ������1
                    {
                       if(OutDelay.DoorLocation == 0)//ǰ��
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //����
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//����
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//����
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //����
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
          break;
          case 6:
                //  if(Ecan_state.Txtimecounti == 0) ///��ʼ���������
                  //if(Ecan_state.sendSuccess==1)
                  {
                      Ecan_state.Txtimecounti = 1;
                      Ecan_state.InitOver = 1; ///��ʼ�����������Խ�����������

                      
                      K400EcanCommAnaly();//ָ�����
                      Ecan_state.Ecan_CommSave.Byte.Mask = Ecan_state.Ecan_RxD0.Byte.By4_Mask;///maskָ���
                      Ecan_state.Ecan_CommSave.Byte.Data = Ecan_state.Ecan_RxD1.Byte.By1_Data;///dataָ���
                     // Ecan_state.PluseRxEn = 1;
                      Ecan_state.PluseTxEn = 1;

                      Ecan_state.Step = 0;
                  }
                 // else
                  {
                  
                  }
          break;
          
          default:break;
        }
        //Ecan_state.sendSuccess = 0;
    }*/
}
/*******************************************************************************
* ��������  : CommandAnaly
* ����      : ��ɶԸ�����Ԫ����Ľ��������õ����յĲ�������
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void CommandAnaly(void)
{
        Uint16 tempcnt,temp = 0;
	static Uint16 switchfalg=0;//buzzerSw = 0;
        
        CanCommAdd = 0;//����ź��ж�

	CTLcmdVar.p = &CTLcmdVar.TerminalCmd;
	switch(CTLcmdVar.CMDSel)
	{
		case 0://�����������
                        //TerminalVar.ValueFinal.all = TerminalVar.Value.all;
                        //���ܲ����ظ�ѡ��
                        if(TerminalVar.InSel[0]==0)
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[0]-1) ) = 0;
                        }
                        else
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[0]-1) ) = TerminalVar.Value.bit.bit0 ;
                        }
                        if(TerminalVar.InSel[1]==0)
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[1]-1) ) = 0 ;
                        }
                        else
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[1]-1) ) = TerminalVar.Value.bit.bit1 ;
                        }
                        if(TerminalVar.InSel[2]==0)
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[2]-1) ) = 0;
                        }
                        else
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[2]-1) ) = TerminalVar.Value.bit.bit2 ;
                        }
                        if(TerminalVar.InSel[3]==0)
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[3]-1) ) = 0 ;
                        }
                        else
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[3]-1) ) = TerminalVar.Value.bit.bit3 ;
                        }
                        /*if(TerminalVar.InSel[4]==0)
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[4]-1) ) = 0;
                        }
                        else
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[4]-1) ) = TerminalVar.Value.bit.bit4 ;
                        }
                        if(TerminalVar.InSel[5]==0)
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[5]-1) ) = 0;
                        }
                        else
                        {
                            * (volatile Uint32 *)(0x22000000  + (Uint32)(&TerminalVar.ValueFinal-0x20000000)*32 +4*(TerminalVar.InSel[5]-1) ) = TerminalVar.Value.bit.bit5 ;
                        }*/
                        
                        if( TerminalVar.Value.bit.bit3==1 )///���ŵ�λ�����ź�����
			{
				CTLcmdVar.TerminalCmd.bit.TerminalCD = 1;
			}
			else
			{
				CTLcmdVar.TerminalCmd.bit.TerminalCD = 0;
			}
                        
                        
                        //�����ж�
			/*if( TerminalVar.ValueFinal.bit.bit3==1 )
			{
				if(g_lOH_Mode==1)//���ӱ���ԭ��״̬
				{
					CTLcmdVar.TerminalCmd.bit.OH = 1;
					
				}
				else
				{
					if( (TerminalVar.ValueFinal.bit.bit0)||(TerminalVar.ValueFinal.bit.bit1)||(TerminalVar.ValueFinal.bit.bit2) )
					{
						CTLcmdVar.TerminalCmd.bit.OH = 0; 
						switchfalg = 0;
						
						CTLcmdVar.TerminalCmd.bit.OD = TerminalVar.ValueFinal.bit.bit0;
						CTLcmdVar.TerminalCmd.bit.CD = TerminalVar.ValueFinal.bit.bit1;
						CTLcmdVar.TerminalCmd.bit.LCD = TerminalVar.ValueFinal.bit.bit2;
						CTLcmdVar.PanelCmd.all = 0;
					}
					else
					{
						if(switchfalg==0)
						{
							switchfalg = 1;
							if( PowerFstVar.PowerFst==0 )   //�����ϵ�����ų���(����������)�󿪵�λ
							{

								CTLcmdVar.TerminalCmd.bit.OD = 0;
							}
							else
							{
								if(g_uPowerFstPulseEn==0)
								{
									CTLcmdVar.TerminalCmd.bit.OD = 0;
								}
								else
								{
									
								}
							}
							
							CTLcmdVar.TerminalCmd.bit.CD = 0;
							CTLcmdVar.TerminalCmd.bit.LCD = 0;
						}
						CTLcmdVar.TerminalCmd.bit.OH = 1;
					}
				}
			}
			else*/
			{
				CTLcmdVar.TerminalCmd.bit.OH = 0;
		

				if( (TerminalVar.ValueFinal.bit.bit0)||(TerminalVar.ValueFinal.bit.bit1)||(TerminalVar.ValueFinal.bit.bit2) )
				{
					CTLcmdVar.TerminalCmd.bit.OD = TerminalVar.ValueFinal.bit.bit0;
					CTLcmdVar.TerminalCmd.bit.CD = TerminalVar.ValueFinal.bit.bit1;
					CTLcmdVar.TerminalCmd.bit.LCD = TerminalVar.ValueFinal.bit.bit2;
				}
				else
				{
						if( PowerFstVar.PowerFst==0 )   //�����ϵ�����ų���(����������)�󿪵�λ
						{
							CTLcmdVar.TerminalCmd.bit.OD = TerminalVar.ValueFinal.bit.bit0;
						}	
						else
						{
							if(g_uPowerFstPulseEn==0)
							{
								CTLcmdVar.TerminalCmd.bit.OD = TerminalVar.ValueFinal.bit.bit0;
							}
							else
							{
								
							}
						}
						CTLcmdVar.TerminalCmd.bit.CD = TerminalVar.ValueFinal.bit.bit1;
						CTLcmdVar.TerminalCmd.bit.LCD = TerminalVar.ValueFinal.bit.bit2;
				}
			}

			if(TerminalVar.ValueFinal.bit.bit0)
			{
				temp++;
			}
			else
			{
			}
			if(TerminalVar.ValueFinal.bit.bit1)
			{
				temp++;
			}
			else
			{
			}
			if(TerminalVar.ValueFinal.bit.bit2)
			{
				temp++;
			}
			else
			{
			}
			if( (temp>=2)&&(g_uODCDSignalEn==0) )
			{
                            CTLcmdVar.TerminalCmd.bit.CD = 0;
                            CTLcmdVar.TerminalCmd.bit.LCD = 0;
                            CTLcmdVar.TerminalCmd.bit.LOD = 0;
                            CTLcmdVar.TerminalCmd.bit.OD = 0;
			}
			else
			{
			}
		break;
		case 1:
		break;
		case 2:
		break;
                case 3:
                  ///Step canbusЭ��
			if((Ecan_state.Ecan_RxD0.Byte.byte1.CMDID == 160) && (OutDelay.DoorLocation == 0)) //��ʶ����ǰ�����ж�
                        {
                            CanCommAdd = Ecan_state.Ecan_RxD0.Byte.byte2.bit.CD + Ecan_state.Ecan_RxD0.Byte.byte2.bit.LCD +
                                         Ecan_state.Ecan_RxD0.Byte.byte2.bit.OD;

                                if(CanCommAdd <= 1)
                                {
                                        CTLcmdVar.CanCmd.bit.OD = Ecan_state.Ecan_RxD0.Byte.byte2.bit.OD;  ///��������
                                        CTLcmdVar.CanCmd.bit.CD = Ecan_state.Ecan_RxD0.Byte.byte2.bit.CD;  ///��������
                                        CTLcmdVar.CanCmd.bit.LCD = Ecan_state.Ecan_RxD0.Byte.byte2.bit.LCD;///����������
                                }
                                else
                                {
                                        Ecan_state.Ecan_RxD0.Byte.byte2.bit.OD = 0;
                                        Ecan_state.Ecan_RxD0.Byte.byte2.bit.CD = 0;
                                        Ecan_state.Ecan_RxD0.Byte.byte2.bit.LCD = 0;
                                        CTLcmdVar.CanCmd.bit.OD = 0;
                                        CTLcmdVar.CanCmd.bit.CD = 0;
                                        CTLcmdVar.CanCmd.bit.LCD = 0;
                                }
                        }

                break;
 
		case 4:
			if(CTLcmdVar.DemoCmd.all==0) CTLcmdVar.DemoCmd.bit.CD = 1;
		break;
		case 5:
		break;
		case 6://��׼����
			OutDelay.StateCmd.bit.CDEndOutEn = 1;
			if( SequenCheckVar.StateCmd.ShortError )
			{
				CTLcmdVar.StandThreeProtocolCmd.all = 0;
				break;
			}
			if( SequenCheckVar.StateCmd.CheckEn )
			{
				break;
			}
			switch( ThreeWireStru.AvaileCMD)
			{
				case 0://���޽�ֹ
				//	CTLcmdVar.StandThreeProtocolCmd.bit.OH = 1;
					CTLcmdVar.FinalCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.OH = 0;
				break;
				case 1://������������ԭ״̬�������λ����������Ҫ������λ�����ж���
					//CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.OH = 1;
			
				break;
				case 2://���ţ�����Ӧ��������
				case 3:
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.CD = 1;
					//CTLcmdVar.StandThreeProtocolCmd.bit.ReOD = 0;//������ֹ
					CTLcmdVar.FinalCmd.bit.ReOD = 0;//������ֹ
				break;
				case 4://ǿ�ȹ���(������)����Ӧ��������
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.LCD = 1;
					//CTLcmdVar.StandThreeProtocolCmd.bit.ReOD = 0;//������ֹ
					CTLcmdVar.FinalCmd.bit.ReOD = 0;//������ֹ
				break;
				case 5://���ţ�������������
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					if( CTLcmdVar.FinalCmd.bit.ReOD==1 )
					{
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
					}
					CTLcmdVar.StandThreeProtocolCmd.bit.CD = 1;
					if(TerminalVar.Value.bit.bit3==1)
					{
						CTLcmdVar.StandThreeProtocolCmd.bit.ReOD = 1;
						CTLcmdVar.FinalCmd.bit.ReOD = 1;
					}
				break;
				case 6://���Ŷ���
				case 7:
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.OD = 1;
				break;
				default:break;
			}
		break;
		case 7://��չ��
			OutDelay.StateCmd.bit.CDEndOutEn = 1;
			if( (SequenCheckVar.StateCmd.ShortError) )//||(SequenCheckVar.StateCmd.CheckEn)
			{
				CTLcmdVar.ExThreeProtocolCmd.all = 0;
				break;
			}
			if( SequenCheckVar.StateCmd.CheckEn )
			{
				break;
			}
			switch( ThreeWireStru.AvaileCMD)
			{
				case 0://���޽�ֹ
					CTLcmdVar.FinalCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					//CTLcmdVar.ExThreeProtocolCmd.bit.OH = 1;
					CTLcmdVar.ExThreeProtocolCmd.bit.OH = 0;
				break;
				case 1://��������
					//CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.OH = 1;
				break;
				case 2://���Ź�ִ���У�ֻ��ӦDOB
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					if( CTLcmdVar.ExThreeProtocolCmd.bit.ReOD==1 )
					{
						CTLcmdVar.ExThreeProtocolCmd.all = 0;
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
					}
					else
					{
						CTLcmdVar.ExThreeProtocolCmd.all = 0;
						CTLcmdVar.FinalCmd.bit.ReOD = 0;
					}
					CTLcmdVar.ExThreeProtocolCmd.bit.CD = 1;
					if(TerminalVar.Value.bit.bit5==1)
					{
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
						CTLcmdVar.FinalCmd.bit.ReOD = 1;
					}
					else
					{
						//CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 0;//������ֹ
						
					}
				break;
				case 3://��������
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.CD = 1;
					CTLcmdVar.FinalCmd.bit.ReOD = 0;
				//	CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 0;//������ֹ
				break;
				case 4://ǿ�ȹ���
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.LCD = 1;
					CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 0;//������ֹ
					CTLcmdVar.FinalCmd.bit.ReOD = 0;//������ֹ
				break;
				case 5://���ţ�����з�������������
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					if( CTLcmdVar.FinalCmd.bit.ReOD==1 )
					{
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
					}
					CTLcmdVar.ExThreeProtocolCmd.bit.CD = 1;
					if( (TerminalVar.Value.bit.bit3==1)||(TerminalVar.Value.bit.bit5==1) )
					{
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
						CTLcmdVar.FinalCmd.bit.ReOD = 1;
					}
				break;
				case 6://���ţ�DOB��Ч
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					if( CTLcmdVar.FinalCmd.bit.ReOD==1 )
					{
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
					}
					CTLcmdVar.ExThreeProtocolCmd.bit.CD = 1;
					if( TerminalVar.Value.bit.bit3==1 )
					{
						CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 1;
						CTLcmdVar.FinalCmd.bit.ReOD = 1;
					}
				break;
				case 7://���Ų���
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.OD = 1;
				break;
				default:break;
			}
		break;
		default:break;
	}
	//������������Ľ�������0
	for(tempcnt=0;tempcnt<CTLcmdVar.Pmax;tempcnt++)
	{	
		//�������������
		if( OCDOverVar.CDSuffType==1 )
		{
			(*(CTLcmdVar.p)).bit.ReOD = CTLcmdVar.FinalCmd.bit.ReOD;
		}
		else
		{
			if( CTLcmdVar.CMDSel==6 )//����ģʽ�Զ�
			{
				(*(CTLcmdVar.p)).bit.ReOD = CTLcmdVar.FinalCmd.bit.ReOD;
			}
			else
			{
				(*(CTLcmdVar.p)).bit.ReOD = 0;
			}
		}
                
                (*(CTLcmdVar.p)).bit.ReCD = CTLcmdVar.FinalCmd.bit.ReCD;
                
		//ͨ��ʵ��
		if(CTLcmdVar.CMDSel!=tempcnt) 
		{
			(*(CTLcmdVar.p)).all = 0;	
		}
		else
		{
			//if( (*(CTLcmdVar.p)).all==0 )
			CTLcmdVar.FinalCmd.bit.OH = (*(CTLcmdVar.p)).bit.OH;
			if( ((*(CTLcmdVar.p)).bit.OD==0)&&((*(CTLcmdVar.p)).bit.LOD==0)&&((*(CTLcmdVar.p)).bit.CD==0)&&((*(CTLcmdVar.p)).bit.LCD==0) )		
			{
				if( PosStateVar.State.bit.ODEnd==1 ) 
				{
					if( CTLcmdVar.HoldTimeCnt>=CTLcmdVar.ODHoldTime )
					{
						if( CTLcmdVar.state.ODHoldFlag==0 )//������Ǳ���״̬
						{
							CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;
						}
					}
					else
					{
						CTLcmdVar.FinalCmd.bit.OD=1;
					}
				}
				else if( PosStateVar.State.bit.CDEnd==1 )
				{
					if( CTLcmdVar.HoldTimeCnt>=CTLcmdVar.CDHoldTime )
					{
						if( CTLcmdVar.state.CDHoldFlag==0 )//������Ǳ���״̬
						{
							CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;
						}
					}
					else
					{
						CTLcmdVar.FinalCmd.bit.CD = 1;
					}
				}
				else
				{
					if( CTLcmdVar.CDHoldMod==0 )
					{
						PosStateVar.State.bit.CmdHoldActive = 0;
						CTLcmdVar.HoldTimeCnt = 0;
						CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;
					}
					else
					{
						if( PosStateVar.State.bit.CmdHoldActive )
						{
							if(CTLcmdVar.FinalCmd.bit.OD)
							{
								if( CTLcmdVar.HoldTimeCnt>=CTLcmdVar.ODHoldTime )
								{
									if( CTLcmdVar.state.ODHoldFlag==0 )
									{
										CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;
										PosStateVar.State.bit.CmdHoldActive = 0;
									}
								}
							}
							else if(CTLcmdVar.FinalCmd.bit.CD)
							{
								if( CTLcmdVar.HoldTimeCnt>=CTLcmdVar.CDHoldTime )
								{
									if( CTLcmdVar.state.CDHoldFlag==0 )
									{
										CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;
										PosStateVar.State.bit.CmdHoldActive = 0;
									}
								}
							}
							else
							{
								CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;

							}
						
						}
						else
						{
							CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;
							PosStateVar.State.bit.CmdHoldActive = 0;
						}
					}

				}
			}
			else
			{
				CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;//����ǰ��Ч����͸������������
				if( (PosStateVar.State.bit.ODEnd==1)&&((CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.LOD==1)) )
				{
					PosStateVar.State.bit.CmdHoldActive = 1;
				}
				else if( (PosStateVar.State.bit.CDEnd==1)&&((CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.bit.LCD==1)) )
				{
					PosStateVar.State.bit.CmdHoldActive = 1;
				}
				else
				{	
					PosStateVar.State.bit.CmdHoldActive = 0;
					CTLcmdVar.HoldTimeCnt = 0;
				}
			}
			
		}
		CTLcmdVar.p++; 
	}
	if( PosStateVar.State.bit.CmdHoldActive==1 )
	{
		CTLcmdVar.FinalCmd.bit.ReOD = 0;
	}
}
/**********************************************************************************
  Function:        DeadCompensate
  Description:     compenstate the dead 
  Calls:           None
  Called By:       MainISR
  Input:           None
  Output:          None
  Return:          None
  Others:          None
*********************************************************************************/
//void DeadCompensate(void)
//{
	/*static int16 pcompenstate = _IQ15(0.005),ncompenstate = _IQ15(-0.008);//001953125
	if(ilg2_vdc1.ImeasW>pcompenstate)
	{
		pwm1.MfuncC1 = pwm1.MfuncC1 + DeadCompenVal;
	}
	else if( ilg2_vdc1.ImeasW<ncompenstate)
	{
		pwm1.MfuncC1 = pwm1.MfuncC1-DeadCompenVal;;
	}
	else
	{	deadcomp = _IQ15mpyI32int(ilg2_vdc1.ImeasW,DeadCompenVal)*deadtest;
		pwm1.MfuncC1 = pwm1.MfuncC1 + deadcomp;
	}
	if(ilg2_vdc1.ImeasV>pcompenstate)
	{

		pwm1.MfuncC2 = pwm1.MfuncC2 + DeadCompenVal;
	}
	else if( ilg2_vdc1.ImeasV<ncompenstate)
	{
		pwm1.MfuncC2 = pwm1.MfuncC2 - DeadCompenVal;
	}
	else
	{
		deadcomp = _IQ15mpyI32int(ilg2_vdc1.ImeasV,DeadCompenVal)*deadtest;
		pwm1.MfuncC2 = pwm1.MfuncC2 +  deadcomp;
	}
	if(ilg2_vdc1.ImeasU>pcompenstate)
	{
		pwm1.MfuncC3 = pwm1.MfuncC3 + DeadCompenVal;
	}
	else if( ilg2_vdc1.ImeasU<ncompenstate)
	{
		pwm1.MfuncC3 = pwm1.MfuncC3 - DeadCompenVal;
	}
	else
	{
		deadcomp = _IQ15mpyI32int(ilg2_vdc1.ImeasU,DeadCompenVal)*deadtest;
		pwm1.MfuncC3 = pwm1.MfuncC3 + deadcomp;
	}

	if( pwm1.MfuncC3>_IQ15(PWMMAXDUTY) )  
	{
		pwm1.MfuncC3=_IQ15(PWMMAXDUTY+PWMCompenstate);
	}
	else
	{
		if( pwm1.MfuncC3<_IQ15(PWMMINDUTY) )  
		{
			pwm1.MfuncC3=_IQ15(PWMMINDUTY-PWMCompenstate);
		}
	}

	if( pwm1.MfuncC2>_IQ15(PWMMAXDUTY) )  
	{
		pwm1.MfuncC2=_IQ15(PWMMAXDUTY+PWMCompenstate);
	}
	else
	{
		if( pwm1.MfuncC2<_IQ15(PWMMINDUTY) )  
		{
			pwm1.MfuncC2=_IQ15(PWMMINDUTY-PWMCompenstate);
		}
	}

	if( pwm1.MfuncC1>_IQ15(PWMMAXDUTY) )  
	{
		pwm1.MfuncC1=_IQ15(PWMMAXDUTY+PWMCompenstate);
	}
	else
	{
		if( pwm1.MfuncC1<_IQ15(PWMMINDUTY) )  
		{
			pwm1.MfuncC1=_IQ15(PWMMINDUTY-PWMCompenstate);
		}
	}*/
//}

/*******************************************************************************
* ��������  : ActionToMTCAnaly
* ����      : �����������ݴ��͸�������õ�������еĲ���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ActionToMTCAnaly(void)
{
	/*if( Door_Ation.Action<(Door_Ation.Pmax) )
	{
		Door_Ation.Point = &(Door_Ation.AciontData.OD_Start) + Door_Ation.Action;
	}
	else
	{
		return;//ָ�����ʱ������ԭ������
	}
	if( (*(Door_Ation.Point)).DataAttribute.Dir==0 )
	{
		if( (*(Door_Ation.Point)).DataAttribute.TorqueLimitEn )
		{
			MTCVar.CurrentMax = (*(Door_Ation.Point)).TorqueLimit;
		}

		if( (*(Door_Ation.Point)).DataAttribute.TargetSpeedEn )
		{
			if( qep1.QepABError==ABSIGNALNONE )
			{
				MTCVar.SpeedRef = (*(Door_Ation.Point)).TargetSpeed;
			}
			else
			{
				 MTCVar.SpeedRef = _IQdiv((DoorStudyVar.SpeedStudey>>1),MotorStru.MotorRatedFreq);
			}
		}
		
	}
	else
	{
		if( (*(Door_Ation.Point)).DataAttribute.TorqueLimitEn )
		{
			MTCVar.CurrentMin = -((*(Door_Ation.Point)).TorqueLimit);
		}

		if( (*(Door_Ation.Point)).DataAttribute.TargetSpeedEn )
		{
			if( qep1.QepABError==ABSIGNALNONE )
			{
				MTCVar.SpeedRef = -((*(Door_Ation.Point)).TargetSpeed);
			}
			else
			{
				MTCVar.SpeedRef = _IQdiv((DoorStudyVar.SpeedStudey>>1),MotorStru.MotorRatedFreq);
			}
		}
	}
	MTCVar.StateCmd.CTLType = 1;
	MTCVar.StateCmd.RunEn = (*(Door_Ation.Point)).DataAttribute.RunEn;
	if( (*(Door_Ation.Point)).DataAttribute.TargetSpeedAccEn )
	{
		MTCVar.SpeedAcc = (*(Door_Ation.Point)).TargetSpeedAcc;
	}*/
}
/*******************************************************************************
* ��������  : ParaCalcNULL
* ����      : ���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ParaCalcNULL(void)
{
}
/*******************************************************************************
* ��������  : ParaInSelCalc
* ����      : ���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ParaInSelCalc(void)
{
}
/*******************************************************************************
* ��������  : Para0100Calc
* ����      : P01.00���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0100Calc(void)
{

//ͨѶģʽ
	if( CTLcmdVar.CMDSel==2 )
	{
		Sci_Strut.RS485EN = 1;
	}
	else
	{
		Sci_Strut.RS485EN = 0;
	}
}

/*******************************************************************************
* ��������  : Para0102Calc
* ����      : P01.02���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0102Calc(void)
{
	MotorStru.MotorOutFreMax = _IQ15mpy(MotorStru.MotorOutFreMax_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//������Ƶ��
	MotorFreMaxQ = _IQdiv(MotorStru.MotorOutFreMax,MotorStru.MotorRatedFreq);
}
/*******************************************************************************
* ��������  : Para0103Calc
* ����      : P01.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0103Calc(void)
{
	g_hCurveGenerate.PolyLineVar.PowerFstLowSpd =	_IQ15mpy(g_hCurveGenerate.PolyLineVar.PowerFstLowSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	g_hCurveGenerate.PolyLineVar.PowerFstLowSpdQ24 = _IQ28div(g_hCurveGenerate.PolyLineVar.PowerFstLowSpd,MotorStru.MotorRatedFreq);
}
/*******************************************************************************
* ��������  : Para0104Calc
* ����      : P01.04���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0104Calc(void)
{
	//������Ż��߼�����
	if( MotorStru.MotorDirSet==1 )
	{
		MotorStru.DoorDirect = -1;
	}
	else
	{
		MotorStru.DoorDirect = 1;
	}

	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}

/*******************************************************************************
* ��������  : Para0203Calc
* ����      : P02.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0203Calc(void)
{
	//������������궨
	g_lRefCurrentQ = _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase);
}	


/*******************************************************************************
* ��������  : Para0204Calc
* ����      : P02.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0204Calc(void)
{
	Para0102Calc();
	Para0103Calc();
	Para0212Calc();
	Para0102Calc();
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}

/*******************************************************************************
* ��������  : Para0206Calc
* ����      : P02.06���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0206Calc(void)
{
	//����ת�������ٶȵ�ϵ����ϵ
	g_hUnitTransform.SpeedHztoLinearVelCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/(MotorStru.Motor_Poles/2)/1000);
	g_hUnitTransform.LinearVeltoSpeedHzCoeff = _IQ15div(_IQ15(1.0),g_hUnitTransform.SpeedHztoLinearVelCoeff);
	
}
/*******************************************************************************
* ��������  : Para0212Calc
* ����      : P02.12���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0212Calc(void)
{
//���ſ���	
	/*FluxWeakVar.Ls = _IQ15( (float32)MotorStru.Motor_LS/1000 );
	FluxWeakVar.Lr = _IQ15( (float32)MotorStru.Motor_LR/1000 );
	FluxWeakVar.Lm = _IQ15( (float32)MotorStru.Motor_LM/1000 );
	FluxWeakVar.BaseCurrentRef = _IQ15( (float32)MotorStru.MotoRatedCurrent/100 );
	FluxWeakVar.BaseFre = _IQ15( (float32)MotorStru.MotorRatedFreq/100 );
	FluxWeakVar.WeakCurrentRef = _IQ15( (float32)MotorStru.ExcitationCurrent/100 );//_IQ15mpy(_IQ15div(IdRef,100),_IQ15div(CtlCurrentBase,100));
//	FluxWeakVar.Vdc = _IQ15div(310,
	FluxCurrentQ24 = _IQdiv(MotorStru.ExcitationCurrent,CtlCurrentBase);
//	FluxWeakVar.PreFre = _IQ15(0.1);
//	FluxWeakVar.VdcDead = _IQtoIQ15(pid1_iq.OutMax);
	FluxWeakVar.calc_const(&FluxWeakVar);*/
	
}
/*******************************************************************************
* ��������  : Para0301Calc
* ����      : P03.01���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0301Calc(void)
{
	DoorStudyVar.SpeedStudey = _IQ15mpy(DoorStudyVar.SpeedStudeyMil,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
}

/*******************************************************************************
* ��������  : Para0303Calc
* ����      : P03.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0303Calc(void)
{
//�ſ����
	DoorStudyVar.MilToPosCoffe	=	PosStateVar.MilToPosCoeff;
	DoorStudyVar.PosToMilCoffe	=	PosStateVar.PosToMilCoeff;
	DoorStudyVar.Width	=	_IQ15mpyI32int(DoorStudyVar.DecPos.DoorMil,DoorStudyVar.MilToPosCoffe);
	
}
/*******************************************************************************
* ��������  : Para0307Calc
* ����      : P03.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0307Calc(void)
{
//����ת�������ٶȵ�ϵ����ϵ
	g_hUnitTransform.SpeedHztoLinearVelCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/(MotorStru.Motor_Poles/2)/1000);
	g_hUnitTransform.LinearVeltoSpeedHzCoeff = _IQ15div(_IQ15(1.0),g_hUnitTransform.SpeedHztoLinearVelCoeff);
	
	//�ſ�ȵ�λת��ϵ������
	//PosStateVar.PosUnitCoeff = _IQ15(WHEEL_DIA*3.1415926/qep1.LineEncoder);
	PosStateVar.PosToMilCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/qep1.LineEncoder);
	PosStateVar.MilToPosCoeff = _IQ15div(_IQ15(1.0),PosStateVar.PosToMilCoeff);
}
/*******************************************************************************
* ��������  : Para0400Calc
* ����      : P04.00���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0400Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODStartDis = (int32)(g_hCurveGenerate.PolyLineVar.ODStartDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0402Calc
* ����      : P04.02���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0402Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0403Calc
* ����      : P04.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0403Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.ODCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0404Calc
* ����      : P04.04���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0404Calc(void)
{
	g_hCurveGenerate.PolyLineVar.RefHighSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0406Calc
* ����      : P04.06���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0406Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.RefHighSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}


/*******************************************************************************
* ��������  : Para0407Calc
* ����      : P04.07���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0407Calc(void)
{
	PosStateVar.ParSet.ODFullPosEn = (int32)(PosStateVar.ParSet.ODFullPosEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* ��������  : Para0408Calc
* ����      : P04.08���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0408Calc(void)
{
	PosStateVar.ParSet.ODFullPosDisEn = (int32)(PosStateVar.ParSet.ODFullPosDisEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* ��������  : Para0409Calc
* ����      : P04.069���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0409Calc(void)
{
	g_hCurveGenerate.OD_ActCurve.HighSpd = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.ODActHighSpd_per,100),g_hCurveGenerate.OD_RefCurve.HighSpd);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0410Calc
* ����      : P04.10���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0410Calc(void)
{
	g_hCurveGenerate.PolyLineVar.FullDis = (int32)(g_hCurveGenerate.PolyLineVar.FullDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* ��������  : Para0411Calc
* ����      : P04.11���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0411Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.ODCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* ��������  : Para0412Calc
* ����      : P04.12���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0412Calc(void)
{
	PosStateVar.ParSet.ODEndPos = (int32)(PosStateVar.ParSet.ODEndPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* ��������  : Para0413Calc
* ����      : P04.13���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0413Calc(void)
{
    g_hCurveGenerate.PolyLineVar.ODRetiringCamDis = (int32)(g_hCurveGenerate.PolyLineVar.ODRetiringCamDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}

/*******************************************************************************
* ��������  : Para0414Calc
* ����      : P04.14���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0414Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
        CurveParaCalc();
}
/*******************************************************************************
* ��������  : Para0415Calc
* ����      : P04.14���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0415Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODRetiringCamOverDis = (int32)(g_hCurveGenerate.PolyLineVar.ODRetiringCamOverDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* ��������  : Para0500Calc
* ����      : P05.00���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0500Calc(void)
{
	g_hCurveGenerate.PolyLineVar.StrapElongate = (int32)(g_hCurveGenerate.PolyLineVar.StrapElongateMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* ��������  : Para0502Calc
* ����      : P05.02���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0502Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
	//λ�û������������	
	pid1_pos.OutMax = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd)*-1;	
	pid1_pos.OutMin = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd);
}
/*******************************************************************************
* ��������  : Para0503Calc
* ����      : P05.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0503Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0506Calc
* ����      : P05.06���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0506Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para05107Calc
* ����      : P05.07���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0507Calc(void)
{
	PosStateVar.ParSet.CDFullPosEn = (int32)(PosStateVar.ParSet.CDFullPosEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* ��������  : Para05108Calc
* ����      : P05.08���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0508Calc(void)
{
	PosStateVar.ParSet.CDFullPosDisEn = (int32)(PosStateVar.ParSet.CDFullPosDisEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* ��������  : Para0509Calc
* ����      : P05.09���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0509Calc(void)
{
	g_hCurveGenerate.CD_HighSpdSave = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.CDActHighSpd_per,100),g_hCurveGenerate.CD_RefCurve.HighSpd);
	g_hCurveGenerate.CD_ActCurve.HighSpd = g_hCurveGenerate.CD_HighSpdSave ;
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0510Calc
* ����      : P05.10���²���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0510Calc(void)
{
	g_hCurveGenerate.NomalrLCD_HighSpdSave = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.NomalLCDActHighSpd_per,100),g_hCurveGenerate.CD_RefCurve.HighSpd);
}

/*******************************************************************************
* ��������  : Para0512Calc
* ����      : P05.12���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0512Calc(void)
{
	PosStateVar.ParSet.CDEndPos = (int32)(PosStateVar.ParSet.CDEndPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* ��������  : Para0511Calc
* ����      : P05.11���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0511Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}

/*******************************************************************************
* ��������  : Para0513Calc
* ����      : P05.13���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0513Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDRetiringCamDis = (int32)(g_hCurveGenerate.PolyLineVar.CDRetiringCamDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}

/*******************************************************************************
* ��������  : Para0514Calc
* ����      : P05.14���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0514Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
}
/*******************************************************************************
* ��������  : Para0515Calc
* ����      : P05.15���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0515Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDis = (int32)(g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}


/*******************************************************************************
* ��������  : Para0601Calc
* ����      : P06.01���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0601Calc(void)
{
	PosStateVar.TorqueInter.OD_HlTorque		= _IQmpy( _IQdiv( PosStateVar.ParSet.OD_HlTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0602Calc
* ����      : P06.02���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0602Calc(void)
{
	PosStateVar.TorqueInter.OD_MaxTorque		=_IQmpy( _IQdiv( PosStateVar.ParSet.OD_MaxTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0603Calc
* ����      : P06.03���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0603Calc(void)
{
	PosStateVar.TorqueInter.OD_HlLastTorque	= _IQmpy( _IQdiv( PosStateVar.ParSet.OD_HlLastTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0606Calc
* ����      : P06.06���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0606Calc(void)
{
	PosStateVar.TorqueInter.CD_HlTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_HlTorquePer,1000 ),g_lRefCurrentQ);
}
/*******************************************************************************
* ��������  : Para0607Calc
* ����      : P06.07���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0607Calc(void)
{
	PosStateVar.TorqueInter.CD_MaxTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_MaxTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0608Calc
* ����      : P06.08���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0608Calc(void)
{
	PosStateVar.TorqueInter.stallCDTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.stallCDTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0609Calc
* ����      : P06.09���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0609Calc(void)
{
	PosStateVar.TorqueInter.CD_HlLastTorque 	=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_HlLastTorquePer,1000 ),g_lRefCurrentQ);
}
/*******************************************************************************
* ��������  : Para0613Calc
* ����      : P06.13���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0613Calc(void)
{
	DoorFianlVar.ODSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.ODswithPer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0614Calc
* ����      : P06.14���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0614Calc(void)
{
	DoorFianlVar.CDSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.CDswithPer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* ��������  : Para0712Calc
* ����      : P07.12���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0712Calc(void)
{
	pid1_pos.Kp = _IQdiv(PosGain,1000);
}
/*******************************************************************************
* ��������  : Para0714Calc
* ����      : P07.14���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0714Calc(void)
{
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para0806Calc
* ����      : P08.06���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0806Calc(void)
{
	PiAdjVar.ProPar.SpeedProportionGain0 = _IQdiv(PiAdjVar.ParSet.SpeedProportionGain0,1000);			
}
/*******************************************************************************
* ��������  : Para0807Calc
* ����      : P08.07���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0807Calc(void)
{
	PiAdjVar.ProPar.SpeedIntegralGain0 = _IQdiv(PiAdjVar.ParSet.SpeedIntegralGain0,1000);		
}
/*******************************************************************************
* ��������  : Para0808Calc
* ����      : P08.08���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0808Calc(void)
{
	PiAdjVar.ProPar.SpeedProportionGain1 = _IQdiv(PiAdjVar.ParSet.SpeedProportionGain1,1000);	
}
/*******************************************************************************
* ��������  : Para0809Calc
* ����      : P08.09���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0809Calc(void)
{
	PiAdjVar.ProPar.SpeedIntegralGain1 = _IQdiv(PiAdjVar.ParSet.SpeedIntegralGain1,1000);	
}
/*******************************************************************************
* ��������  : Para0811Calc
* ����      : P08.11���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0811Calc(void)
{
	PiAdjVar.ProPar.CurrentProportionGain0 = _IQdiv(PiAdjVar.ParSet.CurrentProportionGain0,1000);
	pid1_id.Kp = PiAdjVar.ProPar.CurrentProportionGain0;
	pid1_iq.Kp = PiAdjVar.ProPar.CurrentProportionGain0;
}
/*******************************************************************************
* ��������  : Para0812Calc
* ����      : P08.12���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0812Calc(void)
{
	PiAdjVar.ProPar.CurrentIntegralGain0 = _IQdiv(PiAdjVar.ParSet.CurrentIntegralGain0,1000);
	pid1_id.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);	
	pid1_iq.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);		

}
/*******************************************************************************
* ��������  : Para0814Calc
* ����      : P08.14���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0814Calc(void)
{
	pid1_iq.Kc = _IQdiv(KcTest,100);	
	pid1_id.Kc = pid1_iq.Kc;	
}
/*******************************************************************************
* ��������  : Para0815Calc
* ����      : P08.15���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0815Calc(void)
{
	//pid1_spd.Kd = _IQdiv(KdTestSpd,100);
	pid1_spd.Kc = _IQdiv(KcTestSpd,100);
}
/*******************************************************************************
* ��������  : Para0908Calc
* ����      : P09.08���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0908Calc(void)
{
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* ��������  : Para09011Calc
* ����      : P09.11���²����մ�����
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void Para0911Calc(void)
{
	//�����������ʱ��
	if( CTLcmdVar.ODHoldTime>=HOLDMAX )
	{
		CTLcmdVar.state.ODHoldFlag = 1;
	}
	else
	{
		CTLcmdVar.state.ODHoldFlag = 0;
	}
}
/*******************************************************************************
* ��������  : Para09012Calc
* ����      : P09.11���²����մ�����
* ��?     : None
* ?     : None
* ����      : None.
*******************************************************************************/
void Para0912Calc(void)
{

	if( CTLcmdVar.CDHoldTime>=HOLDMAX )
	{
		CTLcmdVar.state.CDHoldFlag = 1;
	}
	else
	{
		CTLcmdVar.state.CDHoldFlag = 0;
	}
}

/*******************************************************************************
* ��������  : ParSettoCTLAnaly
* ����      : �����������ò���ת��Ϊͬ�����Ʊ���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ParSettoCTLAnaly(void)
{
	//_iq basetemp;
	//����ת�������ٶȵ�ϵ����ϵ
	g_hUnitTransform.SpeedHztoLinearVelCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/(MotorStru.Motor_Poles/2)/1000);
	g_hUnitTransform.LinearVeltoSpeedHzCoeff = _IQ15div(_IQ15(1.0),g_hUnitTransform.SpeedHztoLinearVelCoeff);
	//Para0206Calc();
	//�ſ�ȵ�λת��ϵ������
	//PosStateVar.PosUnitCoeff = _IQ15(WHEEL_DIA*3.1415926/qep1.LineEncoder);
	PosStateVar.PosToMilCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/qep1.LineEncoder);
	PosStateVar.MilToPosCoeff = _IQ15div(_IQ15(1.0),PosStateVar.PosToMilCoeff);
	//Para0307Calc();
	//basetemp = _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase);
	g_lRefCurrentQ = _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase);
	//Para0203Calc();
	//�����л�
	//Para0613Calc();
	//Para0614Calc();
	DoorFianlVar.CDSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.CDswithPer,1000 ),g_lRefCurrentQ);
	DoorFianlVar.ODSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.ODswithPer,1000 ),g_lRefCurrentQ);
	//���ؼ���
	PosStateVar.TorqueInter.OD_MaxTorque	=_IQmpy( _IQdiv( PosStateVar.ParSet.OD_MaxTorquePer,1000 ),g_lRefCurrentQ);
	//Para0602Calc();
	PosStateVar.TorqueInter.OD_HlTorque		= _IQmpy( _IQdiv( PosStateVar.ParSet.OD_HlTorquePer,1000 ),g_lRefCurrentQ);
	Para0601Calc();
	PosStateVar.TorqueInter.OD_HlLastTorque = _IQmpy( _IQdiv( PosStateVar.ParSet.OD_HlLastTorquePer,1000 ),g_lRefCurrentQ);
	//Para0603Calc();

	PosStateVar.TorqueInter.CD_MaxTorque	=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_MaxTorquePer,1000 ),g_lRefCurrentQ);
	//Para0607Calc();
	PosStateVar.TorqueInter.CD_HlTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_HlTorquePer,1000 ),g_lRefCurrentQ);
	//Para0606Calc();
	PosStateVar.TorqueInter.CD_HlLastTorque =  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_HlLastTorquePer,1000 ),g_lRefCurrentQ);
	Para0609Calc();

        
        PosStateVar.TorqueInter.stallCDTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.stallCDTorquePer,1000 ),g_lRefCurrentQ);	
	//�ſ����
	//DoorStudyVar.Width = DoorStudyVar.DecPos.DecH*10000 + DoorStudyVar.DecPos.DecL;
	DoorStudyVar.MilToPosCoffe	=	PosStateVar.MilToPosCoeff;
	DoorStudyVar.PosToMilCoffe	=	PosStateVar.PosToMilCoeff;
	DoorStudyVar.Width	=	_IQ15mpyI32int(DoorStudyVar.DecPos.DoorMil,DoorStudyVar.MilToPosCoffe);
	//Para0303Calc();
	//��λת��

	MotorStru.MotorOutFreMax = _IQ15mpy(MotorStru.MotorOutFreMax_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0102Calc();
	g_hCurveGenerate.PolyLineVar.PowerFstLowSpd =	_IQ15mpy(g_hCurveGenerate.PolyLineVar.PowerFstLowSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0103Calc();
	DoorStudyVar.SpeedStudey = _IQ15mpy(DoorStudyVar.SpeedStudeyMil,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0301Calc();
	g_hCurveGenerate.PolyLineVar.ODCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0402Calc();
	g_hCurveGenerate.PolyLineVar.RefHighSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0404Calc();
	g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0502Calc();
	g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//Para0514Calc();
        g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);


	g_hCurveGenerate.PolyLineVar.ODStartDis = (int32)(g_hCurveGenerate.PolyLineVar.ODStartDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0400Calc();
	g_hCurveGenerate.PolyLineVar.ODCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.ODCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0403Calc();
	PosStateVar.ParSet.ODEndPos = (int32)(PosStateVar.ParSet.ODEndPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0412Calc();
	PosStateVar.ParSet.SGSPos = (int32)(PosStateVar.ParSet.SGSPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0414Calc();
	g_hCurveGenerate.PolyLineVar.StrapElongate = (int32)(g_hCurveGenerate.PolyLineVar.StrapElongateMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0500Calc();
	g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0511Calc();
	PosStateVar.ParSet.CDEndPos = (int32)(PosStateVar.ParSet.CDEndPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0512Calc();
	g_hCurveGenerate.PolyLineVar.CDRetiringCamDis = (int32)(g_hCurveGenerate.PolyLineVar.CDRetiringCamDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0513Calc();
	g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDis = (int32)(g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0515Calc();
        
        g_hCurveGenerate.PolyLineVar.ODRetiringCamDis = (int32)(g_hCurveGenerate.PolyLineVar.ODRetiringCamDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	g_hCurveGenerate.PolyLineVar.ODRetiringCamOverDis = (int32)(g_hCurveGenerate.PolyLineVar.ODRetiringCamOverDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	

	PosStateVar.ParSet.ODFullPosEn = (int32)(PosStateVar.ParSet.ODFullPosEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0407Calc();
	PosStateVar.ParSet.ODFullPosDisEn = (int32)(PosStateVar.ParSet.ODFullPosDisEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0408Calc();
	PosStateVar.ParSet.CDFullPosEn = (int32)(PosStateVar.ParSet.CDFullPosEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0507Calc();
	PosStateVar.ParSet.CDFullPosDisEn = (int32)(PosStateVar.ParSet.CDFullPosDisEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	//Para0508Calc();
	//�����������ʱ��
	if( CTLcmdVar.ODHoldTime>=HOLDMAX )
	{
		CTLcmdVar.state.ODHoldFlag = 1;
	}
	else
	{
		CTLcmdVar.state.ODHoldFlag = 0;
	}
	if( CTLcmdVar.CDHoldTime>=HOLDMAX )
	{
		CTLcmdVar.state.CDHoldFlag = 1;
	}
	else
	{
		CTLcmdVar.state.CDHoldFlag = 0;
	}
	//Para0104Calc();
	//ͨѶģʽ
	if( CTLcmdVar.CMDSel==2 )
	{
		Sci_Strut.RS485EN = 1;
	}
	else
	{
		Sci_Strut.RS485EN = 0;
	}
	//PI��������
	PiAdjVar.ProPar.SpeedIntegralGain0 = _IQdiv(PiAdjVar.ParSet.SpeedIntegralGain0,1000);
	//Para0807Calc();
	PiAdjVar.ProPar.SpeedProportionGain0 = _IQdiv(PiAdjVar.ParSet.SpeedProportionGain0,1000);
	//Para0806Calc();
	PiAdjVar.ProPar.SpeedIntegralGain1 = _IQdiv(PiAdjVar.ParSet.SpeedIntegralGain1,1000);
	//Para0809Calc();
	PiAdjVar.ProPar.SpeedProportionGain1 = _IQdiv(PiAdjVar.ParSet.SpeedProportionGain1,1000);
	//Para0808Calc();
	PiAdjVar.ProPar.CurrentIntegralGain0 = _IQdiv(PiAdjVar.ParSet.CurrentIntegralGain0,1000);
	PiAdjVar.ProPar.CurrentProportionGain0 = _IQdiv(PiAdjVar.ParSet.CurrentProportionGain0,1000);
	
	PiAdjVar.ProPar.SwitchFre = _IQdiv(PiAdjVar.ParSet.SwitchFre,MotorStru.MotorRatedFreq);
	pid1_id.Kp = PiAdjVar.ProPar.CurrentProportionGain0;
	pid1_id.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);	//0.1
// Initialize the PID_REG3 module for Iq
	pid1_iq.Kp = PiAdjVar.ProPar.CurrentProportionGain0;//_IQ(0.5);//0.5  0.1
	pid1_iq.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);//0.2	
	pid1_iq.Kc = _IQdiv(KcTest,100);
	pid1_id.Kc = pid1_iq.Kc;
	pid1_spd.Kc = _IQdiv(KcTestSpd,100);
	pid1_spd.Kd = _IQdiv(KdTestSpd,100);//test
	//Para0812Calc();
	//Para0811Calc();
	//Para0814Calc();
	//Para0815Calc();
	
	

	pid1_pos.Kp = _IQdiv(PosGain,1000);
	//Para0712Calc();

//���ſ���	
	/*FluxWeakVar.Ls = _IQ15( (float32)MotorStru.Motor_LS/1000 );
	FluxWeakVar.Lr = _IQ15( (float32)MotorStru.Motor_LR/1000 );
	FluxWeakVar.Lm = _IQ15( (float32)MotorStru.Motor_LM/1000 );
	FluxWeakVar.BaseCurrentRef = _IQ15( (float32)MotorStru.MotoRatedCurrent/100 );
	FluxWeakVar.BaseFre = _IQ15( (float32)MotorStru.MotorRatedFreq/100 );
	FluxWeakVar.WeakCurrentRef = _IQ15( (float32)MotorStru.ExcitationCurrent/100 );//_IQ15mpy(_IQ15div(IdRef,100),_IQ15div(CtlCurrentBase,100));
//	FluxWeakVar.Vdc = _IQ15div(310,
	FluxCurrentQ24 = _IQdiv(MotorStru.ExcitationCurrent,CtlCurrentBase);
//	FluxWeakVar.PreFre = _IQ15(0.1);
//	FluxWeakVar.VdcDead = _IQtoIQ15(pid1_iq.OutMax);
	FluxWeakVar.calc_const(&FluxWeakVar);*/
	//FluxWeakVar.calc(&FluxWeakVar);
	//Para0212Calc();
	//������Ƶ��
	MotorFreMaxQ = _IQdiv(MotorStru.MotorOutFreMax,MotorStru.MotorRatedFreq);
	//Para0102Calc();
	//������Ż��߼�����
	if( MotorStru.MotorDirSet==1 )
	{
		MotorStru.DoorDirect = -1;
	}
	else
	{
		MotorStru.DoorDirect = 1;
	}


//ͨѶģʽ
	if( CTLcmdVar.CMDSel==2 )
	{
		Sci_Strut.RS485EN = 1;
	}
	else
	{
		Sci_Strut.RS485EN = 0;
	}
	//Para0100Calc();
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
	//λ�û������������
	pid1_pos.OutMax = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd)*-1;
	pid1_pos.OutMin = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd);
}
/*******************************************************************************
* ��������  : ParDefault
* ����      : ��������Ĭ�����ò���ת��Ϊ�ڲ����Ʊ���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ParDefault(Uint16 numstart,Uint16 numEnd)
{
	Uint16 temp;
	
	temp = numstart;
	for( ;temp<numEnd;temp++ )
	{
		/*if(MotorStru.MotorType==0)
		{
                    *(SysParaAttrChar[temp].ParAdd) = SysParaAttrChar[temp].OrgVal0;
		}
		else if( MotorStru.MotorType==1 )
		{
                    *(SysParaAttrChar[temp].ParAdd) = SysParaAttrChar[temp].OrgVal1;
		}*/
                *(SysParaAttrChar[temp].ParAdd) = SysParaAttrChar[temp].OrgVal0;
                
	}
   
}
/*******************************************************************************
* ��������  : ProInitFun
* ����      : �����ϵ��ĳ�ʼ��������Ҫ���E2PROM��д���������ż�У������
*             ����д����Ҫ����λʱ����ʼ������
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ProInitFun(void)
{
	Uint16 temp;
	if( PowerFstVar.ProInit==1 )
	{
		for(temp=0;temp<E2PROM_MAXNUM;temp++)
		{
			if( E2promTask.E2PROM_Distribute[temp].ResponseFlag==1 )
			{
				return;
			}
		}
		VolProtectVar.state.PowerFst = 0;
		if(MotorStru.MotorType==0)//ͬ�����ʱҪ�ȴ������̵�PWM�ź�û������ʱ�ſ��Գ�ʼ��
		{
			if( (gCap.InitMechThetaFlag==2)||(gCap.NoSignalFlag==1) )
			{
				PowerFstVar.ProInit = 0;
				
			}
			else
			{
			
			}
		}
		else
		{
			PowerFstVar.ProInit = 0;
		}
		if( gE2rom.CmdState.WtireReadError==1 )
		{
			ParDefault(0,160);//����Ĭ�ϲ���
			ParSettoCTLAnaly();//��������
		}
	}
	else
	{
		if( Sci_Strut.ProRest==1 )
		{
			for(temp=0;temp<E2PROM_MAXNUM;temp++)
			{
				if( E2promTask.E2PROM_Distribute[temp].ResponseFlag==1 )
				{
					return;
				}
			}
			//if( gE2rom.ControlStatus==I2C_MSGSTAT_IDLE )
                        if( gE2rom.CmdState.OSBusy==FALSE )
			{
                            //DIS_OFF
                                //NVIC_SystemReset();
                            //ProInit();
                            //Sci_Strut.ProRest = 0;
				while(1);

			}
		}
	}
}
/*******************************************************************************
* ��������  : ReadParAll
* ����      : ��ȡ���в���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ReadParAll(void)
{
	Uint16 temp,partemp;
	for(temp=0;temp<22;temp++)
	{
		partemp = temp<<3;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[partemp].ParaPro.E2ROMAdd;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[partemp].ParaPro.E2ROMBlock;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 8;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = READCMD;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[partemp].ParAdd);
		E2PROM_TaskSet(&E2promTask);
	}
}
/*******************************************************************************
* ��������  : WriteParAll
* ����      : д�����в�����Ĭ��ֵ
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void WriteParAll(Uint16 num)
{
	Uint16 temp,partemp;

	ParDefault(0,176);//����Ĭ�ϲ���
	ParSettoCTLAnaly();//��������
	for(temp=0;temp<num;temp++)
	{
            partemp = temp<<3;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[partemp].ParaPro.E2ROMAdd;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[partemp].ParaPro.E2ROMBlock;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 8;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[partemp].ParAdd);
            E2PROM_TaskSet(&E2promTask);
	}
}
/*******************************************************************************
* ��������  : UserWriteParAll
* ����      : �û�д�������Ĭ��ֵ���������ż��Ƕ����ſ����ݡ�
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void UserWriteParAll(Uint16 num)
{
	Uint16 temp,partemp;

	ParDefault(1,4);//������P01.00 P01.04  20200317
	///ParDefault(5,32);//P03.00
        ParDefault(5,6);//������P01.06
	ParDefault(7,8);//������P01.08
	ParDefault(9,32);//P01 P02
	ParDefault(32,35);
	ParDefault(36,47);//������P03.03 P03.15
	ParDefault(48,160);//����Ĭ�ϲ���
	ParSettoCTLAnaly();//��������
	for(temp=0;temp<num;temp++)
	{
		partemp = temp<<3;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[partemp].ParaPro.E2ROMAdd;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[partemp].ParaPro.E2ROMBlock;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 8;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
		E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[partemp].ParAdd);
		E2PROM_TaskSet(&E2promTask);
	}
}
/*******************************************************************************
* ��������  : CMDToActionAnaly
* ����      : ��������Ľ���
* ��?     : None
* ���      : None
* ����      : None.
*******************************************************************************/
void CMDToActionAnaly(void)
{
    if( PowerFstVar.PowerFst==1 )
    {
        StallCheckStru.CheckTime = 1000;//��λ��ʱ����ת���Ϊ1S
    }
    else
    {
        if( (PosStateVar.State.bit.CDEnd==1)||(PosStateVar.State.bit.ODEnd==1) )
        {
            StallCheckStru.CheckTime = 250;//��λ��ʱ����ת���Ϊ250ms
        }
        else
        {
            StallCheckStru.CheckTime = 500;//�ǵ�λ��ʱ����ת���Ϊ500ms�����ٸ��ر䶯�����Ĳ��ɿ�
        }
    }
    if(ADCL_adFlag==1)
    {
        //CTLcmdVar.FinalCmd.all = 0;
        //CTLcmdVar.FinalCmd.bit.OD = 1;
    }
    else
    {
    }
    //�����ʹ��������ֻ��һ��ʹ��
    if( PowerFstVar.PowerFst==1 )
    {
        if( (CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.LOD==1)||(CTLcmdVar.FinalCmd.bit.ReOD) )
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.LOD = 1;
        }
      /// else if( (CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.bit.LCD==1) )
        else if(( (CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.bit.ReCD)||(CTLcmdVar.FinalCmd.bit.LCD==1) )&&(Camstateaction.StateCmd.CamStateActionRun == 0))	
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.LCD = 1;
        }
        else
        {
            
            
            if( CTLcmdVar.PowerFstCmdMod==1 )
            {
                CTLcmdVar.FinalCmd.all = 0;
                CTLcmdVar.FinalCmd.bit.LCD = 1;
            }
            else
            {
                CTLcmdVar.FinalCmd.all = 0;
            }
        }
        if( CTLcmdVar.PowerFstCmdMod==2 )
        {
        CTLcmdVar.FinalCmd.all = 0;
        CTLcmdVar.FinalCmd.bit.LCD = 1;
        }
    }
    else
    {
        if( CTLcmdVar.FinalCmd.bit.ReOD==1 )//����
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.ReOD = 1;
        }
        else if( CTLcmdVar.FinalCmd.bit.OD==1 )//��
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.OD = 1;
        }
       /// else if( CTLcmdVar.FinalCmd.bit.LCD==1 )//����
        else if(( CTLcmdVar.FinalCmd.bit.LCD==1 )&&(Camstateaction.StateCmd.CamStateActionRun == 0))//����	
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.LCD = 1;
        }
        else if((( CTLcmdVar.FinalCmd.bit.CD==1 )||(CTLcmdVar.FinalCmd.bit.ReCD==1))&&(Camstateaction.StateCmd.CamStateActionRun == 0))//����
        ///else if( CTLcmdVar.FinalCmd.bit.CD==1 )//����
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.CD = 1;
        }
        else//������
        {
            CTLcmdVar.FinalCmd.all = 0;
        }
    }

    //����ʱ�Ĵ����д˹���ʱ����ͣ��
    if(QeqResumeVar.RemsumCnt>=RESUMENUM)
    {
        QeqResumeVar.CmdState.ResumeFlag = 0;
        FaultVar.fault.FautFlag0.bit.QEPErrorFlag = 1;
        CTLcmdVar.FinalCmd.all = 0;
        return;
    }
    if( (gCap.ParSetErrorFlag!=0)||(gCap.NoSignalFlag!=0)||(TemperatureVar.state.OverTempSecFlag)||(FaultVar.fault.FautFlag1.bit.TemperMErorr)||
       (TemperaMtureVar.state.OverTempMSecFlag)||(FaultVar.fault.FautFlag0.bit.ADCERROR)||(FaultVar.fault.FautFlag0.bit.ADCREFERROR)||
        (FaultVar.fault.FautFlag0.bit.E2promError)||(FaultVar.fault.FautFlag0.bit.FOErorr)||(FaultVar.fault.FautFlag0.bit.PhaseMiss)||
        (FaultVar.fault.FautFlag0.bit.TemperatureOve)||(FaultVar.fault.FautFlag0.bit.VolOve)||(FaultVar.fault.FautFlag0.bit.QEPErrorFlag)||(FaultVar.fault.FautFlag1.bit.LossSpd))//(FaultVar.fault.FautFlag0.bit.VolLow) ||
    {
        CTLcmdVar.FinalCmd.all = 0;
        MTCVar.StateCmd.RunEn = 0;//�������ʹ
        return;
    }
    if(FaultVar.fault.FautFlag0.bit.VolLow)
    {
        //CTLcmdVar.FinalCmd.all = 0;
        //MTCVar.StateCmd.RunEn = 0;//�������ʹ
        //return;
      
        CTLcmdVar.FinalCmd.all = 0;
        StallCheckStru.StateCmd.ClrStall = 1;
        TorqueProtecVar.StateCmd.ClrEn = 1;
        MTCVar.StateCmd.RunEn = 0;
    }
    else
    {
    }

    if( (QeqResumeVar.CmdState.ResumeEn==0) )
    {
        if(FaultVar.fault.FautFlag0.bit.QEPErrorFlag)
        {
            CTLcmdVar.FinalCmd.all = 0;
            QeqResumeVar.RemsumCnt++;
            if(QeqResumeVar.RemsumCnt>=RESUMENUM)
            {
                QeqResumeVar.CmdState.ResumeFlag = 0;
                FaultVar.fault.FautFlag0.bit.QEPErrorFlag = 1;
                CTLcmdVar.FinalCmd.all = 0;
                return;
            }
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
            return;
        }
    }
    //�жϿ����������Ƿ��л�
    if( (CTLcmdVar.FinalCmdSave.bit.OD==1)||(CTLcmdVar.FinalCmdSave.bit.LOD==1)||(CTLcmdVar.FinalCmdSave.bit.ReOD==1) )
    {
        if( (CTLcmdVar.FinalCmd.bit.LCD==1)||(CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.all==0) )
        {
            CTLcmdVar.state.ODtoCDFlag = 1;
            g_hPosLoop.StateCmd.PosLoop = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.PosLock = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.CmdSwt = 1;
            //��Ҫ�����ת�йأ��������߼����������
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
            TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
            StallCheckStru.StateCmd.StallFlag = 0;
        }
    }
    if( (CTLcmdVar.FinalCmdSave.bit.LCD==1)||(CTLcmdVar.FinalCmdSave.bit.CD==1) )
    {
        if( (CTLcmdVar.FinalCmd.bit.LOD==1)||(CTLcmdVar.FinalCmd.bit.OD==1)||((CTLcmdVar.FinalCmd.bit.ReOD==1))||(CTLcmdVar.FinalCmd.all==0) )
        {
            CTLcmdVar.state.CDtoODFlag = 1;
            g_hCurveGenerate.RealCurveCalc.CmdState.PosLock = 0;
            g_hPosLoop.StateCmd.PosLoop = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.CmdSwt = 1;
            //��Ҫ�����ת�йأ��������߼����������
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
            TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
            StallCheckStru.StateCmd.StallFlag = 0;
        }
    }
    if(CTLcmdVar.FinalCmdSave.all==0)
    {
        if(CTLcmdVar.FinalCmd.all!=0)
        {
            CTLcmdVar.state.CDtoODFlag = 1;
            g_hPosLoop.StateCmd.PosLoop = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.PosLock = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.CmdSwt = 1;
            //��Ҫ�����ת�йأ��������߼����������
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
            TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
            StallCheckStru.StateCmd.StallFlag = 0;
        }
    }
    CTLcmdVar.FinalCmdSave.all = CTLcmdVar.FinalCmd.all;
	

    //����λ�����䣬���״̬�����γɵ������
    if( CTLcmdVar.FinalCmd.bit.CD==1 )
    {
        g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 0;
        OCDOverVar.state.CDCntEn = 1;//���ų�ʱ�������
        OCDOverVar.state.ODCntEn = 0;//���ų�ʱ��ⲻ����
        OCDOverVar.state.ODOverFlag = 0;//���ų�ʱ��־����
        //���������
        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;

        if( PosStateVar.State.bit.CDEnd==1 )
        {
            OCDOverVar.state.CDCntEn = 0;
            OCDOverVar.state.ODCntEn = 0;
            if(g_hPosLoop.FullLockEn==2)//ֱ�Ӷ�λ
            {
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<((int16)g_hCurveGenerate.PolyLineVar.StrapElongate*2)) )
                {
                    g_hPosLoop.StateCmd.PosLoop = 1;//����
                    g_hPosLoop.StateCmd.Step = 1;
                    g_hPosLoop.StateCmd.Full = 0;
                }
                g_hPosLoop.PosRef = g_hCurveGenerate.PolyLineVar.StrapElongate;
            }
            g_hCurveGenerate.RealCurveCalc.CmdState.StallRemember = 0;

            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.CDSwithTorque )
            {
                if(DoorFianlVar.StateCmd.CDHDToHFst==0)//���δ�л���ʹ�ñ�����
                {
                    //���ű�����
                    MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                }
                DoorFianlVar.StateCmd.CDHDToHFst = 1;//�л�ʱ���ʱ��־
            }
            else
            {
                if(DoorFianlVar.StateCmd.CDHDToHFst==0)
                {
                    //���������
                    MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
                }
            }
            if( StallCheckStru.StateCmd.StallFlag==1 )
            {
                //�����ſ��ж���У��
                if( labs(PosStateVar.DoorPosOriginCorrect-(qep1.OutputRawTheta>>2))<g_hCurveGenerate.PolyLineVar.StrapElongate )
                {
                     if(PosStateVar.ODDoorWidth>(DoorStudyVar.Width+g_hCurveGenerate.PolyLineVar.StrapElongate))
                     {
                        DoorStudyVar.Width += (g_hCurveGenerate.PolyLineVar.StrapElongate>>2);
                     }
                     else if(PosStateVar.ODDoorWidth<(DoorStudyVar.Width-g_hCurveGenerate.PolyLineVar.StrapElongate))
                     {
                        DoorStudyVar.Width -= (g_hCurveGenerate.PolyLineVar.StrapElongate>>2);
                     }
                     else
                     {
                        DoorStudyVar.Width = PosStateVar.ODDoorWidth;
                        
                        if((Camstateaction.StateCmd.QepcountInitSamOk == 0)&&(Camstateaction.StateCmd.QepcountSecSamClac == 0))
                        {
                                Camstateaction.QepcountInit = (PosStateVar.DoorPosOrigin << 2);
                                Camstateaction.StateCmd.QepcountInitSamOk = 1;///��������ŵ��߼�����
                        }
                     }
                      PosStateVar.ODDoorWidth = DoorStudyVar.Width;
                      DoorStudyVar.DecPos.DoorMil =	_IQ15mpyI32int(DoorStudyVar.Width,PosStateVar.PosToMilCoeff);
                }
                PosStateVar.DoorPosOriginCorrect = (qep1.OutputRawTheta>>2);


                DoorFianlVar.StateCmd.CDSwithEn = 1;
                if( DoorFianlVar.StateCmd.CDSwithFlag==1 )
                {
                    DoorFianlVar.StateCmd.CDHToHLFst = 1;
                    DoorFianlVar.StateCmd.CDHDToHFst = 1;
                    //�������ձ�����
                    MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque;
                }
                else
                {
                    if(DoorFianlVar.StateCmd.CDHToHLFst==0)
                    {
                        //���ű�����
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                    }
                    else
                    {
                            
                    }
                }
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<(int16)g_hCurveGenerate.PolyLineVar.StrapElongate) )
                {
                    if(g_hPosLoop.FullLockEn==1)
                    {
                        if( g_hPosLoop.StateCmd.PosLoop==0 ) 
                        {
                            g_hPosLoop.StateCmd.PosLoop = 1;//����
                            g_hPosLoop.StateCmd.Step = 1;
                            g_hPosLoop.StateCmd.Full = 1;
                        }
                        
                        ///g_hPosLoop.PosRef = PosStateVar.DoorPosActual0 - g_hCurveGenerate.PolyLineVar.StrapElongate;
                        g_hPosLoop.PosRef = PosStateVar.DoorPosActual0;
                    
                    }
                }
                /*
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<(int16)g_hCurveGenerate.PolyLineVar.StrapElongate) )
                {  
                        if(g_hPosLoop.FullLockEn==1)
                        {
                                if(g_hPosLoop.StateCmd.PosLoop==0)
                                {
                                        g_hPosLoop.StateCmd.PosLoop = 1;//����
                                        g_hPosLoop.StateCmd.Step = 1;
                                }
                                g_hPosLoop.PosRef = PosStateVar.DoorPosActual0;
                        }
                }*/
                
                
                
            }
            else
            {
                DoorFianlVar.StateCmd.CDSwithEn = 0;
            }
            
            
            
            /*///����������0x46�й� �����ε�
            if((OutDelay.StateCmd.bit.CDEndOut == 1) && ( PosStateVar.DoorPosActual >= g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDis))///K400�����ŵ����ŵ�λ����  >=15mm
            {
                //���ű�����
                    MTCVar.CurrentMin = 0;
                    g_hPosLoop.StateCmd.PosLoop = 0;
            }
            else
            {
            
            }*/
            
        }
        else
        {
            g_hPosLoop.StateCmd.PosLoop = 0;
            //�ǹ��ŵ�λ����������Ϊ������󣬲��������ر����л���־
            DoorFianlVar.StateCmd.CDHToHLFst = 0;
            DoorFianlVar.StateCmd.CDHDToHFst = 0;
            MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
            //��ת���书�ܼ��
            if( OCDOverVar.StallEn==1 )
            {
                if( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1 ) )
                {
                    g_hCurveGenerate.RealCurveCalc.CmdState.StallRemember = 1;
                    g_hCurveGenerate.RealCurveCalc.CalcPara.stallpos = PosStateVar.DoorPosActual;
                }
            }
            else
            {
                g_hCurveGenerate.RealCurveCalc.CmdState.StallRemember = 0;
            }
            if(( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1) ) &&(PosStateVar.State.bit.ODEnd==0) )//��ת������ش򿪷�������  20200324 ���ŵ�λ�������δ�״̬
            {
                
                if( OCDOverVar.CDSuffType==1 )//�Ƿ�������
                {
                    CTLcmdVar.FinalCmd.bit.ReOD = 1;
                }
                else
                {
                    CTLcmdVar.FinalCmd.bit.ReOD = 0;
                    //�����ס�������߽���ʱ���Զ�Ѳ����
                    if( (g_hCurveGenerate.RealCurveCalc.CmdState.CurveOver==1) )//(StallCheckStru.StateCmd.StallFlag==1)&&
                    {
                    }
                }
                if( g_hPosLoop.StallLockEn==1 )//���Ŷ�ת�Ƿ�����
                {
                    if( g_hPosLoop.StateCmd.PosLoop==0 )
                    {
                        g_hPosLoop.StateCmd.PosLoop = 1;
                        g_hPosLoop.StateCmd.Step = 1;
                        g_hPosLoop.PosRef = PosStateVar.DoorPosActual0;
                        g_hPosLoop.StateCmd.Full = 0;
                    }
                    g_hPosLoop.StateCmd.StallPosLoop = 1;
                }
                else
                {
                    g_hPosLoop.StateCmd.StallPosLoop = 0;
                }
            }
            else
            {
                g_hPosLoop.StateCmd.PosLoop = 0;//������
            }

        }

    }//end of CD
    else if( (CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.ReOD==1) )
    {
        g_hPosLoop.StateCmd.StallPosLoop = 0;
        g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 0;
        OCDOverVar.state.CDCntEn = 0;//���ų�ʱ��ⲻ����
        OCDOverVar.state.ODCntEn = 1;//���ų�ʱ�������
        OCDOverVar.state.CDOverFlag = 0;//���ų�ʱ��־��0
        //�ƶ������
        MTCVar.CurrentMin = PosStateVar.TorqueInter.OD_MaxTorque*-1;
        if( PosStateVar.State.bit.ODEnd==1 )
        {
            OCDOverVar.state.CDCntEn = 0;
            OCDOverVar.state.ODCntEn = 0;
            if(g_hPosLoop.FullLockEn==2)//ֱ�Ӷ�λ
            {
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0>( (int16)DoorStudyVar.Width-(int16)g_hCurveGenerate.PolyLineVar.StrapElongate*2)) )
                {
                    g_hPosLoop.StateCmd.PosLoop = 1;//����
                    g_hPosLoop.StateCmd.Step = 1;
                    g_hPosLoop.StateCmd.Full = 0;
                }
                g_hPosLoop.PosRef = DoorStudyVar.Width - g_hCurveGenerate.PolyLineVar.StrapElongate;
            }
            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque	)
            {
                if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                {
                    //���ű�����
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                }
                DoorFianlVar.StateCmd.ODHDToHFst = 1;
            }
            else
            {
                if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
                {
                    //���������
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
                }
            }

            if( StallCheckStru.StateCmd.StallFlag==1 )
            {

                PosStateVar.ODDoorWidth = labs((qep1.OutputRawTheta>>2)-PosStateVar.DoorPosOriginCorrect);
                DoorFianlVar.StateCmd.ODSwithEn = 1;
                CTLcmdVar.FinalCmd.bit.ReOD = 0;//���ŵ�λ�������������
                if( DoorFianlVar.StateCmd.ODSwithFlag==1 )
                {
                    DoorFianlVar.StateCmd.ODHToHLFst = 1;
                    DoorFianlVar.StateCmd.ODHDToHFst = 1;
                    //�������ձ�����
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlLastTorque;
                }
                else
                {
                    if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                    {
                        //���ű�����
                        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                    }
                        
                }
                //if(g_hPosLoop.StateCmd.PosLoop==0 )
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0>( (int16)DoorStudyVar.Width-(int16)g_hCurveGenerate.PolyLineVar.StrapElongate)) )
                {
                    //����λ��Ƥ������2/3
                    if(g_hPosLoop.FullLockEn==1)
                    {
                        if(g_hPosLoop.StateCmd.PosLoop==0)
                        {
                            g_hPosLoop.StateCmd.PosLoop = 1;//����
                            g_hPosLoop.StateCmd.Step = 1;
                            g_hPosLoop.StateCmd.Full = 1;
                        }
                      ///  g_hPosLoop.PosRef = PosStateVar.DoorPosActual0-(g_hCurveGenerate.PolyLineVar.StrapElongate*2/3) ;
                        g_hPosLoop.PosRef = PosStateVar.DoorPosActual0 - (g_hCurveGenerate.PolyLineVar.StrapElongate>>2) ;       
                    }
                }
            }
            else
            {
                DoorFianlVar.StateCmd.ODSwithEn = 0;
            }
        }
        else
        {
            //���������
            MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
            DoorFianlVar.StateCmd.ODHToHLFst = 0;
            DoorFianlVar.StateCmd.ODHDToHFst = 0;
            g_hPosLoop.StateCmd.PosLoop = 0;//������
        }
    }//end of OD
    else if( CTLcmdVar.FinalCmd.bit.LCD==1 )
    {
        g_hPosLoop.StateCmd.StallPosLoop = 0;
        //g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
        OCDOverVar.state.CDCntEn = 1;//���ų�ʱ��ⲻ����
        OCDOverVar.state.ODCntEn = 0;//���ų�ʱ�������
      ///  OCDOverVar.state.CDOverFlag = 0;//���ų�ʱ��־����   ����ų�ʱ�澯��ͻ�������ε� 20190809 gw
        ////�ƶ������
        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
        //	���������
        if( PowerFstVar.PowerFst==1 )
        {
            g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
           /// MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
            if(AutoWidthStudyRunCls_initOK == 0)
            {
                  MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
            }
            else
            {
                  MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
            }

            
            //��λ��ʱ����ת���Ϊ1S
            if( (StallCheckStru.StateCmd.StallFlag==1) )
            {
               if((DoorWidthAutoStudy_en == 1)&&(AutoWidthStudyRun_initOver == 0))///ʹ���ſ���ʶ����
              {
                  if((AutoWidthStudyRunCls_initOK == 0) && (AutoWidthStudyRunOpen_initOK == 0))///�����ϵ�����Ϊ��������
                  {
                          PosStateVar.State.bit.CDEnd = 1;
                          PosStateVar.State.bit.Sgs = 1;
                          PosStateVar.State.bit.ODEnd = 0;
                          AutoWidthStudyRunCls_initOK = 1;
                          MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                          DoorStudyVar.PosOrigin = qep1.OutputRawTheta;///��ס��ʼλ�õ�
                  }
                  else if(AutoWidthStudyRunOpen_initOK == 1)///�����ϵ�Ϊ��������ѽ��г���λ�ö�λ���ſ���ʶ�����
                  {
                      ///DoorStudyVar.PosInput = qep1.OutputRawTheta;
                      DoorStudyVar.Width = labs(qep1.OutputRawTheta-DoorStudyVar.PosOrigin);///����ſ�
                      DoorStudyVar.Width = DoorStudyVar.Width>>2;
                      DoorStudyVar.DecPos.DoorMil	=	_IQ15mpyI32int(DoorStudyVar.Width,DoorStudyVar.PosToMilCoffe);
                      if(DoorStudyVar.DecPos.DoorMil < 3000)///Ĭ��ѧϰ����600�ſ�Ϊ����
                      {
                      ///	DoorStudyVar.DecPos.DoorMil = 3000;
                      ///	DoorStudyVar.Width	=	_IQ15mpyI32int(DoorStudyVar.DecPos.DoorMil,DoorStudyVar.MilToPosCoffe);
                              FaultVar.fault.FautFlag1.bit.WidthFstErorr = 1;

                              gCap.InitTime = 100;
                              PowerFstVar.ProInit = 1;
                              PowerFstVar.PowerFst = 1;
                              gCap.InitMechThetaFlag = 0;

                              MTCVar.StateCmd.RunEn = 0;
                      }
                      else
                      {
                              DoorWidthAutoStudy_en = 0;///ѧϰ��ɺ󣬱�־����

                              DoorStudyVar.DecPos.DecH = (DoorStudyVar.Width)/10000;
                              DoorStudyVar.DecPos.DecL = (DoorStudyVar.Width)%10000;

                              PosStateVar.State.bit.CDEnd = 1;
                              PosStateVar.State.bit.Sgs = 1;
                              PosStateVar.State.bit.ODEnd = 0;

                              AutoWidthStudyRun_initOver = 1;

                              StallCheckStru.StateCmd.ClrStall = 1;
                              StallCheckStru.StateCmd.StallFlag = 0;
                              TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
                              TorqueProtecVar.StateCmd.ClrEn = 1;

                              PosStateVar.DoorPosOrigin = qep1.OutputRawTheta>>2;
                              PosStateVar.DoorPosOriginSave = PosStateVar.DoorPosOrigin;
                              PowerFstVar.PowerFst = 0;

                              //�ſ���У����ʼ��
                              PosStateVar.ODDoorWidth = DoorStudyVar.Width;
                              PosStateVar.CDDoorWidth = DoorStudyVar.Width;
                              PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;

                              //�����������ʱ��
                              CTLcmdVar.HoldTimeCnt = 0;

                              LCD_DoorMil = _IQ15mpyI32int(DoorStudyVar.Width,PosStateVar.PosToMilCoeff);

                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[8].ParaPro.E2ROMAdd;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[8].ParaPro.E2ROMBlock;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[8].ParAdd);
                              E2PROM_TaskSet(&E2promTask);

                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[35].ParaPro.E2ROMAdd;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[35].ParaPro.E2ROMBlock;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[35].ParAdd);
                              E2PROM_TaskSet(&E2promTask);
                      }
                    }
                  }
                  else if((DoorWidthAutoStudy_en == 0) || (AutoWidthStudyRun_initOver == 1))
                  {
                      StallCheckStru.StateCmd.ClrStall = 1;
                      StallCheckStru.StateCmd.StallFlag = 0;
                      TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
                      TorqueProtecVar.StateCmd.ClrEn = 1;
                      PosStateVar.DoorPosOrigin = qep1.OutputRawTheta>>2;
                      PosStateVar.DoorPosOriginSave = PosStateVar.DoorPosOrigin;
                      PowerFstVar.PowerFst = 0;
                      //�ſ���У����ʼ��
                      PosStateVar.ODDoorWidth = DoorStudyVar.Width;
                      PosStateVar.CDDoorWidth = DoorStudyVar.Width;
                      PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;
                      //�����������ʱ��
                      CTLcmdVar.HoldTimeCnt = 0;
                  }
                if((Camstateaction.StateCmd.QepcountInitSamOk == 0)&&(Camstateaction.StateCmd.QepcountSecSamClac == 0))
                {
                        ///Camstateaction.QepcountInit = ((Uint32)EQep1Regs.QPOSCNT);///��¼�ŵ����ŵ�λ�����ʼֵ��ֻ��¼һ��
                        ///Camstateaction.QepcountInitSave = Camstateaction.QepcountInit;///�����ϴ�ԭ��ֵ
                        Camstateaction.QepcountInit = qep1.OutputRawTheta;///��ס��ʼλ�õ�(PosStateVar.DoorPosOrigin << 2);
                        Camstateaction.StateCmd.QepcountInitSamOk = 1;
                }
            }
            g_hPosLoop.StateCmd.PosLoop = 0;//������
        }
        else
        { 
            g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 0;
            //��λ�����ص���
            if( PosStateVar.State.bit.CDEnd==1 )
            {
                OCDOverVar.state.CDCntEn = 0;
                OCDOverVar.state.ODCntEn = 0;
                if(g_hPosLoop.FullLockEn==2)//ֱ�Ӷ�λ
                {
                    if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<((int16)g_hCurveGenerate.PolyLineVar.StrapElongate*2)) )
                    {
                        g_hPosLoop.StateCmd.PosLoop = 1;//����
                        g_hPosLoop.StateCmd.Step = 1;
                        g_hPosLoop.StateCmd.Full = 0;
                    }
                    g_hPosLoop.PosRef = g_hCurveGenerate.PolyLineVar.StrapElongate;
                }
                if( StallCheckStru.StateCmd.StallFlag==1 )
                {
                 if((Camstateaction.StateCmd.QepcountInitSamOk == 0)&&(Camstateaction.StateCmd.QepcountSecSamClac == 0))
                  {
                          ///Camstateaction.QepcountInit = ((Uint32)EQep1Regs.QPOSCNT);///��¼�ŵ����ŵ�λ�����ʼֵ��ֻ��¼һ��
                          ///Camstateaction.QepcountInitSave = Camstateaction.QepcountInit;///�����ϴ�ԭ��ֵ
                          Camstateaction.QepcountInit = (PosStateVar.DoorPosOrigin << 2);
                          Camstateaction.StateCmd.QepcountInitSamOk = 1;
                  }
                  
                  DoorFianlVar.StateCmd.CDSwithEn = 1;//�����л���־
                    if( DoorFianlVar.StateCmd.CDSwithFlag==1 )
                    {
                        //	�������ձ�����
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque;
                    }
                    else
                    {
                        //���ű�����
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                    }
                    if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<(int16)g_hCurveGenerate.PolyLineVar.StrapElongate) )
                    {
                            
                        if(g_hPosLoop.FullLockEn==1)
                        {
                            if(g_hPosLoop.StateCmd.PosLoop==0)
                            {
                                g_hPosLoop.StateCmd.PosLoop = 1;//����
                                g_hPosLoop.StateCmd.Step = 1;
                                g_hPosLoop.StateCmd.Full = 1;
                            }
                            g_hPosLoop.PosRef = PosStateVar.DoorPosActual0;
                        }
                    }
                        
                }
                else
                {
                    DoorFianlVar.StateCmd.CDSwithEn = 0;
                    if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.CDSwithTorque	)
                    {
                        //���ű�����
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                    }
                    else
                    {
                        //���������
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
                    }
                }
            }//end of end
            else//�ǹ��ŵ�λ��Ϊ�����������
            {
                MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
                g_hPosLoop.StateCmd.PosLoop = 0;//������
            }

        }
        //���ų�ʱ����
        if( (OCDOverVar.state.CDOverFlag==1)&&(StallCheckStru.StateCmd.StallFlag==1) )
        {
            //���ű�����
            MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
        }
    }//end of LCD
    else if( CTLcmdVar.FinalCmd.bit.LOD==1 )
    {
        g_hPosLoop.StateCmd.StallPosLoop = 0;
        g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
        OCDOverVar.state.CDCntEn = 0;//���ų�ʱ��ⲻ����
        OCDOverVar.state.ODCntEn = 1;//���ų�ʱ�������
        OCDOverVar.state.CDOverFlag = 0;//���ų�ʱ��־����
        ////�ƶ������
        MTCVar.CurrentMin = PosStateVar.TorqueInter.OD_MaxTorque*-1;
        if( PowerFstVar.PowerFst==1 )
        {
            g_hPosLoop.StateCmd.PosLoop = 0;//������
           
             //���������
              if(AutoWidthStudyRunOpen_initOK == 0)
              {
                      MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
              }
              else
              {
                      MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
              }
           
           /*
            //���������
            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque	)
            {
                if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                {
                    //���ű�����
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                }
                DoorFianlVar.StateCmd.ODHDToHFst = 1;
            }
            else
            {
                if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
                {
                    //���������
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
                }
            }*/
            
          
            
            //MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
            //��λ��ʱ����ת���Ϊ1S���ҵ���ת����Ϊ��λ���Ա������������ϵ��־����
            if( (StallCheckStru.StateCmd.StallFlag==1) )
            {
              if((DoorWidthAutoStudy_en == 1) && (AutoWidthStudyRun_initOver == 0))///ʹ���ſ���ʶ����
                {
                    if((AutoWidthStudyRunOpen_initOK == 0) && (AutoWidthStudyRunCls_initOK == 0))///�����ϵ�����Ϊ��������
                    {
                        PosStateVar.State.bit.CDEnd = 0;
                        PosStateVar.State.bit.Sgs = 0;
                        PosStateVar.State.bit.ODEnd = 1;
                        AutoWidthStudyRunOpen_initOK = 1;

                        ///DoorStudyVar.PosInput = qep1.OutputRawTheta;
                        DoorStudyVar.PosOrigin = qep1.OutputRawTheta;///��ס��ʼλ�õ�
                    }
                    else if(AutoWidthStudyRunCls_initOK == 1)///�����ϵ�Ϊ��������ѽ��г���λ�ö�λ���ſ���ʶ�����
                    {
                          ///DoorStudyVar.PosInput = qep1.OutputRawTheta;
                          DoorStudyVar.Width = labs(qep1.OutputRawTheta-DoorStudyVar.PosOrigin);///����ſ�
                          DoorStudyVar.Width = DoorStudyVar.Width>>2;
                          DoorStudyVar.DecPos.DoorMil	=	_IQ15mpyI32int(DoorStudyVar.Width,DoorStudyVar.PosToMilCoffe);
                          if(DoorStudyVar.DecPos.DoorMil < 3000)///Ĭ��ѧϰ����600�ſ�Ϊ����
                          {
                              ///DoorStudyVar.DecPos.DoorMil = 4500;
                              ///DoorStudyVar.Width	=	_IQ15mpyI32int(DoorStudyVar.DecPos.DoorMil,DoorStudyVar.MilToPosCoffe);
                              FaultVar.fault.FautFlag1.bit.WidthFstErorr = 1;

                              gCap.InitTime = 100;
                              PowerFstVar.ProInit = 1;
                              PowerFstVar.PowerFst = 1;
                              gCap.InitMechThetaFlag = 0;

                              MTCVar.StateCmd.RunEn = 0;
                          }
                          else
                          {
                              DoorWidthAutoStudy_en = 0;///ѧϰ��ɺ󣬱�־����

                              DoorStudyVar.DecPos.DecH = (DoorStudyVar.Width)/10000;
                              DoorStudyVar.DecPos.DecL = (DoorStudyVar.Width)%10000;

                              PosStateVar.State.bit.CDEnd = 0;
                              PosStateVar.State.bit.Sgs = 0;
                              PosStateVar.State.bit.ODEnd = 1;
                              AutoWidthStudyRun_initOver = 1;


                              PowerFstVar.PowerFst = 0;
                              StallCheckStru.StateCmd.ClrStall = 1;
                              StallCheckStru.StateCmd.StallFlag = 0;
                              TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
                              TorqueProtecVar.StateCmd.ClrEn = 1;
                              if( PosStateVar.State.bit.DoorDirectionCMD==0 )
                              {
                              PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2) - DoorStudyVar.Width;
                              }
                              else
                              {
                              PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2) + DoorStudyVar.Width;
                              }

                              PosStateVar.DoorPosOriginSave = PosStateVar.DoorPosOrigin;
                              //���������ʱ��
                              CTLcmdVar.HoldTimeCnt = 0;
                              //CTLcmdVar.state.ODHoldFlag = 1;
                              g_hPosLoop.StateCmd.StallPosLoop = 0;
                              //�ſ���У����ʼ��
                              PosStateVar.ODDoorWidth = DoorStudyVar.Width;
                              PosStateVar.CDDoorWidth = DoorStudyVar.Width;
                              PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;

                              LOD_DoorMil = _IQ15mpyI32int(DoorStudyVar.Width,PosStateVar.PosToMilCoeff);

                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[8].ParaPro.E2ROMAdd;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[8].ParaPro.E2ROMBlock;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[8].ParAdd);
                              E2PROM_TaskSet(&E2promTask);

                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[35].ParaPro.E2ROMAdd;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[35].ParaPro.E2ROMBlock;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
                              E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[35].ParAdd);
                              E2PROM_TaskSet(&E2promTask);
                          }
                    }
                }
                else if((DoorWidthAutoStudy_en == 0) || (AutoWidthStudyRun_initOver == 1))
                {
                        PowerFstVar.PowerFst = 0;
                        StallCheckStru.StateCmd.ClrStall = 1;
                        StallCheckStru.StateCmd.StallFlag = 0;
                        TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
                        TorqueProtecVar.StateCmd.ClrEn = 1;
                        if( PosStateVar.State.bit.DoorDirectionCMD==0 )
                        {
                                PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2) - DoorStudyVar.Width;
                        }
                        else
                        {
                                PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2) + DoorStudyVar.Width;
                        }

                        PosStateVar.DoorPosOriginSave = PosStateVar.DoorPosOrigin;
                        //���������ʱ��
                        CTLcmdVar.HoldTimeCnt = 0;
                        //CTLcmdVar.state.ODHoldFlag = 1;
                        g_hPosLoop.StateCmd.StallPosLoop = 0;
                }
                
            }
        }
        else
        {
            //��λ�����ص���
            OCDOverVar.state.CDCntEn = 0;
            OCDOverVar.state.ODCntEn = 0;
            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque )
            {
                if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                {
                    //���ű�����
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                }
                DoorFianlVar.StateCmd.ODHDToHFst = 1;
            }
            else
            {
                if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
                {
                    //���������
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
                }
            }

            if( StallCheckStru.StateCmd.StallFlag==1 )
            {
                DoorFianlVar.StateCmd.ODSwithEn = 1;
                CTLcmdVar.FinalCmd.bit.ReOD = 0;//���ŵ�λ�������������
                if( DoorFianlVar.StateCmd.ODSwithFlag==1 )
                {
                    DoorFianlVar.StateCmd.ODHToHLFst = 1;
                    DoorFianlVar.StateCmd.ODHDToHFst = 1;
                    //�������ձ�����
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlLastTorque;
                }
                else
                {
                    if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                    {
                        //���ű�����
                        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                    }
                        
                }
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0>( (int16)DoorStudyVar.Width-(int16)g_hCurveGenerate.PolyLineVar.StrapElongate)) )
                {
                        
                    if(g_hPosLoop.FullLockEn==1)
                    {
                        g_hPosLoop.StateCmd.PosLoop = 1;//����
                        g_hPosLoop.StateCmd.Full = 1;
                        g_hPosLoop.PosRef = PosStateVar.DoorPosActual0-(g_hCurveGenerate.PolyLineVar.StrapElongate) ;//����λ��Ƥ��������
                    }
                }
            }
            else
            {
                DoorFianlVar.StateCmd.ODSwithEn = 0;
                g_hPosLoop.StateCmd.PosLoop = 0;//������

            }

        }
        //���ų�ʱ����
        if( (OCDOverVar.state.ODOverFlag==1)&&(StallCheckStru.StateCmd.StallFlag==1) )
        {
        //���ű�����
        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
        }

    }
    else//stop
    {
        OCDOverVar.state.CDCntEn = 0;
        OCDOverVar.state.ODCntEn = 0;
        OCDOverVar.state.CDOverFlag = 0;//���ų�����?
        OCDOverVar.state.ODOverFlag = 0;
        g_hPosLoop.StateCmd.StallPosLoop = 0;
    }
    //���͸���������
    if( (CTLcmdVar.FinalCmd.bit.LOD==1)||(CTLcmdVar.FinalCmd.bit.OD)||(CTLcmdVar.FinalCmd.bit.ReOD) )
    {	   
        if(CTLcmdVar.FinalCmd.bit.LOD==1)
        {
            g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 3;
        }
        else
        {
            g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 1;
        }
        MTCVar.StateCmd.RunEn = 1;
            
    }
    else if( (CTLcmdVar.FinalCmd.bit.LCD==1)||(CTLcmdVar.FinalCmd.bit.CD)||(CTLcmdVar.FinalCmd.bit.ReCD) )
    {
        if(CTLcmdVar.FinalCmd.bit.LCD==1)
        {
            if( PowerFstVar.PowerFst==1 )//δ�ҵ�λ��ʱ���ǲ������ߵĹ���
            {
                    g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 2;
            }
            else
            {
                    g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 0;
            }
        }
        else
        {
            g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 0;
        }
        MTCVar.StateCmd.RunEn = 1;
    }
    else
    {
        if(PowStallEn==1)
        {
            MTCVar.StateCmd.RunEn = 0;//�������ʹ
        }
                
        if( StopSel==2 )//����ʱ��ͣ��
        {
            if(FaultVar.fault.FautFlag0.bit.QEPErrorFlag==1)
            {
                g_hPosLoop.StateCmd.PosLoop = 0;
                MTCVar.StateCmd.RunEn = 0;//������йر� 
            }
            else
            {
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PowStallEn==0) )
                {
                    g_hCurveGenerate.RealCurveCalc.CmdState.PosLock = 1;
                    g_hPosLoop.StateCmd.PosLoop = 1;
                    g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
                    g_hPosLoop.StateCmd.Step = 0;
                    
                   /// g_hPosLoop.PosRef = (PosStateVar.DoorPosActual0 - g_hCurveGenerate.PolyLineVar.StrapElongate);///K400�������
                    g_hPosLoop.PosRef = PosStateVar.DoorPosActual0;
                    g_hPosLoop.StateCmd.Full = 0;
                }
            }
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
        }
        else if( StopSel==1 )//�ܺ��ƶ�
        {
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
            MTCVar.StateCmd.RunEn = 0;

        }
        else//ͣ��
        {	
            MTCVar.StateCmd.RunEn = 0;
        }
            
    }
    if( g_hCurveGenerate.RealCurveCalc.CmdState.CmdSwt==1 )//�л�ʱ�����ת������ؼ��
    {
        TorqueProtecVar.StateCmd.ClrEn = 1;
    }
    
    ///ƥ���¹����ŵ��߼�����
	///if(((OutDelay.StateCmd.bit.CDEndOut==1 )&&(QepcountInitSamOk != 0)) || (Camstateaction.StateCmd.CamStateActionRun == 1))///���ŵ�λ���һ�׼ֵ������ɺ�ʵʱ��¼��λ��
	if(Camstateaction.CamStateActionDoneEn == 1)///ʹ���ŵ��߼��������
	{
		if(Camstateaction.StateCmd.QepcountInitSamOk == 1)///��ʼ��������ɺ󣬿�ʵʱ�����ŵ�λ��
		{
			///Camstateaction.QepcountInitSave = Camstateaction.QepcountInit;
			Camstateaction.QepcountSec = ((Uint32)qep1.OutputRawTheta);///((Uint32)EQep1Regs.QPOSCNT);
			Camstateaction.QepcountErr = labs(Camstateaction.QepcountSec - Camstateaction.QepcountInit);
			Camstateaction.QepcountBase = (Uint32)(76980 / g_hUnitTransform.Whell_dia);///4096*70/(234*3.1415926);����5.9mm��Ϊ�ǿ��ŵ�ֵ    76980
			Camstateaction.QepReccountBase = (Uint32)(612788 / g_hUnitTransform.Whell_dia);///4096*470/(254*3.1415926);����47mm��Ϊ�ָ���������ֵ   1173418
			Camstateaction.StateCmd.QepcountSecSamClac = 1;///���ڼ��㣬��׼ֵ�����в���
			if( StallCheckStru.StateCmd.StallFlag==1 )
			{
				Camstateaction.QepCdEndsBase = 1;
			}
		}
		else///�˳����ŵ�λ������׼��������
		{
			///QepcountInitSamOk = 0;///��ʼ������ֻ����һ�Σ�Ӧ����EEPROM��
			///QepcountSecSamClac = 0;
		}

		///if(OutDelay.StateCmd.bit.ODEndOut == 1 )///���ŵ�λ���������²���
		if(( (CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.LOD==1)||(CTLcmdVar.FinalCmd.bit.ReOD) )&&(PosStateVar.State.bit.CDEnd == 0)) ///���ŵ�λ���� ��־������  20200318
		{
			Camstateaction.StateCmd.QepcountInitSamOk = 0;
			Camstateaction.QepcountSec = 0;
			Camstateaction.QepcountErr = 0;
			Camstateaction.StateCmd.QepcountSecSamClac = 0;///���ڼ��㣬��׼ֵ�����в���
			Camstateaction.StateCmd.CamStateActionRun = 0;///20sʱ���־���㣬�������
			Camstateaction.QepCdEndsBase = 0;
			FaultVar.fault.FautFlag1.bit.CamActionFault = 0;//�����߼���������
                        
                        CTLcmdVar.FinalCmd.bit.ReCD = 0;      ///20190912 �Ľ��ŵ��ǿ��󣬿���ָ��㶯���Զ�����BUG
                        Camstateaction.StateCmd.Step = 0;     ///
                        Camstateaction.StateCmd.ReCamen = 0;  /// 
		}

	/*
		if(( OutDelay.StateCmd.bit.CDEndOut==1 )&&(QepcountErr > QepcountBase)&&(CTLcmdVar.FinalCmd.bit.CD==0)
		&&(CTLcmdVar.FinalCmd.bit.LCD==0))///�ݶ�5��mm
		{
			Qepstallcounti++;
			if(Qepstallcounti >= 1)
			{
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque >> 8;///�ŵ�����
				CTLcmdVar.FinalCmd.bit.ReCD = 0;
				Qepstallcounti = 0;
			}
			else
			{
				//	�������ձ�����
				StallCheckStru.StateCmd.StallFlag = 0;
				CTLcmdVar.FinalCmd.bit.ReCD = 1;
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
			}

			OutDelay.StateCmd.bit.CDEndOut = 0;
			QepcountInitSamOk = 0;
			QepcountSecSamClac = 0;
		}*/
	/*
		HMI_CamStep = Camstateaction.StateCmd.Step;*/

		if(Camstateaction.StateCmd.ReCDen == 1)///�յ����ڣ��ָ��յ���ִ�й���ָ��
		{
			CTLcmdVar.FinalCmd.bit.ReCD = 1;
			if((OutDelay.StateCmd.bit.CDEndOut == 1)||(CTLcmdVar.FinalCmd.bit.OD == 1)||(CTLcmdVar.FinalCmd.bit.LOD == 1))
			{
				Camstateaction.StateCmd.ReCDen = 0;
				CTLcmdVar.FinalCmd.bit.ReCD = 0;
			}
		}

		if(Camstateaction.StateCmd.ReCamen == 1)///������������ָ������յ�ָ������
		{
			if((OutDelay.StateCmd.bit.CDEndOut == 1)||(CTLcmdVar.FinalCmd.bit.OD == 1)||(CTLcmdVar.FinalCmd.bit.LOD == 1)||
			  (CTLcmdVar.FinalCmd.bit.CD == 1)||(CTLcmdVar.FinalCmd.bit.LCD == 1))
			{
				Camstateaction.StateCmd.ReCamen = 0;
			}
		}

		switch(Camstateaction.StateCmd.Step)
			{
				case 0:///���������ж�
					///if(( OutDelay.StateCmd.bit.CDEndOut==1 )&&(QepcountInitSamOk != 0)&&(QepcountErr > QepcountBase)&&())///�ڹ��ŵ�λ�����ݶ�7��mm
					if(( OutDelay.StateCmd.bit.CDEndOut==1 )&&(Camstateaction.QepCdEndsBase == 1)&&(Camstateaction.QepcountErr > Camstateaction.QepcountBase))///�ݶ�5��mm������������
					{
						Camstateaction.data.CamStateActionTime = 50;///�˲�50ms
						Camstateaction.StateCmd.CamStateActionRun = 1;///ִ�а��ų��򣬹���ָ�ִ�У�20s����ŵ�λ���˱�־λ����
						Camstateaction.QepCdEndsBase = 0;
						Camstateaction.StateCmd.Step = 1;///������ʱ�ж�
						FaultVar.fault.FautFlag1.bit.CamActionFault = 1;//�����߼�����
						Camstateaction.CamStateActionCount++; //ִ��һ�η����߼�������+1
						if(Camstateaction.CamStateActionCount >=60000 ) Camstateaction.CamStateActionCount = 60001;
					}
					else
					{
						Camstateaction.StateCmd.Step = 0;///����
					}

					if((Camstateaction.QepcountErr < Camstateaction.QepReccountBase) && (Camstateaction.StateCmd.ReCamen == 1))///�ǿ���Ȼ�ڻ�ǿ���ָ��������յ����� G0+47mm
					{
						if(Camstateaction.StateCmd.CamStateActionRun == 1)
						{
							Camstateaction.StateCmd.Step = 0;///����Case0,����
						}
						else
						{
							Camstateaction.StateCmd.Step = 2;///����Case2�����ָ��յ�����
						}

					///	Camstateaction.StateCmd.CamStateActionRun = 1;///ʹ�ܰ��Ŵ����־
						Camstateaction.StateCmd.ReCamen = 0;
					}


				break;
				case 1:///�����ź�ʱ���˲���ִ��˫��������
					if( Camstateaction.data.CamStateActionTime == 0 )
					{
						MTCVar.CurrentMax = (PosStateVar.TorqueInter.OD_HlTorque >> 2); //�л������ű�����
						MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque >> 8;///�ŵ�����
						Camstateaction.StateCmd.CamStateActionRun = 1;///ִ�а��ų��򣬹���ָ�ִ�У�20s����ŵ�λ���˱�־λ����
						if(PowerFstVar.PowerFst==1)
						{
							///PosStateVar.State.bit.CDEnd = 0;
							OutDelay.StateCmd.bit.CDEndOut = 0;
						}

						Camstateaction.StateCmd.Step = 2;///����ָ��յ��߼�
					}
					else
					{
						if(Camstateaction.QepcountErr < Camstateaction.QepcountBase)///�ݶ�7��mm���п���ָ��£�����50ms���ָֻ�λ���ˣ�
						{
							Camstateaction.StateCmd.Step = 0;///����
						}
						else
						{

						}
					}
				break;
				case 2:///���ź�λ����--�˲�ʱ���ж�
					if(Camstateaction.QepcountErr < Camstateaction.QepReccountBase)///��Ȼ�������յ����� G0+47mm
					{
						Camstateaction.data.CamStateActionTime = 1500;///�˲�1000ms
						Camstateaction.StateCmd.Step = 3;///������ʱ�ж�
					}
					else
					{
						Camstateaction.data.CamStateActionTime100ms = 200;///�˲�20s,����Ӧ����ָ��
						Camstateaction.StateCmd.Step = 4;///������ʱ�ж�
					}
				break;
				case 3:///���ź�λ����---���յ����ڣ��ָ��յ�
					if( Camstateaction.data.CamStateActionTime == 0 )///һֱ��G0+47mm�ڣ��ָ��յ�
					{
					///	CTLcmdVar.FinalCmd.bit.ReCD = 1;
					///	MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;///������������
					///	MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
						Camstateaction.StateCmd.CamStateActionRun = 0;///ִ�а��ų���20s����ŵ�λ���˱�־λ����,���Լ�ִ���յ�
						Camstateaction.StateCmd.ReCDen = 1;
						Camstateaction.StateCmd.Step = 0;///����
					}
					else
					{
						if(Camstateaction.QepcountErr > Camstateaction.QepReccountBase)///�ǳ������յ�����G0+47mm
						{
							Camstateaction.data.CamStateActionTime100ms = 50;///200
							Camstateaction.StateCmd.Step = 4;///������ʱ�ж�
						}
					}
				break;
				case 4:///���ź�λ����
					if( Camstateaction.data.CamStateActionTime100ms == 0 )///����G0+47mm�ڣ�������20s��ָ�����
					{
						///if((CTLcmdVar.FinalCmd.bit.CD)||(CTLcmdVar.FinalCmd.bit.LCD))///20s����Ӧ����ָ��
					//	{
						///	MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;///������������
						//	MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
							Camstateaction.StateCmd.CamStateActionRun = 0;///ִ�а��ų���20s����ŵ�λ���˱�־λ����
							Camstateaction.StateCmd.ReCamen = 1;///���ź� 20s��λ��־
					//	}
						///else
						//{

						//}

						Camstateaction.StateCmd.Step = 0;///����
					}
					else
					{
						if(Camstateaction.QepcountErr < Camstateaction.QepReccountBase)///�����»ص������յ����� G0+47mm
						{
							Camstateaction.StateCmd.Step = 2;///�����ж��յ��߼�
						}
					}

				break;
				case 9:///�ŵ������߼�����

					///Camstateaction.data.CamStateActionTime = 0;///50ms/1000msʱ�������
					///Camstateaction.data.CamStateActionTime100ms = 0;///20sʱ�������
					///Camstateaction.StateCmd.Step = 0;///��������
				break;

				default:break;
			}
	}
    
    //TestAdd2 = g_hPosLoop.PosRef ;
}
/*******************************************************************************
* ��������  : ParInit
* ����      : ������ʼ��
* ����      : None
* ��?     : None
* ����      : None.
*******************************************************************************/
void ParInit(void)
{
  Uint16 temp;
  gE2rom.E2promDataSturct.E2ROMAdd =SysProtection[0].ParaPro.E2ROMAdd;
  gE2rom.E2promDataSturct.E2ROMBlock = SysProtection[0].ParaPro.E2ROMBlock;
  gE2rom.E2promDataSturct.nrData = 1;
  gE2rom.E2promDataSturct.parameterlink = (Uint32 *) &(SysProtection[0].ParAdd);
  gE2rom.Command = READCMD;
  while( gE2rom.CmdState.ReadOverFlag==0 )
  {
    gE2rom.updata(&gE2rom);	
    if(gE2rom.CmdState.WtireReadError==1) 
    {
        asm("nop");
        break;
    }
  }
  gE2rom.CmdState.ReadOverFlag = 0;
  if(gE2rom.CmdState.WtireReadError==0)
  {
    if(g_intFstWriterNum==FstNum )//д���������ֱ�Ӷ�ȡ
    {
      ReadParAll();	
    }
    else//��һ��ʱ����д��E2PROM
    {
      WriteParAll(22);
      //ReadParAll();
      g_intFstWriterNum = FstNum;
      E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysProtection[0].ParaPro.E2ROMAdd;//SysParaAttrChar[partemp].ParaPro.E2ROMAdd;
      E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysProtection[0].ParaPro.E2ROMBlock;//SysParaAttrChar[partemp].ParaPro.E2ROMBlock;
      E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
      E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
      E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink = (Uint32 *) &(SysProtection[0].ParAdd);//(Uint16 *) &(SysParaAttrChar[partemp].ParAdd);
      E2PROM_TaskSet(&E2promTask);
    }
  }
  while(1)
  {
    //E2PROM manage---E2PROM���������
    E2prom_TaskManage();
    //E2prom deal
    gE2rom.updata(&gE2rom);	
    if( gE2rom.CmdState.WtireReadError==1 )
    {
      return;
    }
    for(temp=0;temp<E2PROM_MAXNUM;temp++)
    {
      if( E2promTask.E2PROM_Distribute[temp].ResponseFlag==1 )
      {
        break;
      }
    }
    if( (temp==E2PROM_MAXNUM)&&((gE2rom.CmdState.ReadOverFlag==1)||(gE2rom.CmdState.WriteOverFlag==1)) )
    {
      gE2rom.CmdState.ReadOverFlag = 0;
      gE2rom.CmdState.WriteOverFlag = 0;
      return;
    }

  }
}
/*******************************************************************************
* ��������  : OutDeal
* ����      : ����̵�������
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void OutDeal(void)
{
	if( PowerFstVar.PowerFst==0 )
	{
		//�����ŵ�λ����
		if( PosStateVar.State.bit.ODEnd==1 )
		{
			if( OutDelay.ODOutMode==0 )
			{
				if( OutDelay.ODOutDelayCnt>(OutDelay.ODOutDelayTime*100) )
				OutDelay.StateCmd.bit.ODEndOut = 1;
			}
			else if( OutDelay.ODOutMode==1 )
			{
				if( StallCheckStru.StateCmd.StallFlag==1 )
				{
					OutDelay.StateCmd.bit.ODEndOut = 1;
				}
			}
			else
			{
				
				if( ( (CTLcmdVar.FinalCmd.bit.CD)||(CTLcmdVar.FinalCmd.bit.LCD) )  )
				{
					if( PosStateVar.State.bit.ODFullDisEn==1 )
					{
						OutDelay.StateCmd.bit.ODEndOut = 0;
					}
					else
					{
						
					}
				}
				else
				{
					if( PosStateVar.State.bit.ODFullEn==1 )
					{
						if( StallCheckStru.StateCmd.StallFlag==1 )
						{
							OutDelay.StateCmd.bit.ODEndOut = 1;
						}
					}
					else
					{
					}
				}
                        }
			if( (CTLcmdVar.FinalCmd.bit.CD)||(CTLcmdVar.FinalCmd.bit.LCD) )
			{
				OutDelay.StateCmd.bit.ODEndOut = 0;
			}
		}
		else
		{
			OutDelay.StateCmd.bit.ODEndOut = 0;
			OutDelay.ODOutDelayCnt = 0;
			//OutDelay.CDOutDelayCnt = 0;
		}
		if( PosStateVar.State.bit.CDEnd==1 )
		{
		  /* if( StallCheckStru.StateCmd.StallFlag==1 )  
			{
				OutDelay.StateCmd.bit.CDEndOut = 1;
			}
		
			if( (CTLcmdVar.FinalCmd.bit.OD)||(CTLcmdVar.FinalCmd.bit.ReOD)||(CTLcmdVar.FinalCmd.bit.LOD) )
			{
				OutDelay.StateCmd.bit.CDEndOut = 0;
			}*/
                   
                        if( OutDelay.CDOutMode==0 )
			{
				if(OutDelay.CDOutDelayCnt>OutDelay.CDOutDelayTime*100)
					OutDelay.StateCmd.bit.CDEndOut = 1;
			}
			else if( OutDelay.CDOutMode==1 )
			{
				if( StallCheckStru.StateCmd.StallFlag==1 )
				{
					OutDelay.StateCmd.bit.CDEndOut = 1;
				}
			}
                        else if(OutDelay.CDOutMode==2)
			{
				if(( StallCheckStru.StateCmd.StallFlag==1 )&&((CTLcmdVar.TerminalCmd.bit.TerminalCD == 1)||(CTLcmdVar.CMDSel==4)))
				{
					OutDelay.StateCmd.bit.CDEndOut = 1;
				}
			}
			else
			{
				
				if( (CTLcmdVar.FinalCmd.bit.OD)||(CTLcmdVar.FinalCmd.bit.ReOD)||(CTLcmdVar.FinalCmd.bit.LOD)  )
				{
					if( PosStateVar.State.bit.CDFullDisEn==1 )
					{
						OutDelay.StateCmd.bit.CDEndOut = 0;
					}
					else
					{
						
					}
				}
				else
				{
					if( PosStateVar.State.bit.CDFullEn==1 )
					{
						if( StallCheckStru.StateCmd.StallFlag==1 )
						{
							OutDelay.StateCmd.bit.CDEndOut = 1;
						}
					}
					else
					{
					}
				}
			}

			if( ( (CTLcmdVar.FinalCmd.bit.OD)||(CTLcmdVar.FinalCmd.bit.ReOD)||(CTLcmdVar.FinalCmd.bit.LOD)||((CTLcmdVar.TerminalCmd.bit.TerminalCD == 0)&&(OutDelay.CDOutMode==2)&&(CTLcmdVar.CMDSel != 4)) ) )
			{
				OutDelay.StateCmd.bit.CDEndOut = 0;
			}
		}
		else
		{
			OutDelay.StateCmd.bit.CDEndOut = 0;
			//OutDelay.ODOutDelayCnt = 0;
			OutDelay.CDOutDelayCnt = 0;	
		}
		//��ת����
		if( (PosStateVar.State.bit.ODEnd==0)&&(PosStateVar.State.bit.CDEnd==0) )
		{
			if( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1) )//��ת�����ؾ���Ϊ���?			{
			{	
                                if( (g_hCurveGenerate.RealCurveCalc.CmdState.ODCD==1)&&(OutDelay.FaultOutODEn==0) )//���������Ҳ��������ʱ
				{
					OutDelay.StateCmd.bit.StallOut = 0;
				}
				else
				{
					OutDelay.StateCmd.bit.StallOut = 1;
					OutDelay.OutDelayStall = 50;//500ms
				}
			}
			else
			{
				if(OutDelay.OutDelayStall==0)
				{
					OutDelay.StateCmd.bit.StallOut = 0;
				}
			}
		}
		else
		{
			OutDelay.StateCmd.bit.StallOut = 0;
		}
	}
	else
	{
	//�����ŵ�λ����
		if( PosStateVar.State.bit.ODEnd==1 )
		{
			if( OutDelay.ODOutMode==0 )
			{
				if( StallCheckStru.StateCmd.StallFlag==1 )
				{
					OutDelay.StateCmd.bit.ODEndOut = 1;
				}
			}
			else
			{
				if( StallCheckStru.StateCmd.StallFlag==1 )
				{
					OutDelay.StateCmd.bit.ODEndOut = 1;
				}
			}
			if( (CTLcmdVar.FinalCmd.bit.CD)||(CTLcmdVar.FinalCmd.bit.LCD) )
			{
				OutDelay.StateCmd.bit.ODEndOut = 0;
			}
		}
		else
		{
			OutDelay.StateCmd.bit.ODEndOut = 0;
			OutDelay.ODOutDelayCnt = 0;
		}
		if( PosStateVar.State.bit.CDEnd==1 )
		{
		   if( OutDelay.CDOutMode==0 )
			{
				if(OutDelay.CDOutDelayCnt>OutDelay.CDOutDelayTime*100)
					OutDelay.StateCmd.bit.CDEndOut = 1;
			}
			else if( OutDelay.CDOutMode==1 )
			{
				if( StallCheckStru.StateCmd.StallFlag==1 )
				{
					OutDelay.StateCmd.bit.CDEndOut = 1;
				}
			}
			else if(OutDelay.CDOutMode==2)
			{
				if(( StallCheckStru.StateCmd.StallFlag==1 )&&((CTLcmdVar.TerminalCmd.bit.TerminalCD == 1)||(CTLcmdVar.CMDSel==4)))
				{
					OutDelay.StateCmd.bit.CDEndOut = 1;
				}
			}

			if( (CTLcmdVar.FinalCmd.bit.OD)||(CTLcmdVar.FinalCmd.bit.ReOD)||(CTLcmdVar.FinalCmd.bit.LOD)||((CTLcmdVar.TerminalCmd.bit.TerminalCD == 0)&&(OutDelay.CDOutMode==2)&&(CTLcmdVar.CMDSel != 4)) )
			{
				OutDelay.StateCmd.bit.CDEndOut = 0;
			}
		}
		else
		{
			OutDelay.StateCmd.bit.CDEndOut = 0;
			//OutDelay.ODOutDelayCnt = 0;
			OutDelay.CDOutDelayCnt = 0;
		}
		//��ת����
		if( (PosStateVar.State.bit.ODEnd==0)&&(PosStateVar.State.bit.CDEnd==0) )
		{
			if( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1) )//��ת������ؾ���Ϊ��ת
			{
				if( (g_hCurveGenerate.RealCurveCalc.CmdState.ODCD==1)&&(OutDelay.FaultOutODEn==0) )//���������Ҳ��������ʱ
				{
					OutDelay.StateCmd.bit.StallOut = 0;
				}
				else
				{
					OutDelay.StateCmd.bit.StallOut = 1;
					OutDelay.OutDelayStall = 50;//500ms
				}
			}
			else
			{
				if(OutDelay.OutDelayStall==0)
				{
					OutDelay.StateCmd.bit.StallOut = 0;
				}
			}
		}
		else
		{		       
		        OutDelay.StateCmd.bit.StallOut = 0;
		}
	}
	//���ϴ���
	OutDelay.StateCmd.bit.FaultOut = PosStateVar.State.bit.Sgs; ///��ȫ����
	if(FaultVar.fault.FautFlag0.bit.QEPErrorFlag)
	{
		OutDelay.StateCmd.bit.CDEndOut = 0;
		OutDelay.StateCmd.bit.ODEndOut = 0;
                PosStateVar.State.bit.Sgs = 0;
	}

///�������
        
        if(CTLcmdVar.CMDSel!=3)
        {
            //���OUT0L
            if( ValBit(OutDelay.PolarSel,0) )
            {
                    if( OutDelay.StateCmd.bit.ODEndOut==1 )
                    {
                        switch(OutDelay.OutPortSel[0])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                            
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[0])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                            
                    }
            }
            else
            {
                    if( OutDelay.StateCmd.bit.ODEndOut==1 )
                    {
                        
                        switch(OutDelay.OutPortSel[0])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[0])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
            }
            if( ValBit(OutDelay.PolarSel,1) )
            {
                    if( OutDelay.StateCmd.bit.CDEndOut==1 )
                    {
                        switch(OutDelay.OutPortSel[1])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[1])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
            }
            else
            {
                    if( OutDelay.StateCmd.bit.CDEndOut==1 )
                    {
                        switch(OutDelay.OutPortSel[1])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[1])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
            }
            if( ValBit(OutDelay.PolarSel,2) )
            {
                    if( FaultVar.fault.FautFlag1.bit.TemperatureMOve ==1 )
                    {
                        switch(OutDelay.OutPortSel[2])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[2])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
            }
            else
            {
                    if( FaultVar.fault.FautFlag1.bit.TemperatureMOve ==1 )
                    {
                        switch(OutDelay.OutPortSel[2])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[2])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
            }	
                if( ValBit(OutDelay.PolarSel,3) )
            {
                    if( OutDelay.StateCmd.bit.FaultOut==1 )
                    {
                        switch(OutDelay.OutPortSel[3])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[3])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
            }
            else
            {
                    if( OutDelay.StateCmd.bit.FaultOut==1 )
                    {
                        switch(OutDelay.OutPortSel[3])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0L;
                                break;
                            case 2:
                                OUT1L;
                                break;
                            case 3:
                                OUT2L;
                                break;
                            case 4:
                                OUT3L;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        switch(OutDelay.OutPortSel[3])
                        {
                            case 0:
                                break;
                            case 1:
                                OUT0H;
                                break;
                            case 2:
                                OUT1H;
                                break;
                            case 3:
                                OUT2H;
                                break;
                            case 4:
                                OUT3H;
                                break;
                            default:
                                break;
                        }
                    }
            }
        }
        else
        {
         /* if( OutDelay.StateCmd.bit.ODEndOut==1 )  ///���ŵ�λ
          {
                  if(OutDelay.DoorLocation == 0)//ǰ��
                  {
                   //	Ecan_state.Ecan_Tx.bit.FrontDOLEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDOL = 1;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDOLEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDOL = 0;    ///rear.data.bit
                  //	Ecan_state.Ecan_Tx.bit.FrontDCLEN = 0; ///front.mask.bit
                  }
                  else ///����
                  {
                  //	Ecan_state.Ecan_Tx.bit.FrontDOLEN = 0; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDOL = 0;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDOLEN = 1; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDOL = 1;    ///rear.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDCLEN = 0; ///rear.mask.bit
                  }
          }
          else
          {
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDOL = 0;   ///front.data.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDOL = 0;    ///rear.data.bit
          }

          if( OutDelay.StateCmd.bit.CDEndOut==1 ) ///���ŵ�λ
          {
                  if(OutDelay.DoorLocation == 0)//ǰ��
                  {
                  // 	Ecan_state.Ecan_Tx.bit.FrontDCLEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDCL = 1;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDCLEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDCL = 0;    ///rear.data.bit
                  //	Ecan_state.Ecan_Tx.bit.FrontDOLEN = 0; ///front.mask.bit
                  }
                  else ///����
                  {
                  //	Ecan_state.Ecan_Tx.bit.FrontDCLEN = 0; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDCL = 0;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDCLEN = 1; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDCL = 1;    ///rear.data.bit
          //		Ecan_state.Ecan_Tx.bit.RearDOLEN = 0; ///rear.mask.bit
                  }
          }
          else
          {
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDCL = 0;   ///front.data.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDCL = 0;    ///rear.data.bit
          }

          if( OutDelay.StateCmd.bit.StallOut==1 )///��ת ��������>150N
          {
                  if(OutDelay.DoorLocation == 0)//ǰ��
                  {
                  // 	Ecan_state.Ecan_Tx.bit.FrontBRKEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontBRK = 1;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearBRKEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearBRK = 0;    ///rear.data.bit
                  }
                  else ///����
                  {
                  //	Ecan_state.Ecan_Tx.bit.FrontBRKEN = 0; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontBRK = 0;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearBRKEN = 1; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearBRK = 1;    ///rear.data.bit
                  }
          }
          else
          {
                //  Ecan_state.Ecan_Tx.bit.FrontBRKEN = 1; ///front.mask.bit
                  //	Ecan_state.Ecan_Tx.bit.RearBRKEN = 1; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontBRK = 0;   ///front.data.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearBRK = 0;    ///rear.data.bit
          }

          if((TerminalVar.Value.bit.bit1 == 0))     //��Ļ����ȫ����| && (TerminalVar.Value.bit.bit3 == 0)
      {
          if(OutDelay.DoorLocation == 0)//ǰ��
                  {
                  // 	Ecan_state.Ecan_Tx.bit.FrontLTEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontLT = 1;   ///front.data.bit
          //		Ecan_state.Ecan_Tx.bit.RearLTDEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearLTD = 0;    ///rear.data.bit
                  }
                  else ///����
                  {
          //		Ecan_state.Ecan_Tx.bit.FrontLTEN = 0; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontLT = 0;   ///front.data.bit
          //		Ecan_state.Ecan_Tx.bit.RearLTDEN = 1; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearLTD = 1;    ///rear.data.bit
                  }
      }
          else
          {
            //      Ecan_state.Ecan_Tx.bit.FrontLTEN = 1; ///front.mask.bit
          //		Ecan_state.Ecan_Tx.bit.RearLTDEN = 1; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontLT = 0;   ///front.data.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearLTD = 0;    ///rear.data.bit
          }*/
    }
}
/*******************************************************************************
* ��������  : ThreeOutDeal
* ��?     : ���ߵ��������
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ThreeOutDeal(void)
{
	if(SequenCheckVar.StateCmd.CheckEn==1)
	{
		if( SequenCheckVar.StateCmd.DOl )
		{
			if( ValBit(OutDelay.PolarSel,0) )
			{
				OUT1H;
			}
			else
			{
				OUT1L;
			}
		}
		else
		{
			if( ValBit(OutDelay.PolarSel,0) )
			{
				OUT1L;
			}
			else
			{
				OUT1H;
			}
		}
		if( SequenCheckVar.StateCmd.DOS )
		{
			if( ValBit(OutDelay.PolarSel,1) )
			{
				OUT0H;
			}
			else
			{
				OUT0L;
			}
		}
		else
		{
			if( ValBit(OutDelay.PolarSel,1) )
			{
				OUT0L;
			}
			else
			{
				OUT0H;
			}
		}
	}
	else
	{
		if( SequenCheckVar.StateCmd.ShortError )
		{
			return;
		}
		if( PowerFstVar.PowerFst==0	 )
		{
			//�����ŵ�λ����
			if( PosStateVar.State.bit.ODEnd==1 )
			{
				if( OutDelay.ODOutMode==0 )
				{
					OutDelay.StateCmd.bit.ODEndOut = 1;
				}
				else
				{
					if( StallCheckStru.StateCmd.StallFlag==1 )
					{
						OutDelay.StateCmd.bit.ODEndOut = 1;
					}
				}
			}
			else
			{
				OutDelay.StateCmd.bit.ODEndOut = 0;
			}

			if(OCDOverVar.state.PPFlag==1)
			{
				OutDelay.StateCmd.bit.CDEndOut = 1;
			}
			else
			{
				OutDelay.StateCmd.bit.CDEndOut = 0;
			}
			if( CTLcmdVar.FinalCmd.bit.ReOD==0 )
			{
				OCDOverVar.state.PPFlag = 0;
				OutDelay.StateCmd.bit.CDEndOut = 0;
			}
			
			if( (OutDelay.StateCmd.bit.CDEndOutEn==0)||(OutDelay.StateCmd.bit.ODEndOut==1) )
			{
				OutDelay.StateCmd.bit.CDEndOut = 0;
			}

		}

	//OutDelay.StateCmd.bit.FaultOut = PosStateVar.State.bit.Sgs;
		//���OUT0L
	if( ValBit(OutDelay.PolarSel,0) )
	{
		if( OutDelay.StateCmd.bit.ODEndOut==1 )
		{
			OUT1H;
		}
		else
		{
			OUT1L;
		}
	}
	else
	{
		if( OutDelay.StateCmd.bit.ODEndOut==1 )
		{
			OUT1L;
		}
		else
		{
			OUT1H;
		}
	}
	if( ValBit(OutDelay.PolarSel,1) )
	{
		if( OutDelay.StateCmd.bit.CDEndOut==1 )
		{
			OUT0H;
		}
		else
		{
			OUT0L;
		}
	}
	else
	{
		if( OutDelay.StateCmd.bit.CDEndOut==1 )
		{
			OUT0L;
		}
		else
		{
			OUT0H;
		}
	}
	}
		  
}

/**********************************************************************************
  Function:        VdcCal
  Description:     cal the vdc
  Calls:           None
  Called By:       None
  Input:           None
  Output:          None
  Return:          None
  Others:          None
*********************************************************************************/
void VdcCal(void)
{	
	/*_iq tempvol = 0;
	tempvol = _IQ15toIQ(ilg2_vdc1.VdcMeas);

	if( tempvol<VdcSub[2] )
	{
		if( tempvol<VdcSub[0] )
		{
			tempvol = _IQmpy(VdcSubK[0],tempvol)+VdcSubOffset[0];
		}
		else if( tempvol<VdcSub[1] )
		{
			tempvol = _IQmpy(VdcSubK[1],tempvol)-VdcSubOffset[1];
		}
		else
		{
			tempvol = _IQmpy(VdcSubK[2],tempvol)-VdcSubOffset[2];
		}
	}
	else if( tempvol<VdcSub[5] )
	{
		if( tempvol<VdcSub[3] )
		{
			tempvol = _IQmpy(VdcSubK[3],tempvol)-VdcSubOffset[3];
		}
		else if( tempvol<VdcSub[4] )
		{
			tempvol = _IQmpy(VdcSubK[4],tempvol)-VdcSubOffset[4];
		}
		else
		{
			tempvol = _IQmpy(VdcSubK[5],tempvol)-VdcSubOffset[5];
		}
	}
	else
	{
		if( tempvol<VdcSub[6] )
		{
			tempvol = _IQmpy(VdcSubK[6],tempvol)-VdcSubOffset[6];
		}
		else if( tempvol<VdcSub[7] )
		{
			tempvol = _IQmpy(VdcSubK[7],tempvol)-VdcSubOffset[7];
		}
		else if( tempvol<VdcSub[8] )
		{
			tempvol = _IQmpy(VdcSubK[8],tempvol)-VdcSubOffset[8];
		}
		else 
		{
			tempvol = _IQmpy(VdcSubK[9],tempvol)-VdcSubOffset[9];
		}
	}
	ilg2_vdc1.VdcMeas = _IQtoIQ15(tempvol);*/
}

/*******************************************************************************
* ��������  : FaultAnaly
* ����      : ���Ϸ����������ϵ�ʱ������ɶϵ籣��
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void FaultAnaly(void)
{
  Uint16 faultcnt,tempcnt;
  Uint64 faulttemp = 0;
  
  //FaultVar.fault.FautFlag0.bit.ADCERROR = ilg2_vdc1.state.ADCErrorFlag;
  FaultVar.fault.FautFlag0.bit.ADCREFERROR = ilg2_vdc1.state.OffsetErrorFlag;
  FaultVar.fault.FautFlag0.bit.BusErorr = ilg2_vdc1.state.BusVolErrorFlag;
  FaultVar.fault.FautFlag0.bit.E2promError = gE2rom.CmdState.WtireReadError;
  FaultVar.fault.FautFlag1.bit.PowerInLoss = g_hPowerInLoss.state.loss;
  //FaultVar.fault.FautFlag1.bit.ThreeWireError =  SequenCheckVar.StateCmd.ShortError;
  FaultVar.fault.FautFlag1.bit.LossSpd =  g_sLossSpdVar.StateCmd.StallFlag;
	
  if( (MageStudy.StateCmd.MageticStudyError)||(DoorStudyVar.StateCmd.StudyError)||(Magzerostudy.StateCmd.MagZeroPosError) )
  {
    FaultVar.fault.FautFlag0.bit.StudyError =  1;
  }
  else
  {
    FaultVar.fault.FautFlag0.bit.StudyError =  0;
  }
	
  if( ((qep1.QepABError!=ABSIGNALNONE)||(gCap.NoSignalFlag!=0)||(gCap.ParSetErrorFlag!=0)) )
  {
    FaultVar.fault.FautFlag0.bit.QEPErrorFlag = 1;
    if( StudyType!=0 )
    {
      QeqResumeVar.RemsumCnt = RESUMENUM;
    }

  }
  else
  {
    if( (FaultVar.fault.FautFlag0.bit.QEPErrorFlag==1)&&(StudyType!=2) )
    {
      gCap.InitTime = 100;
      PowerFstVar.ProInit = 1;
      gCap.CheckSignalTime = 3000;
      PowerFstVar.PowerFst = 1;
      QeqResumeVar.CmdState.ResumeEn = 0;
      QeqResumeVar.CmdState.ResumeFlag = 0;
      gCap.InitMechThetaFlag = 0;
      QeqResumeVar.RemsumCnt=0;
      FaultVar.fault.FautFlag0.bit.QEPErrorFlag = 0;
      FaultVar.fault.FautFlag0.bit.StallFlag = 0;
      FaultVar.fault.FautFlag0.bit.ForceStall = 0;
      TorqueProtecVar.StateCmd.TorqueOverFlag  = 0;
      TorqueProtecVar.StateCmd.ClrEn = 1;
      StallCheckStru.StateCmd.StallFlag = 0;
      StallCheckStru.StateCmd.ClrStall = 1;
    }
  }
  if( (PosStateVar.State.bit.CDEnd)||(PosStateVar.State.bit.ODEnd)||(PowerFstVar.PowerFst) )
  {
    FaultVar.fault.FautFlag0.bit.StallFlag = 0;
    FaultVar.fault.FautFlag0.bit.ForceStall = 0;
  }
  else
  {
    if( StudyType==0 )
    {
      FaultVar.fault.FautFlag0.bit.StallFlag = StallCheckStru.StateCmd.StallFlag;
      FaultVar.fault.FautFlag0.bit.ForceStall = TorqueProtecVar.StateCmd.TorqueOverFlag;
    }
    else
    {
      FaultVar.fault.FautFlag0.bit.StallFlag = 0;
      FaultVar.fault.FautFlag0.bit.ForceStall = 0;
    }
  }
	
  FaultVar.fault.FautFlag1.bit.ClsTimeLimError =  OCDOverVar.state.CDOverFlag;
  FaultVar.fault.FautFlag1.bit.OpenTimeLimError =  OCDOverVar.state.ODOverFlag;
  if( TemperatureVar.state.OverTempFstFlag==1 )
  {
    FaultVar.fault.FautFlag0.bit.TemperatureOve = TemperatureVar.state.OverTempFstFlag;
  }
  else
  {
    FaultVar.fault.FautFlag0.bit.TemperatureOve = 0;
  }
  FaultVar.fault.FautFlag0.bit.TemperErorr = TemperatureVar.state.TemperError;
  
  if( TemperaMtureVar.state.OverTempMFstFlag==1 )
  {
          FaultVar.fault.FautFlag1.bit.TemperatureMOve = TemperaMtureVar.state.OverTempMFstFlag;
  }
  else
  {
          FaultVar.fault.FautFlag1.bit.TemperatureMOve = 0;
  }
  FaultVar.fault.FautFlag1.bit.TemperMErorr = TemperaMtureVar.state.TemperMError;///����¶ȴ����� err24
	
  if(ilg2_vdc1.state.BusVolErrorFlag==0)
  {
    FaultVar.fault.FautFlag0.bit.VolLow = VolProtectVar.state.LowVoltageFlag;
    FaultVar.fault.FautFlag0.bit.VolOve = VolProtectVar.state.OverVoltageFlag;
  }
  else
  {
    FaultVar.fault.FautFlag0.bit.VolLow = 0;
    FaultVar.fault.FautFlag0.bit.VolOve = 0;
  }

  FaultVar.FaultAll = ((Uint64)FaultVar.fault.FautFlag0.all)+((Uint64)FaultVar.fault.FautFlag1.all<<32);	
  
  if(FaultVar.FaultAll==0 )
  {
    OutDelay.StateCmd.bit.FaultOut = 0;
    //GPIO_SetBits(GPIOA,GPIO_Pin_11);//PA.11 �����
    g_hBuzzerVar.BuzzerEn = 0;
  }
  else
  {
    OutDelay.StateCmd.bit.FaultOut = 1;
    //GPIO_ResetBits(GPIOA,GPIO_Pin_11); /// PA.11 �����  ������
  ///  g_hBuzzerVar.BuzzerEn = 1;
  }
	if( FaultVar.FaultLast!=FaultVar.FaultAll )
	{
          
          g_hBuzzerVar.BuzzerEn = 1;
          
          if( FaultVar.FaultLast<FaultVar.FaultAll )
          {
            faulttemp = (FaultVar.FaultLast)^(FaultVar.FaultAll);
            for(faultcnt=0;faultcnt<64;faultcnt++)
            {
              if((faulttemp&0x0001)==1)//FaultVar.fault.FautFlag0.bit.VolLow==0)
              {
              }
              else
              {
                VolProtectVar.state.SaveFst = 0;
              }
              if( faulttemp&0x0001!=0 )
              {
                if( faultcnt!=0 )
                {
                  for(tempcnt=4;tempcnt>0;tempcnt--)
                  {
                    FaultVar.ParSave.FaultSave[tempcnt] = FaultVar.ParSave.FaultSave[tempcnt-1];
                  }
                  FaultVar.ParSave.FaultSave[0] = faultcnt+1;
                  FaultVar.ParSave.VolSave = VolProtectVar.Vdc;
                  FaultVar.ParSave.FreSave = MTCVar.Dis.SpeedRef;
                  FaultVar.ParSave.IdcSave = MTCVar.Dis.CurrentFdb;
                  FaultVar.ParSave.PosSave = PosStateVar.DoorPosActual;
                }
                else
                {
                }

              }
              if(faulttemp==0)
              {
                break;
              }
              else
              {
                faulttemp = faulttemp>>1;
              }
            }
          }
	}
	//�ϵ籣����ϱ�
	if( VolProtectVar.state.SaveParVol==1 )//ֻ����Ƿѹ��ֻ����һ�Σ�������Ƿѹ�����µĹ�
	{
          if( (VolProtectVar.state.SaveFst==0)&&(FaultVar.fault.FautFlag0.bit.VolLow==0) )
          {
            VolProtectVar.state.SaveFst = 1;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[144].ParaPro.E2ROMAdd;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[144].ParaPro.E2ROMBlock;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 8;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[144].ParAdd);
            E2PROM_TaskSet(&E2promTask);
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[152].ParaPro.E2ROMAdd;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[152].ParaPro.E2ROMBlock;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 6;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[152].ParAdd);
            E2PROM_TaskSet(&E2promTask);
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMAdd = SysParaAttrChar[7].ParaPro.E2ROMAdd;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].E2ROMBlock = SysParaAttrChar[7].ParaPro.E2ROMBlock;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].DataNum = 1;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].ControlCMD = WRITECMD;
            E2promTask.E2PROM_Distribute[E2promTask.TaskWriteSP].parameterlink =(Uint32 *) &(SysParaAttrChar[7].ParAdd);
            E2PROM_TaskSet(&E2promTask);
            if((FaultVar.FaultLast<FaultVar.FaultAll))
            {
              VolProtectVar.state.SaveFst = 0;
            }
          }
	}
	else
	{
          if(FaultVar.fault.FautFlag0.bit.VolLow==0)
          {			
            VolProtectVar.state.SaveFst = 0;
          }
	}
	FaultVar.FaultLast=FaultVar.FaultAll;
	if(FaultVar.FaultLast!=0)
	{
          faulttemp = FaultVar.FaultLast;
          for(faultcnt=0;faultcnt<63;faultcnt++)
          {
            if( faulttemp&0x0001!=0 )
            {
              FaultVar.FaultPre = faultcnt+1;
              break;
            }
            faulttemp = faulttemp>>1;
          }
        }
	else
	{
          FaultVar.FaultPre = 0;
	}
}

/*******************************************************************************
* ��������  : PosCorrect
* ����      : ����Ƥ����ʱ��У��λ�ã��Է�ֹ������ʧЧ������ԭ����λ��ԭ����У��
			  ����ģ������ڴ�?Ȧʱ����ҪУ���������ĵ�ǰλ�ã��Է�ֹƤ����һ��
			  ����ͣ�򻬳��ֵı����̼������
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void PosCorrect(void)
{
	int32 postemp,postemp0;

	postemp = (PosStateVar.DoorPosOriginSave - PosStateVar.DoorPosOrigin)/(int32)qep1.LineEncoder;
	if( labs(postemp)>=1 )
	{
		if( (StallCheckStru.StateCmd.StallFlag==1) )//||(labs(postemp)>(PosStateVar.DoorWidth>>1))
		{
			postemp0 = postemp*(int32)qep1.LineEncoder;
			postemp = postemp*(int32)qep1.QepCountIndex;
                        
			qep1.OutputRawTheta += postemp;
                        STM3210X_QEP_Pos_Init(qep1.OutputRawTheta,&qep1);
			qep1.OutputRawThetaOld = qep1.OutputRawTheta;
                        
                       
			PosStateVar.DoorPosOrigin += postemp0;
			PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;
			g_hPosLoop.StateCmd.PosLoop = 0;
		}
		if( PowerFstVar.PowerFst==0 )
		{
			if( (FaultVar.fault.FautFlag0.bit.QEPErrorFlag==0)&&(QeqResumeVar.CmdState.ResumeFlag==0) )
			{
				FaultVar.fault.FautFlag0.bit.StrapError = 1;
			}
		}
		qep1.CheckClr = 1;
	}
	else
	{
		if( StallCheckStru.StateCmd.StallFlag==1 )
		{
			FaultVar.fault.FautFlag0.bit.StrapError = 0;
		}
	}

	if( PosStateVar.State.bit.CDOverFlow==1 )
	{
		if( StallCheckStru.StateCmd.StallFlag==1 )
		{
			PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2);
			g_hPosLoop.StateCmd.PosLoop = 0;
		}
	}
	else if( PosStateVar.State.bit.ODOverFlow==1 )
	{
		//if( PosStateVar.DoorPosActual>(PosStateVar.DoorWidth+g_hCurveGenerate.PolyLineVar.StrapElongate) )//�ſ�ʵ��λ�ô���ѧϰ�ſ���Ƥ��������֮��
		if( StallCheckStru.StateCmd.StallFlag==1 )
		{
			if( PosStateVar.State.bit.DoorDirectionCMD==0 )
			{
				PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2) - PosStateVar.DoorWidth;
			}
			else
			{
				PosStateVar.DoorPosOrigin = (qep1.OutputRawTheta>>2) + PosStateVar.DoorWidth;
			}
		}
		
	}
	else
	{
	}
	//�Ƕ�У���������ת���AB��PWM������PWMΪ��׼
}
/*******************************************************************************
* ��������  : SpeedPIAdjustFun
* ����      : ASR��PI�ֶε���
* ����      : None
* ���      : None
* ����      : None.
*******************************************************************************/
void SpeedPIAdjustFun(void)
{
	_iq temp,tempdiv;
	temp = _IQabs(speed2.Speed);
// Initialize the PID_REG3 module for speed
	if( temp < PiAdjVar.ProPar.SwitchFre )
	{
		tempdiv = _IQdiv(temp,PiAdjVar.ProPar.SwitchFre);
		pid1_spd.Kp = PiAdjVar.ProPar.SpeedProportionGain0 + _IQmpy((PiAdjVar.ProPar.SpeedProportionGain1 - PiAdjVar.ProPar.SpeedProportionGain0),tempdiv);// _IQ(0.50);         //0.5        
		temp = PiAdjVar.ProPar.SpeedIntegralGain0 + _IQmpy( (PiAdjVar.ProPar.SpeedIntegralGain1 - PiAdjVar.ProPar.SpeedIntegralGain0),tempdiv);
		pid1_spd.Ki =  _IQdiv(_IQmpyI32(Tbase,SpeedLoopPrescaler),temp);//2.0
 
	}
	else
	{
	    pid1_spd.Kp = PiAdjVar.ProPar.SpeedProportionGain1;// _IQ(0.50);         //0.5        
		pid1_spd.Ki =  _IQdiv(_IQmpyI32(Tbase,SpeedLoopPrescaler),PiAdjVar.ProPar.SpeedIntegralGain1);//2.0
	}

}
 

/*******************************************************************************
* ��������  : ODMaxTorqueLimitUp
* ����      : ���Ź�����������޷�
* ����     : None
* ���      : None
* ����      : None.
*******************************************************************************/
void ODMaxTorqueLimitUp(void)
{
	if((CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.ReOD==1)||( CTLcmdVar.FinalCmd.bit.LOD==1))///�������������� ,��û�����ſ�ѧϰ
	{
		if((TorqueProtecVar.StateCmd.TorqueOverFlag==1) && (PosStateVar.State.bit.ODEnd == 0)) //������ ����û�������л� && ( DoorFianlVar.StateCmd.ODSwithFlag==0 )
		{
			if(OD_TorqueUPTime >= 2000)///����2.5s
			{
				//���ű�����
				MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque; //�л������ű�����
				OD_TorqueUPTime = 2600;
				OD_TorqueDWNTime = 2600;
				CTLcmdVar.FinalCmd.bit.ReOD = 0;
			}
		}
		else
		{
			if(OD_TorqueDWNTime <= 100)///����2.5s
			{
				OD_TorqueDWNTime = 100;
				OD_TorqueUPTime = 0;
			}
			else
			{
				//���ű�����
				MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque; //�л������ű�����
			}
		}
	}
	else if((CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.bit.LCD==1))///�ء�����
	{
		if(((TorqueProtecVar.StateCmd.TorqueOverFlag==1)||(StallCheckStru.StateCmd.StallFlag==1)) && (PosStateVar.State.bit.CDEnd == 0)) //������ ����û�������л� && ( DoorFianlVar.StateCmd.ODSwithFlag==0 )
		{
			if(CD_TorqueUPTime >= 2000)///����2.5s
			{
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque; //�л������ű�����

				CD_TorqueUPTime = 2600;
				CD_TorqueDWNTime = 2600;
			}
		}
		else
		{
			if(CD_TorqueDWNTime <= 100)///����2.5s
			{
				CD_TorqueDWNTime = 100;
				CD_TorqueUPTime = 0;
			}
			else
			{
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque; //�л������ű�����
			}
		}
	}
}

/*******************************************************************************
* ��������  : MenuKeySpdAdj
* ����      : �Ż������ٶ��������٣��м�Ĭ���ٶȿɽ����������
* ����     : None
* ���      : None
* ����      : None.
*******************************************************************************/
void MenuKeySpdAdj(void)
{
	///��������ѡ�� ///
	///if(((KeyAvailable&RLKEY)==RLKEY)&&((KeyAvailable&LLKEY)==LLKEY))//�������Ϲ��� �ٶȵ���
        if(1)
        {
            switch(key.val)
              {
                case SPEEDUPKEY://����ѡ��

                    if(Speed_cnt==0)///�������¼�����Ϊ��
                    {
                          Speed_llkey++;

                          if(Speed_llkey > 1)///���ν���˵�����ʾΪ��ǰֵ
                          {
                                  Speed_step++;
                                  if(Speed_step > 3)
                                          {Speed_step = 1;}
                                  else
                                          {}
                          }


                          if(Speed_llkey == 1)
                          {
                                  push_dis();
                                  DisStru.MenuIndex += 4;//ֱ�ӽ�������鿴����
                          }
                          else
                          {}


                          KeyAvailable &= (~DOWNKEY);//����Ч

                          KeyAvailable &= (~LLKEY);//����Ч
                          KeyAvailable &= (~RLKEY);//����Ч
                          KeyAvailable &= (~SPEEDUPKEY);//����Ч
                          KeyAvailable &= (~KeyAvailable);//����Ч
                          key.val = 0;   

                          Speed_cnt = 800;
                          Speed_KeyCountOver = 0;///��������
                        }

                      break;
                      case ODUPKEY://���Ÿ���
                               break;
                      default:break;
                    }
		}

	///�ٶȸ��ݵ�λ����///

	if(((OutDelay.StateCmd.bit.CDEndOut==1)||(OutDelay.StateCmd.bit.ODEndOut==1))&&(Speed_KeyCountOver==0))///���Ż���ŵ�λ���ٶ����ߵ���
	{
		switch(Speed_step)
		{
			case 1://��������
					g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel = 582;//P4.04 ����S���߲ο��ٶ�
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 25;	     //P4.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 23;	 //P4.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 50;//P4.09 ����S���߲ο����ٱ���
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 23;   //P5.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 25;   //P5.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 40;//P5.09  ����S���߲ο����ٱ���
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 45;//P5.11  �������о���
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
					UnusualDcc.ParSet.AccTime = 5;

					Para0404Calc();
			break;
			case 2://�������� ����ֵ ���ֶ�����

				if(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel == 582)//P4.04 ����S���߲ο��ٶ�   ����δ�� Ĭ����������
				{
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 21;	     //P4.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 19;	 //P4.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 55;//P4.09 ����S���߲ο����ٱ���
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 19;   //P5.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 21;   //P5.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 45;//P5.09  ����S���߲ο����ٱ���
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 45;//P5.11  �������о���
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
					UnusualDcc.ParSet.AccTime = 5;

					Para0404Calc();
				}
				else///�ֶ�����
				{}

			break;
			case 3://��������
				
					g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel = 682;//P4.04 ����S���߲ο��ٶ�
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 14;	     //P4.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 14;	 //P4.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 65;//P4.09 ����S���߲ο����ٱ���
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 14;   //P5.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 14;   //P5.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 50;//P5.09  ����S���߲ο����ٱ���
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 45;//P5.11  �������о���
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
                                        UnusualDcc.ParSet.AccTime = 3;
                                        
                                        Para0404Calc();

			break;
                        case 4://����������

                                        g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel = 682;//P4.04 ����S���߲ο��ٶ�
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 14;	     //P4.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 15;	 //P4.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 65;//P4.09 ����S���߲ο����ٱ���
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 14;   //P5.03 ����S���߲ο�����ʱ��
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 14;   //P5.06 ����S���߲ο�����ʱ��
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 55;//P5.09  ����S���߲ο����ٱ���
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 100;//P5.11  �������о���
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
					UnusualDcc.ParSet.AccTime = 3;
					
                                        Para0404Calc();
				
			break;
			default:break;
		}
		Speed_KeyCountOver = 1;///�ٶȼ������
	}

    //������ʾ///
	if(Speed_step==1)
		{Speed_dis = 226;}
	else if(Speed_step==2)
		{Speed_dis = 335;}
	else if(Speed_step==3)
		{Speed_dis = 393;}
        else if(Speed_step==4)
		{Speed_dis = 435;}
	else
		{Speed_dis = 2;}
}








