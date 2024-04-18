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

Uint16 DoorWidthAutoStudy_en = 1;///门宽自学习使能位，默认为1
Uint16 Doorwidthsave = 0;///掉电保存门宽标志位
Uint16 DoorFirstLod_Flag = 1;///初次上电为慢开门标志

Uint16 AutoWidthStudyRunCls_initOK = 0;///门宽自识别初次运行慢关门结束标志
Uint16 AutoWidthStudyRunOpen_initOK = 0;///门宽自识别初次运行慢开门结束标志
Uint16 AutoWidthStudyRun_initOver = 0;///门宽自识别初次运行结束标志
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
extern Uint16 Speed_KeyCountOver;  ///计算完成
Uint16 Speed_cnt=800;
Uint16 Speed_step = 2;
Uint16 Speed_dis = 0;

Uint16 FstNum = 0x55AA;
const Uint16 *FstParAdd = &FstNum; //参数变量地址


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
* 函数名称  : MagStudyMain
* 描述      : 磁极学习，学习成功后保存角度，如果学习不成功则自动退出
* 输入      : None
* 输出      : None
* 返回      : None
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
		StallCheckStru.CheckTime = 2000;//学习时堵转检测时间为2S
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

			//判断门宽与爬行距离关系
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
                                
                                DoorWidthAutoStudy_en = 0;///学习完成后，自学习标志清零
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
		//学习完成后，清堵转与过力矩故障
		StallCheckStru.StateCmd.ClrStall = 1;
		TorqueProtecVar.StateCmd.ClrEn = 1;
	
		g_hCurveGenerate.RealCurveCalc.CmdState.StudyMode  = DoorStudyVar.StateCmd.StudyEn;

		MTCVar.StateCmd.RunEn = DoorStudyVar.StateCmd.Run;

		/*if( DoorStudyVar.StateCmd.RunDir==0)
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 1;//开
			if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque	)
			{
				
				if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
				{
					p_uOD_TorqueSwTime++;
					//开门保持力
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
					//开门最大力
					MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
				}
			}
		}
		else
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 0;//关门
		
		}*////改进自学习门宽逻辑
                
                if( DoorStudyVar.StateCmd.RunDir==0)
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 1;//开门
		}
		else
		{
			g_hCurveGenerate.RealCurveCalc.CmdState.ODCD = 0;//关门
		
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
                  //判断是否一致
                  if(temp==E2promTask.E2PROM_Distribute[E2promTask.TaskReadSP].DataNum*2)
                  {
                    p_charEEPORMStep = EEPROM_OS_END;//成功
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
                        //EE2PROM出错
                        gE2rom.CmdState.WtireReadError = 1;
                        p_charWriteCnt = 0;
                        p_charEEPORMStep = EEPROM_OS_END;//成功
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
* 函数名称  : SpeedCal
* 描述      : 完成对磁编码器速度计算的处理，因为磁编码器信号在速度平稳情况下，也
              会有较大波动，因此要对速卸啻吻笃骄担约跎偎俣绕轿龋涣硗庖?
			  为还要偃速性，因此还要进行快速处理
* 输入      : None
* 输出      : None
* 返回      : None.
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
* 函数名称  : K400EcanCommAnaly
* 描述      : K400CANbus指令优先级处理
* 输入      : None
* 输出      : None
* 返回      : None.
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
* 函数名称  : K400EcanCommErrCkeck
* 描述      : CANBUS主机心跳接收判断
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void K400EcanCommErrCkeck(void)
{
    /*if(((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
      (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0))&& (Ecan_state.InitOver))//判断梯号及前后门	初始化过程结束
      {
          if((((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
          ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))) ///判断前后门一致性
        {  
            if ((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x50) && (Ecan_state.PluseRxEn == 1))	///识别心跳命令功能码
                {
                   if((Ecan_state.Ecan_CommSave.Byte.Mask != Ecan_state.Ecan_RxD0.Byte.By4_Mask) ||///.byte.mask
                      (Ecan_state.Ecan_CommSave.Byte.Data != Ecan_state.Ecan_RxD1.Byte.By1_Data) )///.byte.data
                   {
                          Ecan_state.PluseErrCounti++;
                          if(Ecan_state.PluseErrCounti >= 2)  
                          {
                              Ecan_state.PluseErrCounti = 2;

                              //每一个功能写任务栈   
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low; 
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xfa;
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x07; 
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 4;         //帧长
                              ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                              ECANSendStack.BufHead ++;
                              if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                              {
                                  ECANSendStack.BufHead =0;
                              }   else{}      
                              
                              if(OutDelay.DoorLocation == 0)//前门
                              {
                                  if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//开门
                                  if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//关门
                                  if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //慢关
                              }
                              else
                              {
                                  if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.RearOD;//开门
                                  if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//关门
                                  if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.RearZTZD; //慢关							 
                              }

                                 K400EcanCommAnaly();//指令解析
                                 Ecan_state.Ecan_CommSave.Byte.Mask = Ecan_state.Ecan_RxD0.Byte.By4_Mask;///mask指令保存
                                 Ecan_state.Ecan_CommSave.Byte.Data = Ecan_state.Ecan_RxD1.Byte.By1_Data;///data指令保存
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
* 函数名称  : K400EcanCommStateCkeck
* 描述      : CANBUS状态变化及门控心跳发送
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void K400EcanCommStateCkeck(void)
{
///状态变化发送
  /*if(((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum) || 
     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) && (Ecan_state.InitOver != 0))//判断梯号及前后门
     {
      if(((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
         ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))
              ///判断前后门一致性
        {
          if((Ecan_state.Ecan_Tx_Rear.Byte.By1_Data) != Ecan_state.stateLast)///状态发生变化 
          {	
              if(OutDelay.DoorLocation == 0)//前门
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x53;  
              }
              else
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0xAC;  
              } 
            ///  Ecan_state.Ecan_Tx.Byte.By4_Mask = Ecan_state.stateLast^Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
            /// Ecan_state.PluseSaveMask = Ecan_state.Ecan_Tx.Byte.By4_Mask; 
              
              
              //每一个功能写任务栈   
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0x10;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.Ecan_Tx.Byte.By4_Mask; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 5;         //帧长
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
              ECANSendStack.BufHead ++;
              if(ECANSendStack.BufHead >= ECANBUFLENMAX)
              {
                  ECANSendStack.BufHead =0;
              }   else{} 
              
              Ecan_state.PluseTxEn = 1;//使能心跳	
              Ecan_state.PluseErrTxcount = 5000;
          }
          else if((Ecan_state.PluseErrTxcount == 0) && (Ecan_state.PluseTxEn == 1))///5s
          {
             if(OutDelay.DoorLocation == 0)//前门
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x53;  
              }
              else
              {
                  Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
                  Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                  Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                  Ecan_state.Ecan_Tx.Byte.By4_Mask = 0xAC;  
              } 
            ///  Ecan_state.Ecan_Tx.Byte.By4_Mask = Ecan_state.stateLast^Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
              Ecan_state.PluseSaveMask = Ecan_state.Ecan_Tx.Byte.By4_Mask;   
           
            //每一个功能写任务栈   
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0x50;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.PluseSaveMask; 
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
              ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 5;         //帧长
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
* 函数名称  : K400EcanRradeId
* 描述      : K400读软件版本及电子标签
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void K400EcanRradeId(void)//ASCII码表示
{
  /*  int16 temp1,txi,txj;

     if(((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum) || 
     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)))//判断梯号及前后门
     {
      if(((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
         ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))
      {
          if( Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xe4 )//读软件版本及电子标签
          {
          //获取CPU唯一ID
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
          
          if(OutDelay.DoorLocation == 0)//前门
          {
              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
          }
          else
          {
              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
          }

          switch(Ecan_state.IDStep)///读电子标签步骤
          {
            case 0://K400硬件版本号  C005  0x43, 0x30, 0x30, 0x35
                   
                  //每一个功能写任务栈   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x0; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x43;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x30;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = 0x30;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = 0x35; 
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //帧长
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
                      ECANSendStack.BufHead ++;
                      if(ECANSendStack.BufHead >= ECANBUFLENMAX)
                      {
                          ECANSendStack.BufHead =0;
                      }   else{} 
                     
                     Ecan_state.Txtimecountj = 100;
                     Ecan_state.IDStep = 1;
             break;
             case 1://K400软件版本号  “A01.0” 0x41, 0x30, 0x31, 0x30
                 if(Ecan_state.Txtimecountj == 0)
                 {
                       //每一个功能写任务栈   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x08; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x31;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x30;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = 0x31;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = 0x30; 
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //帧长
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
             
             case 2://K400主控板ID值---E_ID数据长度  
                 if(Ecan_state.Txtimecountj == 0)
                 {
                      //每一个功能写任务栈   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x10; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x18;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x00;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = 0x00;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = 0x00; 
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //帧长
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
             case 3://K400主控板ID值---E_ID数据值 

               if(Ecan_state.Txtimecountj == 0)
                 {  
                        //每一个功能写任务栈   
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe5;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x11 + Ecan_state.IDsendcounti; 
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = ARM_ID[Ecan_state.IDsendcountj];
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = ARM_ID[Ecan_state.IDsendcountj + 1];
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[6] = ARM_ID[Ecan_state.IDsendcountj + 2];
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[7] = ARM_ID[Ecan_state.IDsendcountj + 3];
                      
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 8;         //帧长
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
             
             case 4://结束
                 
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
* 函数名称  : K400EcanAdvanInit
* 描述      : K400高级初始化
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/

void K400EcanAdvanInit(void)
{
  //if(Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x04)//高级初始化标识符
  {
    //if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1)//唤醒标识位
    {
       ;/* Ecan_state.Ecan_InitSave.Byte.Low = Ecan_state.Ecan_RxD0.Byte.By1_LSP;//保存梯号、前后门信息
        Ecan_state.InitOver = 1; ///初始化结束，可以进行正常运行

        Ecan_state.PluseRxEn = 0;
        Ecan_state.PluseTxEn = 0;

        Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x0;//mask位清零
        Ecan_state.Ecan_Tx_Rear.Byte.By1_Data = 0x0;//data位清零
        Ecan_state.stateLast = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;//保存当前状态

        Ecan_state.Ecan_RxD0.Byte.By3_Indix = 0x0;*/
    }
  }
}


/*******************************************************************************
* 函数名称  : K400EcanInit
* 描述      : K400初始化，握手信号
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/

void K400EcanInit(void)
{
   /*if(((OutDelay.DoorLocation == 0) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x01) == 0x01)) ||
      ((OutDelay.DoorLocation == 1) && ((Ecan_state.Ecan_RxD0.bit.FRSdoor&0x02)== 0x02)))///判断前后门一致性
      { 
        if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xf8)) 
        {
            //复位操作
            PosStateVar.State.all = 0;
            
            Ecan_state.Ecan_RxD0.all = 0;
            Ecan_state.Ecan_RxD1.all = 0;
            CTLcmdVar.FinalCmd.all = 0;
            MTCVar.StateCmd.RunEn = 0;//电机运行使用
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
          ///  K400EcanCommAnaly();//指令解析
            
            if(OutDelay.DoorLocation == 0)//前门
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

        //每一个功能写任务栈
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_Tx.Byte.By1_LSP; 
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xf9;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = 0x25; 
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = 0x00;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[5] = 0x0;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 6;         //帧长
            ECANSendStack.BufHead ++;
            ECANSendStack.ECANSendData[ECANSendStack.BufHead].id = (Uint32)OutDelay.TxIDIDNumber[OutDelay.DoorLocation];
            if(ECANSendStack.BufHead >= ECANBUFLENMAX)
            {
                ECANSendStack.BufHead =0;
            }   else{} 

            Ecan_state.Ecan_RxD0.all = 0x0;
            Ecan_state.InitOver = 0;//初始状态位
            Ecan_state.Txtimecounti = 2;
            Ecan_state.Step = 1;
        }
        else
        {
                
        }

        switch(Ecan_state.Step)///初始化步骤
        {
           case 0: ///复位
          
          break;
          case 1:	  ///握手

              if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xe8) )//&&(Ecan_state.sendSuccess==1)&& (Ecan_state.Txtimecounti == 0)) 
              {      
                    if(OutDelay.DoorLocation == 0)//前门
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
                    Ecan_state.Ecan_InitSave.Byte.Low = Ecan_state.Ecan_RxD0.Byte.By1_LSP;//保存梯号、前后门信息

                //每一个功能写任务栈
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xe9;
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                    ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 3;         //帧长
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
                  if(OutDelay.DoorLocation == 0)//前门
                   {
                      if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//开门
                      if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//关门
                      if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //慢关
                    }
                   else
                   {
                      if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//开门
                      if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//关门
                      if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //慢关
                   }
                  Ecan_state.Txtimecounti = 100;
               }  

            break;
            case 2:///邢工主机测试板 暂跳过此命令 20160229
                  if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //判断梯号或广播
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///初始化命令1
                    {
                       if(OutDelay.DoorLocation == 0)//前门
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //慢关
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //慢关
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
          case 3: ///初始化命令1
                  if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //判断梯号或广播
                  {
                        if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xea)&&(Ecan_state.sendSuccess==1) )//&& (Ecan_state.Txtimecounti == 0)) ///初始化命令2
                        {
                          //每一个功能写任务栈
                           if(OutDelay.DoorLocation == 0)//前门
                          {
                              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
                              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x01;
                              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                              Ecan_state.Ecan_Tx.Byte.By4_Mask = 0x53;  
                          }
                          else
                          {
                              Ecan_state.Ecan_Tx.bit.LifeNum = Ecan_state.Ecan_InitSave.bit.LifeNum;//梯号
                              Ecan_state.Ecan_Tx.bit.FRSdoor = 0x02;
                              Ecan_state.Ecan_Tx.bit.Fraprty = 0x01;
                              Ecan_state.Ecan_Tx.Byte.By4_Mask = 0xAC;  
                          } 

                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0x10;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[3] = Ecan_state.Ecan_Tx.Byte.By4_Mask;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[4] = Ecan_state.Ecan_Tx_Rear.Byte.By1_Data;
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                          ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 5;         //帧长
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
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //判断梯号或广播
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///初始化命令1
                    {
                       if(OutDelay.DoorLocation == 0)//前门
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //慢关
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //慢关
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
                  
          break;
          case 4:///初始化命令2
                  //if(Ecan_state.Txtimecounti == 0) ///跏蓟?
                  if(Ecan_state.sendSuccess==1)
                  {
                   //每一个功能写任务栈
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[0] = Ecan_state.Ecan_InitSave.Byte.Low;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[1] = 0x0;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].Buf[2] = 0xeb;
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataVaildFlag =1;//数据有效
                      ECANSendStack.ECANSendData[ECANSendStack.BufHead].BufDataLen = 3;         //帧长
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
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //判断梯号或广播
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///初始化命令1
                    {
                       if(OutDelay.DoorLocation == 0)//前门
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //慢关
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //慢关
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
          break;
          case 5:
                  if((Ecan_state.Ecan_RxD0.bit.LifeNum == Ecan_state.Ecan_InitSave.bit.LifeNum)||
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //判断梯号或广播
                  {
                      if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0xec)&&(Ecan_state.sendSuccess==1))// && (Ecan_state.Txtimecounti == 0)) ///初始化命令2
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
                     (Ecan_state.Ecan_RxD0.bit.LifeNum == 0x0)) //判断梯号或广播
                 {
                    if((Ecan_state.Ecan_RxD0.Byte.By3_Indix == 0x20))//&&(Ecan_state.sendSuccess==1))//&& (Ecan_state.Txtimecounti == 0)) ///初始化命令1
                    {
                       if(OutDelay.DoorLocation == 0)//前门
                       {
                          if(Ecan_state.Ecan_RxD0.bit.FrontODEN == 1) Ecan_state.Ecan_InitSave.bit.OD = Ecan_state.Ecan_RxD1.bit.FrontOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.FrontCDEN == 1) Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.FrontCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.FrontZTZEN == 1) Ecan_state.Ecan_InitSave.bit.LCD = Ecan_state.Ecan_RxD1.bit.FrontZTZ; //慢关
                        }
                       else
                       {
                          if(Ecan_state.Ecan_RxD0.bit.RearODEN == 1)  Ecan_state.Ecan_InitSave.bit.OD  = Ecan_state.Ecan_RxD1.bit.RearOD;//开门
                          if(Ecan_state.Ecan_RxD0.bit.RearCDEN == 1)  Ecan_state.Ecan_InitSave.bit.CD = Ecan_state.Ecan_RxD1.bit.RearCD;	//关门
                          if(Ecan_state.Ecan_RxD0.bit.RearZTZDEN == 1) Ecan_state.Ecan_InitSave.bit.LCD  = Ecan_state.Ecan_RxD1.bit.RearZTZD; //慢关
                       }
                      
                        
                    }
                    else
                    {
                    }
                  }  
          break;
          case 6:
                //  if(Ecan_state.Txtimecounti == 0) ///初始化命令结束
                  //if(Ecan_state.sendSuccess==1)
                  {
                      Ecan_state.Txtimecounti = 1;
                      Ecan_state.InitOver = 1; ///初始化结束，可以进行正常运行

                      
                      K400EcanCommAnaly();//指令解析
                      Ecan_state.Ecan_CommSave.Byte.Mask = Ecan_state.Ecan_RxD0.Byte.By4_Mask;///mask指令保存
                      Ecan_state.Ecan_CommSave.Byte.Data = Ecan_state.Ecan_RxD1.Byte.By1_Data;///data指令保存
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
* 函数名称  : CommandAnaly
* 描述      : 完成对各个单元命令的解析，并得到最终的操作命令
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void CommandAnaly(void)
{
        Uint16 tempcnt,temp = 0;
	static Uint16 switchfalg=0;//buzzerSw = 0;
        
        CanCommAdd = 0;//多个信号判断

	CTLcmdVar.p = &CTLcmdVar.TerminalCmd;
	switch(CTLcmdVar.CMDSel)
	{
		case 0://端子命令解析
                        //TerminalVar.ValueFinal.all = TerminalVar.Value.all;
                        //功能不可重复选择
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
                        
                        if( TerminalVar.Value.bit.bit3==1 )///关门到位端子信号输入
			{
				CTLcmdVar.TerminalCmd.bit.TerminalCD = 1;
			}
			else
			{
				CTLcmdVar.TerminalCmd.bit.TerminalCD = 0;
			}
                        
                        
                        //检修判断
			/*if( TerminalVar.ValueFinal.bit.bit3==1 )
			{
				if(g_lOH_Mode==1)//端子保留原来状态
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
							if( PowerFstVar.PowerFst==0 )   //初次上电给开门撤销(无其他命令)后开到位
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
						if( PowerFstVar.PowerFst==0 )   //初次上电给开门撤销(无其他命令)后开到位
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
                  ///Step canbus协议
			if((Ecan_state.Ecan_RxD0.Byte.byte1.CMDID == 160) && (OutDelay.DoorLocation == 0)) //标识符、前后门判断
                        {
                            CanCommAdd = Ecan_state.Ecan_RxD0.Byte.byte2.bit.CD + Ecan_state.Ecan_RxD0.Byte.byte2.bit.LCD +
                                         Ecan_state.Ecan_RxD0.Byte.byte2.bit.OD;

                                if(CanCommAdd <= 1)
                                {
                                        CTLcmdVar.CanCmd.bit.OD = Ecan_state.Ecan_RxD0.Byte.byte2.bit.OD;  ///开门命令
                                        CTLcmdVar.CanCmd.bit.CD = Ecan_state.Ecan_RxD0.Byte.byte2.bit.CD;  ///关门命令
                                        CTLcmdVar.CanCmd.bit.LCD = Ecan_state.Ecan_RxD0.Byte.byte2.bit.LCD;///慢关门命令
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
		case 6://标准三线
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
				case 0://检修禁止
				//	CTLcmdVar.StandThreeProtocolCmd.bit.OH = 1;
					CTLcmdVar.FinalCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.OH = 0;
				break;
				case 1://检修允许，保持原状态。如果上位机有命令需要根据上位机进行动作
					//CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.OH = 1;
			
				break;
				case 2://关门，不响应反开动作
				case 3:
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.CD = 1;
					//CTLcmdVar.StandThreeProtocolCmd.bit.ReOD = 0;//反开禁止
					CTLcmdVar.FinalCmd.bit.ReOD = 0;//反开禁止
				break;
				case 4://强迫关门(慢关门)不响应反开动作
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.LCD = 1;
					//CTLcmdVar.StandThreeProtocolCmd.bit.ReOD = 0;//反开禁止
					CTLcmdVar.FinalCmd.bit.ReOD = 0;//反开禁止
				break;
				case 5://关门，反开动作允许
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
				case 6://开门动作
				case 7:
					CTLcmdVar.StandThreeProtocolCmd.all = 0;
					CTLcmdVar.StandThreeProtocolCmd.bit.OD = 1;
				break;
				default:break;
			}
		break;
		case 7://扩展三
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
				case 0://检修禁止
					CTLcmdVar.FinalCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					//CTLcmdVar.ExThreeProtocolCmd.bit.OH = 1;
					CTLcmdVar.ExThreeProtocolCmd.bit.OH = 0;
				break;
				case 1://检修允许
					//CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.OH = 1;
				break;
				case 2://关门过执行中，只响应DOB
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
						//CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 0;//反开禁止
						
					}
				break;
				case 3://关门命令
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.CD = 1;
					CTLcmdVar.FinalCmd.bit.ReOD = 0;
				//	CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 0;//反开禁止
				break;
				case 4://强迫关门
					OutDelay.StateCmd.bit.CDEndOutEn = 0;
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.LCD = 1;
					CTLcmdVar.ExThreeProtocolCmd.bit.ReOD = 0;//反开禁止
					CTLcmdVar.FinalCmd.bit.ReOD = 0;//反开禁止
				break;
				case 5://关门，如果有反开操作都允许
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
				case 6://关门，DOB无效
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
				case 7://开门操作
					CTLcmdVar.ExThreeProtocolCmd.all = 0;
					CTLcmdVar.ExThreeProtocolCmd.bit.OD = 1;
				break;
				default:break;
			}
		break;
		default:break;
	}
	//将不进行命令的解析项清0
	for(tempcnt=0;tempcnt<CTLcmdVar.Pmax;tempcnt++)
	{	
		//关门受阻命令处理
		if( OCDOverVar.CDSuffType==1 )
		{
			(*(CTLcmdVar.p)).bit.ReOD = CTLcmdVar.FinalCmd.bit.ReOD;
		}
		else
		{
			if( CTLcmdVar.CMDSel==6 )//三线模式自动
			{
				(*(CTLcmdVar.p)).bit.ReOD = CTLcmdVar.FinalCmd.bit.ReOD;
			}
			else
			{
				(*(CTLcmdVar.p)).bit.ReOD = 0;
			}
		}
                
                (*(CTLcmdVar.p)).bit.ReCD = CTLcmdVar.FinalCmd.bit.ReCD;
                
		//通道实别
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
						if( CTLcmdVar.state.ODHoldFlag==0 )//如果不是保持状态
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
						if( CTLcmdVar.state.CDHoldFlag==0 )//如果不是保持状态
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
				CTLcmdVar.FinalCmd.all = (*(CTLcmdVar.p)).all;//将当前有效命令传送给最终命令变量
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
* 函数名称  : ActionToMTCAnaly
* 描述      : 将动作的数据传送给电机，得到电机运行的参数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void ActionToMTCAnaly(void)
{
	/*if( Door_Ation.Action<(Door_Ation.Pmax) )
	{
		Door_Ation.Point = &(Door_Ation.AciontData.OD_Start) + Door_Ation.Action;
	}
	else
	{
		return;//指针出错时，保持原来动作
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
* 函数名称  : ParaCalcNULL
* 描述      : 更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void ParaCalcNULL(void)
{
}
/*******************************************************************************
* 函数名称  : ParaInSelCalc
* 描述      : 更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void ParaInSelCalc(void)
{
}
/*******************************************************************************
* 函数名称  : Para0100Calc
* 描述      : P01.00更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0100Calc(void)
{

//通讯模式
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
* 函数名称  : Para0102Calc
* 描述      : P01.02更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0102Calc(void)
{
	MotorStru.MotorOutFreMax = _IQ15mpy(MotorStru.MotorOutFreMax_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	//最大输出频率
	MotorFreMaxQ = _IQdiv(MotorStru.MotorOutFreMax,MotorStru.MotorRatedFreq);
}
/*******************************************************************************
* 函数名称  : Para0103Calc
* 描述      : P01.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0103Calc(void)
{
	g_hCurveGenerate.PolyLineVar.PowerFstLowSpd =	_IQ15mpy(g_hCurveGenerate.PolyLineVar.PowerFstLowSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	g_hCurveGenerate.PolyLineVar.PowerFstLowSpdQ24 = _IQ28div(g_hCurveGenerate.PolyLineVar.PowerFstLowSpd,MotorStru.MotorRatedFreq);
}
/*******************************************************************************
* 函数名称  : Para0104Calc
* 描述      : P01.04更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0104Calc(void)
{
	//电机及门机逻辑方向
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
* 函数名称  : Para0203Calc
* 描述      : P02.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0203Calc(void)
{
	//参数电机电流标定
	g_lRefCurrentQ = _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase);
}	


/*******************************************************************************
* 函数名称  : Para0204Calc
* 描述      : P02.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
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
* 函数名称  : Para0206Calc
* 描述      : P02.06更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0206Calc(void)
{
	//计算转速与线速度的系数关系
	g_hUnitTransform.SpeedHztoLinearVelCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/(MotorStru.Motor_Poles/2)/1000);
	g_hUnitTransform.LinearVeltoSpeedHzCoeff = _IQ15div(_IQ15(1.0),g_hUnitTransform.SpeedHztoLinearVelCoeff);
	
}
/*******************************************************************************
* 函数名称  : Para0212Calc
* 描述      : P02.12更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0212Calc(void)
{
//弱磁控制	
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
* 函数名称  : Para0301Calc
* 描述      : P03.01更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0301Calc(void)
{
	DoorStudyVar.SpeedStudey = _IQ15mpy(DoorStudyVar.SpeedStudeyMil,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
}

/*******************************************************************************
* 函数名称  : Para0303Calc
* 描述      : P03.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0303Calc(void)
{
//门宽计算
	DoorStudyVar.MilToPosCoffe	=	PosStateVar.MilToPosCoeff;
	DoorStudyVar.PosToMilCoffe	=	PosStateVar.PosToMilCoeff;
	DoorStudyVar.Width	=	_IQ15mpyI32int(DoorStudyVar.DecPos.DoorMil,DoorStudyVar.MilToPosCoffe);
	
}
/*******************************************************************************
* 函数名称  : Para0307Calc
* 描述      : P03.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0307Calc(void)
{
//计算转速与线速度的系数关系
	g_hUnitTransform.SpeedHztoLinearVelCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/(MotorStru.Motor_Poles/2)/1000);
	g_hUnitTransform.LinearVeltoSpeedHzCoeff = _IQ15div(_IQ15(1.0),g_hUnitTransform.SpeedHztoLinearVelCoeff);
	
	//门宽度单位转化系数计算
	//PosStateVar.PosUnitCoeff = _IQ15(WHEEL_DIA*3.1415926/qep1.LineEncoder);
	PosStateVar.PosToMilCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/qep1.LineEncoder);
	PosStateVar.MilToPosCoeff = _IQ15div(_IQ15(1.0),PosStateVar.PosToMilCoeff);
}
/*******************************************************************************
* 函数名称  : Para0400Calc
* 描述      : P04.00更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0400Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODStartDis = (int32)(g_hCurveGenerate.PolyLineVar.ODStartDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0402Calc
* 描述      : P04.02更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0402Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0403Calc
* 描述      : P04.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0403Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.ODCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0404Calc
* 描述      : P04.04更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0404Calc(void)
{
	g_hCurveGenerate.PolyLineVar.RefHighSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0406Calc
* 描述      : P04.06更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0406Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.RefHighSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}


/*******************************************************************************
* 函数名称  : Para0407Calc
* 描述      : P04.07更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0407Calc(void)
{
	PosStateVar.ParSet.ODFullPosEn = (int32)(PosStateVar.ParSet.ODFullPosEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* 函数名称  : Para0408Calc
* 描述      : P04.08更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0408Calc(void)
{
	PosStateVar.ParSet.ODFullPosDisEn = (int32)(PosStateVar.ParSet.ODFullPosDisEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* 函数名称  : Para0409Calc
* 描述      : P04.069更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0409Calc(void)
{
	g_hCurveGenerate.OD_ActCurve.HighSpd = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.ODActHighSpd_per,100),g_hCurveGenerate.OD_RefCurve.HighSpd);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0410Calc
* 描述      : P04.10更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0410Calc(void)
{
	g_hCurveGenerate.PolyLineVar.FullDis = (int32)(g_hCurveGenerate.PolyLineVar.FullDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* 函数名称  : Para0411Calc
* 描述      : P04.11更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0411Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.ODCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* 函数名称  : Para0412Calc
* 描述      : P04.12更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0412Calc(void)
{
	PosStateVar.ParSet.ODEndPos = (int32)(PosStateVar.ParSet.ODEndPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* 函数名称  : Para0413Calc
* 描述      : P04.13更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0413Calc(void)
{
    g_hCurveGenerate.PolyLineVar.ODRetiringCamDis = (int32)(g_hCurveGenerate.PolyLineVar.ODRetiringCamDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}

/*******************************************************************************
* 函数名称  : Para0414Calc
* 描述      : P04.14更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0414Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.ODRetiringCamSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
        CurveParaCalc();
}
/*******************************************************************************
* 函数名称  : Para0415Calc
* 描述      : P04.14更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0415Calc(void)
{
	g_hCurveGenerate.PolyLineVar.ODRetiringCamOverDis = (int32)(g_hCurveGenerate.PolyLineVar.ODRetiringCamOverDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}
/*******************************************************************************
* 函数名称  : Para0500Calc
* 描述      : P05.00更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0500Calc(void)
{
	g_hCurveGenerate.PolyLineVar.StrapElongate = (int32)(g_hCurveGenerate.PolyLineVar.StrapElongateMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* 函数名称  : Para0502Calc
* 描述      : P05.02更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0502Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
	//位置环锁定最大限制	
	pid1_pos.OutMax = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd)*-1;	
	pid1_pos.OutMin = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd);
}
/*******************************************************************************
* 函数名称  : Para0503Calc
* 描述      : P05.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0503Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0506Calc
* 描述      : P05.06更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0506Calc(void)
{
	//g_hCurveGenerate.PolyLineVar.CDCreepSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDCreepSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para05107Calc
* 描述      : P05.07更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0507Calc(void)
{
	PosStateVar.ParSet.CDFullPosEn = (int32)(PosStateVar.ParSet.CDFullPosEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* 函数名称  : Para05108Calc
* 描述      : P05.08更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0508Calc(void)
{
	PosStateVar.ParSet.CDFullPosDisEn = (int32)(PosStateVar.ParSet.CDFullPosDisEnMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* 函数名称  : Para0509Calc
* 描述      : P05.09更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0509Calc(void)
{
	g_hCurveGenerate.CD_HighSpdSave = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.CDActHighSpd_per,100),g_hCurveGenerate.CD_RefCurve.HighSpd);
	g_hCurveGenerate.CD_ActCurve.HighSpd = g_hCurveGenerate.CD_HighSpdSave ;
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0510Calc
* 描述      : P05.10更新参数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0510Calc(void)
{
	g_hCurveGenerate.NomalrLCD_HighSpdSave = _IQ28mpy(_IQ28div(g_hCurveGenerate.PolyLineVar.NomalLCDActHighSpd_per,100),g_hCurveGenerate.CD_RefCurve.HighSpd);
}

/*******************************************************************************
* 函数名称  : Para0512Calc
* 描述      : P05.12更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0512Calc(void)
{
	PosStateVar.ParSet.CDEndPos = (int32)(PosStateVar.ParSet.CDEndPosMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}
/*******************************************************************************
* 函数名称  : Para0511Calc
* 描述      : P05.11更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0511Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
}

/*******************************************************************************
* 函数名称  : Para0513Calc
* 描述      : P05.13更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0513Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDRetiringCamDis = (int32)(g_hCurveGenerate.PolyLineVar.CDRetiringCamDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}

/*******************************************************************************
* 函数名称  : Para0514Calc
* 描述      : P05.14更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0514Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd = _IQ15mpy(g_hCurveGenerate.PolyLineVar.CDRetiringCamSpd_LinearVel,g_hUnitTransform.LinearVeltoSpeedHzCoeff);
	CurveParaCalc();
}
/*******************************************************************************
* 函数名称  : Para0515Calc
* 描述      : P05.15更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0515Calc(void)
{
	g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDis = (int32)(g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
	
}


/*******************************************************************************
* 函数名称  : Para0601Calc
* 描述      : P06.01更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0601Calc(void)
{
	PosStateVar.TorqueInter.OD_HlTorque		= _IQmpy( _IQdiv( PosStateVar.ParSet.OD_HlTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0602Calc
* 描述      : P06.02更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0602Calc(void)
{
	PosStateVar.TorqueInter.OD_MaxTorque		=_IQmpy( _IQdiv( PosStateVar.ParSet.OD_MaxTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0603Calc
* 描述      : P06.03更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0603Calc(void)
{
	PosStateVar.TorqueInter.OD_HlLastTorque	= _IQmpy( _IQdiv( PosStateVar.ParSet.OD_HlLastTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0606Calc
* 描述      : P06.06更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0606Calc(void)
{
	PosStateVar.TorqueInter.CD_HlTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_HlTorquePer,1000 ),g_lRefCurrentQ);
}
/*******************************************************************************
* 函数名称  : Para0607Calc
* 描述      : P06.07更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0607Calc(void)
{
	PosStateVar.TorqueInter.CD_MaxTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_MaxTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0608Calc
* 描述      : P06.08更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0608Calc(void)
{
	PosStateVar.TorqueInter.stallCDTorque		=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.stallCDTorquePer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0609Calc
* 描述      : P06.09更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0609Calc(void)
{
	PosStateVar.TorqueInter.CD_HlLastTorque 	=  -1*_IQmpy( _IQdiv( PosStateVar.ParSet.CD_HlLastTorquePer,1000 ),g_lRefCurrentQ);
}
/*******************************************************************************
* 函数名称  : Para0613Calc
* 描述      : P06.13更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0613Calc(void)
{
	DoorFianlVar.ODSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.ODswithPer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0614Calc
* 描述      : P06.14更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0614Calc(void)
{
	DoorFianlVar.CDSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.CDswithPer,1000 ),g_lRefCurrentQ);	
}
/*******************************************************************************
* 函数名称  : Para0712Calc
* 描述      : P07.12更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0712Calc(void)
{
	pid1_pos.Kp = _IQdiv(PosGain,1000);
}
/*******************************************************************************
* 函数名称  : Para0714Calc
* 描述      : P07.14更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0714Calc(void)
{
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para0806Calc
* 描述      : P08.06更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0806Calc(void)
{
	PiAdjVar.ProPar.SpeedProportionGain0 = _IQdiv(PiAdjVar.ParSet.SpeedProportionGain0,1000);			
}
/*******************************************************************************
* 函数名称  : Para0807Calc
* 描述      : P08.07更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0807Calc(void)
{
	PiAdjVar.ProPar.SpeedIntegralGain0 = _IQdiv(PiAdjVar.ParSet.SpeedIntegralGain0,1000);		
}
/*******************************************************************************
* 函数名称  : Para0808Calc
* 描述      : P08.08更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0808Calc(void)
{
	PiAdjVar.ProPar.SpeedProportionGain1 = _IQdiv(PiAdjVar.ParSet.SpeedProportionGain1,1000);	
}
/*******************************************************************************
* 函数名称  : Para0809Calc
* 描述      : P08.09更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0809Calc(void)
{
	PiAdjVar.ProPar.SpeedIntegralGain1 = _IQdiv(PiAdjVar.ParSet.SpeedIntegralGain1,1000);	
}
/*******************************************************************************
* 函数名称  : Para0811Calc
* 描述      : P08.11更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0811Calc(void)
{
	PiAdjVar.ProPar.CurrentProportionGain0 = _IQdiv(PiAdjVar.ParSet.CurrentProportionGain0,1000);
	pid1_id.Kp = PiAdjVar.ProPar.CurrentProportionGain0;
	pid1_iq.Kp = PiAdjVar.ProPar.CurrentProportionGain0;
}
/*******************************************************************************
* 函数名称  : Para0812Calc
* 描述      : P08.12更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0812Calc(void)
{
	PiAdjVar.ProPar.CurrentIntegralGain0 = _IQdiv(PiAdjVar.ParSet.CurrentIntegralGain0,1000);
	pid1_id.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);	
	pid1_iq.Ki = _IQdiv(Tbase,PiAdjVar.ProPar.CurrentIntegralGain0);		

}
/*******************************************************************************
* 函数名称  : Para0814Calc
* 描述      : P08.14更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0814Calc(void)
{
	pid1_iq.Kc = _IQdiv(KcTest,100);	
	pid1_id.Kc = pid1_iq.Kc;	
}
/*******************************************************************************
* 函数名称  : Para0815Calc
* 描述      : P08.15更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0815Calc(void)
{
	//pid1_spd.Kd = _IQdiv(KdTestSpd,100);
	pid1_spd.Kc = _IQdiv(KcTestSpd,100);
}
/*******************************************************************************
* 函数名称  : Para0908Calc
* 描述      : P09.08更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0908Calc(void)
{
	CurveParaCalc();
	CurveInitFun(&g_hCurveGenerate);
}
/*******************************************************************************
* 函数名称  : Para09011Calc
* 描述      : P09.11更新参数空处理函数
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void Para0911Calc(void)
{
	//开关门命令保持时间
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
* 函数名称  : Para09012Calc
* 描述      : P09.11更新参数空处理函数
* 淙?     : None
* ?     : None
* 返回      : None.
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
* 函数名称  : ParSettoCTLAnaly
* 描述      : 将控制器设置参数转换为同部控制变量
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void ParSettoCTLAnaly(void)
{
	//_iq basetemp;
	//计算转速与线速度的系数关系
	g_hUnitTransform.SpeedHztoLinearVelCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/(MotorStru.Motor_Poles/2)/1000);
	g_hUnitTransform.LinearVeltoSpeedHzCoeff = _IQ15div(_IQ15(1.0),g_hUnitTransform.SpeedHztoLinearVelCoeff);
	//Para0206Calc();
	//门宽度单位转化系数计算
	//PosStateVar.PosUnitCoeff = _IQ15(WHEEL_DIA*3.1415926/qep1.LineEncoder);
	PosStateVar.PosToMilCoeff = _IQ15(g_hUnitTransform.Whell_dia*3.1415926/qep1.LineEncoder);
	PosStateVar.MilToPosCoeff = _IQ15div(_IQ15(1.0),PosStateVar.PosToMilCoeff);
	//Para0307Calc();
	//basetemp = _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase);
	g_lRefCurrentQ = _IQdiv(MotorStru.MotoRatedCurrent,CtlCurrentBase);
	//Para0203Calc();
	//力矩切换
	//Para0613Calc();
	//Para0614Calc();
	DoorFianlVar.CDSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.CDswithPer,1000 ),g_lRefCurrentQ);
	DoorFianlVar.ODSwithTorque = 	_IQmpy( _IQdiv( DoorFianlVar.ParSet.ODswithPer,1000 ),g_lRefCurrentQ);
	//力矩计算
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
	//门宽计算
	//DoorStudyVar.Width = DoorStudyVar.DecPos.DecH*10000 + DoorStudyVar.DecPos.DecL;
	DoorStudyVar.MilToPosCoffe	=	PosStateVar.MilToPosCoeff;
	DoorStudyVar.PosToMilCoffe	=	PosStateVar.PosToMilCoeff;
	DoorStudyVar.Width	=	_IQ15mpyI32int(DoorStudyVar.DecPos.DoorMil,DoorStudyVar.MilToPosCoffe);
	//Para0303Calc();
	//单位转换

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
	//开关门命令保持时间
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
	//通讯模式
	if( CTLcmdVar.CMDSel==2 )
	{
		Sci_Strut.RS485EN = 1;
	}
	else
	{
		Sci_Strut.RS485EN = 0;
	}
	//PI调整参数
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

//弱磁控制	
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
	//最大输出频率
	MotorFreMaxQ = _IQdiv(MotorStru.MotorOutFreMax,MotorStru.MotorRatedFreq);
	//Para0102Calc();
	//电机及门机逻辑方向
	if( MotorStru.MotorDirSet==1 )
	{
		MotorStru.DoorDirect = -1;
	}
	else
	{
		MotorStru.DoorDirect = 1;
	}


//通讯模式
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
	//位置环锁定最大限制
	pid1_pos.OutMax = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd)*-1;
	pid1_pos.OutMin = _IQ28toIQ(g_hCurveGenerate.CD_RefCurve.OriginSpd);
}
/*******************************************************************************
* 函数名称  : ParDefault
* 描述      : 将控制器默认设置参数转换为内部控制变量
* 输入      : None
* 输出      : None
* 返回      : None.
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
* 函数名称  : ProInitFun
* 描述      : 程序上电后的初始化程序，主要完成E2PROM读写，编码器磁极校正。当
*             参数写入需要程序复位时，初始化程序
* 输入      : None
* 输出      : None
* 返回      : None.
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
		if(MotorStru.MotorType==0)//同步电机时要等待编码盘的PWM信号没有问题时才可以初始化
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
			ParDefault(0,160);//调用默认参数
			ParSettoCTLAnaly();//参数更新
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
* 函数名称  : ReadParAll
* 描述      : 读取所有参数
* 输入      : None
* 输出      : None
* 返回      : None.
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
* 函数名称  : WriteParAll
* 描述      : 写入所有参数的默认值
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void WriteParAll(Uint16 num)
{
	Uint16 temp,partemp;

	ParDefault(0,176);//调用默认参数
	ParSettoCTLAnaly();//参数更新
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
* 函数名称  : UserWriteParAll
* 描述      : 用户写入参数的默认值，不包括磁极角度与门宽数据。
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void UserWriteParAll(Uint16 num)
{
	Uint16 temp,partemp;

	ParDefault(1,4);//不包括P01.00 P01.04  20200317
	///ParDefault(5,32);//P03.00
        ParDefault(5,6);//不包括P01.06
	ParDefault(7,8);//不包括P01.08
	ParDefault(9,32);//P01 P02
	ParDefault(32,35);
	ParDefault(36,47);//不包括P03.03 P03.15
	ParDefault(48,160);//调用默认参数
	ParSettoCTLAnaly();//参数更新
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
* 函数名称  : CMDToActionAnaly
* 描述      : 命令到动作的解析
* 淙?     : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void CMDToActionAnaly(void)
{
    if( PowerFstVar.PowerFst==1 )
    {
        StallCheckStru.CheckTime = 1000;//找位置时，堵转检测为1S
    }
    else
    {
        if( (PosStateVar.State.bit.CDEnd==1)||(PosStateVar.State.bit.ODEnd==1) )
        {
            StallCheckStru.CheckTime = 250;//到位区时，堵转检测为250ms
        }
        else
        {
            StallCheckStru.CheckTime = 500;//非到位区时，堵转检测为500ms，减少负载变动带来的不可靠
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
    //命令处理，使最终命令只有一个使能
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
        if( CTLcmdVar.FinalCmd.bit.ReOD==1 )//反开
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.ReOD = 1;
        }
        else if( CTLcmdVar.FinalCmd.bit.OD==1 )//开
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.OD = 1;
        }
       /// else if( CTLcmdVar.FinalCmd.bit.LCD==1 )//慢关
        else if(( CTLcmdVar.FinalCmd.bit.LCD==1 )&&(Camstateaction.StateCmd.CamStateActionRun == 0))//慢关	
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.LCD = 1;
        }
        else if((( CTLcmdVar.FinalCmd.bit.CD==1 )||(CTLcmdVar.FinalCmd.bit.ReCD==1))&&(Camstateaction.StateCmd.CamStateActionRun == 0))//关门
        ///else if( CTLcmdVar.FinalCmd.bit.CD==1 )//关门
        {
            CTLcmdVar.FinalCmd.all = 0;
            CTLcmdVar.FinalCmd.bit.CD = 1;
        }
        else//无命令
        {
            CTLcmdVar.FinalCmd.all = 0;
        }
    }

    //故障时的处理，有此故障时，需停机
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
        MTCVar.StateCmd.RunEn = 0;//电机运行使
        return;
    }
    if(FaultVar.fault.FautFlag0.bit.VolLow)
    {
        //CTLcmdVar.FinalCmd.all = 0;
        //MTCVar.StateCmd.RunEn = 0;//电机运行使
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
    //判断开关门命令是否切换
    if( (CTLcmdVar.FinalCmdSave.bit.OD==1)||(CTLcmdVar.FinalCmdSave.bit.LOD==1)||(CTLcmdVar.FinalCmdSave.bit.ReOD==1) )
    {
        if( (CTLcmdVar.FinalCmd.bit.LCD==1)||(CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.all==0) )
        {
            CTLcmdVar.state.ODtoCDFlag = 1;
            g_hPosLoop.StateCmd.PosLoop = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.PosLock = 0;
            g_hCurveGenerate.RealCurveCalc.CmdState.CmdSwt = 1;
            //需要清零堵转有关，否则动作逻辑会出现问题
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
            //需要清零堵转有关，否则动作逻辑会出现问题
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
            //需要清零堵转有关，否则动作逻辑会出现问题
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
            TorqueProtecVar.StateCmd.TorqueOverFlag = 0;
            StallCheckStru.StateCmd.StallFlag = 0;
        }
    }
    CTLcmdVar.FinalCmdSave.all = CTLcmdVar.FinalCmd.all;
	

    //与门位置区间，电机状态关联形成电机动作
    if( CTLcmdVar.FinalCmd.bit.CD==1 )
    {
        g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 0;
        OCDOverVar.state.CDCntEn = 1;//关门超时检测允许
        OCDOverVar.state.ODCntEn = 0;//开门超时检测不允许
        OCDOverVar.state.ODOverFlag = 0;//开门超时标志清零
        //开门最大力
        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;

        if( PosStateVar.State.bit.CDEnd==1 )
        {
            OCDOverVar.state.CDCntEn = 0;
            OCDOverVar.state.ODCntEn = 0;
            if(g_hPosLoop.FullLockEn==2)//直接定位
            {
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<((int16)g_hCurveGenerate.PolyLineVar.StrapElongate*2)) )
                {
                    g_hPosLoop.StateCmd.PosLoop = 1;//锁定
                    g_hPosLoop.StateCmd.Step = 1;
                    g_hPosLoop.StateCmd.Full = 0;
                }
                g_hPosLoop.PosRef = g_hCurveGenerate.PolyLineVar.StrapElongate;
            }
            g_hCurveGenerate.RealCurveCalc.CmdState.StallRemember = 0;

            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.CDSwithTorque )
            {
                if(DoorFianlVar.StateCmd.CDHDToHFst==0)//如果未切换则使用保持力
                {
                    //关门保持力
                    MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                }
                DoorFianlVar.StateCmd.CDHDToHFst = 1;//切换时间计时标志
            }
            else
            {
                if(DoorFianlVar.StateCmd.CDHDToHFst==0)
                {
                    //关门最大力
                    MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
                }
            }
            if( StallCheckStru.StateCmd.StallFlag==1 )
            {
                //关门门宽判断与校正
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
                                Camstateaction.StateCmd.QepcountInitSamOk = 1;///允许进行门刀逻辑处理
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
                    //关门最终保持力
                    MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque;
                }
                else
                {
                    if(DoorFianlVar.StateCmd.CDHToHLFst==0)
                    {
                        //关门保持力
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
                            g_hPosLoop.StateCmd.PosLoop = 1;//锁定
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
                                        g_hPosLoop.StateCmd.PosLoop = 1;//锁定
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
            
            
            
            /*///怀疑与主机0x46有关 暂屏蔽掉
            if((OutDelay.StateCmd.bit.CDEndOut == 1) && ( PosStateVar.DoorPosActual >= g_hCurveGenerate.PolyLineVar.CDRetiringCamOverDis))///K400防扒门刀关门到位处理  >=15mm
            {
                //关门保持力
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
            //非关门到位区，关门力为关门最大，并且清力矩保持切换标志
            DoorFianlVar.StateCmd.CDHToHLFst = 0;
            DoorFianlVar.StateCmd.CDHDToHFst = 0;
            MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
            //堵转记忆功能检测
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
            if(( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1) ) &&(PosStateVar.State.bit.ODEnd==0) )//堵转或过力矩打开反开命令  20200324 开门到位区内屏蔽此状态
            {
                
                if( OCDOverVar.CDSuffType==1 )//是否允许反开
                {
                    CTLcmdVar.FinalCmd.bit.ReOD = 1;
                }
                else
                {
                    CTLcmdVar.FinalCmd.bit.ReOD = 0;
                    //如果堵住，这曲线结束时，自动巡曲线
                    if( (g_hCurveGenerate.RealCurveCalc.CmdState.CurveOver==1) )//(StallCheckStru.StateCmd.StallFlag==1)&&
                    {
                    }
                }
                if( g_hPosLoop.StallLockEn==1 )//关门堵转是否锁定
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
                g_hPosLoop.StateCmd.PosLoop = 0;//不锁定
            }

        }

    }//end of CD
    else if( (CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.ReOD==1) )
    {
        g_hPosLoop.StateCmd.StallPosLoop = 0;
        g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 0;
        OCDOverVar.state.CDCntEn = 0;//关门超时检测不允许
        OCDOverVar.state.ODCntEn = 1;//开门超时检测允许
        OCDOverVar.state.CDOverFlag = 0;//开门超时标志清0
        //制动最大力
        MTCVar.CurrentMin = PosStateVar.TorqueInter.OD_MaxTorque*-1;
        if( PosStateVar.State.bit.ODEnd==1 )
        {
            OCDOverVar.state.CDCntEn = 0;
            OCDOverVar.state.ODCntEn = 0;
            if(g_hPosLoop.FullLockEn==2)//直接定位
            {
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0>( (int16)DoorStudyVar.Width-(int16)g_hCurveGenerate.PolyLineVar.StrapElongate*2)) )
                {
                    g_hPosLoop.StateCmd.PosLoop = 1;//锁定
                    g_hPosLoop.StateCmd.Step = 1;
                    g_hPosLoop.StateCmd.Full = 0;
                }
                g_hPosLoop.PosRef = DoorStudyVar.Width - g_hCurveGenerate.PolyLineVar.StrapElongate;
            }
            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque	)
            {
                if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                {
                    //开门保持力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                }
                DoorFianlVar.StateCmd.ODHDToHFst = 1;
            }
            else
            {
                if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
                {
                    //开门最大力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
                }
            }

            if( StallCheckStru.StateCmd.StallFlag==1 )
            {

                PosStateVar.ODDoorWidth = labs((qep1.OutputRawTheta>>2)-PosStateVar.DoorPosOriginCorrect);
                DoorFianlVar.StateCmd.ODSwithEn = 1;
                CTLcmdVar.FinalCmd.bit.ReOD = 0;//开门到位，清除反开命令
                if( DoorFianlVar.StateCmd.ODSwithFlag==1 )
                {
                    DoorFianlVar.StateCmd.ODHToHLFst = 1;
                    DoorFianlVar.StateCmd.ODHDToHFst = 1;
                    //开门最终保持力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlLastTorque;
                }
                else
                {
                    if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                    {
                        //开门保持力
                        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                    }
                        
                }
                //if(g_hPosLoop.StateCmd.PosLoop==0 )
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0>( (int16)DoorStudyVar.Width-(int16)g_hCurveGenerate.PolyLineVar.StrapElongate)) )
                {
                    //锁定位置皮带变形2/3
                    if(g_hPosLoop.FullLockEn==1)
                    {
                        if(g_hPosLoop.StateCmd.PosLoop==0)
                        {
                            g_hPosLoop.StateCmd.PosLoop = 1;//锁定
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
            //开门最大力
            MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
            DoorFianlVar.StateCmd.ODHToHLFst = 0;
            DoorFianlVar.StateCmd.ODHDToHFst = 0;
            g_hPosLoop.StateCmd.PosLoop = 0;//不锁定
        }
    }//end of OD
    else if( CTLcmdVar.FinalCmd.bit.LCD==1 )
    {
        g_hPosLoop.StateCmd.StallPosLoop = 0;
        //g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
        OCDOverVar.state.CDCntEn = 1;//关门超时检测不允许
        OCDOverVar.state.ODCntEn = 0;//开门超时检测允许
      ///  OCDOverVar.state.CDOverFlag = 0;//关门超时标志清零   与关门超时告警冲突，故屏蔽掉 20190809 gw
        ////制动最大力
        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
        //	开门最大力
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

            
            //找位置时，堵转检测为1S
            if( (StallCheckStru.StateCmd.StallFlag==1) )
            {
               if((DoorWidthAutoStudy_en == 1)&&(AutoWidthStudyRun_initOver == 0))///使能门宽自识别功能
              {
                  if((AutoWidthStudyRunCls_initOK == 0) && (AutoWidthStudyRunOpen_initOK == 0))///初次上电运行为关门命令
                  {
                          PosStateVar.State.bit.CDEnd = 1;
                          PosStateVar.State.bit.Sgs = 1;
                          PosStateVar.State.bit.ODEnd = 0;
                          AutoWidthStudyRunCls_initOK = 1;
                          MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                          DoorStudyVar.PosOrigin = qep1.OutputRawTheta;///记住初始位置点
                  }
                  else if(AutoWidthStudyRunOpen_initOK == 1)///初次上电为开门命令，已进行初次位置定位，门宽自识别结束
                  {
                      ///DoorStudyVar.PosInput = qep1.OutputRawTheta;
                      DoorStudyVar.Width = labs(qep1.OutputRawTheta-DoorStudyVar.PosOrigin);///获得门宽
                      DoorStudyVar.Width = DoorStudyVar.Width>>2;
                      DoorStudyVar.DecPos.DoorMil	=	_IQ15mpyI32int(DoorStudyVar.Width,DoorStudyVar.PosToMilCoffe);
                      if(DoorStudyVar.DecPos.DoorMil < 3000)///默认学习超过600门宽，为正常
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
                              DoorWidthAutoStudy_en = 0;///学习完成后，标志清零

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

                              //门宽自校正初始化
                              PosStateVar.ODDoorWidth = DoorStudyVar.Width;
                              PosStateVar.CDDoorWidth = DoorStudyVar.Width;
                              PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;

                              //开关门命令保持时间
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
                      //门宽自校正初始化
                      PosStateVar.ODDoorWidth = DoorStudyVar.Width;
                      PosStateVar.CDDoorWidth = DoorStudyVar.Width;
                      PosStateVar.DoorPosOriginCorrect = PosStateVar.DoorPosOrigin;
                      //开关门命令保持时间
                      CTLcmdVar.HoldTimeCnt = 0;
                  }
                if((Camstateaction.StateCmd.QepcountInitSamOk == 0)&&(Camstateaction.StateCmd.QepcountSecSamClac == 0))
                {
                        ///Camstateaction.QepcountInit = ((Uint32)EQep1Regs.QPOSCNT);///记录门刀关门到位脉冲初始值，只记录一次
                        ///Camstateaction.QepcountInitSave = Camstateaction.QepcountInit;///保存上次原点值
                        Camstateaction.QepcountInit = qep1.OutputRawTheta;///记住初始位置点(PosStateVar.DoorPosOrigin << 2);
                        Camstateaction.StateCmd.QepcountInitSamOk = 1;
                }
            }
            g_hPosLoop.StateCmd.PosLoop = 0;//不锁定
        }
        else
        { 
            g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 0;
            //到位区力矩调整
            if( PosStateVar.State.bit.CDEnd==1 )
            {
                OCDOverVar.state.CDCntEn = 0;
                OCDOverVar.state.ODCntEn = 0;
                if(g_hPosLoop.FullLockEn==2)//直接定位
                {
                    if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<((int16)g_hCurveGenerate.PolyLineVar.StrapElongate*2)) )
                    {
                        g_hPosLoop.StateCmd.PosLoop = 1;//锁定
                        g_hPosLoop.StateCmd.Step = 1;
                        g_hPosLoop.StateCmd.Full = 0;
                    }
                    g_hPosLoop.PosRef = g_hCurveGenerate.PolyLineVar.StrapElongate;
                }
                if( StallCheckStru.StateCmd.StallFlag==1 )
                {
                 if((Camstateaction.StateCmd.QepcountInitSamOk == 0)&&(Camstateaction.StateCmd.QepcountSecSamClac == 0))
                  {
                          ///Camstateaction.QepcountInit = ((Uint32)EQep1Regs.QPOSCNT);///记录门刀关门到位脉冲初始值，只记录一次
                          ///Camstateaction.QepcountInitSave = Camstateaction.QepcountInit;///保存上次原点值
                          Camstateaction.QepcountInit = (PosStateVar.DoorPosOrigin << 2);
                          Camstateaction.StateCmd.QepcountInitSamOk = 1;
                  }
                  
                  DoorFianlVar.StateCmd.CDSwithEn = 1;//力矩切换标志
                    if( DoorFianlVar.StateCmd.CDSwithFlag==1 )
                    {
                        //	关门最终保持力
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque;
                    }
                    else
                    {
                        //关门保持力
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                    }
                    if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0<(int16)g_hCurveGenerate.PolyLineVar.StrapElongate) )
                    {
                            
                        if(g_hPosLoop.FullLockEn==1)
                        {
                            if(g_hPosLoop.StateCmd.PosLoop==0)
                            {
                                g_hPosLoop.StateCmd.PosLoop = 1;//锁定
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
                        //关门保持力
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
                    }
                    else
                    {
                        //关门最大力
                        MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
                    }
                }
            }//end of end
            else//非关门到位区为关门最大力矩
            {
                MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
                g_hPosLoop.StateCmd.PosLoop = 0;//不锁定
            }

        }
        //关门超时处理
        if( (OCDOverVar.state.CDOverFlag==1)&&(StallCheckStru.StateCmd.StallFlag==1) )
        {
            //关门保持力
            MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque;
        }
    }//end of LCD
    else if( CTLcmdVar.FinalCmd.bit.LOD==1 )
    {
        g_hPosLoop.StateCmd.StallPosLoop = 0;
        g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
        OCDOverVar.state.CDCntEn = 0;//关门超时检测不允许
        OCDOverVar.state.ODCntEn = 1;//开门超时检测允许
        OCDOverVar.state.CDOverFlag = 0;//关门超时标志清零
        ////制动最大力
        MTCVar.CurrentMin = PosStateVar.TorqueInter.OD_MaxTorque*-1;
        if( PowerFstVar.PowerFst==1 )
        {
            g_hPosLoop.StateCmd.PosLoop = 0;//不锁定
           
             //开门最大力
              if(AutoWidthStudyRunOpen_initOK == 0)
              {
                      MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
              }
              else
              {
                      MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
              }
           
           /*
            //开门最大力
            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque	)
            {
                if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                {
                    //开门保持力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                }
                DoorFianlVar.StateCmd.ODHDToHFst = 1;
            }
            else
            {
                if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
                {
                    //开门最大力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
                }
            }*/
            
          
            
            //MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
            //找位置时，堵转检测为1S，找到堵转则认为到位，对编码器及初次上电标志清零
            if( (StallCheckStru.StateCmd.StallFlag==1) )
            {
              if((DoorWidthAutoStudy_en == 1) && (AutoWidthStudyRun_initOver == 0))///使能门宽自识别功能
                {
                    if((AutoWidthStudyRunOpen_initOK == 0) && (AutoWidthStudyRunCls_initOK == 0))///初次上电运行为开门命令
                    {
                        PosStateVar.State.bit.CDEnd = 0;
                        PosStateVar.State.bit.Sgs = 0;
                        PosStateVar.State.bit.ODEnd = 1;
                        AutoWidthStudyRunOpen_initOK = 1;

                        ///DoorStudyVar.PosInput = qep1.OutputRawTheta;
                        DoorStudyVar.PosOrigin = qep1.OutputRawTheta;///记住初始位置点
                    }
                    else if(AutoWidthStudyRunCls_initOK == 1)///初次上电为开门命令，已进行初次位置定位，门宽自识别结束
                    {
                          ///DoorStudyVar.PosInput = qep1.OutputRawTheta;
                          DoorStudyVar.Width = labs(qep1.OutputRawTheta-DoorStudyVar.PosOrigin);///获得门宽
                          DoorStudyVar.Width = DoorStudyVar.Width>>2;
                          DoorStudyVar.DecPos.DoorMil	=	_IQ15mpyI32int(DoorStudyVar.Width,DoorStudyVar.PosToMilCoffe);
                          if(DoorStudyVar.DecPos.DoorMil < 3000)///默认学习超过600门宽，为正常
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
                              DoorWidthAutoStudy_en = 0;///学习完成后，标志清零

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
                              //孛劈令保持时间
                              CTLcmdVar.HoldTimeCnt = 0;
                              //CTLcmdVar.state.ODHoldFlag = 1;
                              g_hPosLoop.StateCmd.StallPosLoop = 0;
                              //门宽自校正初始化
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
                        //孛劈令保持时间
                        CTLcmdVar.HoldTimeCnt = 0;
                        //CTLcmdVar.state.ODHoldFlag = 1;
                        g_hPosLoop.StateCmd.StallPosLoop = 0;
                }
                
            }
        }
        else
        {
            //到位区力矩调整
            OCDOverVar.state.CDCntEn = 0;
            OCDOverVar.state.ODCntEn = 0;
            if( _IQabs(Iq_Fir.OutSoure) > DoorFianlVar.ODSwithTorque )
            {
                if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                {
                    //开门保持力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                }
                DoorFianlVar.StateCmd.ODHDToHFst = 1;
            }
            else
            {
                if( DoorFianlVar.StateCmd.ODHDToHFst==0 )
                {
                    //开门最大力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;
                }
            }

            if( StallCheckStru.StateCmd.StallFlag==1 )
            {
                DoorFianlVar.StateCmd.ODSwithEn = 1;
                CTLcmdVar.FinalCmd.bit.ReOD = 0;//开门到位，清除反开命令
                if( DoorFianlVar.StateCmd.ODSwithFlag==1 )
                {
                    DoorFianlVar.StateCmd.ODHToHLFst = 1;
                    DoorFianlVar.StateCmd.ODHDToHFst = 1;
                    //开门最终保持力
                    MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlLastTorque;
                }
                else
                {
                    if( DoorFianlVar.StateCmd.ODHToHLFst==0 )
                    {
                        //开门保持力
                        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
                    }
                        
                }
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PosStateVar.DoorPosActual0>( (int16)DoorStudyVar.Width-(int16)g_hCurveGenerate.PolyLineVar.StrapElongate)) )
                {
                        
                    if(g_hPosLoop.FullLockEn==1)
                    {
                        g_hPosLoop.StateCmd.PosLoop = 1;//锁定
                        g_hPosLoop.StateCmd.Full = 1;
                        g_hPosLoop.PosRef = PosStateVar.DoorPosActual0-(g_hCurveGenerate.PolyLineVar.StrapElongate) ;//锁定位置皮带变形量
                    }
                }
            }
            else
            {
                DoorFianlVar.StateCmd.ODSwithEn = 0;
                g_hPosLoop.StateCmd.PosLoop = 0;//不锁定

            }

        }
        //开门超时处理
        if( (OCDOverVar.state.ODOverFlag==1)&&(StallCheckStru.StateCmd.StallFlag==1) )
        {
        //开门保持力
        MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque;
        }

    }
    else//stop
    {
        OCDOverVar.state.CDCntEn = 0;
        OCDOverVar.state.ODCntEn = 0;
        OCDOverVar.state.CDOverFlag = 0;//关门超鼻辶?
        OCDOverVar.state.ODOverFlag = 0;
        g_hPosLoop.StateCmd.StallPosLoop = 0;
    }
    //发送给曲线命令
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
            if( PowerFstVar.PowerFst==1 )//未找到位置时，是不带曲线的关门
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
            MTCVar.StateCmd.RunEn = 0;//电机运行使
        }
                
        if( StopSel==2 )//锁定时不停机
        {
            if(FaultVar.fault.FautFlag0.bit.QEPErrorFlag==1)
            {
                g_hPosLoop.StateCmd.PosLoop = 0;
                MTCVar.StateCmd.RunEn = 0;//电机运行关闭 
            }
            else
            {
                if( (g_hPosLoop.StateCmd.PosLoop==0)&&(PowStallEn==0) )
                {
                    g_hCurveGenerate.RealCurveCalc.CmdState.PosLock = 1;
                    g_hPosLoop.StateCmd.PosLoop = 1;
                    g_hCurveGenerate.RealCurveCalc.CmdState.curve_line_en = 1;
                    g_hPosLoop.StateCmd.Step = 0;
                    
                   /// g_hPosLoop.PosRef = (PosStateVar.DoorPosActual0 - g_hCurveGenerate.PolyLineVar.StrapElongate);///K400马达锁定
                    g_hPosLoop.PosRef = PosStateVar.DoorPosActual0;
                    g_hPosLoop.StateCmd.Full = 0;
                }
            }
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
        }
        else if( StopSel==1 )//能耗制动
        {
            StallCheckStru.StateCmd.ClrStall = 1;
            TorqueProtecVar.StateCmd.ClrEn = 1;
            MTCVar.StateCmd.RunEn = 0;

        }
        else//停车
        {	
            MTCVar.StateCmd.RunEn = 0;
        }
            
    }
    if( g_hCurveGenerate.RealCurveCalc.CmdState.CmdSwt==1 )//切换时，清堵转与过力矩检测
    {
        TorqueProtecVar.StateCmd.ClrEn = 1;
    }
    
    ///匹配新国标门刀逻辑处理
	///if(((OutDelay.StateCmd.bit.CDEndOut==1 )&&(QepcountInitSamOk != 0)) || (Camstateaction.StateCmd.CamStateActionRun == 1))///关门到位区且基准值采样完成后实时记录门位置
	if(Camstateaction.CamStateActionDoneEn == 1)///使能门刀逻辑处理程序
	{
		if(Camstateaction.StateCmd.QepcountInitSamOk == 1)///初始化采样完成后，可实时计算门的位置
		{
			///Camstateaction.QepcountInitSave = Camstateaction.QepcountInit;
			Camstateaction.QepcountSec = ((Uint32)qep1.OutputRawTheta);///((Uint32)EQep1Regs.QPOSCNT);
			Camstateaction.QepcountErr = labs(Camstateaction.QepcountSec - Camstateaction.QepcountInit);
			Camstateaction.QepcountBase = (Uint32)(76980 / g_hUnitTransform.Whell_dia);///4096*70/(234*3.1415926);暂以5.9mm作为扒开门的值    76980
			Camstateaction.QepReccountBase = (Uint32)(612788 / g_hUnitTransform.Whell_dia);///4096*470/(254*3.1415926);暂以47mm作为恢复关门力的值   1173418
			Camstateaction.StateCmd.QepcountSecSamClac = 1;///正在计算，基准值不进行采样
			if( StallCheckStru.StateCmd.StallFlag==1 )
			{
				Camstateaction.QepCdEndsBase = 1;
			}
		}
		else///退出关门到位区，基准采样清零
		{
			///QepcountInitSamOk = 0;///初始化采样只采样一次，应存在EEPROM内
			///QepcountSecSamClac = 0;
		}

		///if(OutDelay.StateCmd.bit.ODEndOut == 1 )///开门到位后，允许重新采样
		if(( (CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.LOD==1)||(CTLcmdVar.FinalCmd.bit.ReOD) )&&(PosStateVar.State.bit.CDEnd == 0)) ///关门到位区内 标志不清零  20200318
		{
			Camstateaction.StateCmd.QepcountInitSamOk = 0;
			Camstateaction.QepcountSec = 0;
			Camstateaction.QepcountErr = 0;
			Camstateaction.StateCmd.QepcountSecSamClac = 0;///正在计算，基准值不进行采样
			Camstateaction.StateCmd.CamStateActionRun = 0;///20s时间标志清零，允许关门
			Camstateaction.QepCdEndsBase = 0;
			FaultVar.fault.FautFlag1.bit.CamActionFault = 0;//防扒逻辑报警清零
                        
                        CTLcmdVar.FinalCmd.bit.ReCD = 0;      ///20190912 改进门刀扒开后，开门指令点动，自动关门BUG
                        Camstateaction.StateCmd.Step = 0;     ///
                        Camstateaction.StateCmd.ReCamen = 0;  /// 
		}

	/*
		if(( OutDelay.StateCmd.bit.CDEndOut==1 )&&(QepcountErr > QepcountBase)&&(CTLcmdVar.FinalCmd.bit.CD==0)
		&&(CTLcmdVar.FinalCmd.bit.LCD==0))///暂定5个mm
		{
			Qepstallcounti++;
			if(Qepstallcounti >= 1)
			{
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque >> 8;///门刀撤力
				CTLcmdVar.FinalCmd.bit.ReCD = 0;
				Qepstallcounti = 0;
			}
			else
			{
				//	关门最终保持力
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

		if(Camstateaction.StateCmd.ReCDen == 1)///收刀区内，恢复收刀，执行关门指令
		{
			CTLcmdVar.FinalCmd.bit.ReCD = 1;
			if((OutDelay.StateCmd.bit.CDEndOut == 1)||(CTLcmdVar.FinalCmd.bit.OD == 1)||(CTLcmdVar.FinalCmd.bit.LOD == 1))
			{
				Camstateaction.StateCmd.ReCDen = 0;
				CTLcmdVar.FinalCmd.bit.ReCD = 0;
			}
		}

		if(Camstateaction.StateCmd.ReCamen == 1)///有正常开关门指令，重新收刀指令清零
		{
			if((OutDelay.StateCmd.bit.CDEndOut == 1)||(CTLcmdVar.FinalCmd.bit.OD == 1)||(CTLcmdVar.FinalCmd.bit.LOD == 1)||
			  (CTLcmdVar.FinalCmd.bit.CD == 1)||(CTLcmdVar.FinalCmd.bit.LCD == 1))
			{
				Camstateaction.StateCmd.ReCamen = 0;
			}
		}

		switch(Camstateaction.StateCmd.Step)
			{
				case 0:///扒门条件判断
					///if(( OutDelay.StateCmd.bit.CDEndOut==1 )&&(QepcountInitSamOk != 0)&&(QepcountErr > QepcountBase)&&())///在关门到位区，暂定7个mm
					if(( OutDelay.StateCmd.bit.CDEndOut==1 )&&(Camstateaction.QepCdEndsBase == 1)&&(Camstateaction.QepcountErr > Camstateaction.QepcountBase))///暂定5个mm，正常工作后
					{
						Camstateaction.data.CamStateActionTime = 50;///滤波50ms
						Camstateaction.StateCmd.CamStateActionRun = 1;///执行扒门程序，关门指令不执行，20s后或开门到位区此标志位清零
						Camstateaction.QepCdEndsBase = 0;
						Camstateaction.StateCmd.Step = 1;///继续延时判断
						FaultVar.fault.FautFlag1.bit.CamActionFault = 1;//防扒逻辑报警
						Camstateaction.CamStateActionCount++; //执行一次防扒逻辑计数器+1
						if(Camstateaction.CamStateActionCount >=60000 ) Camstateaction.CamStateActionCount = 60001;
					}
					else
					{
						Camstateaction.StateCmd.Step = 0;///结束
					}

					if((Camstateaction.QepcountErr < Camstateaction.QepReccountBase) && (Camstateaction.StateCmd.ReCamen == 1))///扒开依然在或扒开后恢复在允许收刀区内 G0+47mm
					{
						if(Camstateaction.StateCmd.CamStateActionRun == 1)
						{
							Camstateaction.StateCmd.Step = 0;///进入Case0,结束
						}
						else
						{
							Camstateaction.StateCmd.Step = 2;///进入Case2，做恢复收刀处理
						}

					///	Camstateaction.StateCmd.CamStateActionRun = 1;///使能扒门处理标志
						Camstateaction.StateCmd.ReCamen = 0;
					}


				break;
				case 1:///扒门信号时间滤波，执行双向撤力操作
					if( Camstateaction.data.CamStateActionTime == 0 )
					{
						MTCVar.CurrentMax = (PosStateVar.TorqueInter.OD_HlTorque >> 2); //切换至开门保持力
						MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlLastTorque >> 8;///门刀撤力
						Camstateaction.StateCmd.CamStateActionRun = 1;///执行扒门程序，关门指令不执行，20s后或开门到位区此标志位清零
						if(PowerFstVar.PowerFst==1)
						{
							///PosStateVar.State.bit.CDEnd = 0;
							OutDelay.StateCmd.bit.CDEndOut = 0;
						}

						Camstateaction.StateCmd.Step = 2;///进入恢复收刀逻辑
					}
					else
					{
						if(Camstateaction.QepcountErr < Camstateaction.QepcountBase)///暂定7个mm，有开门指令导致，或在50ms内又恢复位置了；
						{
							Camstateaction.StateCmd.Step = 0;///结束
						}
						else
						{

						}
					}
				break;
				case 2:///扒门后复位处理--滤波时间判断
					if(Camstateaction.QepcountErr < Camstateaction.QepReccountBase)///依然在允许收刀区内 G0+47mm
					{
						Camstateaction.data.CamStateActionTime = 1500;///滤波1000ms
						Camstateaction.StateCmd.Step = 3;///继续延时判断
					}
					else
					{
						Camstateaction.data.CamStateActionTime100ms = 200;///滤波20s,再响应关门指令
						Camstateaction.StateCmd.Step = 4;///继续延时判断
					}
				break;
				case 3:///扒门后复位处理---在收刀区内，恢复收刀
					if( Camstateaction.data.CamStateActionTime == 0 )///一直在G0+47mm内，恢复收刀
					{
					///	CTLcmdVar.FinalCmd.bit.ReCD = 1;
					///	MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;///正常开关门力
					///	MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
						Camstateaction.StateCmd.CamStateActionRun = 0;///执行扒门程序，20s后或开门到位区此标志位清零,或自己执行收刀
						Camstateaction.StateCmd.ReCDen = 1;
						Camstateaction.StateCmd.Step = 0;///结束
					}
					else
					{
						if(Camstateaction.QepcountErr > Camstateaction.QepReccountBase)///扒出允许收刀区内G0+47mm
						{
							Camstateaction.data.CamStateActionTime100ms = 50;///200
							Camstateaction.StateCmd.Step = 4;///继续延时判断
						}
					}
				break;
				case 4:///扒门后复位处理
					if( Camstateaction.data.CamStateActionTime100ms == 0 )///超出G0+47mm内，锁定，20s后恢复关门
					{
						///if((CTLcmdVar.FinalCmd.bit.CD)||(CTLcmdVar.FinalCmd.bit.LCD))///20s后响应关门指令
					//	{
						///	MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_MaxTorque;///正常开关门力
						//	MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_MaxTorque;
							Camstateaction.StateCmd.CamStateActionRun = 0;///执行扒门程序，20s后或开门到位区此标志位清零
							Camstateaction.StateCmd.ReCamen = 1;///扒门后 20s复位标志
					//	}
						///else
						//{

						//}

						Camstateaction.StateCmd.Step = 0;///结束
					}
					else
					{
						if(Camstateaction.QepcountErr < Camstateaction.QepReccountBase)///又重新回到允许收刀区内 G0+47mm
						{
							Camstateaction.StateCmd.Step = 2;///重新判断收刀逻辑
						}
					}

				break;
				case 9:///门刀处理逻辑结束

					///Camstateaction.data.CamStateActionTime = 0;///50ms/1000ms时间计数器
					///Camstateaction.data.CamStateActionTime100ms = 0;///20s时间计数器
					///Camstateaction.StateCmd.Step = 0;///步数清零
				break;

				default:break;
			}
	}
    
    //TestAdd2 = g_hPosLoop.PosRef ;
}
/*******************************************************************************
* 函数名称  : ParInit
* 描述      : 参数初始化
* 输入      : None
* 输?     : None
* 返回      : None.
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
    if(g_intFstWriterNum==FstNum )//写入过参数则直接读取
    {
      ReadParAll();	
    }
    else//第一次时，先写入E2PROM
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
    //E2PROM manage---E2PROM的任务管理
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
* 函数名称  : OutDeal
* 描述      : 输出继电器处理
* 输入      : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void OutDeal(void)
{
	if( PowerFstVar.PowerFst==0 )
	{
		//开关门到位处理
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
		//堵转处理
		if( (PosStateVar.State.bit.ODEnd==0)&&(PosStateVar.State.bit.CDEnd==0) )
		{
			if( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1) )//堵转或过力鼐衔伦?			{
			{	
                                if( (g_hCurveGenerate.RealCurveCalc.CmdState.ODCD==1)&&(OutDelay.FaultOutODEn==0) )//开门命令且不允许输出时
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
	//开关门到位处理
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
		//堵转处理
		if( (PosStateVar.State.bit.ODEnd==0)&&(PosStateVar.State.bit.CDEnd==0) )
		{
			if( ( StallCheckStru.StateCmd.StallFlag==1 )||(TorqueProtecVar.StateCmd.TorqueOverFlag==1) )//堵转或过力矩均认为堵转
			{
				if( (g_hCurveGenerate.RealCurveCalc.CmdState.ODCD==1)&&(OutDelay.FaultOutODEn==0) )//开门命令且不允许输出时
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
	//故障处理
	OutDelay.StateCmd.bit.FaultOut = PosStateVar.State.bit.Sgs; ///安全触板
	if(FaultVar.fault.FautFlag0.bit.QEPErrorFlag)
	{
		OutDelay.StateCmd.bit.CDEndOut = 0;
		OutDelay.StateCmd.bit.ODEndOut = 0;
                PosStateVar.State.bit.Sgs = 0;
	}

///输出处理
        
        if(CTLcmdVar.CMDSel!=3)
        {
            //输出OUT0L
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
         /* if( OutDelay.StateCmd.bit.ODEndOut==1 )  ///开门到位
          {
                  if(OutDelay.DoorLocation == 0)//前门
                  {
                   //	Ecan_state.Ecan_Tx.bit.FrontDOLEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDOL = 1;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDOLEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDOL = 0;    ///rear.data.bit
                  //	Ecan_state.Ecan_Tx.bit.FrontDCLEN = 0; ///front.mask.bit
                  }
                  else ///后门
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

          if( OutDelay.StateCmd.bit.CDEndOut==1 ) ///关门到位
          {
                  if(OutDelay.DoorLocation == 0)//前门
                  {
                  // 	Ecan_state.Ecan_Tx.bit.FrontDCLEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontDCL = 1;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearDCLEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearDCL = 0;    ///rear.data.bit
                  //	Ecan_state.Ecan_Tx.bit.FrontDOLEN = 0; ///front.mask.bit
                  }
                  else ///后门
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

          if( OutDelay.StateCmd.bit.StallOut==1 )///堵转 关门阻力>150N
          {
                  if(OutDelay.DoorLocation == 0)//前门
                  {
                  // 	Ecan_state.Ecan_Tx.bit.FrontBRKEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontBRK = 1;   ///front.data.bit
                  //	Ecan_state.Ecan_Tx.bit.RearBRKEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearBRK = 0;    ///rear.data.bit
                  }
                  else ///后门
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

          if((TerminalVar.Value.bit.bit1 == 0))     //光幕、安全触板| && (TerminalVar.Value.bit.bit3 == 0)
      {
          if(OutDelay.DoorLocation == 0)//前门
                  {
                  // 	Ecan_state.Ecan_Tx.bit.FrontLTEN = 1; ///front.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.FrontLT = 1;   ///front.data.bit
          //		Ecan_state.Ecan_Tx.bit.RearLTDEN = 0; ///rear.mask.bit
                          Ecan_state.Ecan_Tx_Rear.bit.RearLTD = 0;    ///rear.data.bit
                  }
                  else ///后门
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
* 函数名称  : ThreeOutDeal
* 描?     : 三线的输出处理
* 输入      : None
* 输出      : None
* 返回      : None.
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
			//开关门到位处理
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
		//输出OUT0L
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
* 函数名称  : FaultAnaly
* 描述      : 故障分析处理，当断电时，并完成断电保存
* 输入      : None
* 输出      : None
* 返回      : None.
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
  FaultVar.fault.FautFlag1.bit.TemperMErorr = TemperaMtureVar.state.TemperMError;///马达温度传感器 err24
	
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
    //GPIO_SetBits(GPIOA,GPIO_Pin_11);//PA.11 输出高
    g_hBuzzerVar.BuzzerEn = 0;
  }
  else
  {
    OutDelay.StateCmd.bit.FaultOut = 1;
    //GPIO_ResetBits(GPIOA,GPIO_Pin_11); /// PA.11 输出低  喇叭响
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
	//断电保存故障表
	if( VolProtectVar.state.SaveParVol==1 )//只有在欠压且只保存一次，或者在欠压下有新的故
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
* 函数名称  : PosCorrect
* 描述      : 当因皮带打滑时，校正位置，以防止开关门失效；当最原来的位置原点与校正
			  正后的，相差大于大?圈时，需要校正编码器的当前位置，以防止皮带往一个
			  方向不停打滑出现的编码盘计数溢出
* 输入      : None
* 输出      : None
* 返回      : None.
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
		//if( PosStateVar.DoorPosActual>(PosStateVar.DoorWidth+g_hCurveGenerate.PolyLineVar.StrapElongate) )//门宽实际位置大于学习门宽与皮带变形量之和
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
	//角度校正当电机堵转后比AB与PWM，并以PWM为基准
}
/*******************************************************************************
* 函数名称  : SpeedPIAdjustFun
* 描述      : ASR的PI分段调整
* 输入      : None
* 输出      : None
* 返回      : None.
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
* 函数名称  : ODMaxTorqueLimitUp
* 描述      : 开门过程最大力矩限幅
* 输入     : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void ODMaxTorqueLimitUp(void)
{
	if((CTLcmdVar.FinalCmd.bit.OD==1)||(CTLcmdVar.FinalCmd.bit.ReOD==1)||( CTLcmdVar.FinalCmd.bit.LOD==1))///开、反开、慢开 ,暂没考虑门宽学习
	{
		if((TorqueProtecVar.StateCmd.TorqueOverFlag==1) && (PosStateVar.State.bit.ODEnd == 0)) //过力矩 且力没有最终切换 && ( DoorFianlVar.StateCmd.ODSwithFlag==0 )
		{
			if(OD_TorqueUPTime >= 2000)///超过2.5s
			{
				//开门保持力
				MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque; //切换至开门保持力
				OD_TorqueUPTime = 2600;
				OD_TorqueDWNTime = 2600;
				CTLcmdVar.FinalCmd.bit.ReOD = 0;
			}
		}
		else
		{
			if(OD_TorqueDWNTime <= 100)///超过2.5s
			{
				OD_TorqueDWNTime = 100;
				OD_TorqueUPTime = 0;
			}
			else
			{
				//开门保持力
				MTCVar.CurrentMax = PosStateVar.TorqueInter.OD_HlTorque; //切换至开门保持力
			}
		}
	}
	else if((CTLcmdVar.FinalCmd.bit.CD==1)||(CTLcmdVar.FinalCmd.bit.LCD==1))///关、慢关
	{
		if(((TorqueProtecVar.StateCmd.TorqueOverFlag==1)||(StallCheckStru.StateCmd.StallFlag==1)) && (PosStateVar.State.bit.CDEnd == 0)) //过力矩 且力没有最终切换 && ( DoorFianlVar.StateCmd.ODSwithFlag==0 )
		{
			if(CD_TorqueUPTime >= 2000)///超过2.5s
			{
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque; //切换至关门保持力

				CD_TorqueUPTime = 2600;
				CD_TorqueDWNTime = 2600;
			}
		}
		else
		{
			if(CD_TorqueDWNTime <= 100)///超过2.5s
			{
				CD_TorqueDWNTime = 100;
				CD_TorqueUPTime = 0;
			}
			else
			{
				MTCVar.CurrentMin = PosStateVar.TorqueInter.CD_HlTorque; //切换至关门保持力
			}
		}
	}
}

/*******************************************************************************
* 函数名称  : MenuKeySpdAdj
* 描述      : 门机运行速度三档调速，中间默认速度可进入参数调节
* 输入     : None
* 输出      : None
* 返回      : None.
*******************************************************************************/
void MenuKeySpdAdj(void)
{
	///参数按键选择 ///
	///if(((KeyAvailable&RLKEY)==RLKEY)&&((KeyAvailable&LLKEY)==LLKEY))//按键复合功能 速度调整
        if(1)
        {
            switch(key.val)
              {
                case SPEEDUPKEY://高速选择

                    if(Speed_cnt==0)///按键按下计数器为零
                    {
                          Speed_llkey++;

                          if(Speed_llkey > 1)///初次进入菜单，显示为当前值
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
                                  DisStru.MenuIndex += 4;//直接进入参数查看界面
                          }
                          else
                          {}


                          KeyAvailable &= (~DOWNKEY);//清无效

                          KeyAvailable &= (~LLKEY);//清无效
                          KeyAvailable &= (~RLKEY);//清无效
                          KeyAvailable &= (~SPEEDUPKEY);//清无效
                          KeyAvailable &= (~KeyAvailable);//清无效
                          key.val = 0;   

                          Speed_cnt = 800;
                          Speed_KeyCountOver = 0;///启动计算
                        }

                      break;
                      case ODUPKEY://开门高速
                               break;
                      default:break;
                    }
		}

	///速度根据档位调节///

	if(((OutDelay.StateCmd.bit.CDEndOut==1)||(OutDelay.StateCmd.bit.ODEndOut==1))&&(Speed_KeyCountOver==0))///开门或关门到位区速度曲线调整
	{
		switch(Speed_step)
		{
			case 1://低速运行
					g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel = 582;//P4.04 开门S曲线参考速度
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 25;	     //P4.03 开门S曲线参考加速时间
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 23;	 //P4.06 开门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 50;//P4.09 开门S曲线参考高速比例
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 23;   //P5.03 关门S曲线参考加速时间
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 25;   //P5.06 关门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 40;//P5.09  关门S曲线参考高速比例
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 45;//P5.11  关门爬行距离
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
					UnusualDcc.ParSet.AccTime = 5;

					Para0404Calc();
			break;
			case 2://中速运行 出厂值 可手动调节

				if(g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel == 582)//P4.04 开门S曲线参考速度   参数未变 默认中速运行
				{
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 21;	     //P4.03 开门S曲线参考加速时间
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 19;	 //P4.06 开门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 55;//P4.09 开门S曲线参考高速比例
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 19;   //P5.03 关门S曲线参考加速时间
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 21;   //P5.06 关门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 45;//P5.09  关门S曲线参考高速比例
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 45;//P5.11  关门爬行距离
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
					UnusualDcc.ParSet.AccTime = 5;

					Para0404Calc();
				}
				else///手动调节
				{}

			break;
			case 3://高速运行
				
					g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel = 682;//P4.04 开门S曲线参考速度
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 14;	     //P4.03 开门S曲线参考加速时间
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 14;	 //P4.06 开门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 65;//P4.09 开门S曲线参考高速比例
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 14;   //P5.03 关门S曲线参考加速时间
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 14;   //P5.06 关门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 50;//P5.09  关门S曲线参考高速比例
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 45;//P5.11  关门爬行距离
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
                                        UnusualDcc.ParSet.AccTime = 3;
                                        
                                        Para0404Calc();

			break;
                        case 4://超高速运行

                                        g_hCurveGenerate.PolyLineVar.RefHighSpd_LinearVel = 682;//P4.04 开门S曲线参考速度
					g_hCurveGenerate.OD_RefCurve.SupAccTim = 14;	     //P4.03 开门S曲线参考加速时间
					g_hCurveGenerate.OD_RefCurve.SdnDecTim = 15;	 //P4.06 开门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.ODActHighSpd_per = 65;//P4.09 开门S曲线参考高速比例
					g_hCurveGenerate.CD_RefCurve.SupAccTim = 14;   //P5.03 关门S曲线参考加速时间
					g_hCurveGenerate.CD_RefCurve.SdnDecTim = 14;   //P5.06 关门S曲线参考减速时间
					g_hCurveGenerate.PolyLineVar.CDActHighSpd_per = 55;//P5.09  关门S曲线参考高速比例
					g_hCurveGenerate.PolyLineVar.CDCreepDisMil = 100;//P5.11  关门爬行距离
					g_hCurveGenerate.PolyLineVar.CDCreepDis = (int32)(g_hCurveGenerate.PolyLineVar.CDCreepDisMil*(int32)PosStateVar.MilToPosCoeff)>>15;
					UnusualDcc.ParSet.AccTime = 3;
					
                                        Para0404Calc();
				
			break;
			default:break;
		}
		Speed_KeyCountOver = 1;///速度计算结束
	}

    //界面显示///
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








