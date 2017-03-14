#include	"Git_Repository_Fecal\Include\Motor.h"
#include	"Git_Repository_Fecal\Include\Global.h"
#include  "Git_Repository_Fecal\Include\IO_Expand.h"
#include  "Git_Repository_Fecal\Include\RS422.h"
#include	"Git_Repository_Fecal\Include\MCU_Init.h"

#define			IO_TEST
#define			RELAY_TEST
float 	code 		KsTab_25M[ADCON_LEVEL] ={10.49,5.24,3.50,2.62,2.10,1.75,1.50,1.31,1.17,1.05,0.95,0.87,0.81,0.75,0.70,0.66,0.62,0.58,0.55,0.52};
float 	data		Ks;
uchar   data    Kv;
uchar		data 		tmpRelay[2];
bit		mixFlag;
bit		sampleFlag;
bit		cleanFlag;

void IOCtrl()
{
	ui0 =0;
	ui1 =1;
	gPtr =&tmpRelay;
	while(ui0 <RELAYNUM)
	{
		if(*gPtr ==0 && *(gPtr+1) ==0)
		{
			ui0 +=8;
		}
		else
		{
			if(*gPtr !=0)
			{				
				if((*gPtr &ui1) !=0)
				{
					RelayOffCnt[ui0] =relayTdelay[ui0];
				}
				ui1 <<=1;
			}
			ui0 ++;					
		}
		if((ui0 &7) ==0)
		{
			ui1 =1;
			YY0 |=*gPtr;	
			*gPtr =0;
			gPtr ++;
			YY0 &= ~(*gPtr);
			*gPtr =0;
			gPtr ++;
		}
	}		
	gwrite_Port(PLC_OL, ~YY0);
	gwrite_Port(PLC_OH, ~YY1);
	IO_YY[0] =YY0;
	IO_YY[1] =YY1;
	//ϡ��Һ����ϴҺ:1,����;0,�����㣻��Һ:1,���;0,����
	XX =gread_Port(PLC_I);
	IO_XX[0] =XX;
}

void ReadAxis()
{
	gwrite_Port(gPLSCMD, 0x01);
	for(ui0 =0;ui0 <MOTORNUM;ui0 ++)
	{
		AxisV[ui0].Xin = gread_Port( ADDR_AXIS0 +ui0*16 +DRV_STATE);
		AxisV[ui0].Pos.Byte[1] = gread_Port( ADDR_AXIS0 + ui0*16 + PLS_CNT_L);
		AxisV[ui0].Pos.Byte[0] = gread_Port( ADDR_AXIS0 + ui0*16 + PLS_CNT_H);
		AxisV[ui0].Pos.intD += AxisP[ui0].refpos.intD;
	}	
}

void PositioningStart(unsigned char axis)
{
	MotorY =AxisV[axis].Yout;
	if(AxisV[axis].Speed !=0)	//��ֹ��ѯ
	{
		//Error
		return;
	}
	AxisV[axis].TargetPos.intD = sii1;
	MotorY |=0x03;
	if(sii1 > AxisV[axis].Pos.intD)  // + positioning
	{
		MovDir =1;
	}
	else if(sii1 < AxisV[axis].Pos.intD)  // - positioning
	{
		MovDir =0;
	}
	else
	{
		return;
	}
	
	Positioning =1;
	AxisV[axis].Yout =MotorY;
}

void PositioningCompleted(unsigned char axis)
{
	CurV = 0;
	AxisV[axis].Speed = CurV;
	gwrite_Port(ADDR_AXIS0 + axis*16 +SPEEDCMD, CurV);		
	AxisV[axis].Yout &= 0x3c;	//clear Decflag & Positioning
	gwrite_Port(ADDR_AXIS0 + axis*16 +STATECMD, AxisV[axis].Yout);
	
	AxisV[axis].TargetV = 0;
}

//������ٵ㣬����λ�ÿ���
//�˶���ʼʱ�����÷����Ŀ��λ��
//using uii0;
uchar ChkSpeed(unsigned char axis,unsigned char targetV)
{
	MotorY = AxisV[axis].Yout;
	TargPos.intD = AxisV[axis].TargetPos.intD;
	CurPos.intD = AxisV[axis].Pos.intD;
	if(MovDir)
	{
		if(TargPos.intD > CurPos.intD)
		{
			RemDis.uintD = TargPos.intD - CurPos.intD;
		}
		else
		{
			RemDis.uintD = 0;
			PositioningCompleted(axis);
		}	
	}
	else
	{
		if(TargPos.intD < CurPos.intD)
		{
			RemDis.uintD = CurPos.intD - TargPos.intD;
		}
		else
		{
			RemDis.uintD = 0;
			PositioningCompleted(axis);
		}
	}
	
	CurV =AxisV[axis].Speed;
	if(RemDis.uintD ==0)
	{
		if(CurV !=0) //��λ���
		{
			PositioningCompleted(axis);
			return 0; //ֹͣ�˶�
		}
		else
			return 0; //û���˶�
	}
	
	//δ�ﵽĿ��λ�ã��ٶȲ���Ϊ0
	KsMap(axis);
	KvMap(axis);
	uii0 =CurV *CurV;
	uii0 =uii0 >>Kv;
	if(uii0 >=(RemDis.uintD *Ks))//���٣�λ��ģʽ�£�����ʱ�䳣��������Ks��
	{
		DecFlag =1;		
		if(CurV > AxisP[axis].DecSpeed)
			CurV  -=AxisP[axis].DecSpeed;
		
		uii0 = CurV *CurV;
		uii0 =uii0 >>Kv;
		if(uii0 >=(RemDis.uintD *Ks))
		{
			if(CurV > AxisP[axis].DecSpeed)
				CurV  -=AxisP[axis].DecSpeed;
		}
	}
	else if(CurV < targetV && DecFlag ==0) //������
	{
		if(AxisV[axis].Acon < AxisP[axis].AccConP)
		{
			AxisV[axis].Acon ++;
		}
		else
		{
			AxisV[axis].Acon = 1;
			CurV +=AxisP[axis].AccSpeed;
		}				
	}
	else
	{
		AxisV[axis].Acon = 1;
	}
	
	//��ˢ���ٶȶ˿ڣ���������λ���㣻
	AxisV[axis].Speed =CurV;
	return CurV;
}

/*
unsigned char PosLimit(unsigned char axis)
{
	unsigned char speed;
	if(RefDone)
	{
		//if()
		//ChkSpeed(axis,AxisV[axis].TargetV);	//��sLimit���мӼ��ٿ��ƣ��˶���ʼʱ�����÷������λĿ��λ�ã�AxisV[axis].TargetPos.intD��
	}
	else//δͨ���ο��㣬���ٿ���
	{
	}
	return speed;
}
*/
void SpeedModeRun(unsigned char axis,unsigned char targetV)
{
	CurV = AxisV[axis].Speed;
	if(CurV ==0 && targetV ==0)	//û���˶�ָ��
		return;
	
	//PosLimit();	//�ı�targetV
	
	MotorY = AxisV[axis].Yout;
	DecFlag =0;
	if(targetV ==0)	//������
	{
		if(AxisV[axis].Dcon < AxisP[axis].DecConP)
		{
			AxisV[axis].Dcon ++;
		}
		else
		{
			AxisV[axis].Dcon = 1;
			if(CurV > AxisP[axis].DecSpeed)
				CurV  -= AxisP[axis].DecSpeed;
			else
			{	
				CurV =0;  //���ٵ�0���˶�ֹͣ
				AxisV[axis].Yout &= 0x3c;
				AxisV[axis].TargetPos =AxisV[axis].Pos; //����ģʽ�л�ʱ�����˶�
			}
			gwrite_Port(ADDR_AXIS0 + axis*16 +SPEEDCMD, CurV);
			gwrite_Port(ADDR_AXIS0 + axis*16 +STATECMD, AxisV[axis].Yout);
		}
	}
	else	//���١�������
	{
//		MotorX = AxisV[axis].Xin;	
		if(CurV < targetV) 
		{
			if(AxisV[axis].Acon < AxisP[axis].AccConP)
			{
				AxisV[axis].Acon ++;
			}
			else
			{
				AxisV[axis].Acon = 1;
				CurV +=AxisP[axis].AccSpeed;	
			}							
		}
		gwrite_Port(ADDR_AXIS0 + axis*16 +STATECMD, AxisV[axis].Yout);
		gwrite_Port(ADDR_AXIS0 + axis*16 +SPEEDCMD, CurV);
	}
	AxisV[axis].Speed =CurV;
//	AxisV[axis].Yout =MotorY;
}
void StartRef()
{
	if(AxisV[ui2].Xin&8 || AxisV[ui2].RunStep ==4)	//���ѹ���ο��㿪���ϻ��ߵ��ֹͣ�˶�
	{
		MovDir = ~MovDir;
		AxisV[ui2].RunStep = 5;											
		gwrite_Port( ui2*16 + ADDR_AXIS0 + YOUT2 , 0x01);
	}
	else
	{
		gwrite_Port( ui2*16 + ADDR_AXIS0 + YOUT2 , 0x00);
		AxisV[ui2].RunStep = 3;							
	}
	MotorY |= 0x03;
	AxisV[ui2].Yout = MotorY;
	AxisV[ui2].TargetV = AxisP[ui2].MannualSpeed;
}

//using ui2
void MotorRst()
{
	for(ui2 =0;ui2 <MOTORNUM;ui2++)
	{
		MotorX = AxisV[ui2].Xin;
		MotorY = AxisV[ui2].Yout;
		/*		
		if(StepRunFlag)
		{
			if((AxisV[ui2].RunStep&1)==0 && ContinueRun == 0)
				return;
		}
		*/
		/*
		if((AxisV[ui2].RunStep&1)!=0)
		{
			AxisV[ui2].RunCnt --;
			if(AxisV[ui2].RunCnt ==0)
			{
				//Error
				AxisV[ui2].TargetV = 0;
				AxisV[ui2].RunStep = 0;
			}
		}
		else
		{
			AxisV[ui2].RunCnt =4000;
		}
		*/
		switch(AxisV[ui2].RunStep)
		{
			case 0:
			case 16:
				break;
			case 2://����
				if(AxisV[ui2].Speed ==0) //�����ֹ״̬�ſ��������˶�
				{
					if(AxisP[ui2].BitPara&1)
						MovDir = 1;
					else
						MovDir = 0;
					switch(ui2)
					{
						case 1:	//Y����
						case 2: //X����
							if(AxisV[ui2-1].Xin&4) //Z������λ��ɺ󣬷��ɸ�λY������Y������λ��ɺ󣬷��ɸ�λX������
							{
								StartRef();
							}
							break;
						case 4: //ȡ����C����
							if((AxisV[MOTOR_ARM_Z].Xin&4) && (AxisV[MOTOR_ARM_Z].Pos.intD >= (AxisP[MOTOR_ARM_Z].workpos0.intD -3)))//ȡ�����ڸ�λ
							{
								StartRef();
							}						
							break;
						case 7: //�̵��
							if((AxisV[MOTOR_ARM_Z].Xin&4) && (AxisV[MOTOR_Mix].Xin&4))
							{
								if(AxisV[MOTOR_ARM_Z].Pos.intD >= (AxisP[MOTOR_ARM_Z].workpos0.intD -3) && (AxisV[MOTOR_Mix].Pos.intD >= (AxisP[MOTOR_Mix].workpos0.intD -3)))
								{
									StartRef();
								}
							}
							break;
						default: //Z����,������,ȡ���۴�ֱ���,�����õ��
							StartRef();
							break;
					}
				}
				break;
			case 3:
				if(AxisV[ui2].Xin&8) //ѹ�ڲο��㿪����
				{
					AxisV[ui2].TargetV = 0;	
					AxisV[ui2].RunStep ++;
				}
				break;
			case 4:
				if(AxisV[ui2].Speed ==0) //����˶�ֹͣ
				{
					StartRef();						
				}	
				break;
			case 5:
				if((AxisV[ui2].Xin&4) !=0) //�ҵ��ο����ֹͣ,RefEn =0 
				{
					AxisV[ui2].TargetV = 0;
					AxisV[ui2].RunStep ++;
					gwrite_Port( ui2*16 + ADDR_AXIS0 + YOUT2 , 0x00);
				}
				break;
			case 6: //������λ
				if(AxisV[ui2].Speed ==0)
				{
					sii1 =AxisP[ui2].workpos0.intD;
					PositioningStart(ui2);
					AxisV[ui2].RunStep ++;
				}
				break;
			case 7:
				if((AxisV[ui2].Yout&0x40) ==0)
				{
					AxisV[ui2].RunStep =16;
					sii1 =AxisV[ui2].Pos.intD - AxisP[ui2].workpos0.intD;
					if(sii1<3 && sii1>-3)
					{
						AxisV[ui2].Errcode =0x06;
					}
				}
				break;
			default:
				break;
		}
		
		/*
		if((StepRunFlag ==1) && (AxisV[ui2].RunStep&1)==0)
			ContinueRun = 0;
		*/
	}
}

void MixCtrl()
{
		switch(AxisV[MOTOR_Mix].RunStep)
		{
			case 0://����,δ�ҵ��ο���
			case 16://����,�ҵ��ο���
			case 64://���У�����ִ�����
			{
				mixFlag =0;
			};break;
			//����۵���λ
			case 18:
			{
				if(ContinueRun)
				{
					if((AxisV[MOTOR_Mix].Xin&4) !=0)
					{
						mixFlag =1;
						sii1 =AxisP[MOTOR_Mix].workpos1.intD;
						PositioningStart(MOTOR_Mix);
						AxisV[MOTOR_Mix].RunStep ++;
					}
					else
					{
						AxisV[MOTOR_Mix].RunStep =0;
					}					
				}
			};break;
			//����������˶�����λ
			case 19:
			{
				if((AxisV[MOTOR_Mix].Yout &0x40) ==0)
				{
					AxisV[MOTOR_Mix].RunStep ++;
				}
			};break;
			//��ͨ1��ˮ��1
			case 20:
			{
				tmpRelay[1] =0x01;
				AxisV[MOTOR_Mix].RunStep ++;				
			};break;
			case 21:
			{
				AxisV[MOTOR_Mix].RunStep ++;					
			};break;
			//��ˮ��1
			case 22:
			{
				if(ContinueRun)
				{
					tmpRelay[0] =0x04;
					AxisV[MOTOR_Mix].RunStep ++;					
				}
			};break;
			//ˮ��1������Һ
			case 23:
			{
				if(RelayOffCnt[SHUIBENG1] ==0)
				{				
					AxisV[MOTOR_Mix].RunStep ++;
				}				
			};break;
			//��ͨ1�����ã�������
			case 24:
			{
				if(ContinueRun)
				{
					tmpRelay[0] =0x03;
					AxisV[MOTOR_Mix].RunStep ++;						
				}			
			};break;
			//�������ڴ�������
			case 25:
			{
				if(RelayOffCnt[QIBENG1] ==0)
				{
					AxisV[MOTOR_Mix].RunStep ++;
				}
			};break;
			//��ͨ1��ˮ��
			case 26:
			{
				tmpRelay[1] =0x01;
				AxisV[MOTOR_Mix].RunStep ++;
			};break;
			case 27:
			{
				AxisV[MOTOR_Mix].RunStep ++;				
			};break;
			//��ˮ��1
			case 28:
			{
				if(ContinueRun)
				{
					tmpRelay[0] =0x04;
					AxisV[MOTOR_Mix].RunStep ++;					
				}
			};break;
			//ˮ��1������Һ			
			case 29:
			{
				if(RelayOffCnt[SHUIBENG1] ==0)
				{				
					AxisV[MOTOR_Mix].RunStep ++;
				}					
			};break;	
			//��ͨ1�����ã�������
			case 30:
			{
				if(ContinueRun)
				{
					tmpRelay[0] =0x03;
					AxisV[MOTOR_Mix].RunStep ++;						
				}			
			};break;
			case 31:
			{
				if(RelayOffCnt[QIBENG1] ==0)
				{
					AxisV[MOTOR_Mix].RunStep ++;
				}
			};break;
			//����۵���λ
			case 32:
			{
				if(ContinueRun)
				{
					sii1 =AxisP[MOTOR_Mix].workpos0.intD;
					PositioningStart(MOTOR_Mix);
					AxisV[MOTOR_Mix].RunStep ++;					
				}
			};break;
			//��������ڵ���λ
			case 33:
			{
				if((AxisV[MOTOR_Mix].Yout &0x40) ==0)
				{
					AxisV[MOTOR_Mix].RunStep =64;
				}
			};break;
		}
		//Ӧ�ڴ�������������ʱ���ƺ�ż���������ơ�20170205tt
		if(AxisV[MOTOR_Mix].RunStep&1)
		{
			//��ʱ����???
		}
		else
		{
			if(StepRunFlag ==1)
			{
				if((AxisV[MOTOR_Mix].RunStep >=18) && (AxisV[MOTOR_Mix].RunStep <64))
				{
					ContinueRun = 0;						
				}	
			}
		}
}

void SampleCtrl()
{
	switch(AxisV[MOTOR_PUMP].RunStep)
	{
		case 0:
		case 16:
		case 64:
		{
			sampleFlag =0;
		};break;
		//ȡ����Z����λ
		case 18:
		{
			if(ContinueRun)
			{
				if((AxisV[MOTOR_ARM_Z].Xin &AxisV[MOTOR_ARM_C].Xin &AxisV[MOTOR_PUMP].Xin &AxisV[MOTOR_PLATE].Xin &4) !=0)
				{
					sampleFlag =1;
					if(AxisV[MOTOR_ARM_Z].Pos.intD ==AxisP[MOTOR_ARM_Z].workpos0.intD)
					{
						AxisV[MOTOR_PUMP].RunStep = AxisV[MOTOR_PUMP].RunStep + 2;
					}
					else
					{
						sii1 =AxisP[MOTOR_ARM_Z].workpos0.intD;
						PositioningStart(MOTOR_ARM_Z);
						AxisV[MOTOR_PUMP].RunStep ++;					
					}
				}
				else
				{
					AxisV[MOTOR_PUMP].RunStep =0;
				}				
			}
		};break;
		//ȡ����Z�����˶�����λ
		case 19:
		{
			if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;
			}
		};break;
		//ȡ����C����ȡλ
		case 20:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_ARM_C].workpos2.intD;
				PositioningStart(MOTOR_ARM_C);
				AxisV[MOTOR_PUMP].RunStep ++;
			}
		};break;
		//ȡ����C�����˶�����ȡλ
		case 21:
		{
			if((AxisV[MOTOR_ARM_C].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;				
			}
		};break;
		//�����ó�ȡ20ul��������ֹ��Ʒ����ϴҺϡ��
		case 22:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_PUMP].workpos0.intD;
				PositioningStart(MOTOR_PUMP);
				AxisV[MOTOR_PUMP].RunStep ++;				
			}
		};break;
		//���������ڳ�ȡ����
		case 23:
		{
			if((AxisV[MOTOR_PUMP].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;				
			}
		};break;
		//ȡ����Z���¹�λ
		case 24:
		{
			if(ContinueRun)
			{
				sii1 = AxisP[MOTOR_ARM_Z].workpos1.intD;
				PositioningStart(MOTOR_ARM_Z);
				AxisV[MOTOR_PUMP].RunStep ++;					
			}
		};break;
		//ȡ����Z�����˶����¹�λ
		case 25:
		{
			if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;	
			}
		};break;
		//�����ó�ȡ����λ
		case 26:
		{
			if(ContinueRun)
			{
				sii1 = AxisP[MOTOR_PUMP].workpos1.intD;
				PositioningStart(MOTOR_PUMP);		
				AxisV[MOTOR_PUMP].RunStep ++;
			}
		};break;
		//���������ڳ�Һ
		case 27:
		{
			if((AxisV[MOTOR_PUMP].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;		
			}
		};break;
		//ȡ����Z�˶�����λ
		case 28:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_ARM_Z].workpos0.intD;
				PositioningStart(MOTOR_ARM_Z);
				AxisV[MOTOR_PUMP].RunStep ++;					
			}
		};break;
		//ȡ����Z�����˶�����λ
		case 29:
		{
			if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;				
			}
		};break;
		//�̵���һ��ȡ��λ
		case 30:
		{
			if(ContinueRun)
			{
				sii1 =AxisV[MOTOR_PLATE].Pos.intD + AxisP[MOTOR_PLATE].workpos1.intD; //???�Ƿ������ۻ����???
				PositioningStart(MOTOR_PLATE);
				AxisV[MOTOR_PUMP].RunStep ++;	
			}		
		};break;
		//�����ڶ�λ
		case 31:
		{
			if((AxisV[MOTOR_PLATE].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;		
			}
		};break;
		//ȡ����C����λ
		case 32:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_ARM_C].workpos1.intD;
				PositioningStart(MOTOR_ARM_C);
				AxisV[MOTOR_PUMP].RunStep ++;					
			}
		};break;
		//ȡ����C�������е���λ
		case 33:
		{
			if((AxisV[MOTOR_ARM_C].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;		
			}
		};break;
		//ȡ����Z������λ
		case 34:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_ARM_Z].workpos2.intD;
				PositioningStart(MOTOR_ARM_Z);
				AxisV[MOTOR_PUMP].RunStep ++;					
			}
		};break;
		//ȡ����Z�����˶�������λ
		case 35:
		{
			if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;			
			}
		};break;
		//��������Һ����50ul
		case 36:
		{
			if(ContinueRun)
			{			
				sii1 =AxisP[MOTOR_PUMP].workpos2.intD;
				PositioningStart(MOTOR_PUMP);
				AxisV[MOTOR_PUMP].RunStep ++;		
			}
		};break;	
		//������������Һ		
		case 37:
		{
			if((AxisV[MOTOR_PUMP].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;			
			}
		};break;
		//�����ó�ȡ5ul����,��ֹ��Һ
		case 38:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_PUMP].workpos3.intD;
				PositioningStart(MOTOR_PUMP);
				AxisV[MOTOR_PUMP].RunStep ++;	
			}
		};break;
		//���������ڳ�ȡ����
		case 39:
		{
			if((AxisV[MOTOR_PUMP].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;			
			}
		};break;
		//ȡ����Z����λ
		case 40:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_ARM_Z].workpos0.intD;
				PositioningStart(MOTOR_ARM_Z);
				AxisV[MOTOR_PUMP].RunStep ++;	
			}
		};break;
		//ȡ����Z�����˶�����λ
		case 41:
		{
			if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep ++;		
			}			
		};break;
		//ȡ����C����ϴλ
		case 42:
		{
			if(ContinueRun)
			{
				sii1 =AxisP[MOTOR_ARM_C].workpos0.intD;
				PositioningStart(MOTOR_ARM_C);
				AxisV[MOTOR_PUMP].RunStep ++;	
			}
		};break;
		//ȡ����C���ڵ���ϴλ
		case 43:
		{
			if((AxisV[MOTOR_ARM_C].Yout &0x40) ==0)
			{
				AxisV[MOTOR_PUMP].RunStep =64;			
			}			
		};break;	
	}
	if(AxisV[MOTOR_PUMP].RunStep&1)
	{
		//��ʱ����???
	}
	else
	{
		if(StepRunFlag ==1)
		{
			if((AxisV[MOTOR_PUMP].RunStep >=18) && (AxisV[MOTOR_PUMP].RunStep <64))
			{
				ContinueRun = 0;						
			}				
		}
	}
}

void CleanCtrl()
{
		switch(AxisV[MOTOR_ARM_C].RunStep)
		{
			case 0:
			case 16:
			case 64:
			{
				cleanFlag =0;
			};break;
			case 18:
			{
				if((AxisV[MOTOR_ARM_Z].Xin &AxisV[MOTOR_ARM_C].Xin &AxisV[MOTOR_PUMP].Xin &AxisV[MOTOR_PLATE].Xin &4) !=0)
				{
					cleanFlag =1;
					sii1 =AxisP[MOTOR_ARM_Z].workpos0.intD;
					PositioningStart(MOTOR_ARM_Z);
					AxisV[MOTOR_ARM_C].RunStep ++;
				}
				else
				{
					AxisV[MOTOR_ARM_C].RunStep =0;
				}
			};break;
			case 19:
			{
				if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
				{
					sii1 =AxisP[MOTOR_ARM_C].workpos0.intD;
					PositioningStart(MOTOR_ARM_C);
					AxisV[MOTOR_ARM_C].RunStep ++;			
				}			
			};break;
			case 20:
			{
				if((AxisV[MOTOR_ARM_C].Yout &0x40) ==0)
				{
					sii1 =AxisP[MOTOR_ARM_Z].workpos1.intD;
					PositioningStart(MOTOR_ARM_Z);
					AxisV[MOTOR_ARM_C].RunStep ++;			
				}	
			};break;
			case 21:
			{
				if((AxisV[MOTOR_ARM_Z].Yout &0x40) ==0)
				{
					sii1 =0;
					PositioningStart(MOTOR_PUMP);
					tmpRelay[0] =0x80;
					AxisV[MOTOR_ARM_C].RunStep ++;	
				}
			};break;
			case 22:
			{
				if((AxisV[MOTOR_PUMP].Yout &0x40) ==0 && RelayOffCnt[SHUIBENG2] ==0)
				{
					AxisV[MOTOR_ARM_C].RunStep =64;
				}
			};break;			
		}
}







