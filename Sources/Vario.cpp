#include "Vario.h"

#include "hal.h"
#include "voice.h"
#include <string.h>
#include <stdio.h>


#ifdef VARIO
typedef struct VarioBuff_t {
  uint32_t ToneTimeLeft;
  uint32_t TonePeriod;
  uint32_t ToneHalfPeriod;
  uint32_t ToneTimePause;
} VarioBuff_t;

const Vario_table VarioTab = {
/*Silence Zone
  Lo, Hi   (M/S*100)*/
{-60, 10},
/*Table
M/S*100,Hertz,Cycle,Duty*/
{
/*
 {-1000,200 ,200,100},
 {-300 ,290 ,200,100},
 {-200 ,360 ,200,100},
 {-100 ,440 ,200,100},
 {-50  ,470 ,600,100},
 {-0   ,490 ,600, 50},
 { 50  ,550 ,550, 50},
 { 100 ,590 ,500, 50},
 { 200 ,670 ,400, 50},
 { 300 ,740 ,310, 50},
 { 500 ,880 ,250, 50},
 { 1000,1100,200, 50}}
*/
 {-1000,288 ,650, 80},
 {-300 ,416 ,650, 80},
 {-200 ,448 ,650, 80},
 {-100 ,480 ,650, 80},
 {-50  ,496 ,650, 80},
 { 0   ,512 ,600, 50},
 { 50  ,550 ,600, 50},
 { 100 ,590 ,550, 50},
 { 200 ,670 ,400, 50},
 { 300 ,704 ,310, 50},
 { 500 ,880 ,250, 50},
 { 1000,1100,200, 50}}
/*
tone=-10.00,288,650,80
tone=-3.00,416,650,80
tone=-2.00,448,650,80
tone=-1.00,480,650,80
tone=-0.50,496,650,80
tone=0.00,512,600,50
tone=0.50,544,550,50
tone=1.00,576,500,50
tone=2.00,640,400,50
tone=3.00,704,310,50 
tone=5.00,832,240,50 
tone=10.0,1152,240,50
*/
};
/*------------------------------------------------------------------------------------------*/
static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*------------------------------------------------------------------------------------------*/
static void Vario_GetData(int32_t Vspeed, Vario_data *data) {
  uint8_t MinPoint;
  uint8_t MaxPoint;
  // Range check
  if (Vspeed < VarioTab.Data[0].MS)
    Vspeed = VarioTab.Data[0].MS;
  if (Vspeed > VarioTab.Data[11].MS)
    Vspeed = VarioTab.Data[11].MS;
  // Interpolation
  uint8_t i = 0;
  while (!(Vspeed <= VarioTab.Data[i].MS)) {
    i++;
  }
  if (Vspeed == VarioTab.Data[i].MS) {
    data->Hertz = VarioTab.Data[i].Hertz;
    data->Cycle = VarioTab.Data[i].Cycle;
    data->Duty = VarioTab.Data[i].Duty;
  } else {
    MinPoint = i - 1;
    MaxPoint = i;
    data->Hertz = map(Vspeed, VarioTab.Data[MinPoint].MS,
        VarioTab.Data[MaxPoint].MS, VarioTab.Data[MinPoint].Hertz,
        VarioTab.Data[MaxPoint].Hertz);
    data->Cycle = map(Vspeed, VarioTab.Data[MinPoint].MS,
        VarioTab.Data[MaxPoint].MS, VarioTab.Data[MinPoint].Cycle,
        VarioTab.Data[MaxPoint].Cycle);
    data->Duty = map(Vspeed, VarioTab.Data[MinPoint].MS,
        VarioTab.Data[MaxPoint].MS, VarioTab.Data[MinPoint].Duty,
        VarioTab.Data[MaxPoint].Duty);
  }
}
/*------------------------------------------------------------------------------------------*/
VarioBuff_t WorkBuff;
VarioBuff_t LoadBuff;
bool VarioPlay = false;
/*------------------------------------------------------------------------------------------*/
void Vario_play(int32_t Vspeed, bool play) {
  Vario_data data;
  if ((play == false) | ((Vspeed > VarioTab.SilenceZone.Lo) & (Vspeed < VarioTab.SilenceZone.Hi))) {
    VarioPlay = false;
    return;
  }
  VarioPlay = true;
  Vario_GetData(Vspeed, &data);
  LoadBuff.TonePeriod = 1000000 / 128 / data.Hertz;
  LoadBuff.ToneHalfPeriod = (LoadBuff.TonePeriod / 2);
  LoadBuff.ToneTimeLeft = data.Cycle * 1000 / 128;
  LoadBuff.ToneTimePause = LoadBuff.ToneTimeLeft - (LoadBuff.ToneTimeLeft * data.Duty / 100);
}
/*------------------------------------------------------------------------------------------*/
void Vario_driver(void) {
  static uint32_t toneCounter = 0;
 
  if ((WorkBuff.ToneTimeLeft == 0) & (VarioPlay)) {
    WorkBuff = LoadBuff;
  }
  if (WorkBuff.ToneTimeLeft > WorkBuff.ToneTimePause) {
    if (toneCounter == WorkBuff.ToneHalfPeriod) {
      Buzzer_ClrVal();
    } else
     if (toneCounter >= WorkBuff.TonePeriod) {
      toneCounter = 0;
      Buzzer_SetVal();
    }
   toneCounter++;
   } else {
    Buzzer_ClrVal();
  }
  if (WorkBuff.ToneTimeLeft)
    WorkBuff.ToneTimeLeft--;
}
/*------------------------------------------------------------------------------------------*/

#endif




void DoSum(uint8_t* Str, uint8_t len)
{
	uint16_t xorsum = 0;
	uint8_t i;

	for (i = 0; i < len; i++)
	{
		xorsum = xorsum + Str[i];
	}
	xorsum = 0 - xorsum;
	*(Str + i) = (uint8_t)(xorsum >> 8);
	*(Str + i + 1) = (uint8_t)(xorsum & 0x00ff);

	
}

uint8_t play_cmd_buf[] = {
0x7E,0xFF ,0x06 ,0x12 ,
0x00 ,0x00 ,0x01 ,0xFE ,
0xE8 ,0xEF };
/********************************************************************************************
 - 功能描述： 串口向外发送命令[包括控制和查询]
 - 隶属模块： 外部
 - 参数说明： CMD:表示控制指令，请查阅指令表，还包括查询的相关指令
			  feedback:是否需要应答[0:不需要应答，1:需要应答]
			  data:传送的参数
 - 返回说明：
 - 注：
********************************************************************************************/
int Uart_SendCMD(uint8_t CMD, uint8_t feedback, uint16_t dat)
{
	//DWORD numWritten = 0;
	//DWORD readsize;

	char buffer[10];
///	play_cmd_buf[1] = 0xff;    //保留字节 
	play_cmd_buf[2] = 0x06;    //长度
	play_cmd_buf[3] = CMD;     //控制指令
	play_cmd_buf[4] = feedback;//是否需要反馈
	play_cmd_buf[5] = (uint8_t)(dat >> 8);//datah
	play_cmd_buf[6] = (uint8_t)(dat);     //datal
	DoSum(&play_cmd_buf[1], 6);        //校验

        send_voice_cmd(play_cmd_buf,10);
//	SendCmd(8);       //发送此帧数据
	//if (WriteFile(g_hCom1, play_cmd_buf, 10, &numWritten, NULL))
	//{
	//	printf("numwrite %u\n", numWritten);
	
	//	{
	//		printf("%02X,%02X\n", play_cmd_buf[7], play_cmd_buf[8]);
	//		ReadFile(g_hCom1, buffer, 10, &readsize, NULL);
	//	}
	//	for (int i = 0; i < readsize; i++)
	//	{
	//		printf("%02x ", buffer[i]);
	//	}
	//	printf("\n");
	//}
	//else
	//{
	//	printf("write failed %x\n", GetLastError());
	//	return 0;
	//}

	return 1;
}
 

//void playmp3(uint8_t x)
//{
//	Uart_SendCMD(0x12, 0, x);
//}

#define playmp3(x) Uart_SendCMD(0x12, 0, x)

void play_voice(int category, int value,int nfrac,int unit)
{
	playmp3(category);
	
        /*
	char sznumber[25];
	int len = 0;
	if (nfrac > 0)
	{
		uint32_t expo = 1;
		for (int i = 0; i < nfrac; i++)
		{
			expo *= 10;
		}
		double fvalue = (double)value / (double)expo;
		//len = sprintf(sznumber,"%.1f", fvalue);
	}
	else
	{
		//len = sprintf(sznumber, "%d", value);
	}
	
	int number_decors[] =
	{
		voice_decro_number_ten ,
		voice_decro_number_hundred ,
		voice_decro_number_kilo ,
		voice_decro_number_wan ,
		voice_decro_number_ten ,
		voice_decro_number_hundred ,
		voice_decro_number_kilo,
		voice_decro_number_yi,
		voice_decro_number_ten ,
	};
	int numberstart = value<0 ? 1: 0;
	int numberend = len;

	char* szdot = strstr(sznumber, ".");
	if (szdot != NULL)
		numberend = szdot - sznumber;
	bool islastzero = false;
	for (int i = 0; i < len; i++)
	{
		char x = sznumber[i];
		if (x == '-')
		{
			playmp3(voice_decro_negative);
		}
		else if (x == '.')
		{
			playmp3(voice_decro_dot);
		}
		else if (x >= '0' && x <= '9')
		{
			if (!(x == '0' && !islastzero))
			{
				playmp3(x - '0' + 1);
			}
			
		
			if (i>=numberstart && i< numberend  && x!='0')
			{
				int y = numberend - i-2;
				if(y>=0)
					playmp3(number_decors[y]);
			}

			if (x == '0')
			{
				islastzero = true;
			}
			else
			{
				islastzero = false;
			}
		}  

		
	}

	if(unit>0)
		playmp3(unit);
                */
	
}