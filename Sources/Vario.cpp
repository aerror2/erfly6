#include "Vario.h"

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