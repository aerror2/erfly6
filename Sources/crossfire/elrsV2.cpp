/**
 * ExpressLRS V2 lua configuration script port to C.
 * 
 * 
 * Not supported features:
 * - multiple devices, only ExpressLRS transmitters,
 * - no integer/float/string fields support, ExpressLRS uses only selection anyway,
 * - field unit ie.: "mW" is not displayed,
 * - info fields display only label without value
 */

#include <stdio.h>
#include "crossfire.h"

#include "../er9x.h"
#include "../lcd.h"
#include "../menus.h"
#include "../en.h"


struct FieldProps {
  uint16_t nameOffset;     
  uint8_t valuesLength;   
  uint16_t valuesOffset;   
  uint8_t nameLength;     
  uint8_t parent;
  uint8_t type : 4;       
  uint8_t value : 4; 
  uint8_t valueMax:4;       
  uint8_t id : 5;         
  uint8_t hidden : 1;     
  uint8_t spare : 2;     
} PACKED;

uint8_t badPkt=0;
uint32_t goodBadPkt=0;
uint8_t elrsFlags = 0;
char elrsFlagsInfo[16] = ""; 

FieldProps fields[25];
uint8_t curfieldId = 1;
uint8_t curFieldChunk =0;
uint8_t curNumSelection = 0;
static uint32_t fieldTimeout= 0;
static uint32_t linkstatTimeout = 0;
uint8_t fieldData[72]; 
uint8_t fieldDataLen = 0;
int8_t expectedChunks = -1;
uint8_t statusComplete = 0; 
//char elrs_deviceName[21];
char fields_count=0;
char allParamsLoaded=0;
bool bMenuShown = false;

#define COL1_CHAR_LEN 12
#define COL2_CHAR_LEN 9
  
#define COL2           12*FW

static uint8_t reusableBuffer[612];
//uint8_t *namesBuffer = reusableBuffer; 
static uint16_t reusableBufferOffset = 0;
//uint8_t *valuesBuffer = &reusableBuffer[256]; 
//uint8_t valuesBufferOffset = 0;

#define  lcdDrawText lcd_putsAtt
#define lcdDrawSizedText lcd_putsnAtt  


void crossfireTelemetryPush4(const uint8_t cmd, const uint8_t third, const uint8_t fourth) {
  //TRACE("crsf push %x", cmd);
  uint8_t crsfPushData[4] =  { deviceId, handsetId, third, fourth };
  crossfireTelemetryPush(cmd, crsfPushData, 4);
}


void crossfireTelemetryPing(){
  uint8_t crsfPushData[2] = { 0x00, 0xEA };
  crossfireTelemetryPush(0x28, crsfPushData, 2);
}
void parseDeviceInfoMessage(uint8_t* data) {
  uint8_t offset;
  uint8_t id = data[2];
 // TRACE("parseDeviceInfoMessage %x", id);
  offset = strlen((char*)&data[3]) + 1 + 3; 
  if ( deviceId == id) { 
  //   strcpy(elrs_deviceName, (char *)&data[3]);
    if(data[offset+12]>0)
    {
      if(fields_count==0)
      {
        crossfireTelemetryPush4(0x2C, curfieldId, curFieldChunk); 
        fieldTimeout   = g_tmr10ms+50;
      }
      fields_count = data[offset+12];
      
    }
     
  }
}



#define fieldTextSelectionSave( field)  crossfireTelemetryPush4(0x2D, field->id, field->value)




void parseParameterInfoMessage(uint8_t* data, uint8_t length)
{

 if (data[2] != deviceId || data[3] != curfieldId) {
    fieldDataLen = 0; 
    curFieldChunk = 0;
    return;
  }
  if (fieldDataLen == 0) {
    expectedChunks = -1;
  }
  FieldProps* field = &fields[curfieldId - 1];
  uint8_t chunks = data[4];
  if (field == 0 || (chunks != expectedChunks && expectedChunks != -1)) {
    return; 
  }
  expectedChunks = chunks - 1;
  for (uint32_t i=5; i< length; i++) {
    fieldData[fieldDataLen++] = data[i];
  }
  if (chunks > 0) {
    curFieldChunk ++;
    statusComplete = 0;
  } else { 
    curFieldChunk = 0;
    if (fieldDataLen < 4) { 
      fieldDataLen = 0; 
      return; 
    }

    field->id = curfieldId;
    uint8_t parent = fieldData[0]; 
    uint8_t type = fieldData[1] & 0x7F;
    uint8_t hidden = (fieldData[1] & 0x80) ? 1 : 0; 
    uint8_t offset;
    if (field->nameLength != 0) { 
      if (field->parent != parent || field->type != type || field->hidden != hidden) {
        fieldDataLen = 0; 
        return; 
      }
    }
    field->parent = parent;
    field->type = type;
    field->hidden = hidden;
    offset = strlen((char*)&fieldData[2]) + 1 + 2; 
    if (field->nameLength == 0) {  
      field->nameLength = offset - 3;
      if(field->nameLength > COL1_CHAR_LEN)
      {
         field->nameLength = COL1_CHAR_LEN;
      }
      field->nameOffset = reusableBufferOffset;
      memcpy(&reusableBuffer[reusableBufferOffset], &fieldData[2], field->nameLength); 
      reusableBufferOffset += field->nameLength;
    }
    if (field->type ==9 ) {

     
      
       
       int len=strlen((char*)&fieldData[offset]);
       if(field->valuesLength ==0)
       {
         field->valuesOffset = reusableBufferOffset;
          reusableBuffer[reusableBufferOffset++]=COL2_CHAR_LEN;

         int lastStart = 0;
        // int maxcharlen =COL2_CHAR_LEN;
         //for(int i=0;i<len;i++)
         // {
         //   if(fieldData[offset+i]==';'||i==len-1)
         //   {
         //       int flen = i-lastStart;
         //       if(len-1==i) flen +=1;
         //       if(flen>COL2_CHAR_LEN)
         //       {
         //         flen = COL2_CHAR_LEN;
         //         break;
         //       }
 
         //       if(flen > maxcharlen)
         //       {
         //         maxcharlen = flen;
         //       }
                 
         //   }
         // }

          for(int i=0;i<len;i++)
          { 

            if(fieldData[offset+i]==';'||i==len-1)
            {
                int flen = i-lastStart;
                if(len-1==i) flen +=1;
                if(flen>COL2_CHAR_LEN)
                {
                  flen = COL2_CHAR_LEN;
                }
                memcpy(&reusableBuffer[reusableBufferOffset], &fieldData[offset+lastStart], flen);
                reusableBufferOffset+=flen;
                for(int  j=flen;j<COL2_CHAR_LEN;j++)
                {
                    reusableBuffer[reusableBufferOffset++] = ' ';
                }
                lastStart =  i+1;
                field->valueMax++;
            }
          }
          reusableBuffer[reusableBufferOffset++]=0;
          field->valuesLength = reusableBufferOffset-field->valuesOffset;
        }
        offset += len + 1;
        field->value = fieldData[offset];

        curNumSelection ++;
    }

    curfieldId ++; //1 + (curfieldId % fields_count);
    if (curfieldId > fields_count) {

      allParamsLoaded = 1;
      curfieldId = 1;
    } 
  

    statusComplete = 1;
    fieldDataLen = 0; 
  }
}

void parseElrsInfoMessage(uint8_t* data)
{
 if (data[2] != deviceId) {
    fieldDataLen = 0; 
    curFieldChunk = 0;
    return;
  }

  uint8_t badPkt = data[3];
  uint16_t goodPkt = (data[4]*256) + data[5];
  elrsFlags = data[6];
  strcpy(elrsFlagsInfo, (char*)&data[7]); 
}

void runCrossfireTelemetryCallback(uint8_t command, uint8_t* data, uint8_t length) {

    //g_elrs_lag = (uint32_t)command;
   if(!bMenuShown) return;
  if (command == 0x29) {
    parseDeviceInfoMessage(data);
  } else if (command == 0x2B) {
    parseParameterInfoMessage(data, length);
    if (allParamsLoaded < 1 || statusComplete == 0) {
      fieldTimeout = 0; 
    }
  } else if (command == 0x2E) {
    parseElrsInfoMessage(data);
  }

}

void crossfileMenu(MState2 &mstate2,uint8_t  event, uint8_t sub,  uint8_t subN, uint8_t y )
{
    if(!bMenuShown)
    {
      bMenuShown  = true;
    }
    uint8_t attr =0;
    if( fields_count <=0)
    {
           mstate2.check_columns(event, 1);
         lcd_putsAtt(0,y,PSTR(STR_LOADING),0);
         /*
           lcd_outdezAtt(10 * FW,y,field_count,0);
          y += FH;
          lcd_outdezAtt(0 * FW,y,g_elrs_update_rate,0);
           y += FH;
          lcd_outdezAtt(0 * FW,y,g_elrs_lag,0);
         */
         if(linkstatTimeout < g_tmr10ms )
         {
            linkstatTimeout = g_tmr10ms + 100;
            crossfireTelemetryPing();
         }
    }
    else
    {

     //   lcd_outdezAtt(17 * FW,0,g_elrs_update_rate,0);
        lcd_outdezAtt(21 * FW,0,g_elrs_lag,0);


        if(!allParamsLoaded)
        { 
           if(fieldTimeout < g_tmr10ms )
           {
               fieldTimeout = g_tmr10ms + 5;
               crossfireTelemetryPush4(0x2C, curfieldId, curFieldChunk); 
           }
           if(linkstatTimeout  < g_tmr10ms )
           {
                linkstatTimeout = g_tmr10ms + 10;
                crossfireTelemetryPush4(0x2D, 0x0, 0x0);
           }
        }

        mstate2.check_columns(event, curNumSelection);
        int startDrawIndx = 0;
        int drawedNum = 0;
        int type9idx =0;
         if(sub >7)
         {
            startDrawIndx = sub-7;
         }
        int numfield =allParamsLoaded? fields_count:(curfieldId-1);
        for(int i=0;i<numfield;i++)
        {
          FieldProps *field = &fields[i];
          if(field->type ==9)
          {
              type9idx ++;
              if(type9idx > startDrawIndx && drawedNum < 7)
              {
                drawedNum ++;
                if (sub == subN) {
                    uint8_t modvalue =  checkIncDec_0(field->value,  field->valueMax-1);
                    attr = InverseBlink;
                    if(modvalue !=  field->value)
                    {
                        field->value  = modvalue;
                        fieldTextSelectionSave(field);
                    }
                }
                lcdDrawSizedText(0, y,(char *)&reusableBuffer[field->nameOffset], field->nameLength, 0);
                lcd_putsAttIdx(COL2, y, (char *)&reusableBuffer[field->valuesOffset], field->value, (sub == subN ? InverseBlink : 0));
                y += FH;
             
              } 
              
              attr = 0;
              subN ++;
             
          }
        }

     }
}


void resetElrsMenuVariables()
{

  curfieldId = 1;
  curFieldChunk =0;
  curNumSelection = 0;
  fieldTimeout= 0;
  linkstatTimeout = 0;
   fieldDataLen = 0;
   expectedChunks = -1;
   statusComplete = 0; 
  //char elrs_deviceName[21];
  fields_count=0;
  allParamsLoaded=0;
   reusableBufferOffset = 0;
   bMenuShown = false;
}