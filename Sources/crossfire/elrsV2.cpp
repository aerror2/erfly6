#include <stdio.h>
#include "crossfire.h"
#include "../er9x.h"
#include "../lcd.h"
#include "../menus.h"
#include "../en.h"
#include "../voice.h"
#include "tiny_string.cpp"

#define PACKED __attribute__((packed))
#define DUMP(...) {}

#define WARNING_LINE_LEN               20
#define WARNING_LINE_X                 16
#define WARNING_LINE_Y                 3*FH

struct FieldProps
{
  uint8_t nameOffset;     
  uint8_t nameLength;
  uint8_t valuesOffset;  
  uint8_t valuesLength;
  uint8_t parent;
  uint8_t type;
  uint8_t value;
  uint8_t id; 
} PACKED;

struct FieldFunctions
{
  void (*load)(FieldProps*, uint8_t *, uint8_t);
  void (*save)(FieldProps*);
  void (*display)(FieldProps*, uint8_t, uint8_t);
};

uint8_t badPkt=0;
uint32_t goodPkt=0;

/*FieldProps fields[25];
uint8_t curfieldId = 1;
uint8_t curselectIdx = 0;
uint8_t curFieldChunk =0;
uint8_t curNumSelection = 0;
static uint32_t fieldTimeout= 0;
static uint32_t linkstatTimeout = 0;
uint8_t fieldData[72]; 
uint8_t fieldDataLen = 0;
int8_t expectedChunks = -1;
uint8_t statusComplete = 0; 
char fields_count=0;
char allParamsLoaded=0;
bool itemmodified = false;
bool bMenuShown = false;*/

#define COL1_CHAR_LEN 12
#define COL2_CHAR_LEN 9
#define COL2           12*FW

static constexpr uint8_t maxLineIndex  =  6;
static constexpr uint8_t textXoffset   =  0;
static constexpr uint8_t textYoffset   =  3;
static constexpr uint8_t textSize      =  8;
static uint8_t reusableBuffer[1200];

static constexpr uint8_t RESULT_OK = 2;
static constexpr uint8_t RESULT_CANCEL = 1;

static constexpr uint8_t NAMES_BUFFER_SIZE  = 192;
static constexpr uint8_t VALUES_BUFFER_SIZE = 176;
static uint8_t *namesBuffer = reusableBuffer;
uint8_t namesBufferOffset = 0;
static uint8_t *valuesBuffer = &reusableBuffer[NAMES_BUFFER_SIZE];
uint8_t valuesBufferOffset = 0;

static constexpr uint8_t FIELD_DATA_MAX_LEN = (512 - NAMES_BUFFER_SIZE - VALUES_BUFFER_SIZE);
static uint8_t *fieldData = &reusableBuffer[NAMES_BUFFER_SIZE + VALUES_BUFFER_SIZE];
uint8_t fieldDataLen = 0;

static constexpr uint8_t FIELDS_MAX_COUNT = 32;
static FieldProps fields[FIELDS_MAX_COUNT];
uint8_t fieldsLen = 0;

static constexpr uint8_t DEVICES_MAX_COUNT = 8;
static uint8_t deviceIds[DEVICES_MAX_COUNT];
uint8_t devicesLen = 0;
uint8_t otherDevicesId = 255;
uint8_t deviceId = 0xEE;
uint8_t handsetId = 0xEF;
uint8_t deviceIsELRS_TX = 0;

static constexpr uint8_t DEVICE_NAME_MAX_LEN = 20;
static char deviceName[DEVICE_NAME_MAX_LEN];
uint8_t lineIndex = 1;
uint8_t pageOffset = 0;
uint8_t edit = 0; 
uint8_t charIndex = 1;
static FieldProps * fieldPopup = 0;
tmr10ms_t fieldTimeout = 0; 
uint8_t fieldId = 1;
uint8_t fieldChunk = 0;

uint8_t fields_count = 0;
uint8_t backButtonId = 2; 
tmr10ms_t devicesRefreshTimeout = 50; 
uint8_t allParamsLoaded = 0; 
uint8_t folderAccess = 0; 
uint8_t statusComplete = 0; 
int8_t expectedChunks = -1;
tmr10ms_t linkstatTimeout = 100;
uint8_t reloadFolder = 0;

#define tostring(c)       (char)(c + 48)
#define getTime           get_tmr10ms
#define EVT_VIRTUAL_EXIT  EVT_KEY_BREAK(KEY_EXIT)
#define EVT_VIRTUAL_ENTER EVT_KEY_BREAK(KEY_MENU)
#define EVT_VIRTUAL_NEXT  EVT_KEY_FIRST(KEY_UP)
#define EVT_VIRTUAL_PREV  EVT_KEY_FIRST(KEY_DOWN)

#define  lcdDrawText lcd_putsAtt
#define lcdDrawSizedText lcd_putsnAtt  

void crossfireTelemetryPush4(const uint8_t cmd, const uint8_t third, const uint8_t fourth)
{
  uint8_t crsfPushData[4] =  { deviceId, handsetId, third, fourth };
  crossfireTelemetryPush(cmd, crsfPushData, 4);
}

void crossfireTelemetryPing()
{
  uint8_t crsfPushData[2] = { 0x00, 0xEA };
  crossfireTelemetryPush(0x28, crsfPushData, 2);
}

void allocateFields()
{
  fieldsLen = fields_count + 2U;
  TRACE("allocateFields: len %d", fieldsLen);
  for (uint32_t i = 0; i < fieldsLen; i++)
  {
    fields[i].nameLength = 0;
    fields[i].valuesLength = 0;
  }
  backButtonId = fieldsLen - 1;
  TRACE("add back btn at %d", backButtonId);
  fields[backButtonId].id = backButtonId + 1;
  fields[backButtonId].nameLength = 1; 
  fields[backButtonId].type = 14;
  fields[backButtonId].parent = (folderAccess == 0) ? 255 : folderAccess;
}

void reloadAllField()
{
  allParamsLoaded = 0;
  fieldId = 1;
  fieldChunk = 0;
  fieldDataLen = 0; 
  namesBufferOffset = 0;
  valuesBufferOffset = 0;
}

static FieldProps * getField(const uint8_t line)
{
  uint32_t counter = 1;
  for (uint32_t i = 0; i < fieldsLen; i++)
  {
    FieldProps * field = &fields[i];
    if (folderAccess == field->parent && field->nameLength != 0)
    {
      if (counter < line)
      {
        counter = counter + 1;
      }
      else
      {
        return field;
      }
    }
  }
  return nullptr;
}

static uint8_t getSemicolonCount(const char * str, const uint8_t len)
{
  uint8_t count = 0;
  for (uint32_t i = 0; i < len; i++)
  {
    if (str[i] == ';') count++;
  }
  return count;
}

static void incrField(int8_t step)
{
  FieldProps * field = getField(lineIndex);
  uint8_t min = 0, max = 0;
  if (field->type == 9)
  {
    max = getSemicolonCount((char *)&valuesBuffer[field->valuesOffset], field->valuesLength); 
  }
  field->value = limit<uint8_t>(min, field->value + step, max);
}

void selectField(int8_t step)
{
  int8_t newLineIndex = lineIndex;
  FieldProps * field;
  do
  {
    newLineIndex = newLineIndex + step;
    if (newLineIndex <= 0)
    {
      newLineIndex = fieldsLen - 1;
    }
    else if (newLineIndex == 1 + fieldsLen)
    {
      newLineIndex = 1;
      pageOffset = 0;
    }
    field = getField(newLineIndex);
  }
  while (newLineIndex != lineIndex && (field == 0 || field->nameLength == 0));
  lineIndex = newLineIndex;
  if (lineIndex > maxLineIndex + pageOffset)
  {
    pageOffset = lineIndex - maxLineIndex;
  }
  else if (lineIndex <= pageOffset)
  {
    pageOffset = lineIndex - 1;
  }
}

static uint8_t getDevice(uint8_t devId)
{
  TRACE("getDevice %x", devId);
  for (uint8_t i = 0; i < devicesLen; i++)
  {
    if (deviceIds[i] == devId)
    {
      return deviceIds[i];
    }
  }
  return 0;
}

static uint8_t strRemove(char * src, const char * str, const uint8_t len)
{
  const char strLen = strlen(str);
  char * srcStrPtr = src;
  uint8_t removedLen = 0;
  while ((srcStrPtr = strstr(srcStrPtr, str)) && (srcStrPtr < src + len))
  {
    memcpy(srcStrPtr, srcStrPtr + strLen, (src + len) - (srcStrPtr + strLen));
    removedLen += strLen;
  }
  return removedLen;
}

void fieldTextSelectionLoad(FieldProps * field, uint8_t * data, uint8_t offset)
{
  uint8_t len = strlen((char*)&data[offset]);
  field->value = data[offset + len + 1];
  len -= strRemove((char*)&data[offset], "UX", len);
  if (field->valuesLength == 0)
  {
    memcpy(&valuesBuffer[valuesBufferOffset], (char*)&data[offset], len);
    field->valuesOffset = valuesBufferOffset;
    field->valuesLength = len;
    valuesBufferOffset += len;
  }
}

void fieldTextSelectionSave(FieldProps * field)
{
  crossfireTelemetryPush4(0x2D, field->id, field->value);
}

static uint8_t semicolonPos(const char * str, uint8_t last)
{
  uint8_t pos = 0;
  while ((str[pos] != ';') && (pos < last)) pos++;
  return pos + 1;
}

void fieldTextSelectionDisplay(FieldProps * field, uint8_t y, uint8_t attr)
{
  uint8_t start = field->valuesOffset;
  uint8_t len;
  uint32_t i = 0;
  while (i++ < field->value)
  {
    start += semicolonPos((char *)&valuesBuffer[start], field->valuesLength - (start - field->valuesOffset));
    if (start - field->valuesOffset >= field->valuesLength)
    {
      lcdDrawText(COL2, y, "ERR", attr);
      return;
    }
  }
  len = semicolonPos((char *)&valuesBuffer[start], field->valuesLength - (start - field->valuesOffset)) - 1;
  lcdDrawSizedText(COL2, y, (char *)&valuesBuffer[start], len , attr);
}

void fieldStringDisplay(FieldProps * field, uint8_t y, uint8_t attr)
{
  /*const char* backPat = "[Back]";
  const char* folderPat = "> %s";
  const char* cmdPat = "[%s]";
  const char *pat;
  uint8_t textIndent = textXoffset + 9;
  if (field->type == 11)
  {
    pat = folderPat;
    textIndent = textXoffset;
  } 
  else if (field->type == 14)
  {
    pat = backPat;
  } 
  else if (field->type == 15)
  {
    pat = cmdPat;
  }
  char stringTmp[24];
  tiny_sprintf((char *)&stringTmp, pat, field->nameLength, 1, (char *)&namesBuffer[field->nameOffset]);
  lcdDrawText(textIndent, y, (char *)&stringTmp, attr);
  if (field->type == 9)
  {*/
    lcdDrawSizedText(COL2, y, (char *)&valuesBuffer[field->valuesOffset], field->valuesLength, attr);
  //}
}

void fieldFolderOpen(FieldProps * field)
{
  TRACE("fieldFolderOpen %d", field->id);
  lineIndex = 1;
  pageOffset = 0;
  folderAccess = field->id;
  fields[backButtonId].parent = folderAccess;
  for (uint32_t i = 0; i < backButtonId; i++)
  {
    fields[i].valuesLength = 0;
  }
  reloadAllField();
}

void fieldFolderDeviceOpen(FieldProps * field)
{
  fields_count = devicesLen;
  devicesLen = 0;
  fieldsLen = 0;
  crossfireTelemetryPing();
  return fieldFolderOpen(field);
}

void noopOpen(FieldProps * field)
{
}

void fieldCommandLoad(FieldProps * field, uint8_t * data, uint8_t offset)
{
  field->value = data[offset];
  field->valuesOffset = data[offset+1]; 
  strcpy((char *)&fieldData[FIELD_DATA_MAX_LEN - 24 - 1], (char *)&data[offset+2]); 
  if (field->value == 0)
  { 
    fieldPopup = 0; 
  }
}

void fieldCommandSave(FieldProps * field)
{
  if (field->value < 4)
  {
    field->value = 1; 
    fieldTextSelectionSave(field);
    fieldPopup = field;
    fieldPopup->valuesLength = 0;
    fieldTimeout = getTime() + field->valuesOffset;
  }
}

void fieldUnifiedDisplay(FieldProps * field, uint8_t y, uint8_t attr)
{
  const char* backPat = "[Back]";
  const char* folderPat = ">%s";
  const char* otherPat = ">Other Devices";
  const char* cmdPat = "[%s]";
  const char *pat;
  uint8_t textIndent = textXoffset;// + 9;
  if (field->type == 11)
  {
    pat = folderPat;
    textIndent = textXoffset;
  }
  else if (field->type == 16)
  {
    pat = otherPat;
    textIndent = textXoffset;
  }
  else if (field->type == 14)
  {
    pat = backPat;
  }
  else
  {
    pat = cmdPat;
  }
  char stringTmp[24];
  tiny_sprintf((char *)&stringTmp, pat, field->nameLength, 1, (char *)&namesBuffer[field->nameOffset]);
  lcdDrawText(textIndent, y, (char *)&stringTmp, attr);
}

void UIbackExec(FieldProps * field = 0)
{
  folderAccess = 0;
  fields[backButtonId].parent = 255;
  for (uint32_t i = 0; i < backButtonId; i++)
  {
    fields[i].valuesLength = 0;
  }
  reloadAllField();
  devicesLen = 0;
  fields_count = 0;
}

/*void changeDeviceId(uint8_t newdevId)
{
  TRACE("changeDeviceId %x", newdevId);
  folderAccess = 0;
  deviceIsELRS_TX = 0;
  if (newdevId == 0xEE)
  {
    handsetId = 0xEF;
  }
  else
  {
    handsetId = 0xEA;
  }
  deviceId = newdevId;
  fields_count = 0;
}

void fieldDeviceIdSelect(FieldProps * field)
{
  changeDeviceId(field->id);
  crossfireTelemetryPing();
}

void createDeviceFields()
{
  TRACE("createDeviceFields %d", devicesLen);
  fields[fields_count + 2].id = fields[backButtonId].id;
  fields[fields_count + 2].nameLength = fields[backButtonId].nameLength;
  fields[fields_count + 2].type = fields[backButtonId].type;
  fields[fields_count + 2].parent = fields[backButtonId].parent;
  backButtonId = fields_count + 2;
  fieldsLen = fields_count + 2 + 1;
}*/

void parseDeviceInfoMessage(uint8_t* data)
{
  uint8_t offset;
  uint8_t id = data[2];
  offset = strlen((char*)&data[3]) + 1 + 3;
  uint8_t devId = getDevice(id);
  if (!devId)
  {
    deviceIds[devicesLen] = id;
    if (folderAccess == otherDevicesId)
    {
      fields[devicesLen].id = id;
      fields[devicesLen].type = 15;
      fields[devicesLen].nameLength = offset - 4;
      fields[devicesLen].nameOffset = namesBufferOffset;
      memcpy(&namesBuffer[namesBufferOffset], &data[3], fields[devicesLen].nameLength);
      namesBufferOffset += fields[devicesLen].nameLength;
      if (fields[devicesLen].id == deviceId)
      {
        fields[devicesLen].parent = 255;
      }
      else
      {
        fields[devicesLen].parent = otherDevicesId;
      }
      if (devicesLen == fields_count - 1)
      {
        allParamsLoaded = 1;
        fieldId = 1;
        //createDeviceFields();
      }
    }
    devicesLen++;
  }
  if (deviceId == id && folderAccess != otherDevicesId)
  {
    memcpy(deviceName, (char *)&data[3], DEVICE_NAME_MAX_LEN);
    deviceIsELRS_TX = ((memcmp(&data[offset], "ELRS", 4) == 0) && (deviceId == 0xEE)) ? 1 : 0;
    uint8_t newFieldCount = data[offset+12];
    reloadAllField();
    if (newFieldCount != fields_count || newFieldCount == 0)
    {
      fields_count = newFieldCount;
      allocateFields();
      otherDevicesId = fields_count+0+1;
      fields[fields_count+0].id = otherDevicesId;
      fields[fields_count+0].nameLength = 1;
      fields[fields_count+0].parent = 255;
      fields[fields_count+0].type = 16;
      if (newFieldCount == 0)
      {
        allParamsLoaded = 1;
        fieldId = 1;
        //createDeviceFields();
      }
    }
  }
}

static const FieldFunctions functions[] =
{
  { .load=fieldTextSelectionLoad, .save=fieldTextSelectionSave, .display=fieldTextSelectionDisplay },
  { .load=nullptr, .save=noopOpen, .display=fieldStringDisplay },
  { .load=nullptr, .save=fieldFolderOpen, .display=fieldUnifiedDisplay },
  { .load=fieldTextSelectionLoad, .save=noopOpen, .display=fieldStringDisplay },
  { .load=fieldCommandLoad, .save=fieldCommandSave, .display=fieldUnifiedDisplay },
  { .load=nullptr, .save=UIbackExec, .display=fieldUnifiedDisplay },
  //{ .load=nullptr, .save=fieldDeviceIdSelect, .display=fieldUnifiedDisplay },
  //{ .load=nullptr, .save=fieldFolderDeviceOpen, .display=fieldUnifiedDisplay }
};

void parseParameterInfoMessage(uint8_t* data, uint8_t length)
{
  if (data[2] != deviceId || data[3] != fieldId)
  {
    fieldDataLen = 0; 
    fieldChunk = 0;
    return;
  }
  if (fieldDataLen == 0)
  {
    expectedChunks = -1;
  }
  if (fieldId == reloadFolder)
  {
    reloadFolder = 0;
  }
  FieldProps* field = &fields[fieldId - 1];
  uint8_t chunks = data[4];
  if (field == 0 || (chunks != expectedChunks && expectedChunks != -1))
  {
    return; 
  }
  expectedChunks = chunks - 1;
  for (uint32_t i = 5; i < length; i++)
  {
    fieldData[fieldDataLen++] = data[i];
  }
  TRACE("length %d", length);
  if (chunks > 0)
  {
    fieldChunk = fieldChunk + 1;
    statusComplete = 0;
  }
  else
  {
    TRACE("%d, %s, %d", fieldId, &fieldData[2], fieldDataLen);
    DUMP(fieldData, fieldDataLen);
    fieldChunk = 0;
    if (fieldDataLen < 4)
    { 
      fieldDataLen = 0; 
      return; 
    }
    field->id = fieldId;
    uint8_t parent = fieldData[0]; 
    uint8_t type = fieldData[1] & 0x7F;
    uint8_t hidden = (fieldData[1] & 0x80) ? 1 : 0; 
    uint8_t offset;
    if (field->nameLength != 0)
    {
      if (field->parent != parent || field->type != type)
      {
        fieldDataLen = 0; 
        return; 
      }
    }
    field->parent = parent;
    field->type = type;
    offset = strlen((char*)&fieldData[2]) + 1 + 2;
    if (parent != folderAccess || type < 9)
    {
      field->nameLength = 0;
    }
    else
    {
      if (field->nameLength == 0 && !hidden)
      {
        field->nameLength = offset - 3;
        field->nameOffset = namesBufferOffset;
        memcpy(&namesBuffer[namesBufferOffset], &fieldData[2], field->nameLength); 
        namesBufferOffset += field->nameLength;
      }
      if (field->type >= 9 && functions[field->type - 9].load)
      {
        functions[field->type - 9].load(field, fieldData, offset);
      }
    }
    if (fieldPopup == 0)
    { 
      if (fieldId == fields_count)
      {
        TRACE("namesBufferOffset %d", namesBufferOffset);
        DUMP(namesBuffer, NAMES_BUFFER_SIZE);
        TRACE("valuesBufferOffset %d", valuesBufferOffset);
        DUMP(valuesBuffer, VALUES_BUFFER_SIZE);
        allParamsLoaded = 1;
        fieldId = 1;
        //createDeviceFields();
      }
      else if (allParamsLoaded == 0)
      {
        fieldId++;
      }
      else if (reloadFolder != 0)
      {
        fieldId = reloadFolder;
        fieldChunk = 0;
        statusComplete = 0;
      }
      fieldTimeout = getTime() + 200;
    }
    else
    {
      fieldTimeout = getTime() + fieldPopup->valuesOffset; 
    }
    if (reloadFolder == 0)
    {
      statusComplete = 1;
    }
    fieldDataLen = 0; 
  }
}

void runCrossfireTelemetryCallback(uint8_t command = 0, uint8_t* data = 0, uint8_t length = 0)
{
  if (command == 0x29)
  {
    parseDeviceInfoMessage(data);
  }
  else if (command == 0x2B && folderAccess != otherDevicesId)
  {
    parseParameterInfoMessage(data, length);
    if (allParamsLoaded < 1 || statusComplete == 0)
    {
      fieldTimeout = 0; 
    }
  }
  tmr10ms_t time = getTime();
  if (fieldPopup != 0)
  {
    if (time > fieldTimeout && fieldPopup->value != 3)
    {
      crossfireTelemetryPush4(0x2D, fieldPopup->id, 6); 
      fieldTimeout = time + fieldPopup->valuesOffset; 
    }
  } 
  else if (time > devicesRefreshTimeout && fields_count < 1)
  {
    devicesRefreshTimeout = time + 100; 
    crossfireTelemetryPing(); 
  }
  else if (time > fieldTimeout && fields_count != 0 && !edit)
  {
    if (allParamsLoaded < 1 || statusComplete == 0)
    {
      crossfireTelemetryPush4(0x2C, fieldId, fieldChunk); 
      fieldTimeout = time + 50;
    }
  }
}

void handleDevicePageEvent(uint8_t event)
{
  if (fieldsLen == 0)
  { 
    return;
  }
  else
  {
    if (fields[backButtonId].nameLength == 0)
    { 
      return;
    }
  }
  if (event == EVT_VIRTUAL_EXIT)
  {
    if (edit)
    {
      edit = 0;
      FieldProps * field = getField(lineIndex);
      fieldTimeout = getTime() + 200; 
      fieldId = field->id;
      fieldChunk = 0;
      fieldDataLen = 0; 
      crossfireTelemetryPush4(0x2C, fieldId, fieldChunk); 
    }
    else
    {
      if (folderAccess == 0 && allParamsLoaded == 1)
      {
        //if (deviceId != 0xEE)
        //{
          //changeDeviceId(0xEE);
        //}
        //else 
        //{
          //reloadAllField();
        //}
        crossfireTelemetryPing();
      }
      UIbackExec();
    }
  }
  else if (event == EVT_VIRTUAL_ENTER)
  {
    FieldProps * field = getField(lineIndex);
    if (field != 0 && field->nameLength > 0 && field->type >= 9)
    {
      if (field->type == 10)
      {}
      else if (field->type < 11)
      {
        edit = 1 - edit;
      }
      if (!edit)
      {
        if (field->type < 11 || field->type == 13)
        {
          fieldTimeout = getTime() + 20;
          fieldId = field->id;
          fieldChunk = 0;
          statusComplete = 0;
          if (field->parent)
          {
            reloadFolder = field->parent;
            fields[field->parent - 1].nameLength = 0;
          }
          fieldDataLen = 0;
        }
        functions[field->type - 9].save(field);
      }
    }
  }
  else if (edit)
  {
    if (event == EVT_VIRTUAL_NEXT)
    {
      incrField(1);
    }
    else if (event == EVT_VIRTUAL_PREV)
    {
      incrField(-1);
    }
  }
  else
  {
    if (event == EVT_VIRTUAL_NEXT)
    {
      selectField(-1);
    }
    else if (event == EVT_VIRTUAL_PREV)
    {
      selectField(1);
    }
  }
}

void runDevicePage(uint8_t event)
{
  handleDevicePageEvent(event);
  FieldProps * field;
  if (devicesLen > 1)
  {
    fields[fields_count+0].parent = 0;
  }
  for (uint32_t y = 1; y < maxLineIndex+2; y++)
  {
    if (pageOffset+y >= fieldsLen) break;
    field = getField(pageOffset+y);
    if (field == 0)
    {
      break;
    }
    else if (field->nameLength > 0)
    {
      uint8_t attr = (lineIndex == (pageOffset+y)) ? ((edit && BLINK) + INVERS) : 0;
      if (field->type < 11 or field->type == 12)
      {
        lcdDrawSizedText(textXoffset, y*textSize+textYoffset, (char *)&namesBuffer[field->nameOffset], field->nameLength, 0);
      }
      if (field->type >= 9 && functions[field->type - 9].display)
      {
        functions[field->type - 9].display(field, y*textSize+textYoffset, attr);
      }
    }
  }
}

static uint8_t popupCompat(uint8_t event)
{
  //showMessageBox((char *)&fieldData[FIELD_DATA_MAX_LEN - 24 - 1]);
  lcdDrawText(0, WARNING_LINE_Y, ((char *)&fieldData[FIELD_DATA_MAX_LEN - 24 - 1]), 0);
  //lcdDrawText(0, WARNING_LINE_Y+2*FH, STR_YES_NO_MENU_EXIT, 0);
  if (event == EVT_VIRTUAL_EXIT) {
    return RESULT_CANCEL; 
  }
  else if (event == EVT_VIRTUAL_ENTER)
  {
    return RESULT_OK; 
  }
  return 0; 
}

void runPopupPage(uint8_t event)
{
  if (event == EVT_VIRTUAL_EXIT)
  {
    crossfireTelemetryPush4(0x2D, fieldPopup->id, 5);
    fieldTimeout = getTime() + 200; 
  }
  uint8_t result = 0;
  if (fieldPopup->value == 0 && fieldPopup->valuesLength != 0)
  {
    popupCompat(event);
    reloadAllField();
    fieldPopup = 0;
  }
  else if (fieldPopup->value == 3)
  {
    result = popupCompat(event);
    if (fieldPopup != 0)
    {
      fieldPopup->valuesLength = fieldPopup->value;
    }
    if (result == RESULT_OK)
    {
      crossfireTelemetryPush4(0x2D, fieldPopup->id, 4); 
      fieldTimeout = getTime() + fieldPopup->valuesOffset; 
      fieldPopup->value = 4; 
    }
    else if (result == RESULT_CANCEL)
    {
      fieldPopup = 0;
    }
  }
  else if (fieldPopup->value == 2)
  { 
    result = popupCompat(event);
    if (fieldPopup != 0)
    {
      fieldPopup->valuesLength = fieldPopup->value;
    }
    if (result == RESULT_CANCEL)
    {
      crossfireTelemetryPush4(0x2D, fieldPopup->id, 5); 
      fieldTimeout = getTime() + fieldPopup->valuesOffset; 
      fieldPopup = 0;
    }
  }
}

void ELRSV2_stop()
{
  //runCrossfireTelemetryCallback(nullptr);
  UIbackExec(); 
  fieldPopup = 0;
  deviceId = 0xEE;
  handsetId = 0xEF;
  memset(reusableBuffer, 0, 512);
  popMenu(false);
}

void ELRSV2_run(uint8_t event)
{
  lcd_puts_Pleft(0, "ELRS LUA");
  if (event == EVT_KEY_LONG(KEY_EXIT))
  {
    ELRSV2_stop();
  }
  else
  {
    if (fieldPopup != 0)
    {
      runPopupPage(event);
    }
    else
    {
      runDevicePage(event);
    }
    runCrossfireTelemetryCallback();
  }
}

void crossfileMenu(MState2 &mstate2,uint8_t  event, uint8_t sub,  uint8_t subN, uint8_t y )
{
  mstate2.check_columns(event, 1);
  lcd_puts_Pleft(y, "[Load ELRS]");
  if (sub == subN)
  {
    lcd_char_inverse(0, y, 65, 0);
    if (event == EVT_KEY_FIRST(KEY_MENU))
    {
      pushMenu(ELRSV2_run);
    }
  }
}
