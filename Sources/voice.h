#ifndef _VOICE_H_
#define _VOICE_H_

/* voice card related stuffs */

// voice queue masks
#define VQ_FNMASK     0x3FFF  // filenum mask for 4 digit decimal (0000-9999)
#define VQ_CMDMASK    0xF000  // short command mask

// upto 12 commands (0x4000,..,0xF000) and each cmd can carry upto 12b data
#define VQ_VOLUME     0x4000  // volume command + volume (0-7)
#define VQ_CONFIG     0x5000  // config command + 8b config info
  #define VB_MEGASOUND  0x01  // serial MegaSound card installed
  #define VB_BACKLIGHT  0x02  // let XPB1 controls BACKLIGHT
  #define VB_TRIM_LV    0x04  // TRIM_ON_DATA?
  #define VB_RECEIVED   0x80  // received @cop328

// serial voice commands
// er9x to cop328
#define VCMD_PLAY       0x1F  // voice file playback command
#define VCMD_BACKLIGHT  0x1D  // backlight on-off command
#define VCMD_BOOTREASON 0x1B  // BootReason command
  #define VOP_BACKUP    0x1B  // BootReason: model backup-restore operation
  #define VOP_UNKNOWN   0x1C  // BootReason: unknown
#define VCMD_GOBOOT     0x30  // goto bootloader command
  #define VOP_GOBOOT    0x20  // operand of goto bootloader (VCMD_GOBOOT)

// cop328 to er9x
#define XCMD_STATUS     0x1F  // (MSB:Busy,XPD7,XPC0,XPB1,XPD4,XPD3,XPD2,XPB0)
#define XCMD_FEATURE    0x1E  // MegaSound firmware features (8b)
  #define VC_MBACKUP    0x01  // model backup feature
  #define VC_RTC        0x02  // RTC DS1302(CE:XPC1/XPB2,SCLK:XPC2,SIO:XPC3)
  #define VC_DATALOG    0x04  // data logging feature
  #define VC_LSLIDER    0x10  // left-slider (XPC0/XPC2)
  #define VC_RSLIDER    0x20  // right-slider (XPC1)
  #define VC_CHANGED    0x80  // feature changed @cop328
  #define VC_RECEIVED   0x80  // received @er9x



  
enum voice_category {
	
	voice_cat_rx_voltage=18,
	voice_cat_tx_voltage = 19,
	voice_cat_ground_speed=21,
	voice_cat_rssi = 22,
	voice_cat_v_speed = 23,
	voice_cat_alt = 24,
	voice_cat_gps_alt = 25,
	voice_cat_gps_lat = 26,
	voice_cat_gps_log = 27,
	voice_cat_long_time_no_op = 32,
	voice_cat_welcome = 33,
	voice_cat_roll=34,
	voice_cat_pitch=35,
	voice_cat_yaw = 36,
	
};
enum voice_number_decro
{
	voice_decro_dot = 16,
	voice_decro_low = 17,
	voice_decro_verylow = 19,
	voice_decro_negative = 37,
	voice_decro_number_ten=11,
	voice_decro_number_hundred=12,
	voice_decro_number_kilo =13,
	voice_decro_number_wan=14,
	voice_decro_number_yi=15,
	
};
enum voice_unit {
	voice_unit_meter = 28,
	voice_unit_kl_per_hours = 29,
	voice_unit_degree = 30,
	voice_unit_voltage = 31,
	
};
void play_voice(int category, int value,int nfrac,int unit);

#endif  // _VOICE_H_
