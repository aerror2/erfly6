
Here is a project mainly writting for flysky I6 to add two features for it:

1. crossfire for ELRS TX for FSI6
2. SPI for 127X for implement ELRS TX embeded for FSI6



This was forked from  https://KotelloRC/erfly6.git

Firstly ,thanks to Mr Kotello,  without his efforts  there simply would not have been this project .

Mr Kotello have done a great job and lot of work. 

Mr Kotello privided a lot help and advise while I was working on this project.


The codes of crossfire protocol is a porting version from https://github.com/OpenI6X/opentx
Thanks to Maria and Janek, they have done a great job for FSI6X.

The codes of ELRS is from https://github.com/ExpressLRS ,
Thanks to all the contributors of ElRS project also. :)



You can get help from https://www.rcgroups.com/forums/showthread.php?3961635-ER9X-for-FS-I6-and-FS-I6X-(ERFly6)

Here is the manual for flashing firmware from Kotello : https://github.com/aerror2/erfly6/blob/main/ER9XFlySky%20I6En.pdf

POTOCOL SUPPORTED CURRENTLY:
1. PPM
2. AFDHS
3. AFDHS2A
4. CROSSFIRE FOR ELRS TX 2.0


Here is original readme:


ErFly6 - new (good forgeted old) project porting er9X for family of radio FlySky FS-i6, FS-i6x.
Build firmware - Segger Embedded Sudio
Flash firmware via J-Link (clone) or ST-Link (for FS-i6X).
To build firmware install  https://www.segger.com/downloads/embedded-studio/

In File menu choose Open Solution, and Select FSI6.emProject from folder where it placed.

Choose Active Project which suitable for your radio - FSI6 or FSI6X. Build project.
If build successful flash firmware.

RF module A7105 only wit AFDHS 2A protocol. AFDHS protocol now not support.
Trainer mode (PPM in and PPM out) supported.


Self-made 2.4G ELRS TX tuner, built in Fuss i6


Production: easy

I. Hardware:

1. LoRa radio frequency module, sx1280: E28-2G4M27S

2. MCU Wifi module: ESP-WROOM32

3. Several silicone flexible cords of various colors

4.1k resistor

5. Fusi FS I6 remote control

6. One JLink, for flashing firmware

7. One 2.4G antenna

II.software:

1. EpressLRS Configurator  https://github.com/ExpressLRS/ExpressLRS-Configurator

2. The FS i6 firmware I wrote, ERFLY6,  GitHub-aerror2/erfly6: ER9X for I6 and I6X of Flysky

3. JLink-FLash,  J-Link Flash Download

III. TX circuit diagram:



IV. TX and FSI6 Transmitter wiring:

The completed TX only needs to connect 3 wires to the remote control, a signal wire (SPORT), a 3.3V power supply, and a ground wire, as shown in the figure below



V. Production steps:

1. By "III. TX Circuit Diagram" to connect E28-2G4M27S and ESP-WROOM32 with wires.

2. By "IV. TX and remote control wiring" to connect the three wires and fix them on the bottom plate with hot melt glue.

3. Use the USB cable to connect ESP-WROOM32 to the computer, use EpressLRS Configurator to flash the ELRS 2.0 firmware, hardware selection: DIY 2400 ESP32- E28, then fill in your binding password, then click Build & FLASH, watch Log, and go to flash During the firmware, you need to hold down the Boot0 button on the ESP-WROOM32 development board, or it will fail.

 

4. Flash the FSi6 firmware According  to  erfly6/ER9XFlySky I6En.pdf ,  The  FSI6 firmware is at https://github.com/aerror2/erfly6/blob/main/Output/Release/Exe/FSI6.hex

5. Complete

There are still some issues, such as heat dissipation issues, battery capacity issues, and switching tuner issues that have not been dealt with.





Self-made 2.4 ELRS receiver, no need to board, easy to make


Difficulty: medium, mainly because the device is too small, solding requires patience

I. Hardware

1. LoRa RF module, sx1280: E28-2G4M12S

2. MCU Wifi module: ESP-01F

3. Several silicone flexible cords of various colors

4.1k resistor

5. TTL to serial port CH341 (CH340 or FTDI can be any one.)

6. DC-DC3.3v step-down module

7. One LED light.

II. software:

1. EpressLRS Configurator https://github.com/ExpressLRS/ExpressLRS-Configurator

III. RX circuit diagram



IV.  Steps

1. First, connect the RST and BOOT (IO0) of the esp-01f to two wires. RST is used for grounding and restarting. BOOT must be grounded , When you flash the ELRS firmware, .

2. Connect the TX, RX, VCC, GND of ESP-01F to the RX, TX, 3.3V and GND of CH341, and connect to the computer. If there is no CH341 driver, install the driver. 

3. Use EpressLRS Configurator to flash the firmware, Device selection: DIY 2400 RX ESP8285 SX1280, remember to ground the boot line when flashing, and then "Build And flash", if it is not successful, you can connect RST to ground , it will restart the ESP-01F.



 After flashing, the wiring of ESP-01F and CH341 can be disconnected.

4. Connect E28-2G4M12S and ESP-01F with wires by "III. RX circuit diagram" , add resistors to connect the LED lights, 2G4M12S does not have TCX0EN, just don't need to connect it.

5. Connect VCC and GND to the 3.3V output of the DC-DC module. The step-down module requires ripple less than 20mv

6. The line is connected as shown in the figure:



 7. Leave GND, VCC, TX, RX, and stack the three modules with tape. The weight control is not bad, only 5G:
