
/*
 * Expression Multiplexer Firmware
 * Lucas H. 07.04.2020
 * v0.1
 * 
 * Exp Multiplexer has 4 Output Channels and 3 Operating Modes.
 * Each Output Channel can have different Potentiometer Taper settings applied - either
 * lin, inv. lin, log or inv. log can be set for each channel separately.
 * In the "normal" operating mode, or "channel mode", the select button steps through the 4 channels
 * and the on/off button activates/deactivates the selected channel. Each channel can have the pot taper assigned
 * via the selector knob.
 * 
 * In "Patch Mode", activated/deactivated by pressing both buttons at once, the Select Button steps
 * through 4 patches (indicated by the "Mode" Leds), where each patch has the on/off and Mode info for each
 * channel saved. The patch is activated with the on/off button. Logically, only one patch can be active at
 * a time. It is still possible to deactivate all patches.
 * 
 * The third mode, "Save Mode" can only be entered from normal (channel) mode, not Patch mode. It is used
 * to set up the patches for patch mode. This is done by first setting up the channels on/off and modes as
 * desired and then holding the Select buton for ~2s. Then the desired saving spot for the patch can be selected
 * via the selector knob and by pressing the select button again for ~2s the patch is saved to that spot, overwriting
 * any previosly saved patch.
 * 
 */

 //Libraries
#include <TimerOne.h>
#include <EEPROM.h>
#include <SPI.h>

 //Pin Definitions:
 #define nCS0 A5 //DigPot0 Chip Select
 #define nCS1 A4 //DigPot1 Chip Select
 #define nCS2 A3 //DigPot2 Chip Select
 #define nCS3 A2 //DigPot3 Chip Select
 
 #define Btn0 2 //On/Off Button
 #define Btn1 3 //Select Button

 #define nSHDWN 4 //DigPot Wiper Shutdown - active low

 #define SelIn A1 //Selector Pot ADC in
 #define ExpIn A0 //Expression Pedal ADC in

 #define Led0 5 //LED GPIOs
 #define Led1 6
 #define Led2 7
 #define Led3 8
 #define Led4 9
 #define Led5 10
 #define Led6 11
 #define Led7 12

 //Lookup Table for LogPot-Mode
 byte LogPot[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6,
 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 
 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 
 18, 18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 
 24, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32, 
 32, 33, 34, 35, 35, 36, 37, 38, 38, 39, 40, 41, 
 42, 42, 43, 44, 45, 46, 47, 47, 48, 49, 50, 51, 
 52, 53, 54, 55, 56, 56, 57, 58, 59, 60, 61, 62, 
 63, 64, 65, 66, 67, 68, 69, 70, 71, 73, 74, 75, 
 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 
 89, 91, 92, 93, 94, 95, 97, 98, 99, 100, 102, 
 103, 104, 105, 107, 108, 109, 111, 112, 113, 
 115, 116, 117, 119, 120, 121, 123, 124, 126, 
 127, 128, 130, 131, 133, 134, 136, 137, 139, 
 140, 142, 143, 145, 146, 148, 149, 151, 152, 
 154, 155, 157, 158, 160, 162, 163, 165, 166, 
 168, 170, 171, 173, 175, 176, 178, 180, 181, 
 183, 185, 186, 188, 190, 192, 193, 195, 197, 
 199, 200, 202, 204, 206, 207, 209, 211, 213, 
 215, 217, 218, 220, 222, 224, 226, 228, 230, 
 232, 233, 235, 237, 239, 241, 243, 245, 247, 
 249, 251, 253, 255};
 
 //Variable declarations
 word ExpInputVal = 0; //Input ADC Value
 word SelInputVal = 0; //Input ADC Value
 byte ExpVal = 0; //Exp Pedal Input Value
 byte SelVal = 0; //Selector Wheel input Value
 
 //Mode: 0=lin, 1=inv. lin, 2=log, 3=inv. log
 //State: 0=off, 1=on
 byte PatchModeState[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; //in Patch mode - modes of channels for each patch
 bool PatchChnlState[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; //in Patch mode - state of channels for each patch
 byte PatchActive = 255; //in patch mode - index, which patch is active
 bool ChnlState[4] = {0, 0, 0, 0}; //in normal mode - state of each channel
 byte ChnlMode[4] = {0, 0, 0, 0}; //in normal mode - mode of each channel
 byte SelIndex = 0; //normal + patch mode - index, which patch/channel is currently selected by the Btn1 (or Selector wheel in patch mode)

 byte TimerCount = 0; //timer overflow counter to derive tasks with multiple time periods from one base timer
 byte SaveModeCounter = 0; //counter variable to determine if Btn1 was pressed long enough to enter save mode

 byte NvmData[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //EEPROM Data Array
 /*
  * NvmData Structure
  * NvmData stores Mode and Patch Info in non-volatile Memory
  * Byte[0] contains only Mode information for standard mode
  *  -> 2bit per channel
  * Byte[1..8] contains Patch/Mode information for patch mode
  *  -> each Patch is divided into 2Bytes
  *  -> LSB contains Channel on/off info (4bits)
  *  -> MSB contains Mode Info for each channel (2bit per channel, LSB = CH0)
  * Byte[9] contains operating mode information 
  *  -> 1bit if standard or patch mode
  */

 //Btn0 = On/Off Button
 bool Btn0State = false;
 bool Btn0Pressed = false;
 bool Btn0Released = false;
 //Btn1 = Selector Button
 bool Btn1State = false;
 bool Btn1Pressed = false;
 bool Btn1Released = false; 

 //check if selector has been moved
 bool SelValChng = false;
 byte LastSelVal = 0;

 bool PatchMode = false; //True if in patch mode, false if in normal mode
 bool SaveMode = false; //True if in SaveMode (for Patch saving)
 bool SvMdCntrActive = false; //State of Timer Counter for entering/exiting SaveMode

 //4 Leds for Channels, 4 Leds for Mode/Patch
 //4 Modi each (off = 0, on = 1, slow blink = 2, fast blink = 3)
 byte ChnlLedState[4] = {0, 0, 0, 0}; //Leds0..3
 byte ModeLedState[4] = {0, 0, 0, 0}; //Leds4..7


void setup() {
  //Init Pins
  pinMode(nCS0, OUTPUT);
  digitalWrite(nCS0, HIGH);
  pinMode(nCS1, OUTPUT);
  digitalWrite(nCS1, HIGH);
  pinMode(nCS2, OUTPUT);
  digitalWrite(nCS2, HIGH);
  pinMode(nCS3, OUTPUT);
  digitalWrite(nCS3, HIGH);

  pinMode(Btn0, INPUT);
  pinMode(Btn1, INPUT);

  pinMode(nSHDWN, OUTPUT);
  digitalWrite(nSHDWN, LOW);

  pinMode(SelIn, INPUT);
  pinMode(ExpIn, INPUT);

  pinMode(Led0, OUTPUT);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Led3, OUTPUT);
  pinMode(Led4, OUTPUT);
  pinMode(Led5, OUTPUT);
  pinMode(Led6, OUTPUT);
  pinMode(Led7, OUTPUT);

  //Init SPI with 2MHz, MSB-First, SPI-Mode 0,0
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  //Init ADC Values
  SelVal = (analogRead(SelIn) >> 8);
  LastSelVal = SelVal;
  ExpVal = (analogRead(ExpIn) >> 2);

  //Init Buttons
  CheckBtns();

  //Init DigPots
  DigPotsInit();

  //Get NVM Data from Memory
  ReadNvmData();

  //Init Timer to fire Interrupt every 10ms (=10000us)
  Timer1.initialize(10000);
  Timer1.attachInterrupt(IntTmr1);
}

void IntTmr1(void){
  //Timer1 Interrupt Routine
  //Refresh Timer and execute periodic tasks
  TimerCount++;
  Tmr10msTask();
  if(TimerCount % 10 == 0){
    Tmr100msTask();
  }
  if(TimerCount % 25 == 0){
    Tmr250msTask();
  }
  if(TimerCount % 50 == 0){
    Tmr500msTask();
    TimerCount = 0;
  }
}

void loop() {
  //Read 10bit ADC Value to 8bit variable
  ExpInputVal = analogRead(ExpIn);
  ExpVal = ExpInputVal >> 2;
  CheckSelector();
  
  //check if running on normal, patch or Save mode
  if(PatchMode){
    //Patch Mode
     if(Btn0Pressed && Btn0Released && !Btn1Pressed && !Btn1Released){
      //On/off Button activated
      if(SelIndex == PatchActive){
        //if current patch is selected and active, deactivate it
        PatchActive = 255;
      }else{
        //if current patch is not active, set it active - this deactivates the previous patch (max. one patch can be active at a time)
        PatchActive = SelIndex;
      }
      CheckLeds();
      ClrBtns();
    } else if(!Btn0Pressed && !Btn0Released && Btn1Pressed && Btn1Released){
      //Select Button activated
      SelIndex++;
      //select next patch
      if(SelIndex > 3){
        SelIndex = 0;
      }
      CheckLeds();
      ClrBtns();
    } else if(Btn0Pressed && !Btn0Released && Btn1Pressed && !Btn1Released){
      //On/Off + Select Button pressed together -> Mode Change
      PatchMode = false;
      //if in patch mode -> go to normal mode
      SelIndex = 0;
      CheckLeds();
      while(!Btn0Released && !Btn1Released){
        CheckBtns();
        delay(10);
      }
      delay(10);
      UpdateNvmData();
      ClrBtns();
    }
    //Check if Mode Selector Value changed -> used to select patch
    if(SelValChng){
      SelIndex = SelVal;
      CheckLeds();
      SelValChng = false;
      }
    //check if a patch is active
    if(PatchActive < 5){
    //step through channels and update DigPots with latest Exp. Values
    for(byte i = 0; i < 4; i++){
      if(PatchChnlState[PatchActive][i] == true){
        //check which Mode is active for each channel for the selected patch
        switch(PatchModeState[PatchActive][i]){
          case 0: //linear
           WritePotSPI(i, ExpVal);
           break;
          case 1: //inv. linear
           WritePotSPI(i, 255 - ExpVal);
           break;
          case 2: //log
           WritePotSPI(i, LogPot[ExpVal]);
           break;
          case 3: //inv. log
           WritePotSPI(i, LogPot[255 - ExpVal]);
           break;
          default:
           break;
        }
      }
    }
    }
    //end Patch Mode
  }else if(!PatchMode && !SaveMode){
    //Normal Mode
    if(Btn0Pressed && Btn0Released && !Btn1Pressed && !Btn1Released){
      //On/off Button activated
      ChnlState[SelIndex] = !ChnlState[SelIndex];
      //turn selected channel on/off
      SvMdCntrActive = false;
      SaveModeCounter = 0;
      CheckLeds();
      ClrBtns();
    } else if(!Btn0Pressed && !Btn0Released && Btn1Pressed && Btn1Released){
      //Select Button activated
      SelIndex++;
      //jump to next channel
      if(SelIndex > 3){
        SelIndex = 0;
      }
      SvMdCntrActive = false;
      SaveModeCounter = 0;
      CheckLeds();
      ClrBtns();
    } else if(Btn0Pressed && !Btn0Released && Btn1Pressed && !Btn1Released){
      //On/Off + Select Button pressed together -> Mode Change
      //go to patch mode
      SvMdCntrActive = false;
      SaveModeCounter = 0;
      SelIndex = 0;
      PatchMode = true;
      CheckLeds();
      while(!Btn0Released && !Btn1Released){
        CheckBtns();
        delay(10);
      }
      delay(10);
      UpdateNvmData();
      ClrBtns();
    } else if(!Btn0Pressed && !Btn0Released && Btn1Pressed && !Btn1Released){
      //check if SaveMode has been entered (holding select button for ~2s)
      SvMdCntrActive = true;
      if(SaveModeCounter > 3){
        //if SelectButton is pressed for 2 Seconds, enter SaveMode
        SvMdCntrActive = false;
        SaveModeCounter = 0;
        SelIndex = 0;
        SvMdCntrActive = false;
        SaveMode = true;
        CheckLeds();
        while(!Btn1Released){
          CheckBtns();
          delay(10);
        }
        delay(10);
        UpdateNvmData();
        ClrBtns();
      }      
    }
    //Check if Mode Selector Value changed on currently selected channel
    if(SelValChng){
      ChnlMode[SelIndex] = SelVal;
      SelValChng = false;
      //update mode of current channel
      CheckLeds();
      UpdateNvmData();
      }
    //step through channels and update DigPots with latest Exp. Values
    for(byte i = 0; i < 4; i++){
      if(ChnlState[i] == true){
        //check mode for each active channel and set values accordingly
        switch(ChnlMode[i]){
          case 0: //linear
           WritePotSPI(i, ExpVal);
           break;
          case 1: //inv. linear
           WritePotSPI(i, 255 - ExpVal);
           break;
          case 2: //log
           WritePotSPI(i, LogPot[ExpVal]);
           break;
          case 3: //inv. log
           WritePotSPI(i, LogPot[255 - ExpVal]);
           break;
          default:
           break;
        }
      }
    }
    //end Normal Mode
  }else{
    //Save Mode
    if(Btn1Pressed && !Btn1Released){
      SvMdCntrActive = true;
      //check if SaveMode has been exited
      if(SaveModeCounter > 3){
        //if SelectButton is pressed for 2 Seconds, exit SaveMode
        SvMdCntrActive = false;
        SaveModeCounter = 0;
        //copy channel and mode info to patch data of selected patch
        for(byte i = 0; i < 4; i++){
          PatchModeState[SelIndex][i] = ChnlMode[i];
          PatchChnlState[SelIndex][i] = ChnlState[i];
        }
        SelIndex = 0;
        SelValChng = false;
        SaveMode = false;
        CheckLeds();
        while(!Btn1Released){
          CheckBtns();
          delay(10);
        }
        delay(10);
        UpdateNvmData();
        ClrBtns();
      }
  }
    SelIndex = SelVal;
    //in SaveMode, just update the selector value, rest is handled on long button press of select button
    SelValChng = false;
  }
  //end Save Mode
}

void Tmr10msTask(void){
  //FastTask - Check if Buttons are operated
  CheckBtns();
}

void Tmr100msTask(void){
  //update Leds
  CheckLeds();
  //Fast Led Blinking with 5Hz - Timer is Toggle Frequency, so BlinkFrequency is half
  LedsFastBlink();
}

void Tmr250msTask(void){
  //Slow Led Blinking with 2Hz - Timer is Toggle Frequency, so BlinkFrequency is half
  LedsSlowBlink(); 
}

void Tmr500msTask(void){
  //check if select button was held down long enough for SaveMode
  CheckSaveMode(); 
}

void CheckSelector(void){
  //read analog value of selector pot and convert to 2bit value (4 states)
  SelInputVal = analogRead(SelIn);
  SelVal = SelInputVal >> 8;
  //check if value changed
  if(SelVal != LastSelVal){
    SelValChng = true;
  }
  LastSelVal = SelVal;
}

void CheckBtns(void){
  //Read and compute Button states
  Btn0State = digitalRead(Btn0);
  Btn1State = digitalRead(Btn1);

  //Buttons are active LOW
  if(!Btn0State && !Btn0Pressed && !Btn0Released){
    Btn0Pressed = true;
  }
  if(Btn0State && Btn0Pressed && !Btn0Released){
    Btn0Released = true;
  }
  if(!Btn1State && !Btn1Pressed && !Btn1Released){
    Btn1Pressed = true;
  }
  if(Btn1State && Btn1Pressed && !Btn1Released){
    Btn1Released = true;
  }
}

void ClrBtns(void){
 //Clear Button States
 Btn0Pressed = false;
 Btn0Released = false;
 Btn1Pressed = false;
 Btn1Released = false; 
}

void CheckSaveMode(void){
  //check if "Select" Button has been pressed long enough to enter/exit SaveMode
  if(SvMdCntrActive){
    SaveModeCounter++;
  }
}

void CheckLeds(void){
  if(PatchMode){
    //PatchMode
    for(byte i = 0; i < 4; i++){
      //cycle through channel Leds and turn them on/off according to patch
      ChnlLedState[i] = PatchChnlState[PatchActive][i];

      //check Patch Leds 
      if(SelIndex != PatchActive && PatchActive == i){
        ModeLedState[i] = 1;
      } else if(SelIndex != i && PatchActive != i){
        ModeLedState[i] = 0;
      } else if(SelIndex == i && PatchActive == i){
        ModeLedState[i] = 3;
      } else if(SelIndex == i && PatchActive != i){
        ModeLedState[i] = 2;
      }
    }
    
  }else if(!PatchMode && !SaveMode){
    //Normal Mode
    for(byte i = 0; i < 4; i++){
      //Check Channel Leds on/off/blink
      if(SelIndex != i && ChnlState[i] == true){
        ChnlLedState[i] = 1;
      } else if(SelIndex != i && ChnlState[i] == false){
        ChnlLedState[i] = 0;
      } else if(SelIndex == i && ChnlState[i] == true){
        ChnlLedState[i] = 3;
      } else if(SelIndex == i && ChnlState[i] == false){
        ChnlLedState[i] = 2;
      }
      //reset Mode Led
      ModeLedState[i] = 0;
    }
    //activate Led of current mode
    ModeLedState[ChnlMode[SelIndex]] = 1;
      
  }else{
    //Save Mode
    for(byte i = 0; i < 4; i++){
      //check which patch save spot is selected and fast blink that Led. Other Mode/Patch Leds off
      if(SelIndex == i){
        ModeLedState[i] = 3;
      }else{
        ModeLedState[i] = 0;
      }
      //Maintain Channel Leds on/off from normal mode
      ChnlLedState[i] = ChnlState[i];
    }
  }
  //turn Leds on/off
  for(byte i = 0; i < 4; i++){
    if(ChnlLedState[i] == 0){
      digitalWrite(i + Led0, LOW);
    } else if(ChnlLedState[i] == 1){
      digitalWrite(i + Led0, HIGH);
    }
    if(ModeLedState[i] == 0){
      digitalWrite(i + Led4, LOW);
    }else if(ModeLedState[i] == 1){
      digitalWrite(i + Led4, HIGH);
    }
  }
}

void LedsSlowBlink(void){
  //Slow Blink Leds
  for(byte i = 0; i < 4; i++){
    //step through Leds and check, if any of them need to be blinked slow
    if(ChnlLedState[i] == 2){
      digitalWrite(i + Led0, !digitalRead(i + Led0));
    }
    if(ModeLedState[i] == 2){
      digitalWrite(i + Led4, !digitalRead(i + Led4));
    }
  }
}

void LedsFastBlink(void){
  //Fast Blink Leds
  for(byte i = 0; i < 4; i++){
    //step through Leds and check, if any of them need to be blinked fast
    if(ChnlLedState[i] == 3){
      digitalWrite(i + Led0, !digitalRead(i + Led0));
    }
    if(ModeLedState[i] == 3){
      digitalWrite(i + Led4, !digitalRead(i + Led4));
    }
  }
}

void DigPotsInit(void){
  //Init DigPots on zero position and activate wiper
  for(byte i = 0; i < 4; i++){
    WritePotSPI(i, ExpVal);
  }
  //activate wiper
  digitalWrite(nSHDWN, HIGH);
}

void WritePotSPI(byte PotID, byte Data){
  //write wiper data to DigPots
  switch(PotID){
    case 0:
     digitalWrite(nCS0, LOW);
     delayMicroseconds(3);
     SPI.transfer(0x00); //write command to wiper register
     SPI.transfer(Data); //transmit data
     delayMicroseconds(3);
     digitalWrite(nCS0, HIGH);
     break;
    case 1:
     digitalWrite(nCS1, LOW);
     delayMicroseconds(3);
     SPI.transfer(0x00); //write to wiper register
     SPI.transfer(Data); //transmit data
     delayMicroseconds(3);
     digitalWrite(nCS1, HIGH);
     break;
    case 2:
     digitalWrite(nCS2, LOW);
     delayMicroseconds(3);
     SPI.transfer(0x00); //write to wiper register
     SPI.transfer(Data); //transmit data
     delayMicroseconds(3);
     digitalWrite(nCS2, HIGH);
     break;
    case 3:
     digitalWrite(nCS3, LOW);
     delayMicroseconds(3);
     SPI.transfer(0x00); //write to wiper register
     SPI.transfer(Data); //transmit data
     delayMicroseconds(3);
     digitalWrite(nCS3, HIGH);
     break;
    default:
      break;
  }
}

void ReadNvmData(void){
  //Reads EEPROM Data to corresponding variables
  for(byte i = 0; i < 10; i++){
    NvmData[i] = EEPROM.read(i);
  }

  //Read back Channel Mode info for normal mode
  for(byte i = 0; i < 4; i++){
    ChnlMode[i] = (NvmData[0] >> (i * 2)) & 0x03; 
  }

  //Read back channel on/off info for patch mode
  for(byte i = 0; i < 4; i++){
    for(byte j = 0; j < 4; j++){
      PatchChnlState[i][j] = (NvmData[1 + (i * 2)] >> j) & 0x01;
    }
  }

  //Read back mode info for patch mode
  for(byte i = 0; i < 4; i++){
    for(byte j = 0; j < 4; j++){
      PatchModeState[i][j] = (NvmData[2 + (i * 2)] >> (j * 2)) & 0x03; 
    }
  }

  //Read back Operating Mode
  PatchMode = NvmData[9];
}

void UpdateNvmData(void){
  //Saves changed NVM Data back to EEPROM
  //Read back variables to NVM Structure
  NvmData[0] = (ChnlMode[3] << 6) | (ChnlMode[2] << 4) | (ChnlMode[1] << 2) | ChnlMode[0];

  //put Patch Channel on/off info into every second byte of NvmData Structure
  for(byte i = 0; i < 4; i++){
    byte helper = 0;
    for(byte j = 0; j < 4; j++){
      helper = helper | (PatchChnlState[i][j] << j);
    }
    NvmData[1 + (i * 2)] = helper;
  }

  //put Patch Mode info into every second byte of NvmData Structure
  for(byte i = 0; i < 4; i++){
    byte helper = 0;
    for(byte j = 0; j < 4; j++){
      helper = helper | (PatchModeState[i][j] << (j * 2));
    }
    NvmData[2 + (i * 2)] = helper;
  }
  
  NvmData[9] = PatchMode;
  
  //Update NV-Memory
  for(byte i = 0; i < 10; i++){
    EEPROM.update(i, NvmData[i]);
  }
}
