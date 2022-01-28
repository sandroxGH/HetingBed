/*
* Firmware based on Atmega 328P boarda nano 
* Display Oled 128x32 I2C two button + - 
*
*/

//#define FDebug    //Fast debugging

#include <Arduino.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <EEPROM.h>


#define I2C_ADDRESS 0x3C

//___________PinDefinition_________
#define Ntc     A1
#define LedBIn  13
#define LedR    6
#define LedV    7
#define UpButt  2
#define DwButt  3    

#define MaxTemp 150

//___________Ntc Parameter________
#define NtcNominal  100000   // resistance at 25 degrees C    
#define TempNominal 25   // temp. for nominal resistance (almost always 25 C)
#define Sample      5
#define BCoeff      3950 // The beta coefficient of the thermistor (usually 3000-4000)
#define SerRes      10000  
   

//___________Timer_________
#define TScanTemp   2000
#define TDispOff    30000
unsigned long TimeScanTemp = 0;
unsigned long TimeDispOff = 0;
unsigned long TimeTSave = 0;

//___________All Management_____________ 
#define Pat_No_All	3000
#define Pat_All1    1000
#define Pat_All2    500
#define Pat_All3    200
#define TPatAll     10000
unsigned long TimeBlink = 0;
unsigned long TimePatt  = 0;
unsigned int Pattern    = Pat_No_All;
bool PattAllOn          = 0;

//___________Button Management____________
#define SelPatt		  50      // Selection minimal time 
#define EntPatt			1500    // Enter
#define TPulButt		100     // Time for auto toogles button  
#define TTSave      5000
unsigned long TimeUpButt = 0;
unsigned long TimeDwButt = 0;
unsigned long TimePulseUp = 0;
unsigned long TimePulseDw  = 0;
char UpButtState, DwButtState;

#define DeltaT    4
float ActTemp     = 0;
union { int ival; byte bval[2];} SetTemp;
int ModSetTemp    = 0;
bool DispOn       = 1;

char DispBuff[30];

SSD1306AsciiAvrI2c Disp;
//------------------------------------------------------------------------------

void PrintDisplay(float Data1,int Data2){  
    Disp.clear();
    Disp.print("Act Temp ");
    Disp.println (Data1,1);
    Disp.setCursor(0,2);
    Disp.print("Set Temp ");
    Disp.println(Data2);
}


char ButtonState(char PinN, unsigned long *Time, bool Pulse = 0 , unsigned long *Time1 = 0) {
  
  if ((*Time==0) && !digitalRead(PinN))  *Time = millis();
  if (Pulse && !digitalRead(PinN) && ((millis() -*Time) > (EntPatt * 3 )))  {
    if (*Time1 == 0 ) *Time1 = millis();
    if ((millis() - *Time1)>TPulButt) {
      *Time1 = millis();
#if defined FDebug
      Serial.print(F("3 "));
#endif
      DispOn =1;
      return 1;
    }
  }
  if (Pulse && digitalRead(PinN)) *Time1 = 0;
  if (digitalRead(PinN) && (*Time)) {
    if ((millis() - *Time)> EntPatt) {
      *Time = 0;
#if defined FDebug 
      Serial.println(F("2"));
#endif
      DispOn =1;
      return 2;
    }
    if ((millis() - *Time)> SelPatt) {
      *Time = 0;
#if defined FDebug 
      Serial.println(F("1"));
#endif
      DispOn =1;
      return 1;
    }
    return 0;
  }  
  else return 0;
}

void EpromRead(){
  SetTemp.bval[0]=  EEPROM.read(0);
  SetTemp.bval[1]=  EEPROM.read(1);
  // SetTemp.bval[2]=  EEPROM.read(2);
  // SetTemp.bval[3]=  EEPROM.read(3);
}
void EpromWrite(){
  EEPROM.write(0,SetTemp.bval[0]);
  EEPROM.write(1,SetTemp.bval[1]);
  // EEPROM.write(2,SetTemp.bval[2]);
  // EEPROM.write(3,SetTemp.bval[3]);
}

float ReadTemp(){
  uint8_t i;
  float average =0;
  float steinhart =0;

   for (i=0; i< Sample; i++) {
   average += analogRead(Ntc);
   delay(10);
  }
  average /= Sample;

  // Serial.print("Average analog reading "); 
  // Serial.println(average);
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SerRes / average;
  // Serial.print("Thermistor resistance "); 
  // Serial.println(average);
  
  
  steinhart = average / NtcNominal;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCoeff;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TempNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  
  // Serial.print("Temperature "); 
  // Serial.print(steinhart);
  // Serial.println(" *C");
 return steinhart;


}

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);
  pinMode(UpButt, INPUT_PULLUP);
  pinMode(DwButt, INPUT_PULLUP);
  pinMode(LedBIn, OUTPUT);
  pinMode(LedR, OUTPUT);
  pinMode(LedV, OUTPUT);

  EpromRead();

  Disp.begin(&Adafruit128x32, I2C_ADDRESS);
  Disp.setFont(Adafruit5x7);
  Disp.clear();
  Disp.println("Msystem");
  Disp.println("PCB Heathing bed");
  Disp.println(__DATE__);
  
  delay(1000);
  while(!digitalRead(UpButt));

  digitalWrite(LedR,LOW);
  Pattern=Pat_No_All;
  ModSetTemp=SetTemp.ival;
  DispOn=1;

  //SetTemp.fval = 99.0;
  //EpromWrite();

 
}
//------------------------------------------------------------------------------
void loop() {  
  if ( DispOn){
    DispOn=0;
    TimeDispOff = millis();
    Disp.ssd1306WriteCmd(SSD1306_DISPLAYON);
  }
  
  if(TimeDispOff && ((millis() - TimeDispOff) > TDispOff)){
    TimeDispOff = 0;
    Disp.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
  }

  if((millis() - TimeScanTemp) > TScanTemp){
    TimeScanTemp = millis();
    ActTemp = ReadTemp();
    if(!TimeTSave) PrintDisplay(ActTemp,SetTemp.ival);
  } 
  
  if(!digitalRead(LedR)&& ActTemp < (SetTemp.ival-(DeltaT/2))){
    digitalWrite(LedR,HIGH);
    Pattern = Pat_No_All;
  }

  if(ActTemp > (SetTemp.ival+(DeltaT/2))){
    digitalWrite(LedR,LOW);
    Pattern = Pat_All2;
  }  

  UpButtState = ButtonState(UpButt, &TimeUpButt , 1, &TimePulseUp);
  DwButtState = ButtonState(DwButt, &TimeDwButt , 1, &TimePulseDw);
 
  if(UpButtState && TimeDispOff){
    UpButtState=0;
    TimeTSave  = millis();
    if(ModSetTemp<MaxTemp)  ModSetTemp++;
    PrintDisplay(ActTemp,ModSetTemp);
  }
  if(DwButtState && TimeDispOff){
    DwButtState=0;
    TimeTSave  = millis();
    if (ModSetTemp)ModSetTemp--;
    PrintDisplay(ActTemp,ModSetTemp);
  }
  if(TimeTSave && ((millis()-TimeTSave)>TTSave)) {
     TimeTSave =0;
     SetTemp.ival=ModSetTemp;
     EpromWrite();
     Pattern = Pat_All3;     
   }

  if ((millis()-TimeBlink)>Pattern) {
    digitalWrite(LedV, (digitalRead(LedV) ^ 1));
    if ((Pattern != Pat_No_All) && !PattAllOn) {
      TimePatt = millis();
      PattAllOn = 1;
#if defined FDebug
      Serial.println(F("ALL"));
#endif
    }
    else TimeBlink = millis();

    if ((millis()- TimePatt)>TPatAll) {
      Pattern = Pat_No_All;
      PattAllOn = 0;
    }
  }
}