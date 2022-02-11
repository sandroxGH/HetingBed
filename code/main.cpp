/*
* Firmware based on Atmega 328P boarda nano 
* Display Oled 128x32 I2C two button + - 
*
*/

//#define FDebug    //Fast debugging
#define SDebug      //Scheduled debug by time
//#define TempDeb     //Used to debug the program withou Ntc

#include <Arduino.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <EEPROM.h>


#define I2C_ADDRESS 0x3C

//___________PinDefinition_________
#define Ntc     A0
#define LedBIn  13
#define Triac   6
#define LedV    7
#define UpButt  2
#define DwButt  3 
#define Rel     5

#define MaxTemp 200

//___________Ntc Parameter________
#define NtcNominal  100000   // resistance at 25 degrees C    
#define TempNominal 25   // temp. for nominal resistance (almost always 25 C)
#define Sample      5
#define BCoeff      3950 // The beta coefficient of the thermistor (usually 3000-4000)
#define SerRes      10000 
#if defined TempDeb
float NtcDeb =20;
bool UpTemp =1;
bool DwTemp =0;
#endif

   

//___________Timer_________
#define TScanTemp   2000
#define TDispOff    30000
#define TDispOn     60000
unsigned long TimeScanTemp = 0;
unsigned long TimeDispOff = 0;
unsigned long TimeDispOn = 0;
unsigned long TimeTSave = 0;

//___________All Management_____________ 
#define Pat_No_All	    3000
#define Pat_All1        1000
#define Pat_All2        500
#define Pat_All3        200
#define TPatAll         10000
unsigned long TimeBlink = 0;
unsigned long TimePatt  = 0;
unsigned int Pattern    = Pat_No_All;
bool PattAllOn          = 0;
#define TCkTemp         30000
unsigned long TimeCkTemp =0;
#define DelCkTemp       3
float PrevTemp          =0;
bool AllMaxTemp         =0;
unsigned long TPulseSum =0;

//___________Button Management____________
#define SelPatt		      50      // Selection minimal time 
#define EntPatt			    1500    // Enter
#define TPulButt		    100     // Time for auto toogles button  
#define TTSave          5000
unsigned long TimeUpButt = 0;
unsigned long TimeDwButt = 0;
unsigned long TimePulseUp = 0;
unsigned long TimePulseDw = 0;
char UpButtState        =0; 
char DwButtState        =0;

//___________Program Parameter____________
#define DeltaT    2
#define TempMaxLimit 20
bool AllTemp      = 0;
float ActTemp     = 0;
union { int ival; byte bval[2];} SetTemp;
int ModSetTemp    = 0;
bool DispOn       = 1;
char DispBuff[30];
#define TDeb      2000
unsigned long TimeDeb =0;


//____________PID Parameter___________
#define TPid  3000
unsigned long TimePid =0;
unsigned long TimePulse =0;
int TPulse =0;
float DeltaTimePid=0;
float PidErr = 0;
float PrevErr = 0;
#define MaxErr 150
#define MinErr -150
float SumErr =0;

#define Kp 90
#define Ki 10
#define Kd 3500
int KpVal = 0;
int KiVal = 0;
int KdVal = 0;
#define PidMaxRes 10000
#define PidMinRes 0
int PidRes = 0;
bool FirstUp=0;


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

#if defined TempDeb
float ReadTemp(){
if (UpTemp)   NtcDeb+=random(1, 2) / 100.0;
if (DwTemp)   NtcDeb-=random(1, 250) / 100.0;
if (UpTemp && NtcDeb>(SetTemp.ival+2)) {
  UpTemp=0;
  DwTemp=1;
}
if (DwTemp && NtcDeb<(SetTemp.ival-2)) {
  UpTemp=1;
  DwTemp=0;
}
return NtcDeb;
}
#else

float ReadTemp(){
  uint8_t i;
  float average =0;
  float steinhart =0;

  for (i=0; i< Sample; i++) {
    average += analogRead(Ntc);
    delay(10);
  }
  average /= Sample;
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SerRes / average;
  
  steinhart = average / NtcNominal;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCoeff;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TempNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  return steinhart;
}
#endif

#if defined SDebug
void Debug() {
  Serial.println(F("debug"));
  // Serial.print(F("ERR "));
  // Serial.println(PidErr);
  // Serial.print(F("SummErr "));
  // Serial.println(SumErr);
  // Serial.print(F("KpVal "));
  // Serial.println(KpVal, DEC);
  // Serial.print(F("KiVal "));
  // Serial.println(KiVal, DEC);
  // Serial.print(F("KdVal "));
  // Serial.println(KdVal, DEC);
  // Serial.print(F("PidRes "));
  // Serial.println(PidRes, DEC);
  // Serial.print(F("TPulse "));
  // Serial.println(TPulse, DEC);
  // Serial.print(F("ActTemp "));
  // Serial.println(ActTemp);

  Serial.print("TPulseSum ");
  Serial.println(TPulseSum);
  Serial.print("ActTemp ");
  Serial.println(ActTemp);
  Serial.print("PrepTemp ");
  Serial.println(PrevTemp);


}
#endif

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);
  pinMode(UpButt, INPUT_PULLUP);
  pinMode(DwButt, INPUT_PULLUP);
  pinMode(LedBIn, OUTPUT);
  pinMode(Triac, OUTPUT);
  pinMode(LedV, OUTPUT);
  pinMode(Rel, OUTPUT);

  EpromRead();

  Disp.begin(&Adafruit128x32, I2C_ADDRESS);
  Disp.setFont(Adafruit5x7);
  Disp.clear();
  Disp.println("Msystem");
  Disp.println("PCB Heathing bed");
  Disp.println(__DATE__);
  
  delay(1000);
  while(!digitalRead(UpButt));

  digitalWrite(Triac,LOW);
  Pattern=Pat_No_All;
  ModSetTemp=SetTemp.ival;
  DispOn=1;
  digitalWrite(Rel, HIGH);
  //PrevTemp=ReadTemp()+10;

  //SetTemp.fval = 99.0;
  //EpromWrite();

 
}
//------------------------------------------------------------------------------
void loop() { 
#if defined SDebug
  //------------Time Debug Mng---------
  if((millis()-TimeDeb)>TDeb){
    TimeDeb = millis();
    Debug();
  }
#endif
  if(TimeDispOn && ((millis() - TimeDispOn) > TDispOn)){
    TimeDispOn = 0;
    DispOn=1;
  }

  if (DispOn){
    DispOn=0;
    UpButtState = DwButtState =0;
    TimeDispOff = millis();
    Disp.ssd1306WriteCmd(SSD1306_DISPLAYON);
  }
  
  if(TimeDispOff && ((millis() - TimeDispOff) > TDispOff)){
    TimeDispOff = 0;
    TimeDispOn = millis();
    Disp.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
  }

  if((millis() - TimePid) > TPid){
    ActTemp = ReadTemp();
    if(!TimeTSave) PrintDisplay(ActTemp,SetTemp.ival);
    else PrintDisplay(ActTemp,ModSetTemp);
    DeltaTimePid= (millis()-TimePid) / 1000;  //To Caluculate the derivative value
    TimePid = millis();
    if(!FirstUp && (ActTemp>SetTemp.ival)){
      FirstUp=1;
      SumErr=0;
    } 
    PidErr = (SetTemp.ival- ActTemp);
    SumErr += PidErr;
    KpVal = Kp * PidErr;
    if(SumErr>MaxErr) SumErr = MaxErr;
    if(SumErr<MinErr) SumErr = MinErr;
    
    //if((-3 < PidErr) ||(PidErr<3)) KiVal +=   (Ki * PidErr);
    KiVal = Ki* SumErr;
    KdVal = Kd*((PidErr - PrevErr)/DeltaTimePid);
    PidRes = KpVal + KiVal + KdVal;
    PrevErr=PidErr;
    if(PidRes < PidMinRes)  PidRes = PidMinRes; 
    if(PidRes > PidMaxRes)  PidRes = PidMaxRes;
    
    TPulse=PidRes;
    TimePulse = millis();  
    if(PidErr>20){SumErr=0;TPulse=(TPid+100);}  
    TPulseSum+=TPulse; 
  } 
   
  if(ActTemp>SetTemp.ival || (millis()-TimePulse) > float(TPulse))digitalWrite(Triac,LOW);
  else digitalWrite(Triac, HIGH);
   
   
  //if((abs(SumErr) >= MaxErr)&&((millis() - TimeCkTemp) > TCkTemp)){
  if(ActTemp<(SetTemp.ival-10)){
    if(TPulseSum > TCkTemp){
      TPulseSum=0;
      if(ActTemp < (PrevTemp+DelCkTemp)){
        digitalWrite(Triac, LOW);
        delay(50);
        digitalWrite(Rel, LOW); 
        Disp.clear();
        Disp.set2X();
        Disp.println(F("Allarme"));
        Disp.println(F("Sonda Temp !!!"));
        Disp.ssd1306WriteCmd(SSD1306_DISPLAYON);
        while(1){
        delay(500);
        digitalWrite(LedV, !digitalRead(LedV));
        }   
      }
      PrevTemp=ActTemp;
    }   
  }
  else TPulseSum=0;

  if(ActTemp > (SetTemp.ival+TempMaxLimit)){
    digitalWrite(Triac, LOW);
    delay(50);
    digitalWrite(Rel, LOW); 
    Disp.clear();
    //Disp.set2X();
    Disp.println(F("Allarme"));
    Disp.println(F("Max Temp!!!"));
    Disp.ssd1306WriteCmd(SSD1306_DISPLAYON);
    // while(1){
    //   delay(500);
    //   digitalWrite(LedV, !digitalRead(LedV));
    // }
  }
    if(ActTemp < 0){
    digitalWrite(Triac, LOW);
    delay(50);
    digitalWrite(Rel, LOW); 
    // Disp.clear();
    // Disp.set2X();
    // Disp.println(F("Allarme"));
    // Disp.println(F("Min Temp!!!"));
    // Disp.ssd1306WriteCmd(SSD1306_DISPLAYON);
    while(1){
      delay(500);
      digitalWrite(LedV, !digitalRead(LedV));
    }
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