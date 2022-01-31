
// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
//  #include <Adafruit_I2CDevice.h>

// #define DispAdd		0x3C

// Adafruit_SSD1306 display(128, 32, &Wire);

// void setup(){
//   pinMode(13,OUTPUT); 
//   digitalWrite(13,HIGH);
//   Serial.begin (115200);
//   Serial.println ("Ok");
   
//   Wire.begin();
//   display.clearDisplay();
//   display.println(F("Ciao"));
//   display.display(); 
// } 

// void loop() {}

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

#define OLED_RESET 4
Adafruit_SSD1306 display;
#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
void setup()   {    
          
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //Impostare l'indirizzo i2c annotato in precedenza
  display.clearDisplay();  //Pulisce il buffer da inviare al display
  display.setTextSize(1);  //Imposta la grandezza del testo
  display.setTextColor(WHITE); //Imposta il colore del testo (Solo bianco)
  display.setCursor(0,0); //Imposta la posizione del cursore (Larghezza,Altezza)
  display.println("LORENZOCASABURO.IT"); //Stringa da visualizzare
  display.display(); //Invia il buffer da visualizzare al display
}
void loop() {
 
}