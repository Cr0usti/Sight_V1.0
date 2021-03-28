

/*initialisation Neopixel Avant et Arrière*/

#include <Adafruit_NeoPixel.h>

#include <Arduino.h>



#define ESP32_RTOS  // Uncomment this line if you want to use the code with freertos only on the ESP32
                    // Has to be done before including "OTA.h"

#include "OTA.h"

#include <Keypad.h>





/*-------------------------------------------------NEOPIXEL---------------------------------------------------------*/
#define BacklightPin 16 //Pin arrière
#define FrontlightPin 4 //Pin avant
#define IndicateurPin 2 // Indicateur télécommande


#define BacklightCount 30 //Nombre LED arrière
#define FrontlightCount 30//Nombre LED avant
#define IndicateurCount 3

int brightness = 255 ;  
int brightnessIndicateur = 255;


Adafruit_NeoPixel Backlight(BacklightCount , BacklightPin , NEO_GRBW + NEO_KHZ400); //initialisation lumière arrière
Adafruit_NeoPixel Frontlight(FrontlightCount , FrontlightPin , NEO_GRBW + NEO_KHZ400); //initialisation lumière arrière
Adafruit_NeoPixel Indicateur(IndicateurCount , IndicateurPin , NEO_BRG + NEO_KHZ800);

/*-----------------------------------------*/


/*création des couleurs*/


uint32_t white = Frontlight.Color(255 , 255 , 255 , 0);    //Blanc avec Led blanche
/*uint32_t red = strip.Color(255 , 0 , 0 , 0);      //Rouge
uint32_t orange strip.Color(150 , 150 , 0 , 0);   //Orange*/
uint32_t otaneo = Indicateur.Color(0,255,150);

/*-----------------------------------------------------------NEOPIXEL--------------------------------------------------------------*/




//IHM
int PowerIndicateurLed = 13 ; //Pin LED indicateur ON/FF
int PowerButton = 12; //Pin bouton ON/OFF
int PowerButtonState = 0 ; //Etat bouton ON/OFF


int TempUPPin = 5 ;//Pin temp up

int TempDownPin = 34 ;
int TempDownval = 0 ;
int TempDown = 0 ;

int TempBatPin = 33 ; //Pin temp batterie
int TempBatPin2 = 35 ;


int UbatPin = 32 ; //Tension batterie Pin

int i = 0; //variable pour effet lumière
int i_front = 0 ;
int i_back = 0 ;



/*------------------------------------KEYPAD---------------------------------------------------------------*/
const byte ROWS = 3 ;
const byte COLS = 2 ;

char hexaKeys[ROWS][COLS] = {
  {'1','2'},
  {'3','4'},
  {'5','6'}
};

byte rowPins[ROWS] = { 17 , 18 , 5 };
byte colPins[COLS]= { 19 , 21 };

Keypad customKeypad = Keypad(makeKeymap(hexaKeys  ),rowPins,colPins,ROWS,COLS);

/*--------------------------------------------------KEYPAD-----------------------------------------------------*/



void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  setupOTA("Sight_ESP32_V1.0_1", "esp32", "Crousti8804"); //Setup pour  pour librairie OTA (nom réseau ota, nom réseau auquel se connecter, mot de passe réseau)
  // attention ne pas décommenter problème : la carte passe en mode sleep automatique et ne se réveille pas normalement
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_12,HIGH);//Active le réveille externe par le bouton power

  Backlight.begin();
  Frontlight.begin();
  Indicateur.begin();
  Backlight.setBrightness(brightness);
  Frontlight.setBrightness(brightness);
  Indicateur.setBrightness(brightnessIndicateur);
  Backlight.fill(white,0,30);
  Frontlight.fill(white,0,30);
  Indicateur.fill(white,0,3);
  Backlight.show();
  Frontlight.show();
  Indicateur.show();
  delay(2000);

  pinMode(PowerIndicateurLed,OUTPUT);
  digitalWrite(PowerIndicateurLed,HIGH);

  //Bouton Power ON/OFF
  pinMode(PowerButton,INPUT);

}

void loop() {

#ifdef defined(ESP32_RTOS) && defined(ESP32)
#else // If you do not use FreeRTOS, you have to regulary call the handle method.
  ArduinoOTA.handle();
#endif
  TelnetStream.println("Loop");


  char customKey = customKeypad.getKey();
  if(customKey){
    Serial.println(customKey);
    TelnetStream.println(customKey);
  }

  /*if(PowerButtonState=HIGH){
    delay(2000);//Appuie long pour éteindre
    if(PowerButtonState=HIGH){
      Serial.println("Mode deep sleep");
      TelnetStream.println("Mode deep sleep");
      esp_deep_sleep_start();//mode sommeil profond

    }


  }*/

/*
if(PowerButtonState == HIGH){
  delay(1500);
  if(PowerButtonState == HIGH){
    ArduinoOTA.handle();
    Serial.println("OTA handle...");
    i=0;
    Indicateur.clear();
    Indicateur.show();
    while(i != 4){
      i++;
      Indicateur.setPixelColor(i,0,255,150);
      Indicateur.show();
      delay(250);
    }
    delay(1000);
    Indicateur.clear();
    Indicateur.show();
  }
}*/
  
  PowerButtonState = digitalRead(PowerButton);
  Serial.print("PowerButtonState :");
  Serial.println(PowerButtonState);

/*--------------------------------------------CAPTEUR DE TEMPERATURE---------------------------------------*/
  //Température UP
  /*Serial.println("-------------");
  int TempUPRead = analogRead(TempUPPin);//Récupérer tension de sortie du capteur

  float VinTempUP = (TempUPRead * 3.3) / (4095) ;//Convertir cette valeur en Tension

  Serial.print(VinTempUP);
  Serial.println("Volt");  //communiquer la tension


  float tempUP = (VinTempUP - 424)/6.25;
  Serial.print("tempUP :");
  Serial.println(tempUP);
  Serial.println("-------------");
*/

  //Température down
  /*TempDownval = analogRead(TempDown);
  TempDown = 5*TempDownval/1024;
  Serial.print("temDown =");
  Serial.println(TempDownval);
  Serial.print("TEMPDOWN °C =");
  Serial.println(TempDown);*/


  //tempétature batterie
  Serial.println("------------------");

  int TempBatRead = analogRead(TempBatPin);//Récupérer tension de sortie du capteur
  TelnetStream.print("TempbatRead : ");
  TelnetStream.println(TempBatRead);

  float VinTempBat = (TempBatRead * 3.3) / (4095) ;//Convertir cette valeur en Tension
  TelnetStream.print("VinTempBat");
  TelnetStream.println(VinTempBat);

  Serial.print(VinTempBat);     
  Serial.println("Volt");  //communiquer la tension


  float tempCBat = (VinTempBat - 0.424)/0.00625;
  TelnetStream.print("TempCBat");
  TelnetStream.println(tempCBat);
  Serial.print("tempCBat :");
  Serial.println(tempCBat);
  Serial.println("-------------");
/*------------------------------------CAPTEUR DE TEMPERATURE------------------------------------------------*/


  //Tension batterie
  int UbatRead = analogRead(UbatPin);
  Serial.print("UbatRead = ");
  Serial.println(UbatRead);
  float UbatVin = (UbatRead * 3.3)/4095;
  Serial.print("UbatVin = ");
  Serial.println(UbatVin); 
  float Ubat = UbatVin * 1.121 ;
  Serial.print("Ubat =");
  Serial.println(Ubat);

  delay(3000);//delay pour print serial



  /*Frontlight.show();
  Frontlight.fill(white,1,30);
  Frontlight.show();
  delay(1000);
  Frontlight.clear();
  Frontlight.show();*/
  Indicateur.fill(white,0,3);
  Indicateur.show();
  Frontlight.clear();
  Frontlight.show();
  while(i<30){
    i++;
    Frontlight.setPixelColor(i,255,150,0);
    Frontlight.show();
    delay(50);
  }
  Frontlight.clear();
  Frontlight.show();
  i=0;



/*
  if(customKeypad=1){

  }


  if(customKeypad=2){
    
  }


  if(customKeypad=3){


  }


  if(customKeypad=4){

  }

*/

  if(customKeypad=5)
  {
    while(i_front<30 and i_back<15)
    {
      i_front++;
      i_back++;
      Frontlight.setPixelColor(i_front,255,150,0);
      Backlight.setPixelColor(i_back,255,150,0);
      Frontlight.show();
      Backlight.show();
      delay(50);
    }  
  }



  if(customKeypad=6){
    i_front = 30 ;
    i_back = 30 ;
    while(i_front<0 and i_back<15)
    {
      i_front--;
      i_back--;
      Frontlight.setPixelColor(i_front,255,150,0);
      Backlight.setPixelColor(i_back,255,150,0);
      Frontlight.show();
      Backlight.show();
      delay(50);

    }

  }

}


/* switch
 * 
 * Each time the input pin goes from LOW to HIGH (e.g. because of a push-button
 * press), the output pin is toggled from LOW to HIGH or HIGH to LOW.  There's
 * a minimum delay between toggles to debounce the circuit (i.e. to ignore
 * noise).  
 *
 * David A. Mellis
 * 21 November 2006
 */

int inPin = 2;         // the number of the input pin
int outPin = 13;       // the number of the output pin

int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

void setup()
{
  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);
}

void loop()
{
  reading = digitalRead(inPin);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == HIGH)
      state = LOW;
    else
      state = HIGH;

    time = millis();    
  }

  digitalWrite(outPin, state);

  previous = reading;
}
