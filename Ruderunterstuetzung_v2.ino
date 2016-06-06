//
// Ruderunterstützung v2.5
// Autor: Kai Laborenz
//
// Steuert zwei Fahrtregler so, dass bei Kurvenfahrt ein Motor langsamer läuft und
// bei Stillstand auf der Stelle gedreht werden kann
//
// Mit Unterstützung von rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details
//#include <PinChangeInt.h>

#include <Servo.h>
#include <EEPROM.h>


#include <Wire.h>
#include <GOFi2cOLED.h>

GOFi2cOLED GOFoled;


// Basiswerte für RC-Signale, falls noch keine ermittelt wurden
#define RC_NEUTRAL 1500
#define RC_MAX 2000
#define RC_MIN 1000

#define RC_DEADBAND 40
#define START_MOD 200     // Wert für Ruderausschlag, ab dem die Runderunterstützung beginnt
#define MAX_MOD   50      // Wert, an dem die Runderunterstützung maximal ist (Ruder hart BB oder STB)

// Basiswerte für die Steuerung der ESCs
#define PWM_MIN 0
#define PWM_MAX 255

// Startwerte für Speed
uint16_t speedMin = RC_MIN;
uint16_t speedMax = RC_MAX;
uint16_t speedCenter = RC_NEUTRAL;

// Startwerte für Ruder
uint16_t rudderMin = RC_MIN;
uint16_t rudderMax = RC_MAX;
uint16_t rudderCenter = RC_NEUTRAL;

// Eingangspins für RC-Signale
// todo: PinChange-Library entfernen
#define SPEED_IN_PIN 2
#define RUDDER_IN_PIN 3

// Assign your channel out pins
#define ESC_STB_PIN 6
#define ESC_BBB_PIN 7

#define PROGRAM_PIN 4 // Schalter, um den Programmiermodus zu starten
#define MODE_PIN 8 // Schalter, um den Betriebmodus umzuschalten

// Servo-Objekte zum Steuern der ESCs
Servo escStb;
Servo escBb;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define SPEED_FLAG 1
#define RUDDER_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t speedInShared;
volatile uint16_t rudderInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t speedStart;
uint32_t rudderStart;

// Index into the EEPROM Storage assuming a 0 based array of uint16_t
// Data to be stored low byte, high byte
#define EEPROM_INDEX_SPEED_MIN 0
#define EEPROM_INDEX_SPEED_MAX 1
#define EEPROM_INDEX_SPEED_CENTER 2

#define EEPROM_INDEX_RUDDER_MIN 3
#define EEPROM_INDEX_RUDDER_MAX 4
#define EEPROM_INDEX_RUDDER_CENTER 5

// Programmodi
#define MODE_RUN 0      // Normaler Programmablauf, Betriebszustand
#define MODE_PROGRAM 1  // Programmiermodus, in dem die RC-Kanäle gelesen und die individuellen Maximalwerte bestimmt werden

#define MODE_TURN 2     // Auf der Stelle drehen durch gegenläufige Propeller
#define MODE_BS 3       // Nur Bugstrahlruder 

uint8_t gMode = MODE_RUN; // Zum Start erst einmal in den RUN-Modus
uint32_t ulProgramModeExitTime = 0; 

// Mit den Modifikatoren werden die Signale für die ESCs verringert,
// um bei Kurvenfahrt den jeweils inneren Motor langsamer laufen zu lassen
float modStb=1;         // Modifikation des ESC-Signals für Steuerbord
float modBb=1;          // Modifikation des ESC-Signals für Backbord


// constants won't change. Used here to set a pin number :
const int ledPin =  13;      // the number of the LED pin

// Variables will change :
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 100;           // interval at which to blink (milliseconds)


// Zum Einblenden der seriellen Debugausgaben
// die folgende Zeile auskommentieren
//#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)    Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x) 
#endif

#define DEBUG_OLED

void setup()
{
  Serial.begin(9600);
  
  Serial.println("Ruderhelfer 2.1");

  pinMode(ledPin, OUTPUT);

  // attach the interrupts to read the channels 
  pinMode(SPEED_IN_PIN, INPUT_PULLUP);
  pinMode(RUDDER_IN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_IN_PIN), calcThrottle,CHANGE); 
  attachInterrupt(digitalPinToInterrupt(RUDDER_IN_PIN), calcSteering,CHANGE);

  // attach servo objects
  escStb.attach(ESC_STB_PIN);
  escBb.attach(ESC_BBB_PIN);

  readSettingsFromEEPROM();
  DEBUG_PRINTLN("Startwerte aus EEPROM");
  DEBUG_PRINT("Speed: ");
  DEBUG_PRINT(speedMin);
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(speedMax);
  DEBUG_PRINT(" | ");
  DEBUG_PRINTLN(speedCenter);
  
  DEBUG_PRINT("Ruder: ");
  DEBUG_PRINT(rudderMin);
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(rudderMax);
  DEBUG_PRINT(" | ");
  DEBUG_PRINTLN(rudderCenter);

  #ifdef DEBUG_OLED
    GOFoled.init(0x3C);
    // init done
    
    delay(2000);
    GOFoled.clearDisplay();
    
    GOFoled.setTextSize(1);
    GOFoled.setTextColor(WHITE);
    GOFoled.setCursor(0,0);
    GOFoled.println("   Ruderhelfer 2.1");
  #endif
  
  delay(2000);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop.
  static uint16_t speedIn;
  static uint16_t rudderIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
    
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
    
    if(bUpdateFlags & SPEED_FLAG)
    {
      speedIn = speedInShared;
    }
    
    if(bUpdateFlags & RUDDER_FLAG)
    {
      rudderIn = rudderInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
    
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }
  #ifdef DEBUG_OLED
    GOFoled.setCursor(85,8);
    GOFoled.print("Mode: ");
    GOFoled.clearArea(121,8,5,7);
    GOFoled.print(String(gMode));
    GOFoled.display();    
    
    GOFoled.setCursor(0,16);
    GOFoled.print("RC in: " );
    GOFoled.clearArea(40,16,55,7);
    GOFoled.print(String(speedIn));
    GOFoled.print(" " );
    GOFoled.println(String(rudderIn));
    GOFoled.display();
  #endif

 //
 // Programmodus zum Kalibrieren der Maxima-, Minimal und Nullwerte
 //
 
  if(!digitalRead(PROGRAM_PIN))
  {
    // alle Kanäle auf Mittelstellung, dann Programmknopf drücken und
    // 10 Sekunden Zeit zum Bewegen aller Knüppel auf Maximalwerte in beide Richtungen
    
    ulProgramModeExitTime = millis() + 8000;
    gMode = MODE_PROGRAM;
    
    speedMin = RC_NEUTRAL;
    speedMax = RC_NEUTRAL;
    speedCenter = speedIn; 

    rudderMin = RC_NEUTRAL;
    rudderMax = RC_NEUTRAL;
    rudderCenter = rudderIn; 
    
    DEBUG_PRINTLN("Kalibrierung startet!");
    
    #ifdef DEBUG_OLED
      GOFoled.clearDisplay();
      GOFoled.setCursor(0,0);
      GOFoled.print(" Kalibrierung startet" );
    #endif
    delay(20);
    digitalWrite(ledPin, HIGH);
  }
  
  if(gMode == MODE_PROGRAM)
  {
   if(ulProgramModeExitTime < millis())
   {
     // set to 0 to exit program mode
     ulProgramModeExitTime = 0;
     gMode = MODE_RUN;
     
     writeSettingsToEEPROM();
     
     DEBUG_PRINTLN("Kalibrierung beendet");
    #ifdef DEBUG_OLED
      GOFoled.clearDisplay();
      GOFoled.setCursor(0,0);
      GOFoled.print("Kalibrierung beendet" );
    #endif
    
     delay(6000);
     digitalWrite(ledPin, LOW);
     
     #ifdef DEBUG_OLED
      GOFoled.clearDisplay();
      GOFoled.setCursor(0,0);
      GOFoled.println("   Ruderhelfer 2.1");
    #endif
   }
   else
   {
     // Maximalwerte für alle Kanäle ermitteln 
     // Speed
     if(speedIn > speedMax && speedIn <= RC_MAX)
     {
       speedMax = speedIn;
     }
     else if(speedIn < speedMin && speedIn >= RC_MIN)
     {
       speedMin = speedIn;
     }
     // Ruder
     if(rudderIn > rudderMax && rudderIn <= RC_MAX)
     {
       rudderMax = rudderIn;
     }
     else if(rudderIn < rudderMin && rudderIn >= RC_MIN)
     {
       rudderMin = rudderIn;
     }
   }
  }
    
 if(gMode == MODE_RUN)
  { 
  digitalWrite(ledPin, LOW);  
  if(bUpdateFlags)
  {
    
    if(escBb.readMicroseconds() != rudderIn) // ??
    {
      
      // Ruder hart Steuerbord
      if (rudderIn > (rudderMax - MAX_MOD)) {
        if (speedIn > speedCenter) { // vorwärts
          modBb=0.85;
          modStb=1.15;
        }
        else { //rückwärts
          modStb=0.85;
          modBb=1.15;        
        }
      }    
      // Ruder Steuerbord
      else if (rudderIn > (rudderMax - START_MOD)) {
        if (speedIn > speedCenter) { // vorwärts
          modBb=0.95;
          modStb=1.05;
        }
        else { //rückwärts
          modStb=0.95;
          modBb=1.05;        
        }
      }
      
      // Ruder hart Backbord
      else if (rudderIn < (rudderMin + MAX_MOD)) {
        if (speedIn > speedCenter) { // vorwärts
          modBb=1.15;
          modStb=0.85;
        }
        else { //rückwärts
          modStb=1.15;
          modBb=0.85;
        }
      }
      // Ruder Backbord
      else if (rudderIn < (rudderMin + START_MOD)) {
        if (speedIn > speedCenter) { // vorwärts
          modBb=1.05;
          modStb=0.95;
        }
        else { //rückwärts
          modStb=1.05;
          modBb=0.95;
        }
      }
           
      // Wenn Ruder und Speed beide nahe dem Mittelpunkt sind, wird in den "Auf-der-Stelle"-Modus geschaltet
      else if (((rudderIn > (rudderCenter - RC_DEADBAND)) && (rudderIn < (rudderCenter + RC_DEADBAND))) && (speedIn > (speedCenter - RC_DEADBAND)) && (speedIn < (speedCenter + RC_DEADBAND)))
      {
        gMode = MODE_TURN;
        }
      // beide Motoren ohne Modifikation
      else 
      {
        modStb=1;
        modBb=1;
      }
    }
          
    if(escStb.readMicroseconds() != speedIn) // ??
      {
        DEBUG_PRINT(gMode);
        DEBUG_PRINT(" | IN Speed/Ruder: ");
        DEBUG_PRINT(speedIn);
        DEBUG_PRINT(" / ");
        DEBUG_PRINT(rudderIn);
        DEBUG_PRINT(" | Mod Stb/Bb: ");
        DEBUG_PRINT(modStb);
        DEBUG_PRINT(" / ");
        DEBUG_PRINT(modBb);

        #ifdef DEBUG_OLED
          GOFoled.setCursor(0,26);
          GOFoled.print("Mod Stb/Bb: " );
          GOFoled.clearArea(60,26,65,7);
          GOFoled.print(String(modStb));
          GOFoled.print(" " );
          GOFoled.println(String(modBb));
          GOFoled.display();
        #endif
        
        // Motoren modifiziert ansteuern
        if (speedIn > speedCenter) {
          escStb.writeMicroseconds(constrain(speedIn*modStb,speedCenter,speedMax));
          escBb.writeMicroseconds(constrain(speedIn*modBb,speedCenter,speedMax));
          
        DEBUG_PRINT(" | ESCSTB: ");
        DEBUG_PRINT(constrain(speedIn*modStb,speedCenter,speedMax));
        DEBUG_PRINT(" | ESCBB: ");
        DEBUG_PRINTLN(constrain(speedIn*modBb,speedCenter,speedMax));
        
        #ifdef DEBUG_OLED
          GOFoled.setCursor(0,36);
          GOFoled.print("ESC Stb: " );
          GOFoled.clearArea(45,36,55,7);
          GOFoled.println(String(constrain(speedIn*modStb,speedCenter,speedMax)));
          GOFoled.print("ESC Bb: " );
          GOFoled.clearArea(45,44,55,7);
          GOFoled.print(String(constrain(speedIn*modBb,speedCenter,speedMax)));
          GOFoled.display();
        #endif
          
        }
        else {
          escStb.writeMicroseconds(constrain(speedIn*modStb,speedMin,speedCenter));     
          escBb.writeMicroseconds(constrain(speedIn*modBb,speedMin,speedCenter));
          
        DEBUG_PRINT(" | ESCSTB: ");
        DEBUG_PRINT(constrain(speedIn*modStb,speedMin,speedCenter));
        DEBUG_PRINT(" | ESCBB: ");
        DEBUG_PRINTLN(constrain(speedIn*modBb,speedMin,speedCenter));

        #ifdef DEBUG_OLED
          GOFoled.setCursor(0,36);
          GOFoled.print("ESC Stb: " );
          GOFoled.clearArea(45,36,75,7);
          GOFoled.println(String(constrain(speedIn*modStb,speedCenter,speedMax)));
          GOFoled.print("ESC Bb: " );
          GOFoled.clearArea(45,44,75,7);
          GOFoled.print(String(constrain(speedIn*modBb,speedCenter,speedMax)));
          GOFoled.display();
        #endif  
        }
      }
  }

  bUpdateFlags = 0;
  } // ende MODE_RUN

  if(gMode == MODE_TURN)
  {
    digitalWrite(ledPin, HIGH);
    // Wenn der Speedkanal benutzt wird, zurück zum normalen Fahrmodus schalten
    if ((speedIn < (speedCenter - RC_DEADBAND)) || (speedIn > (speedCenter + RC_DEADBAND)))
      {
        gMode = MODE_RUN;
        }
    // Motoren gegenläufig steuern
    // todo: hier müssen beide Richtungen berücksichtigt werden  
    else {
      if(bUpdateFlags){
        if(escBb.readMicroseconds() != rudderIn) {
          
          DEBUG_PRINT(gMode);
          DEBUG_PRINT(" | ESC-Werte: ");
          DEBUG_PRINT(rudderIn);
          DEBUG_PRINT(" / ");
          DEBUG_PRINTLN(map(rudderIn, 1000,2000,2000,1000));

          #ifdef DEBUG_OLED
            GOFoled.setCursor(0,54);
            GOFoled.print("Turn: " );
            GOFoled.clearArea(37,54,80,7);
            GOFoled.print(String(rudderIn));
            GOFoled.print(" " );
            GOFoled.print(String(map(rudderIn, 1000,2000,2000,1000)));
            GOFoled.display();
          #endif 
          
          escStb.writeMicroseconds(rudderIn);
          escBb.writeMicroseconds(map(rudderIn, 1000,2000,2000,1000));
        }
      }
    }  
  } // ende MODE_TURN
} // ende loop


// simple interrupt service routine
void calcThrottle() {
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(SPEED_IN_PIN) == HIGH) { 
    speedStart = micros();
  }
  else {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    speedInShared = (uint16_t)(micros() - speedStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= SPEED_FLAG;
  }
}

void calcSteering() {
  if(digitalRead(RUDDER_IN_PIN) == HIGH) { 
    rudderStart = micros();
  }
  else {
    rudderInShared = (uint16_t)(micros() - rudderStart);
    bUpdateFlagsShared |= RUDDER_FLAG;
  }
}

// Speicherung der ausgelesenen Maximal- und Minimalwerte für die RC-Kanäle im EEPROM

void readSettingsFromEEPROM() {
  // Speed
  speedMin = readChannelSetting(EEPROM_INDEX_SPEED_MIN);
  if(speedMin < RC_MIN || speedMin > RC_NEUTRAL) {
    speedMin = RC_MIN;
  }
  speedMax = readChannelSetting(EEPROM_INDEX_SPEED_MAX);
  if(speedMax > RC_MAX || speedMax < RC_NEUTRAL) {
    speedMax = RC_MAX;
  }
  speedCenter = readChannelSetting(EEPROM_INDEX_SPEED_CENTER);
  if(speedCenter < speedMin || speedCenter > speedMax) {
    speedCenter = RC_NEUTRAL;
  }

  // Ruder
  rudderMin = readChannelSetting(EEPROM_INDEX_RUDDER_MIN);
  if(rudderMin < RC_MIN || rudderMin > RC_NEUTRAL) {
    rudderMin = RC_MIN;
  }
  rudderMax = readChannelSetting(EEPROM_INDEX_RUDDER_MAX);
  if(rudderMax > RC_MAX || rudderMax < RC_NEUTRAL) {
    rudderMax = RC_MAX;
  }
  rudderCenter = readChannelSetting(EEPROM_INDEX_RUDDER_CENTER);
  if(rudderCenter < rudderMin || rudderCenter > rudderMax) {
    rudderCenter = RC_NEUTRAL;
  } 
}

void writeSettingsToEEPROM()
{ 
  writeChannelSetting(EEPROM_INDEX_SPEED_MIN,speedMin);
  writeChannelSetting(EEPROM_INDEX_SPEED_MAX,speedMax);
  writeChannelSetting(EEPROM_INDEX_SPEED_CENTER,speedCenter);

  writeChannelSetting(EEPROM_INDEX_RUDDER_MIN,rudderMin);
  writeChannelSetting(EEPROM_INDEX_RUDDER_MAX,rudderMax);
  writeChannelSetting(EEPROM_INDEX_RUDDER_CENTER,rudderCenter);

  DEBUG_PRINT("Speed: ");
  DEBUG_PRINT(speedMin);
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(speedMax);
  DEBUG_PRINT(" | ");
  DEBUG_PRINTLN(speedCenter);
  
  DEBUG_PRINT("Ruder: ");
  DEBUG_PRINT(rudderMin);
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(rudderMax);
  DEBUG_PRINT(" | ");
  DEBUG_PRINTLN(rudderCenter);

  #ifdef DEBUG_OLED
    GOFoled.setCursor(0,26);
    GOFoled.println("Speed: " );
    GOFoled.print(String(speedMin));
    GOFoled.print(" " );
    GOFoled.print(String(speedMax));
    GOFoled.print(" " );
    GOFoled.println(String(speedCenter));
    GOFoled.setCursor(0,44);
    GOFoled.println("Ruder: " );
    GOFoled.print(String(rudderMin));
    GOFoled.print(" " );
    GOFoled.print(String(rudderMax));
    GOFoled.print(" " );
    GOFoled.print(String(rudderCenter));
    
    GOFoled.display();
  #endif 

  
}


uint16_t readChannelSetting(uint8_t nStart) {
  uint16_t unSetting = (EEPROM.read((nStart*sizeof(uint16_t))+1)<<8);
  unSetting += EEPROM.read(nStart*sizeof(uint16_t));
  
  return unSetting;
}

void writeChannelSetting(uint8_t nIndex,uint16_t unSetting) {
  EEPROM.write(nIndex*sizeof(uint16_t),lowByte(unSetting));
  EEPROM.write((nIndex*sizeof(uint16_t))+1,highByte(unSetting));
}
