#include <Arduino.h>
#include <Arduino_LSM9DS1.h>

bool isRunning= true;
volatile int counter = 0;
bool isOn = false;
volatile bool useAudio = false;
bool useIMU = true;
const int buzzerPin = 2;
const int buttonInteruptPin = 4;
int freq = 1000;
float x, y, z;


//sound code from https://github.com/suskind/arduino-knight_rider_theme/blob/master/knight_rider.ino
#include "pitches.h" 

/* NOTES AND TONES */
typedef struct {
  int note;
  int tempo;
} notesType;

const notesType aNotes[] = {
    // 1
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 125}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_GS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 125}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_GS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_G4, 250}, {NOTE_GS4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 125}, {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_FS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 250}, {NOTE_GS4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 125}, {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_FS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    // 2
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 125}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_GS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 125}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_GS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_G4, 250}, {NOTE_GS4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 125}, {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_FS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 250}, {NOTE_GS4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 125}, {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_FS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    // 3
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 125}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_GS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, 
    {NOTE_A4, 125}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_GS4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, {NOTE_A4, 125}, 
    {NOTE_G4, 250}, {NOTE_GS4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 125}, {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_FS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 250}, {NOTE_GS4, 125}, {NOTE_G4, 125}, 
    {NOTE_G4, 125}, {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_GS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    {NOTE_FS4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, {NOTE_G4, 125}, 
    // solo 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_E5, 1500}, 
    {NOTE_A5, 250}, {NOTE_AS5, 125}, {NOTE_A5, 125}, {NOTE_E5, 1500}, 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_E5, 250}, {NOTE_A5, 250}, {NOTE_G5, 2000}, 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_E5, 1500}, 
    {NOTE_A5, 250}, {NOTE_AS5, 125}, {NOTE_A5, 125}, {NOTE_E5, 1500}, 
    {NOTE_A4, 250}, {NOTE_AS4, 125}, {NOTE_A4, 125}, {NOTE_E5, 250}, {NOTE_A5, 250}, {NOTE_AS5, 2500}, {NOTE_G5, 250}, {NOTE_A5, 500}
};

int noteIndex = -1;
int totalNotes;

unsigned long tonePrevTime = 0;
int noteDelay = aNotes[(noteIndex + 1)].tempo;
unsigned long curTime = 0; 

//servo
#include <Servo.h>
Servo servo;
const bool withServo = true;
const int SERVO_PIN     = 3;  // (needs PWM pin)

//ble
#include "ArduinoBLE.h"
BLEService ControllerService = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLECharCharacteristic Movement = BLECharCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

//Light Dependent Resistor (LDR)
volatile bool useLDR = true;
const int LDR_PIN = A0;

void SetServo(int state){
  switch (state)
  {
    case 0:
      servo.write(90); //0 degrees
    break;
    case 1:
      servo.write(45); //0 degrees
    break;
    case 2:
      servo.write(0); //0 degrees
    break;
    default: 
    break;
  }
}

void ToggleSound() {
//static unsigned long last_interrupt_time = 0;
//unsigned long interrupt_time = millis();

 // If interrupts come faster than 200ms, assume it's a bounce and ignore. 
 //Can we do better? Yes, but then we need to go even lower level with manual interupt on/off.
 //if (interrupt_time - last_interrupt_time > 200) {
  useAudio= !useAudio;
 //}
 
 //last_interrupt_time = interrupt_time;
}

void ToggleLED() {
  isOn = !isOn;
  digitalWrite(LED_BUILTIN, isOn);
}

void PlaySound() {
if (counter > 20)
{
   if(curTime - tonePrevTime >= noteDelay) {
        if (noteIndex +1 <totalNotes)
        {
        noteIndex++;
        tonePrevTime = curTime;
        noTone(buzzerPin);
        tone(buzzerPin, aNotes[noteIndex].note, aNotes[noteIndex].tempo);

        noteDelay = aNotes[noteIndex].tempo;
        }
        else {
          if (withServo)
            { 
              SetServo(0);
              delay(2000);
              SetServo(1);
              delay(2000);
              SetServo(2);
              delay(2000);
              SetServo(0);
              delay(2000);            
            }
            noteIndex = -1;
            noteDelay = aNotes[(noteIndex + 1)].tempo;
            useAudio = false;
            counter = 0;
        }
    }
}
else
{
  noTone(buzzerPin);
  tone(buzzerPin, freq + counter*100, 500);
  delay(2000);
}
}

void ShowGyro() {
if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(", ");
        Serial.println(z);
    }
}

void ReadLDR()
{
    int sensorValue = analogRead(LDR_PIN);
    float voltage = sensorValue * (5.0 / 1023.0); //3.3 volt divided 10 bit =1024 values (using 10 bit ADC converter) 
    Serial.println(voltage);
}

void setup() {
  //Set pin 13 (buildin LED) to output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonInteruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonInteruptPin), ToggleSound, CHANGE);

  //Start serial
  Serial.begin(115200);

  // Wait for serial connection to be ready (eg. open your terminal)
  //while (!Serial);
  
  if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU");
        while (1);
    }
    
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Gyroscope in degrees/second");
    Serial.println("X, Y, Z"); 

    //sound
    totalNotes = sizeof(aNotes) / sizeof(notesType); 

    //servo
    if (withServo)
    {
    servo.attach(SERVO_PIN);
    SetServo(1);
    delay(1000);
    }

    //ble
    if (!BLE.begin()) {
      Serial.println("starting BLE failed!");    
    }  
    else {
      // set local name and advertised service for BLE:
      BLE.setLocalName("IRTS-BOT");
      BLE.setAdvertisedService(ControllerService);
    
      // add the characteristic and service:
      ControllerService.addCharacteristic(Movement);
      BLE.addService(ControllerService);
    
      // start advertising
      BLE.advertise(); 
    }

    //LDR
    pinMode(A0, INPUT);
}

void loop() {
  if (isRunning)
  {
    BLEDevice central = BLE.central();

    if (central) {
          // print the central's BT address:
          Serial.print("Connected to central: ");
          Serial.println(central.address());
      
          while (central.connected()) {
            if (Movement.written())
            {
              unsigned char receivedContent = Movement.value();
              Serial.print(receivedContent);
        
              if (withServo)
              {
              servo.write(receivedContent); //180 deg max. What is the max of receivedContent?
              delay(100);
              }
            }
            else
            {
                //do nothing
            }
          }
    }
    else
    {
      counter++;
      curTime = millis();

      ToggleLED();

      
      if (useIMU)
        ShowGyro();

      if (useAudio)
        PlaySound();
      else
        delay(2000);

      if (useLDR)
        ReadLDR();
    }
  }
}



