/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Circuit:
  - Arduino Nano 33 BLE board, or
  - Arduino Nano RP2040 Connect, or
  - Arduino Portenta H7 board plus Portenta Vision Shield

  This example code is in the public domain.
*/

#include <PDM.h>
#include <Arduino_APDS9960.h>
#include <Servo.h>


/////////////PDM///////
// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;




unsigned long currentTime = millis();
//////////servoA

Servo servoA;
int servoAPin = 4;
int increment1 = 3;
int increment2 = 3;
unsigned long lastUpdate1 = 0;
unsigned long lastUpdate2 = 0;
int posA = 0;


////////////servoB
Servo servoB;
int servoBPin = A3;
unsigned long lastUpdate3 = 0;
int posB = 0;
int increment3 = 1;


/////////LED A
int ledAPin = 3;
int brightnessA = 0;
int fadeAmount1 = 5;
int lastLit1 = 0;


/////////LED B
int ledBPin = 2;
int brightnessB = 0;
int fadeAmount2 = 6;
int lastLit2 = 0;


////////LED C
int ledCPin = A2;
int brightnessC = 0;
int fadeAmount3 = 8;
int lastLit3 = 0;

////////LED D
int ledDPin = A5;
int brightnessD = 0;
int fadeAmount4 = 7;
int lastLit4 = 0;

void setup() {
  Serial.begin(9600);
  //  while (!Serial);

  servoA.attach(servoAPin);
  servoB.attach(servoBPin);
  pinMode(ledAPin, OUTPUT);
  pinMode(ledBPin, OUTPUT);

  //////////////// Configure the data receive callback for PDM (audio sensor)
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }


  //////////////// APDS sensor for proximity
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor!");
  }
}

void loop() {
  unsigned long currentTime = millis();
  // Wait for samples to be read
  if (samplesRead) {

    // Print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++) {

      int soundVol = constrain(sampleBuffer[i], 10, 1600);

      /////////servoA behaviour
      if (soundVol < 150) {
        if (currentTime - lastUpdate1 > 1000) {
          Serial.println("Sound1:");
          Serial.println(soundVol);

          servoA.write(posA);
          posA += increment1;
          lastUpdate1 = currentTime;

          if (posA <= 0 || posA >= 180) {
            increment1 = - increment1;
          }
        }
      }
  if (soundVol >= 150 && soundVol < 300) {
        if (currentTime - lastUpdate1 > 100) {

          Serial.println("Sound2:");
          Serial.println(soundVol);

          servoA.write(posA);
          posA += increment2;
          lastUpdate1 = currentTime;

          if (posA <= 0 || posA >= 180) {
            increment2 = -increment2;
          }
        }
      }
      if (soundVol >= 300) {
        if (currentTime - lastUpdate1 > 100) {
          Serial.println("Sound3:");
          Serial.println(soundVol);

          servoA.write(random(50, 90));
          lastUpdate1 = currentTime;
        }
      }
    }

  }
  // Clear the read count
  samplesRead = 0;




  // check if a proximity reading is available
  if (APDS.proximityAvailable()) {
    int proximity = APDS.readProximity();

    /////////////servoB behaviour
    if (currentTime - lastUpdate3 > 100) {


      if (proximity > 240) {
        posB += increment3;
        if (posB <= 0 || posB >= 180) {
          increment3 = -increment3;
        }
      }
      if (proximity <= 240) {
        posB = map(proximity, 3, 240, 0, 180);
      }
      if (proximity < 3){
        posB = random (80,110);
        }
      servoB.write(posB);
      lastUpdate3 = currentTime;
      
    }


    /////////////led A
    if (currentTime - lastLit1 > 100) {
      if (proximity > 240) {
        brightnessA += fadeAmount1;
        if (brightnessA <= 0 || brightnessA >= 70) {
          fadeAmount1 = -fadeAmount1;
        }
      }
      if (proximity <= 240) {
        brightnessA = map(proximity, 0, 240, 255, 0);
      }

      analogWrite(ledAPin, brightnessA);
      lastLit1 = currentTime;
    }
    /////////////led B
    if (currentTime - lastLit2 > 120) {
      if (proximity > 240) {
        brightnessB += fadeAmount2;
        if (brightnessB <= 0 || brightnessB >= 60) {
          fadeAmount2 = -fadeAmount2;
        }
      }
      if (proximity <= 240) {
        brightnessB = map(proximity, 0, 240, 255, 0);
      }

      analogWrite(ledBPin, brightnessB);
      lastLit2 = currentTime;
    }
    
    /////////////led C
    if (currentTime - lastLit3 > 110) {
      if (proximity > 240) {
        brightnessC += fadeAmount3;
        if (brightnessC <= 0 || brightnessC >= 70) {
          fadeAmount3 = -fadeAmount3;
        }
      }
      if (proximity <= 240) {
        brightnessC = map(proximity, 0, 240, 255, 0);
      }

      analogWrite(ledCPin, brightnessC);
      lastLit3 = currentTime;
    }
    
    /////////////led D
    if (currentTime - lastLit4 > 115) {
      if (proximity > 240) {
        brightnessD += fadeAmount4;
        if (brightnessD <= 0 || brightnessD >= 80) {
          fadeAmount4 = -fadeAmount4;
        }
      }
      if (proximity <= 240) {
        brightnessD = map(proximity, 0, 240, 255, 0);
      }

      analogWrite(ledDPin, brightnessD);
      lastLit4 = currentTime;
    }
  }

}




/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
