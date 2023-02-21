#include <SPI.h>
#include <SD.h>
#include <Arduino_MKRENV.h>


const char filename[] = "test.csv";
File myFile;

// const int chipSelect = SDCARD_SS_PIN;
float launchPadElevation; // Elevation of the launch pad (in m). Rocket's altitude is measured relative to this elevation.
bool launchOccurred = false; // False until a launch is detected.

// initialize variables to store measurements
float time;
float startTime;
float temperature;
float humidity;
float pressure;
float currentElevation;
float altitude; 
float illuminance;
float uva;
float uvb;
float uvIndex;

// ALTIMETER settings
bool write2SDCard = false; // Will write to SD card if true.
float triggerAltitude = 0.50; // Arduino will only start recording once it crosses this altitude threshold (this is in meters).
int padSetUpTime = 5000; // Length of time between sensor set up and launch pad elevation calibration (in ms) -- nominal value: 60000 (1 min)
int samplingInterval = 10; // time between readings (in ms) -- nominal value: 10 
int sampleNumber = 25; // number of readings to average -- nominal value: 25
int padElevationSamplingInterval = 500; // time between averaged readings to determine pad elevation (in ms) -- nominal value: 500
int padElevationSampleNumber = 20; // number of averaged readings to determine pad elevation -- nominal value: 20

void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Check if the ENV shield is responsive 
  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }

  // Will only run if write2SDCard is true
  if (write2SDCard) {
    Serial.print("Initializing SD card...");
    
    if (!SD.begin(28)) {
      Serial.println("initialization failed!");
      while (1);
    }
    Serial.println("initialization done.");
     // delete the old version of the file
    SD.remove("test.csv");

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    myFile = SD.open("test.csv", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) {
      Serial.print("Writing to test.txt...");
      myFile.println("Time (s), Altitude (m), Pressure (kPa), Temperature (C)");
      // close the file:
      myFile.close();
      Serial.println("done.");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
    }
  }
  
  // re-open the file for reading:
  // myFile = SD.open("test.csv");
  // if (myFile) {
  //   Serial.println("test.csv:");

  //   // read from the file until there's nothing else in it:
  //   while (myFile.available()) {
  //     Serial.write(myFile.read());
  //   }
  //   // close the file:
  //   myFile.close();
  // } else {
  //   // if the file didn't open, print an error:
  //   Serial.println("error opening test.csv");
  // }

  // Give visual indication that all checks have been passed
  int onTime = 1000; // time the LED is on (in ms)
  int offTime = 500; // time the LED is off (in ms)
  int repeat = 5; // number of times the LED should blink
  blink(onTime, offTime, repeat);
  
  // Do nothing for the next X seconds. This give students time to put the altimeter inside the rocket
  delay(padSetUpTime); 

  // At this point the altimeter sits inside the rocket on the pad.
  // Calculate elevation of launchPad by averaging the pressure readings taken in the next x seconds
  
  launchPadElevation = getPadElevation(padElevationSamplingInterval, padElevationSampleNumber);
  String elevationMessageHeader = "Pad Elevation (m) is: ";
  String elevationMessage = elevationMessageHeader + launchPadElevation;
  Serial.println(elevationMessage); // for debugging only
}

void loop() {
  // Obtain measurements from sensor 

  time = -1; // default value of time 

  if (launchOccurred){
    time = (millis() - startTime)/1000;
  } 
  getMeasurements(samplingInterval, sampleNumber); // obtains pressure and temperature measurements 
  currentElevation = getAltitude(pressure); // calculates elevation (in m) relative to sea level based on pressure reading
  altitude = currentElevation - launchPadElevation; // calculates elevation (in m) relative to launch pad

  Serial.print(time);
  Serial.print(", ");
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(temperature);
  Serial.println();

  if (!launchOccurred){
    // Check if the trigger altitude has been crossed. If so, consider the rocket launched. 
    if (currentElevation - launchPadElevation > triggerAltitude) {
      launchOccurred = true; 
      startTime = millis(); // measures how long the arduino has been on before launch 
      Serial.println("Launch detected!");
    }     
  } 
  else { 
    Serial.println("Altimeter is in flight");
    if (write2SDCard) {
      // Start writing to SD Card
      Serial.println('writing to SD card'); 
      myFile = SD.open("test.csv", FILE_WRITE);
      myFile.print(time);
      myFile.print(",");
      myFile.print(altitude);
      myFile.print(",");
      myFile.print(pressure);
      myFile.print(",");
      myFile.println(temperature);
      myFile.close();
    }
  }
  // delay(measurementInterval);
}

void blink(int onTime, int offTime, int repeat) {
  for (int i = 0; i < repeat; i++) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(onTime);                      // wait 
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(offTime);                     // wait
  }
}
void getMeasurements(int samplingInterval, int sampleNumber){
  /*
  Function to return averaged values of sensor readings (raw sensor readings show too much noise)
  */
  float pressure_avg = 0;
  float temperature_avg = 0; 

  for (int i = 0; i < sampleNumber; i++){ 
    float pressureReading = ENV.readPressure();
    float temperatureReading = ENV.readTemperature();
    // Serial.println(pressureReading); // for debugging only
    pressure_avg += pressureReading;
    temperature_avg += temperatureReading;
    delay(samplingInterval);    
  }
  pressure = pressure_avg / sampleNumber;
  temperature = temperature_avg / sampleNumber;
} 

float getAltitude(float pressure){
  /*
  Function for converting pressure readings into elevation. 
  Returns elevation in meters.
  */
  return 145366.45*(1-pow(pressure*10/1013.25,(0.190284)));
}

float getPadElevation(int padElevationSamplingInterval, int padElevationSampleNumber){
  /*
  Function for obtaining launch pad elevation. 
  Averages multiple pressure readings and reports the corresponding elevation (in meters).
  */
  float pressure_avg = 0;
  float temperature_avg = 0;
  for (int i = 0; i < sampleNumber; i++){ 
    getMeasurements(samplingInterval, sampleNumber);
    pressure_avg += pressure;
    temperature_avg += temperature;
    delay(samplingInterval);    
  }
  pressure_avg = pressure_avg / sampleNumber;
  // Serial.println(pressure_avg); // for debugging only
  return getAltitude(pressure_avg);  
}