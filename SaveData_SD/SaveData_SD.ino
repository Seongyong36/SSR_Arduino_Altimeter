// Import necessary libraries 
#include <SPI.h>
#include <SD.h> // library to write and read to SD card
#include <Arduino_MKRENV.h> // library to use the MKR ENV shield
#include "ArduinoLowPower.h" // library to use Low Power modes 

// initialize variables to store measurements
float time;
float startTime;
float pressure;
float currentElevation;
float altitude; 
float voltage;  // voltage of battery (in volts)

// altimeter parameters DO NOT EDIT THESE UNLESS YOU KNOW EXACTLY WHAT YOU ARE DOING!
File myFile;
String fileNameHeader = "Launch_"; // All launch data files will start with the words "Launch_..."
String fileType = ".csv"; // Launch data files will be written in .csv files
String fileName = "";
String dataFileHeader = "Time (s), Altitude (m), Pressure (kPa), Temperature (C), Voltage (V)"; // First row of launch data file
const int chipSelect = 4; //SDCARD_SS_PIN;
bool write2SDCard = true; // Will write to SD card if true.
float triggerAltitude = 3.00; // Arduino will only start recording once it crosses this altitude threshold (this is in meters).
float launchPadElevation = -1; // Elevation of the launch pad (in m). Rocket's altitude is measured relative to this elevation.
bool launchDetected = false; // False until a launch is detected. 
bool launchEnded = false; // False until the end of the launch is detected.
int padSetUpTime = 60000; // Length of time between sensor set up and launch pad elevation calibration (in ms) -- nominal value: 60000 (1 min)
int samplingInterval = 1; // time between readings (in ms) -- nominal value: 1
int sampleNumber = 15; // number of readings to average -- nominal value: ?? 
int padElevationSamplingInterval = 1000; // time between averaged readings to determine pad elevation (in ms) -- nominal value: 1000
int padElevationSampleNumber = 25; // number of averaged readings to determine pad elevation -- nominal value: 25
float batteryMaxVoltage = 4.30; // Voltage of a fully charged battery -- nominal value 4.30  
int deepSleepDuration = 10000; // Go to deep sleep after launch 
 
void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  // Check if the ENV shield is responsive 
  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }

  // Will only run if write2SDCard is true
  if (write2SDCard) {

    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
    Serial.println("initialization done.");

    // Find a file name that is not taken already. Use while loop to increment launch number.
    int launchNumber = 1;    
    fileName = fileNameHeader + String(launchNumber) + fileType;
    while (SD.exists(fileName)) {
      // Check if this file contains launch data, if not, stop incrementing the launch number.
      File file = SD.open(fileName);
      int fileOutput = file.read();
      file.close();
      if (fileOutput == -1){
        break;
      }     
      launchNumber++;
      fileName = fileNameHeader + String(launchNumber) + fileType;
      Serial.println(fileName); 
    }

    // Open that file. If the file opened okay, write to it:
    myFile = SD.open(fileName, FILE_WRITE);
    if (!myFile) {
      // if the file didn't open, print an error:
      Serial.println("error opening " + fileName);
    }
  }

  // Blink the indicator LED to give visual indication that all checks have been passed. 
  // While the LED is still blinking, the students should put the altimeter inside the rocket
  int onTime = 1000; // time the LED is on (in ms)
  int offTime = 1000; // time the LED is off (in ms)
  int repeat = padSetUpTime / (onTime + offTime); // number of times the LED should blink
  blink(onTime, offTime, repeat);

  // At this point the altimeter sits inside the rocket on the pad.
  // Calculate elevation of launchPad by averaging the pressure readings taken in the next x seconds
  launchPadElevation = getPadElevation(padElevationSamplingInterval, padElevationSampleNumber);
  String messageHeader = "Pad Elevation (m) is: ";
  String elevationMessage = messageHeader + launchPadElevation;
  Serial.println(elevationMessage); // for debugging only
}

void loop() {

  // Obtain measurements 
  getMeasurements(samplingInterval, sampleNumber); // obtains time and pressure measurements 

  currentElevation = getAltitude(pressure); // calculates elevation (in m) relative to sea level based on pressure reading
  altitude = currentElevation - launchPadElevation; // calculates elevation (in m) relative to launch pad
  String dataString = "Measurements: ";
  dataString = dataString + String(time) + " s, " + String(altitude) + " m, " + String(pressure) + " kPa" + String(voltage) + " V";
  Serial.println(dataString);
  
  // Run if launch has not been detected
  if (!launchDetected){

    // Check if the trigger altitude has been crossed. This block of code ends up only running once.  
    if (altitude > triggerAltitude) {
      Serial.println("Launch detected!");
      launchDetected = true; 
      startTime = millis(); // measures how long the arduino has been on before launch 
      // Write the file headers
      myFile = SD.open(fileName, FILE_WRITE);
      myFile.println(dataFileHeader);
      myFile.close();
    }
  } 
  
  // Runs if launch has been detected and has not ended.
  else if (!launchEnded) { 
    
    // Check that the rocket is still in flight 
    if (altitude < triggerAltitude){
      launchEnded = true;
    }

    Serial.println("Altimeter is in flight");
    if (write2SDCard) {
      // Start writing to SD Card
      Serial.println("writing to SD card"); 
      myFile = SD.open(fileName, FILE_WRITE);
      myFile.println(dataString);
      myFile.close();
    }
  }
  
  // run if launch has been detected but it has ended
  else {
    Serial.println("Post launch!");

    // Go to deep sleep to conserve battery until the rocket is retrieved.
    LowPower.deepSleep(deepSleepDuration);
  }
}

void blink(int onTime, int offTime, int repeat) {
  for (int i = 0; i < repeat; i++) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(onTime);                      // wait 
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(offTime);                     // wait
  }
}
float getBatteryVoltage() {
  /*
  Function to read voltage of the external Lipo Battery
  */
  int sensorValue = analogRead(ADC_BATTERY); // read the input on analog pin 0:
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  voltage = sensorValue * (batteryMaxVoltage / 1023.0);
}
void getMeasurements(int samplingInterval, int sampleNumber) {
  /*
  Function to return averaged values of sensor readings (raw sensor readings show too much noise) AND time
  */
  float pressure_avg = 0;
  float temp_avg = 0;
  int time1 = millis(); // time at the beginning of measurement
  for (int i = 0; i < sampleNumber; i++){ 
    float pressureReading = ENV.readPressure(KILOPASCAL); // return pressure reading in kilopascals
    // float tempReading = ENV.readTemperature(CELSIUS); // return temperature reading in Celsius
    pressure_avg += pressureReading;
    // temp_avg += tempReading;     
    delay(samplingInterval); 
  }
  int time2 = millis(); // time at the end of measurement
  int time_avg = (time1 + time2)/2;
  time = (time_avg - startTime)/1000;
  if (!launchDetected){
    time = -1;    
  } 
  pressure = pressure_avg / sampleNumber;
  getBatteryVoltage();
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
  for (int i = 0; i < padElevationSampleNumber; i++){ 
    getMeasurements(samplingInterval, sampleNumber);
    pressure_avg += pressure;

    delay(padElevationSamplingInterval);    
  }
  pressure_avg = pressure_avg / padElevationSampleNumber;
  return getAltitude(pressure_avg);  
}