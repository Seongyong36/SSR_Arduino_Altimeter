#include <SPI.h>
#include <SD.h>
#include <Arduino_MKRENV.h>


const char filename[] = "test.csv";
const float collectionTime = 600000;



File myFile;

// const int chipSelect = SDCARD_SS_PIN;

float startTime;
float endTime;

float temperature;
float humidity;
float pressure;
float illuminance;
float uva;
float uvb;
float uvIndex;
float altitude;

void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }

  Serial.print("Initializing SD card...");
  // Serial.print(chipSelect);
  
  for (s)
  
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
    myFile.println("temperature, pressure, humidity, altitude");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  
  // re-open the file for reading:
  myFile = SD.open("test.csv");
  if (myFile) {
    Serial.println("test.csv:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.csv");
  }
  
  startTime = millis();
}

void loop() {
  endTime = millis();
  while (endTime - startTime < collectionTime) {

    temperature = ENV.readTemperature();
    pressure    = ENV.readPressure();
    altitude    = 145366.45*(1-pow(pressure*10/1013.25,(0.190284)));
    
    myFile = SD.open("test.csv", FILE_WRITE);
    myFile.print(temperature);
    myFile.print(",");
    myFile.print(pressure);
    myFile.print(",");
    myFile.println(altitude);
    myFile.close();
    
    
    // Serial.println(temperature);
    // Serial.println(pressure);
    // Serial.println(humidity);
    // Serial.println();
    
    endTime = millis();
  }

  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000); 
}
