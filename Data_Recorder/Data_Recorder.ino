/*
  Refrigerator Predictive Maintenance - Data Recorder (BLE Peripheral)

  This code receives string of sensor values via BLE and save it
  to a file.

  The circuit:
  - Arduino Portenta H7

  Creator:
  - Swapnil Verma <usav.swapnil[at]gmail.com>
*/

#include <ArduinoBLE.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"

// SD card
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");
FILE *myFile;

// BLE
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* stringCharacteristicUuid = "1A3AC131-31EF-758B-BC51-54A61958EF82";

const int value_length = 20;

BLEService datasetRecorder(deviceServiceUuid);

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic stringCharacteristic(stringCharacteristicUuid, BLERead | BLEWrite, value_length);

const int ledPin = LED_BUILTIN; // Pin to use for the LED

// Set to false if not in Debug mode
static const bool debug_mode = true;

void print(const String information, bool new_line = true)
{
  if (!debug_mode) return;
  
  if (new_line) Serial.println(information);
  else Serial.print(information);
}

void setup() {

  // Setup LEDs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  digitalWrite(LEDR, HIGH);   // will turn the LED off
  digitalWrite(LEDG, HIGH);   // will turn the LED off
  digitalWrite(LEDB, HIGH);   // will turn the LED off  

  // Start Serial communication in debug mode
  if (debug_mode)
  {
    Serial.begin(9600);
    while (!Serial) blinkLED(LEDR, 1);
  }

  // Setup SD Card
  print("Mounting SDCARD...");
  int err =  fs.mount(&block_device);
  if (err) {
    // Reformat if we can't mount the filesystem
    // this should only happen on the first boot
    print("No filesystem found, formatting... ");

    print("Do you want to format the SD card? (Y/N): ");
    if (readInput() == 'Y') err = fs.reformat(&block_device);
  }
  if (err) {
     print("Error formatting SDCARD ");
     while(1) blinkLED(LEDR, 1);
  }

  // Open/Create a file on the SD Card
  print("Opening a file");
  myFile = fopen("fs/dataset.csv", "a");
  if (myFile)
  {
    print("File open successfully");
    fprintf(myFile, "******** Refrigerator Predictive Maintenance ********\r\n");
    fprintf(myFile, "Temperature,Humidity,Illumination\r\n");
  }
  else
  {
    print("Could not open a file");
    while(1) blinkLED(LEDR, 1);
  }

  // Set LED pin to output and make it HIGH so it's off.
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Initialise BLE communication
  if (!BLE.begin()) {
    print("Starting Bluetooth® Low Energy failed!");
    while (1) blinkLED(LEDR, 1);
  }

  // Set advertised local name and service UUID:
  BLE.setLocalName("Dataset_Recorder");
  BLE.setAdvertisedService(datasetRecorder);

  // Add the characteristic to the service
  datasetRecorder.addCharacteristic(stringCharacteristic);

  // Add service
  BLE.addService(datasetRecorder);

  // start advertising
  BLE.advertise();
  blinkLED(LEDG, 3);
  print("Dataset Recorder Ready");
}

void loop() {
  // Listen for Bluetooth® Low Energy peripherals to connect:
  blinkLED(LEDB, 1);
  BLEDevice central = BLE.central();

  // If a central is connected to peripheral:
  if (central) {
    print("Connected to central: " + central.address());
    blinkLED(LEDB, 2);

    // While the central is still connected to peripheral:
    while (central.connected()) {
      if (stringCharacteristic.written())
      {
        char read_buff[value_length] = {'\0'};
        stringCharacteristic.readValue(read_buff, stringCharacteristic.valueLength());
        fprintf(myFile, read_buff);
        fprintf(myFile, "\r\n");
        print(String(read_buff));
        shouldIQuit();        
      }
    }

    // When the central disconnects, print it out:
    print("Disconnected from central: " + central.address()); 
  }
  shouldIQuit();
}

/* Function to blink led */
void blinkLED(int led_colour, int blink_count)
{
  for (int i = 0; i < blink_count; ++i)
  {
    digitalWrite(led_colour, LOW);
    delay(150);
    digitalWrite(led_colour, HIGH);
    delay(150);
  }
}

char readInput()
{
  char incoming;
  while(1)
  {
    if (Serial.available() > 0)
    {
      incoming = Serial.read();
      break;
    }
    blinkLED(LEDR, 1);
  }
  return incoming;
}

void shouldIQuit()
{
  if (Serial.available() > 0)
  {
    if (Serial.read() == 'Q')
    {
      print("Closing File");
      delay(1000);
      fclose(myFile);
      
      while(1) blinkLED(LEDG, 1);
    }
  }
}
