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
  BLEDevice central = BLE.central();

  // If a central is connected to peripheral:
  if (central) {
    print("Connected to central: " + central.address());
    blinkLED(LEDB, 2);

    // While the central is still connected to peripheral:
    char read_buff[value_length];
    while (central.connected()) {
      if (stringCharacteristic.written())
      {
        stringCharacteristic.readValue(read_buff, stringCharacteristic.valueLength());
        print(String(read_buff));
      }
    }

    // When the central disconnects, print it out:
    print("Disconnected from central: " + central.address()); 
    blinkLED(LEDR, 2);
  }
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
