/*
  Refrigerator Predictive Maintenance - Data Collector

  This code reads various sensor values of the Arduino BLE Sense
  and send it to a BLE peripheral device in a CSV format.

  The circuit:
  - Arduino Nano 33 BLE Sense

  Creator:
  - Swapnil Verma <usav.swapnil[at]gmail.com>
*/

#include <Arduino_HTS221.h> ///< Temperature and Humidity sensor
#include <PDM.h>  ///< To read digital microphone output
#include <Arduino_APDS9960.h> ///< Proximity and Gesture detection sensor
#include <ArduinoBLE.h>

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;

// BLE constants
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* stringCharacteristicUuid = "1A3AC131-31EF-758B-BC51-54A61958EF82";

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

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Initialise humidity and temperature sensor
  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1) blinkLED(LEDR, 1);
  }

  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1) blinkLED(LEDR, 1);
  }

  // Initialise gesture and proximity detection sensor
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor!");
    while (1) blinkLED(LEDR, 1);
  }

  // Initialise BLE communication
  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1) blinkLED(LEDR, 1);
  }
  
  BLE.setLocalName("BLESense_Data_Collector"); 
  BLE.advertise();

  blinkLED(LEDG, 3);  ///< Indicate that everything is fine
}

void loop() {
  BLEDevice peripheral;
  Serial.println("Discovering peripheral device...");

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    peripheral = BLE.available();
  } while (!peripheral);

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName()); 
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // stop scanning
    BLE.stopScan();

    connectPeripheral(peripheral);

    // peripheral disconnected, start scanning again
  }
}

void connectPeripheral(BLEDevice peripheral)
{
    // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  // BLECharacteristic ledCharacteristic = peripheral.characteristic(deviceServiceCharacteristicUuid);
  BLECharacteristic stringCharacteristic = peripheral.characteristic(stringCharacteristicUuid);

  if (/*!ledCharacteristic ||*/ !stringCharacteristic) {
    Serial.println("Peripheral device does not have required characteristic!");
    peripheral.disconnect();
    return;
  } else if (/*!ledCharacteristic.canWrite() || */!stringCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable characteristic!");
    peripheral.disconnect();
    return;
  }

  char val[5] = "Hiya";
  while (peripheral.connected()) {
    sendData(stringCharacteristic);
    // if (stringCharacteristic.writeValue(val)) Serial.println("Sent");
    // else Serial.println("Not Sent");
    // delay(2000);
  }

  Serial.println("Peripheral disconnected");
}

void sendData(BLECharacteristic stringCharacteristic)
{
  String sensorReading;
  // read all the sensor values
  float temperature = HTS.readTemperature();
  float humidity    = HTS.readHumidity();

  sensorReading += "T" + String(temperature, 2) + ",H" + String(humidity, 2) + ",";
  // print each of the sensor values
//  Serial.print("Temperature = ");
//  Serial.print(temperature);
//  Serial.println(" °C");
//
//  Serial.print("Humidity    = ");
//  Serial.print(humidity);
//  Serial.println(" %");
  
  // Wait for samples to be read
  /*
  if (samplesRead) {    
    // Print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++) {
      if(channels == 2) {
        Serial.print("L:");
        Serial.print(sampleBuffer[i]);
        Serial.print(" R:");
        i++;
      }

      // Serial.print(-500);
      // Serial.print(" ");
      // Serial.print(sampleBuffer[i]);
      // Serial.print(" ");
      // Serial.println(500);
    }
    

    // Clear the read count
    samplesRead = 0;
  }
  */

  // Ambient light detection
  if (APDS.colorAvailable())
  {
    int r, g, b, a;
    APDS.readColor(r, g, b, a);
    Serial.print("Ambient Light = ");
    Serial.println(a);

    // print("Ambient Light = " + String(a));

    sensorReading += "A" + String(a);
  }

  // print an empty line
  Serial.print("Sending = ");
  Serial.println(sensorReading);
  stringCharacteristic.writeValue(sensorReading.c_str());

  // wait 1 second to print again
  delay(1000);
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
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
