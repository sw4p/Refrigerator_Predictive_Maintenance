/*
  HTS221 - Read Sensors

  This example reads data from the on-board HTS221 sensor of the
  Nano 33 BLE Sense and prints the temperature and humidity sensor
  values to the Serial Monitor once a second.

  The circuit:
  - Arduino Nano 33 BLE Sense

  This example code is in the public domain.
*/

#include <Arduino_HTS221.h>
#include <PDM.h>
#include <Arduino_APDS9960.h>

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }

  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor!");
    while (1);
  }
}

void loop() {
  // read all the sensor values
  float temperature = HTS.readTemperature();
  float humidity    = HTS.readHumidity();

  // print each of the sensor values
  // Serial.print("Temperature = ");
  // Serial.print(temperature);
  // Serial.println(" °C");

  // Serial.print("Humidity    = ");
  // Serial.print(humidity);
  // Serial.println(" %");
  
  // Wait for samples to be read
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

  // Ambient light detection
  int r, g, b, a;
  if (APDS.colorAvailable())
  {
    APDS.readColor(r, g, b, a);
    Serial.println(a);
  }

  // print an empty line
  Serial.println();

  // wait 1 second to print again
  delay(100);
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
