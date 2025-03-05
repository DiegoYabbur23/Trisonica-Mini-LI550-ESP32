/*
 * ----------------------------------------------------------------------------
 * Project: Trisonica LI-550 - ESP32 Read Sensor
 * Company: Abyssal Innovation
 * Author: Diego Yabbur
 * 
 * Date: March 5, 2025
 * 
 * Description: 
 * This program is designed to interface with the Trisonica LI-550 sensor 
 * system, reading and logging environmental data such as wind speed, 
 * direction, temperature, humidity, and other atmospheric parameters 
 * through an ESP32 microcontroller. The data is processed and displayed 
 * on the serial monitor for further analysis.
 * 
 * The system utilizes UART2 for communication and supports various sensor 
 * tags for efficient data extraction and presentation.
 * 
 * Contact Information:
 * Email: diegoyabbur@abyssalinnovation.com
 * Website: www.abyssalinnovation.com
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>

#define BAUD_RATE 115200
#define SERIAL2_RXPIN 13  // RX pin for UART2
#define SERIAL2_TXPIN 15  // TX pin for UART2

HardwareSerial mySerial(2);  // Use UART2 of the ESP32

// Function declarations
float extractValue(String data, String tag);
bool isMessageComplete(String data);
void readSensorData();

// Timer variables
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval of 1 second

// List of expected tags in the message
const String expectedTags[] = {"S", "D", "U", "V", "W", "T", "H", "P", "PI", "RO", "MD"};
const int numTags = sizeof(expectedTags) / sizeof(expectedTags[0]);

void setup() {
  Serial.begin(115200);  // Initialize the serial monitor
  mySerial.begin(BAUD_RATE, SERIAL_8N1, SERIAL2_RXPIN, SERIAL2_TXPIN);  // Initialize UART2 with the correct pins

  Serial.println("Waiting for sensor data...");
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time

  // Check if 1 second has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the previous time

    // Call the function to read sensor data
    readSensorData();
  }
}

void readSensorData() {
  if (mySerial.available()) {
    String data = "";
    // Read data until newline character is encountered
    while (mySerial.available()) {
      char c = mySerial.read();
      data += c;
      if (c == '\n') {  // End of line
        break;
      }
    }

    // Check if the message is complete
    if (isMessageComplete(data)) {
      // Display the raw data from the sensor
      Serial.println("Received Data: ");
      Serial.println(data);

      // Process the data to extract specific values
      float windSpeed = extractValue(data, "S");
      float windDirection = extractValue(data, "D");
      float windU = extractValue(data, "U");
      float windV = extractValue(data, "V");
      float windW = extractValue(data, "W");
      float temperature = extractValue(data, "T");
      float humidity = extractValue(data, "H");
      float pressure = extractValue(data, "P");
      float pitch = extractValue(data, "PI");
      float roll = extractValue(data, "RO");
      float magneticHeading = extractValue(data, "MD");

      // Display the extracted values with aligned text
      Serial.println("--------------------------------------------------");
      Serial.printf("Wind Speed      : %.2f m/s\n", windSpeed);
      Serial.printf("Wind Direction  : %.2f degrees\n", windDirection);
      Serial.printf("U Component     : %.2f\n", windU);
      Serial.printf("V Component     : %.2f\n", windV);
      Serial.printf("W Component     : %.2f\n", windW);
      Serial.printf("Temperature     : %.2f °C\n", temperature);
      Serial.printf("Humidity        : %.2f %%\n", humidity);
      Serial.printf("Pressure        : %.2f hPa\n", pressure);
      Serial.printf("Pitch Angle     : %.2f degrees\n", pitch);
      Serial.printf("Roll Angle      : %.2f degrees\n", roll);
      Serial.printf("Magnetic Heading: %.2f degrees\n", magneticHeading);
      Serial.println("--------------------------------------------------");
    } else {
      Serial.println("Incomplete message, ignoring...");
    }
  }
}

// Function to extract a value from the data based on the tag
float extractValue(String data, String tag) {
  int startIndex = data.indexOf(tag);
  if (startIndex == -1) {
    return -1;  // If the tag is not found, return -1
  }

  // Skip spaces or tabs after the tag
  startIndex += tag.length();
  while (startIndex < data.length() && (data[startIndex] == ' ' || data[startIndex] == '\t')) {
    startIndex++;
  }

  // Find the end of the value
  int endIndex = startIndex;
  while (endIndex < data.length() && data[endIndex] != ' ' && data[endIndex] != '\t' && data[endIndex] != '\n') {
    endIndex++;
  }

  // Extract the value as a substring
  String valueStr = data.substring(startIndex, endIndex);

  // Convert the value to float and return
  return valueStr.toFloat();
}

// Function to check if the message contains all expected tags
bool isMessageComplete(String data) {
  for (int i = 0; i < numTags; i++) {
    if (data.indexOf(expectedTags[i]) == -1) {
      return false;  // If any tag is missing, the message is incomplete
    }
  }
  return true;  // All tags are present
}
