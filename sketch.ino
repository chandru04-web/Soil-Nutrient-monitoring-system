#define BLYNK_TEMPLATE_NAME "smart soil nutrition monitoring system  for sustai"
#define BLYNK_AUTH_TOKEN "UgIOVt3CJlP-FAiavn8Fao72LVpvlTUf"
#define BLYNK_TEMPLATE_ID "TMPL3-ylmStED"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
#include <HardwareSerial.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int DHT_PIN = 15;
DHTesp dhtSensor;

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
const int potPin = 34;
const int pHSensorPin = 32;
// Calibration parameters (you need to calibrate your sensor)
const float acidVoltage = 2032.44;  // voltage at pH 4.0
const float neutralVoltage = 1500.0; // voltage at pH 7.0

// variable for storing the potentiometer value
int potValue = 0;

#define ncom 3 //  number of commands.
char commar[ncom] = {0x1, 0x3, 0x5}; // Actual commands
// Response Strings can be stored like this
char respar[ncom][30] = {"Phosphorous value is: ", "Potassium value is: ", "Nitrogen value is: "};
uint8_t rtValue[ncom]; // Store the return values from the custom chip in here. you can use the same
//values to forward to the IOT part.

BlynkTimer timer;

int displayState = 0; // Variable to keep track of the current display state

float temperature, humidity, pHValue;

void setup() {
  Serial.begin(115200);
  Serial2.begin(15200, SERIAL_8N1, 16, 17); //initialize the custom chip communication line.

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  timer.setInterval(500L, sendData); // Send data every 1/2 seconds
  timer.setInterval(2000L, updateLCD); // Change LCD display every 3 seconds

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C address 0x3C

  Serial.println("Hello, ESP32!");

  display.clearDisplay();                    // Clear the buffer
  display.display();                         // Display initial buffer

  // Display title for 5 seconds
  displayTitle();
  delay(5000);

  // Display team members for 5 seconds
  displayTeamMembers();
  delay(5000);


  // Display "Thank you" message
  displayThankYou();
  delay(5000);

  // Turn off the OLED display
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  // Initialize the LCD
  Wire.begin(23, 22);
  lcd.init();
  lcd.backlight();

  Serial.println("LCD Initialized.");
}

void displayTitle()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Smart Soil Monitoring");
  display.println("System Using IoT");
  display.display();
}

void displayTeamMembers()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Team Members:");
  display.println("Chandru M");
  display.println("Vijayakumar");
  display.println("Rizidev P");
  display.println("Vel Murugan G");
  display.display();
}

void displayThankYou() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println("Thank you!");
  display.display();
}

void sendData() {
  temperature = dhtSensor.getTemperature();
  humidity = dhtSensor.getHumidity();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  potValue = analogRead(potPin);
  Serial.println("Moisture: " + String(potValue));

  for (uint8_t i = 0; i < ncom; i++) {
    Serial2.print((char)commar[i]); // send the command stored in ncom array through serial2
    if (Serial2.available()) { //if serial2 data is there
      rtValue[i] = Serial2.read(); // read serial2
      Serial2.flush(); // flush serial2, very important. otherwise extra bits may interfere with communication
      Serial.print(respar[i]); // print the response array to the console.
      Serial.println(rtValue[i]); // print the return value with newline at console
    }
  }

  int analogValue = analogRead(pHSensorPin);

  // Convert the analog value to voltage
  float voltage = analogValue * (3.3 / 4095.0); // 3.3V reference, 12-bit ADC

  // Convert the voltage to pH value
  pHValue = (voltage * 14.0) / 3.3;
  // Print the analog value, voltage, and pH value
  Serial.print("Analog Value: ");
  Serial.print(analogValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print("V | pH Value: ");
  Serial.println(pHValue, 2);

  //send data to blynk
  Blynk.virtualWrite(V0, temperature);  //Temperature
  Blynk.virtualWrite(V1, humidity);  //Humidity
  Blynk.virtualWrite(V2, potValue); //soil Moisture
  Blynk.virtualWrite(V4, rtValue[0]); //Phosphorous
  Blynk.virtualWrite(V3, rtValue[2]); //Nitrogen
  Blynk.virtualWrite(V5, rtValue[1]); //Potassium
}

void updateLCD() {
  // Switch to the next display state
  displayState = (displayState + 1) % 3;

  // Update the LCD display based on the current state
  lcd.clear();
  switch (displayState) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Moisture: ");
      lcd.print(potValue);
      lcd.setCursor(0, 1);
      lcd.print("pH: ");
      lcd.print(pHValue, 2);
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Nitrogen: ");
      lcd.print(rtValue[2]);
      lcd.setCursor(0, 1);
      lcd.print("Phos:");
      lcd.print(rtValue[0]);
      lcd.setCursor(8, 1);
      lcd.print("Potas:");
      lcd.print(rtValue[1]);
      break;
  }
}

void loop()
{
  Blynk.run();
  timer.run();
}
