//For scrolling:
// - https://forums.adafruit.com/viewtopic.php?f=25&t=112175

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "config.h"


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

// Used for software SPI
#define LIS3DH_CLK 33
#define LIS3DH_MISO 32
#define LIS3DH_MOSI 15
// Used for hardware & software SPI
#define LIS3DH_CS 14
float prevY = 1.0;
int steps = 0;
// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// Setting up our AdafruitIO Feed. The IFTT Rule Reads from this feed
// Then posts to twitter.
AdafruitIO_Feed *digital = io.feed("digital");

int numSamples = 0;
int timeStamp = 0;
int prevPeakTime = -1;

const int numReadings = 50;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
double accelMag = 0.0;

int left = 0;
int mid = 0;
int right = 0;
int forward = 0;
int back = 0;

void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  Serial.begin(9600);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  display.println("Screen initialized!");
  display.display();
  delay(3000);

  display.println("Initializing accelerometer...");
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");

    // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}

void loop() {
  // put your main code here, to run repeatedly:

  
  

    printAccel();
    delay(5);
   
}

void printBattery(){
  int rawValue = analogRead(A13);

  // Reference voltage on ESP32 is 1.1V
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-calibration
  // See also: https://bit.ly/2zFzfMT
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  
  //Serial.println((String)"Raw:" + rawValue + " Voltage:" + voltageLevel + "V Percent: " + (batteryFraction * 100) + "%");
  display.println((String)"Bat: " + (batteryFraction * 100) + "%");
}

void printAccel() {
  // Then print out the raw data
  
//  display.print(lis.x);
//  display.print(", ");
//  
//  display.print(lis.y);
//  display.print(", ");
//  display.print(lis.z);
//  display.println();


  lis.read();      // get X Y and Z data at once
  long x = lis.x;
  
  long y = lis.y;
  long z = lis.z;
  double accelMag  = sqrt(x * x + y * y + z*z);  

  //accelMag = sqrt((sq(lis.x) + sq(lis.y) + sq(lis.z)));

  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = accelMag;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  if(right == 0){
    right = average;
    return;
  } else if (mid == 0) {
    mid = right;
    right = average;
    return;
  } else {
        left = mid;
        mid = right;
        right = average;
  }
  
  timeStamp = millis();
  forward = right - mid;
  back = mid - left;
  if (forward < 0 && back > 0 && average > 8120) {
    Serial.println("Peak Detected!");
    int timeDiff = timeStamp - prevPeakTime;
    if (prevPeakTime == -1 |  timeDiff > 500) {
          steps++;
                Serial.println("VALID PEAK!");
                Serial.println();

          prevPeakTime = timeStamp;
          if (steps > 100) {
              // save the current state to the 'digital' feed on adafruit io
              Serial.println();
              Serial.println();

              Serial.print("sending step count -> ");
              Serial.println(steps);
              digital->save(steps);
              Serial.println();
              Serial.println();
          }
    } else {
    }
      Serial.print("  \tSteps:  "); Serial.print(steps);
      Serial.print("  \tAvg:  "); Serial.print(average);
      Serial.println();

      display.clearDisplay();
      display.setCursor(0, 0); 
      display.print("Steps: ");
      display.print(steps); 
      printBattery();

      
      display.println();
    
      display.display();
  }
      Serial.print("  \tSteps:  "); Serial.print(steps);
      Serial.print("  \tAvg:  "); Serial.print(average);
      Serial.println();


}
