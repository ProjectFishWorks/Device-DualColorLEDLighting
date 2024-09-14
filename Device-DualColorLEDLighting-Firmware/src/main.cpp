#include <Arduino.h>
#include <NodeControllerCore.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>


RTC_DS3231 rtc; // Create the RTC object

    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define NODE_ID 0xA1            // 161
#define WHITE_MESSAGE_ID 0x0A00 // 2560
#define BLUE_MESSAGE_ID 0x0A01  // 2561

#define debuging

// LED pins
#define WHITE_PWM_PIN 3
#define BLUE_PWM_PIN 2
// RELAY pins
#define BLUE_RELAY 4
#define WHITE_RELAY 5

//  Max LED Intensities
float MAX_WHITE_PWM = 255.0; // 255 is max
float MAX_BLUE_PWM = 255.0;  //      ""

//  loop light durations
int maxPWM = 255;
int blueOnlyDuration = 10000;
int sunriseFadeDuration = 15000;
float blueOnlyMaxIntensity = (maxPWM * 0.5); //  Percent of max brightness
int highNoonDuration = 10000.0;
int sunsetFadeDuration = 10000.0;
int nightTime = 5000.0;

// Node controller core object
NodeControllerCore core;

// put function declarations here:
// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

void DemoLoop();


void setup()
{
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the I2C bus for RTC
  
    Wire.begin(5, 10);  // Wire.begin(SDA, SCL)
      if (! rtc.begin(&Wire))
      {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      }
  
  pinMode(WHITE_PWM_PIN, OUTPUT);
  pinMode(BLUE_PWM_PIN, OUTPUT);
  analogWrite(WHITE_PWM_PIN, 225); // set the PWM value to dim
  analogWrite(BLUE_PWM_PIN, 225);  // set the PWM value to dim

  // RELAY pins
  pinMode(BLUE_RELAY, OUTPUT);
  pinMode(WHITE_RELAY, OUTPUT);
  digitalWrite(WHITE_RELAY, 1); // turn on the relay
  digitalWrite(BLUE_RELAY, 1);  // turn on the relay

  // Create the node controller core object
  core = NodeControllerCore();
  Serial.println("test");
  // Initialize the node controller core object
  if (core.Init(receive_message, NODE_ID))
  {
    Serial.println("Driver device initialized");
  }
  else
  {
    Serial.println("Failed to initialize driver");
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}

//  
void loop()
{
  DemoLoop();

  
      DateTime now = rtc.now();

      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(" (");
      Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
      Serial.print(") ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();

      Serial.print(" since midnight 1/1/1970 = ");
      Serial.print(now.unixtime());
      Serial.print("s = ");
      Serial.print(now.unixtime() / 86400L);
      Serial.println("d");

      // calculate a date which is 7 days, 12 hours, 30 minutes, 6 seconds into the future
      DateTime future (now + TimeSpan(7,12,30,6));

      Serial.print(" now + 7d + 12h + 30m + 6s: ");
      Serial.print(future.year(), DEC);
      Serial.print('/');
      Serial.print(future.month(), DEC);
      Serial.print('/');
      Serial.print(future.day(), DEC);
      Serial.print(' ');
      Serial.print(future.hour(), DEC);
      Serial.print(':');
      Serial.print(future.minute(), DEC);
      Serial.print(':');
      Serial.print(future.second(), DEC);
      Serial.println();

      Serial.print("Temperature: ");
      Serial.print(rtc.getTemperature());
      Serial.println(" C");

      Serial.println();
      delay(3000);
      
}

// put function definitions here:

void DemoLoop()
{
  analogWrite(BLUE_PWM_PIN, 255);
  analogWrite(WHITE_PWM_PIN, 255);
  Serial.println("Blue LED off");
  Serial.println("White LED off");
  unsigned long currentTime = millis();
  unsigned long startTime = millis();
  unsigned long blueEndTime = millis();
  int currentBlueIntensity;
  int currentWhiteIntensity;

  //  Fade blueOnly up
  while (startTime + blueOnlyDuration > currentTime)
  {
    delay(100);
    currentTime = millis();
    currentBlueIntensity = map(currentTime, startTime, startTime + blueOnlyDuration, 5, blueOnlyMaxIntensity);
    analogWrite(BLUE_PWM_PIN, 255 - currentBlueIntensity);

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
#endif
  }

  //  Fade blue and white to highNoon
  startTime = millis();
  blueEndTime = millis();
  while (startTime + sunriseFadeDuration > currentTime)
  {
    delay(100);
    currentTime = millis();
    //                     map( inputValue, low range input, high range input, low range output, high range output);
    currentBlueIntensity = map(currentTime, startTime, blueEndTime + sunriseFadeDuration, blueOnlyMaxIntensity, MAX_BLUE_PWM);
    analogWrite(BLUE_PWM_PIN, 255 - currentBlueIntensity);
    currentWhiteIntensity = map(currentTime, startTime, startTime + sunriseFadeDuration, 0, MAX_WHITE_PWM);
    analogWrite(WHITE_PWM_PIN, 255 - currentWhiteIntensity);

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
#endif
  }

  //  highNoon
  startTime = millis();
  while (startTime + highNoonDuration > currentTime)
  {
    delay(100);
    currentTime = millis();
    analogWrite(BLUE_PWM_PIN, 255 - MAX_BLUE_PWM);
    analogWrite(WHITE_PWM_PIN, 255 - MAX_WHITE_PWM);

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
#endif
  }

  //  Fade blue and white to sunset
  startTime = millis();
  while (startTime + sunsetFadeDuration > currentTime)
  {
    delay(100);
    currentTime = millis();
    //                         map( inputValue, low range input, high range input, low range output, high range output);
    currentBlueIntensity = map(currentTime, startTime, startTime + sunsetFadeDuration, MAX_BLUE_PWM, blueOnlyMaxIntensity);
    analogWrite(BLUE_PWM_PIN, 255 - currentBlueIntensity);
    currentWhiteIntensity = map(currentTime, startTime + sunsetFadeDuration, startTime, MAX_WHITE_PWM, 0);
    analogWrite(WHITE_PWM_PIN, 255 - currentWhiteIntensity);

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
#endif
  }

  //  Fade blueOnly down
  startTime = millis();
  while (startTime + blueOnlyDuration > currentTime)
  {
    delay(100);
    currentTime = millis();
    currentBlueIntensity = map(currentTime, startTime, startTime + blueOnlyDuration, blueOnlyMaxIntensity, 0);
    analogWrite(BLUE_PWM_PIN, currentBlueIntensity);

    if (currentBlueIntensity < 5)
    {
      digitalWrite(BLUE_RELAY, 0);
    }
    else
    {
      digitalWrite(BLUE_RELAY, 1);
    }

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
#endif
  }
  //  nightTime
  startTime = millis();
  while (startTime + nightTime > currentTime)
  {
    delay(100);
    currentTime = millis();
    analogWrite(BLUE_PWM_PIN, 255);
    analogWrite(WHITE_PWM_PIN, 255);

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
#endif
  }
}

void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
  Serial.println("Message received callback");

  // Check if the message is for this node
  if (nodeID == NODE_ID)
  {
    Serial.println("Message received to self");
    // Check the message ID for the LED control messages
    switch (messageID)
    {
    case WHITE_MESSAGE_ID:
      Serial.println("WHITE_LED to " + String(255 - data));
      // PWM control of the LED based on the received data
      // TODO: Add a check for the data range
      if (data < 5)
      {
        digitalWrite(WHITE_RELAY, 0);
        delay(1000);
        analogWrite(WHITE_PWM_PIN, 255 - data);
      }
      else
      {
        digitalWrite(WHITE_RELAY, 1);
        analogWrite(WHITE_PWM_PIN, 255 - data);
      }
      break;

    case BLUE_MESSAGE_ID:
      Serial.println("LED 2 to " + String(255 - data));
      if (data < 5)
      {
        digitalWrite(BLUE_RELAY, 0);
        delay(1000);
        analogWrite(BLUE_PWM_PIN, 255 - data);
      }
      else
      {
        digitalWrite(BLUE_RELAY, 1);
        analogWrite(BLUE_PWM_PIN, 255 - data);
      }
      break;
    default:
      break;
    }
  }
}