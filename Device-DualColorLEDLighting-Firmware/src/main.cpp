00#include <Arduino.h>
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

//#define debuging

// LED pins
#define WHITE_PWM_PIN 3
#define BLUE_PWM_PIN 2
// RELAY pins
#define BLUE_RELAY 4
#define WHITE_RELAY 5

//  Max LED Intensities
float MAX_WHITE_PWM = 255.0; // 255 is max
float MAX_BLUE_PWM = 255.0; //      ""
int scaleFactor = 1000000000;

//  loop light durations
int blueOnlyDuration = 10000;
int sunriseFadeDuration = 15000;
float blueOnlyMaxIntensity = 0.25;  //  Percent of max brightness
int highNoonDuration = 10000.0;
int sunsetFadeDuration = 10000.0;
int nightTime = 5000.0;
int scaledBlueOnlyDuration = (blueOnlyDuration * scaleFactor);
int scaledFadeValueBlue = (((MAX_BLUE_PWM * scaleFactor) * scaledBlueOnlyDuration));

// Node controller core object
NodeControllerCore core;

// put function declarations here:
// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

void DemoLoop()
{
  unsigned long currentTime = millis();
  unsigned long endTime = millis();
  unsigned long blueStartTime = millis();
  unsigned long whiteStartTime = millis();
  unsigned long testTime = millis();

  int scaledCurrentBlueIntensity;
  int scaledCurrentWhiteIntensity;
  float currentBlueIntensity;
  float currentWhiteIntensity;

  //  Fade blueOnly up
  while(blueStartTime + blueOnlyDuration > currentTime)
  {
    Serial.println(currentTime);
    currentTime = millis();
    scaledCurrentBlueIntensity = (((float)currentTime - (float)blueStartTime)+(scaledFadeValueBlue));
    analogWrite(BLUE_PWM_PIN, (int)currentBlueIntensity);
    testTime = millis();
    Serial.println(testTime);
    delay(1);


    #ifdef debuging
    Serial.println(fadeValueBlue);
    Serial.println("fadeValueBlue");
    Serial.println(currentBlueIntensity);
    Serial.println("currentBlueIntensity");
    Serial.println(endTime);
    Serial.println("endTime");
    Serial.println(currentTime);
    Serial.println("currentTime");
    Serial.println(currentBlueIntensity);
    Serial.println("currentBlueIntensity");
    // check for interupts
    delay(500);
    #endif
    
    endTime = (millis() - currentTime);
  }
  /*
  //  Fade blue and white to highNoon
  while(startTime + blueOnlyDuration + sunriseFadeDuration > currentTime)
  {
    currentTime = millis();
    float fadeValueBlue = ((MAX_BLUE_PWM * blueOnlyMaxIntensity)/blueOnlyDuration + sunriseFadeDuration);
    Serial.println(fadeValueBlue);
    Serial.println("fadeValueBlue");
    currentBlueIntensity = ((blueStartTime + sunriseFadeDuration-startTime)+fadeValueBlue);
    analogWrite(BLUE_PWM_PIN, currentBlueIntensity);
    Serial.println(currentBlueIntensity);
    Serial.println("currentBlueIntensity");
    
    unsigned long fadeValueWhite = (MAX_WHITE_PWM / blueOnlyDuration + sunriseFadeDuration);
    Serial.println(fadeValueWhite);
    Serial.println("fadeValueWhite");
    currentWhiteIntensity = ((blueStartTime + sunriseFadeDuration-startTime)+fadeValueBlue);
    analogWrite(BLUE_PWM_PIN, currentBlueIntensity);
    Serial.println(currentBlueIntensity);
    Serial.println("currentBlueIntensity");
    // check for interupts
    delay(1000);
  }
*/
}

/*
void ChkForInterupts()
{
  if();
  {
    //  Check for interupts
  } 
}
*/
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

   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

}

void loop()
{
  //DemoLoop();


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
      if (data < 26)
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
      if (data < 26)
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