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

#define NODE_ID 0xA1                              // 161
#define WHITE_MESSAGE_ID 0x0A00                   // 2560
#define BLUE_MESSAGE_ID 0x0A01                    // 2561
#define DAWN_MESSAGE_ID 0x0A02                    // 2562
#define DUSK_MESSAGE_ID 0x0A03                    // 2563
#define SUNRISE_MESSAGE_ID 0x0A04                 // 2564
#define SUNSET_MESSAGE_ID 0x0A05                  // 2565
#define HIGH_NOON_MESSAGE_ID 0x0A06               // 2566
#define NIGHT_TIME_MESSAGE_ID 0x0A07              // 2567
#define BLUE_ONLY_MAX_INTENSITY_MESSAGE_ID 0x0A08 // 2568
#define CURRENT_WHITE_MESSAGE_ID 0x0A09           // 2569
#define CURRENT_BLUE_MESSAGE_ID 0x0A0A            // 2570
#define MANUAL_LED_CONTROL_OVERRIDE_SWITCH 0x0A0B // 2571
#define OVERRIDE_WHITE_INTENSITY 0x0A0C           // 2572
#define OVERRIDE_BLUE_INTENSITY 0x0A0D            // 2573

/*        public bool ManualLEDControlOverrideSwitch
        {
            get
            {
                return _ManualLEDControlOverrideSwitch;
            }
            set
            {
                _ManualLEDControlOverrideSwitch = value;
                sendMessageData(nodeID, 2571, (ulong)(_ManualLEDControlOverrideSwitch ? 1 : 0));
            }
        }

        public int OverrideWhiteIntensity
        {
            get
            {
                return _OverrideWhiteIntensity;
            }
            set
            {
                _OverrideWhiteIntensity = value;
                sendMessageData(nodeID, 2572, (ulong)_OverrideWhiteIntensity);
            }
        }

        public int OverrideBlueIntensity
        {
            get
            {
                return _OverrideBlueIntensity;
            }
            set
            {
                _OverrideBlueIntensity = value;
                sendMessageData(nodeID, 2573, (ulong)_OverrideBlueIntensity);
            }
        }*/

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
int off = maxPWM;                             // 0 is max
float blueOnlyMaxIntensity = (maxPWM * 0.75); //  Percent of max brightness
int dawnBlueOnlyDuration = 5000;
int sunriseFadeDuration = 5000;
int highNoonDuration = 5000;
int sunsetFadeDuration = 5000;
int duskBlueOnlyDuration = 5000;
int nightTime = 5000;

int sendMessageLEDIntensityDelay = 500; // frequency of sending LED intensity messages to the App
int wait = 100;                         // delay time in milliseconds
int currentBlueIntensity;
int currentWhiteIntensity;
bool ManualLEDControlOverrideSwitch = false;

// Node controller core object
NodeControllerCore core;

// put function declarations here:
// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

void SendLEDIntensities(void *parameters);

void DemoLoop();

void chkManualLEDControlOverrideSwitch();

//-------------------------------------------------------- setup -------------------------------------------------------------------

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the I2C bus for RTC

  Wire.begin(5, 10); // Wire.begin(SDA, SCL)
  if (!rtc.begin(&Wire))
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
  digitalWrite(WHITE_RELAY, 0); // turn on the relay
  digitalWrite(BLUE_RELAY, 0);  // turn on the relay

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

  xTaskCreate(
      SendLEDIntensities,   /* Task function. */
      "SendLEDIntensities", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */
                            /* pin task to core 0 */
}

//  --------------------------------------------------------------  Loop  -------------------------------------------------------------------

void loop()
{
  DemoLoop();

  /*
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
    DateTime future(now + TimeSpan(7, 12, 30, 6));

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
    */
}

////////////////////////////////////////////// put function definitions here:  /////////////////////////////////////////////////////////

void DemoLoop()
{
  //-------------------------------------------------------- sunrise -------------------------------------------------------------------
  chkManualLEDControlOverrideSwitch();

  analogWrite(BLUE_PWM_PIN, off);
  analogWrite(WHITE_PWM_PIN, off);
  digitalWrite(BLUE_RELAY, 0);
  digitalWrite(WHITE_RELAY, 0);
#ifdef debuging
  Serial.println("Blue LED off");
  Serial.println("White LED off");
#endif
  unsigned long currentTime = millis();
  unsigned long startTime = millis();

  //  Fade blueOnly up
  while (startTime + dawnBlueOnlyDuration > currentTime)
  {
    delay(wait);
    currentTime = millis();
    currentBlueIntensity = map(currentTime, startTime, startTime + dawnBlueOnlyDuration, 0, blueOnlyMaxIntensity);
    analogWrite(BLUE_PWM_PIN, maxPWM - currentBlueIntensity);

    if (currentBlueIntensity < 5)
    {
      digitalWrite(BLUE_RELAY, 0);
      delay(wait);
    }
    else
    {
      digitalWrite(BLUE_RELAY, 1);
    }

#ifdef debuging
    Serial.print("dawnBlueOnlyDuration = ");
    Serial.println(dawnBlueOnlyDuration);
    Serial.print("blueOnlyMaxIntensity = ");
    Serial.println(blueOnlyMaxIntensity);
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
#endif
  }

  //--------------------------------------------------------- Fade blue and white to highNoon --------------------------------------------------
  chkManualLEDControlOverrideSwitch();

  startTime = millis();
  while (startTime + sunriseFadeDuration > currentTime)
  {
    delay(wait);
    currentTime = millis();
    //                     map( inputValue, low range input, high range input, low range output, high range output);
    currentBlueIntensity = map(currentTime, startTime, startTime + sunriseFadeDuration, blueOnlyMaxIntensity, MAX_BLUE_PWM);
    analogWrite(BLUE_PWM_PIN, maxPWM - currentBlueIntensity);
    currentWhiteIntensity = map(currentTime, startTime, startTime + sunriseFadeDuration, 0, MAX_WHITE_PWM);
    analogWrite(WHITE_PWM_PIN, maxPWM - currentWhiteIntensity);

    if (currentWhiteIntensity < 5)
    {
      digitalWrite(WHITE_RELAY, 0);
      delay(wait);
    }
    else
    {
      digitalWrite(WHITE_RELAY, 1);
    }

#ifdef debuging
    Serial.print("sunriseFadeDuration = ");
    Serial.println(sunriseFadeDuration);
    Serial.print("blueOnlyMaxIntensity = ");
    Serial.println(blueOnlyMaxIntensity);
    Serial.print("sunriseFadeDuration = ");
    Serial.println(sunriseFadeDuration);
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
#endif
  }

  //---------------------------------------------------------------- highNoon ---------------------------------------------------------------
  chkManualLEDControlOverrideSwitch();
  
  startTime = millis();
  while (startTime + highNoonDuration > currentTime)
  {
    delay(wait);
    currentTime = millis();
    analogWrite(BLUE_PWM_PIN, maxPWM - MAX_BLUE_PWM);
    analogWrite(WHITE_PWM_PIN, maxPWM - MAX_WHITE_PWM);

#ifdef debuging
    Serial.print("highNoonDuration = ");
    Serial.println(highNoonDuration);
    Serial.print("blueOnlyMaxIntensity = ");
    Serial.println(blueOnlyMaxIntensity);
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
#endif
  }

  //----------------------------------------------------------- Fade blue and white to sunset --------------------------------------------------
  chkManualLEDControlOverrideSwitch();

  startTime = millis();
  while (startTime + sunsetFadeDuration > currentTime)
  {
    delay(wait);
    currentTime = millis();
    //                         map( inputValue, low range input, high range input, low range output, high range output);
    currentBlueIntensity = map(currentTime, startTime, startTime + sunsetFadeDuration, MAX_BLUE_PWM, blueOnlyMaxIntensity);
    analogWrite(BLUE_PWM_PIN, maxPWM - currentBlueIntensity);
    currentWhiteIntensity = map(currentTime, startTime, startTime + sunsetFadeDuration, MAX_WHITE_PWM, 0);
    analogWrite(WHITE_PWM_PIN, maxPWM - currentWhiteIntensity);

    if (currentWhiteIntensity < 5)
    {
      digitalWrite(WHITE_RELAY, 0);
      delay(wait);
    }
    else
    {
      digitalWrite(WHITE_RELAY, 1);
    }

#ifdef debuging
    Serial.print("sunsetFadeDuration = ");
    Serial.println(sunsetFadeDuration);
    Serial.print("blueOnlyMaxIntensity = ");
    Serial.println(blueOnlyMaxIntensity);
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
#endif
  }

  //------------------------------------------------------- Fade blueOnly down -----------------------------------------------------------------------
  chkManualLEDControlOverrideSwitch();

  startTime = millis();
  while (startTime + dawnBlueOnlyDuration > currentTime)
  {
    delay(wait);
    currentTime = millis();
    currentBlueIntensity = map(currentTime, startTime, startTime + dawnBlueOnlyDuration, (int)blueOnlyMaxIntensity, 0);
    analogWrite(BLUE_PWM_PIN, maxPWM - currentBlueIntensity);

    if (currentBlueIntensity < 50)
    {
      digitalWrite(BLUE_RELAY, 0);
      delay(wait);
      analogWrite(BLUE_PWM_PIN, maxPWM - currentBlueIntensity);
    }
    else
    {
      digitalWrite(BLUE_RELAY, 1);
    }

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
#endif
  }
  //------------------------------------------------------- nightTime -----------------------------------------------------------------------
  chkManualLEDControlOverrideSwitch();
  
  startTime = millis();
  while (startTime + nightTime > currentTime)
  {
    delay(wait);
    currentTime = millis();
    analogWrite(BLUE_PWM_PIN, off);
    analogWrite(WHITE_PWM_PIN, off);

#ifdef debuging
    Serial.print("currentBlueIntensity = ");
    Serial.println(currentBlueIntensity);
    Serial.print("currentWhiteIntensity = ");
    Serial.println(currentWhiteIntensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
#endif
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Callback function for received messages from the CAN bus
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
      // ---------------------Demo override control messages-------------------------
    case WHITE_MESSAGE_ID:
      Serial.println("WHITE_LED to " + String(maxPWM - data));
      // PWM control of the LED based on the received data
      // TODO: Add a check for the data range
      if (data < 5)
      {
        digitalWrite(WHITE_RELAY, 0);
        delay(wait);
        analogWrite(WHITE_PWM_PIN, maxPWM - data);
      }
      else
      {
        digitalWrite(WHITE_RELAY, 1);
        analogWrite(WHITE_PWM_PIN, maxPWM - data);
      }
      break;

    case BLUE_MESSAGE_ID:
      Serial.println("LED 2 to " + String(maxPWM - data));
      if (data < 5)
      {
        digitalWrite(BLUE_RELAY, 0);
        delay(wait);
        analogWrite(BLUE_PWM_PIN, maxPWM - data);
      }
      else
      {
        digitalWrite(BLUE_RELAY, 1);
        analogWrite(BLUE_PWM_PIN, maxPWM - data);
      }
      break;

      //  -----------------Demo control messages--------------------------------

    case DAWN_MESSAGE_ID:
      dawnBlueOnlyDuration = data;
      Serial.println("Dawn duration set to " + String(data));
      break;

    case DUSK_MESSAGE_ID:
      duskBlueOnlyDuration = data;
      Serial.println("Dusk duration set to " + String(data));
      break;

    case SUNRISE_MESSAGE_ID:
      sunriseFadeDuration = data;
      Serial.println("Sunrise duration set to " + String(data));
      break;

    case SUNSET_MESSAGE_ID:
      sunsetFadeDuration = data;
      Serial.println("Sunset duration set to " + String(data));
      break;

    case HIGH_NOON_MESSAGE_ID:
      highNoonDuration = data;
      Serial.println("High Noon duration set to " + String(data));
      break;

    case NIGHT_TIME_MESSAGE_ID:
      nightTime = data;
      Serial.println("Night Time duration set to " + String(data));
      break;

    case BLUE_ONLY_MAX_INTENSITY_MESSAGE_ID:
      blueOnlyMaxIntensity = data;
      Serial.println("Blue Only Max Intensity set to " + String(data));
      break;

    case MANUAL_LED_CONTROL_OVERRIDE_SWITCH:
      ManualLEDControlOverrideSwitch = data;
      Serial.println("Manual LED Control Override Switch set to " + String(data));
      break;

    default:
      break;
    }
  }
}

// ----------------------------Send the LED intensities to App-------------------------

void SendLEDIntensities(void *parameters)
{
  while (1)
  {
    uint64_t WhiteIntensity;
    uint64_t BlueIntensity;
    WhiteIntensity = currentWhiteIntensity;
    BlueIntensity = currentBlueIntensity;
    core.sendMessage(CURRENT_WHITE_MESSAGE_ID, &WhiteIntensity); // Send the white LED intensity
    delay(sendMessageLEDIntensityDelay);
    core.sendMessage(CURRENT_BLUE_MESSAGE_ID, &BlueIntensity);
    delay(sendMessageLEDIntensityDelay);
  }
}

void chkManualLEDControlOverrideSwitch()
{
  if (ManualLEDControlOverrideSwitch == true)
  {
    digitalWrite(BLUE_RELAY, 1);
    digitalWrite(WHITE_RELAY, 1);
    analogWrite(WHITE_PWM_PIN, maxPWM - OVERRIDE_WHITE_INTENSITY);
    analogWrite(BLUE_PWM_PIN, maxPWM - OVERRIDE_BLUE_INTENSITY);
    delay(wait);

#ifdef debuging
    Serial.println("Manual LED control override switch is on");
    Serial.println("Blue relay on");
    Serial.println("White relay on");
    Serial.print("Override white intensity = ");
    Serial.println(OVERRIDE_WHITE_INTENSITY);
    Serial.print("Override blue intensity = ");
    Serial.println(OVERRIDE_BLUE_INTENSITY);
#endif
  }
}
