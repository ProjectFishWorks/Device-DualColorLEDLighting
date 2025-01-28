#include <Arduino.h>
#include <NodeControllerCore.h>
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <Adafruit_MCP4728.h>

#define debuging
#define debugingTime // comment out to remove time debuging

//  RELAY logic, change if inverted logic
#define RELAY_ON 1
#define RELAY_OFF 0

//  DAC object
Adafruit_MCP4728 MCP12bitDAC; //  create the MCP4728 object 12 bits of resolution = 4096 steps, so MAX_DAC = 4095
int MAX_DAC = 4095;           //  this can be changed if there is a different resolution than 12bit
int DAC_OFF = MAX_DAC;        //  change this to 0 if the DAC is not inverted

// USE int FOR I2C PIN DEFINITIONS
int I2C_SDA = 2;
int I2C_SCL = 3;

// define the node ID and message IDs
#define BASESTATION_ID 0x00                        // 0
#define NODE_ID 0xA1                               // 161
#define UNIX_TIME_MESSAGE_ID 2000                  // 2000
#define UPDATE_TIMEZONE_OFFSET_MESSAGE_ID 2001     // 2001
#define DAWN_MINUTES_MESSAGE_ID 2560               // 0xA00
int dawnMinutes;                                   // variable to store the dawn minutes message data
#define DAWN_HOURS_MESSAGE_ID 2561                 // 0xA01
int dawnHours;                                     // variable to store the dawn hours message data
#define DUSK_MINUTES_MESSAGE_ID 2562               // 0xA02
int duskMinutes;                                   // variable to store the dusk minutes message data
#define DUSK_HOURS_MESSAGE_ID 2563                 // 0xA03
int duskHours;                                     // variable to store the dusk hours message data
#define SUNRISE_MINUTES_MESSAGE_ID 2564            // 0xA04
int sunriseMinutes;                                // variable to store the sunrise minutes message data
#define SUNRISE_HOURS_MESSAGE_ID 2565              // 0xA05
int sunriseHours;                                  // variable to store the sunrise hours message data
#define SUNSET_MINUTES_MESSAGE_ID 2566             // 0xA06
int sunsetMinutes;                                 // variable to store the sunset minutes message data
#define SUNSET_HOURS_MESSAGE_ID 2567               // 0xA07
int sunsetHours;                                   // variable to store the sunset hours message data
#define HIGH_NOON_MINUTES_MESSAGE_ID 2568          // 0xA08
int highNoonMinutes;                               // variable to store the high noon minutes message data
#define HIGH_NOON_HOURS_MESSAGE_ID 2569            // 0xA09
int highNoonHours;                                 // variable to store the high noon hours message data
#define NIGHT_TIME_MINUTES_MESSAGE_ID 2570         // 0xA0A
int nightTimeMinutes;                              // variable to store the night time minutes message data
#define NIGHT_TIME_HOURS_MESSAGE_ID 2571           // 0xA0B
int nightTimeHours;                                // variable to store the night time hours message data
#define BLUE_1_MAX_INTENSITY_MESSAGE_ID 2572       // 0xA0C
int blue_1_MaxIntensity;                           // variable to store the blue_1 max intensity message data
#define BLUE_2_MAX_INTENSITY_MESSAGE_ID 2573       // 0xA0D
int blue_2_MaxIntensity;                           // variable to store the blue_2 max intensity message data
#define WHITE_1_MAX_INTENSITY_MESSAGE_ID 2574      // 0xA0E
int white_1_MaxIntensity;                          // variable to store the white_1 max intensity message data
#define WHITE_2_MAX_INTENSITY_MESSAGE_ID 2575      // 0xA0F
int white_2_MaxIntensity;                          // variable to store the white_2 max intensity message data
#define CURRENT_WHITE_1_MESSAGE_ID 2576            // 0xA10
int currentWhite_1_Intensity;                      // variable to store the current white_1 intensity message data
#define CURRENT_WHITE_2_MESSAGE_ID 2577            // 0xA11
int currentWhite_2_Intensity;                      // variable to store the current white_2 intensity message data
#define CURRENT_BLUE_1_MESSAGE_ID 2578             // 0xA12
int currentBlue_1_Intensity;                       // variable to store the current blue_1 intensity message data
#define CURRENT_BLUE_2_MESSAGE_ID 2579             // 0xA13
int currentBlue_2_Intensity;                       // variable to store the current blue_2 intensity message data
#define MANUAL_OVERRIDE_SWITCH_MESSAGE_ID 2580     // 0xA14
bool manualOverrideSwitch = false;                 // variable to store the manual override switch message data
#define OVERRIDE_WHITE_1_INTENSITY_MESSAGE_ID 2581 // 0xA15
int overrideWhite_1_Intensity;                     // variable to store the override white_1 intensity message data
#define OVERRIDE_WHITE_2_INTENSITY_MESSAGE_ID 2582 // 0xA16
int overrideWhite_2_Intensity;                     // variable to store the override white_2 intensity message data
#define OVERRIDE_BLUE_1_INTENSITY_MESSAGE_ID 2583  // 0xA17
int overrideBlue_1_Intensity;                      // variable to store the override blue_1 intensity message data
#define OVERRIDE_BLUE_2_INTENSITY_MESSAGE_ID 2584  // 0xA18
int overrideBlue_2_Intensity;                      // variable to store the override blue_2 intensity message data
#define MIN_WHITE_VALUE_MESSAGE_ID 2585            // 0xA19
int minWhiteValue;                                 // variable to store the min white value message data
#define MIN_BLUE_VALUE_MESSAGE_ID 2586             // 0xA1A
int minBlueValue;                                  // variable to store the min blue value message data

// colored and numbered names for the DAC channels to take values from 0-4095
#define WHITE_1_DAC MCP4728_CHANNEL_A
#define WHITE_2_DAC MCP4728_CHANNEL_B
#define BLUE_1_DAC MCP4728_CHANNEL_C
#define BLUE_2_DAC MCP4728_CHANNEL_D

// RELAY pins
#define BLUE_RELAY 3
#define WHITE_RELAY 2

#define sendMqttMessageUpdateUI 5000 // frequency of sending LED intensity messages to the App
#define updateLEDs 100               // delay time in milliseconds
#define MessageGap 1000              // delay time in milliseconds

// Globals for time keeping
char localTime[64];          // Local time string
String localTimeZone;        // "UTC+" + localTimeZoneOffset
int localTimeZoneOffset = 8; // Timezone offset
char buf_localTimeZone[8];   // Char array Buffer for timezone string
struct tm timeinfo;          // Time structure
time_t UNIXtime;             // UNIX time
int curTimeSec;              //  get the start time in seconds since midnight
int dawnStart;               //  start time of dawn in seconds since midnight
int duskStart;               //  start time of dusk in seconds since midnight
int sunriseStart;            //  start time of sunrise in seconds since midnight
int sunsetStart;             //  start time of sunset in seconds since midnight
int highNoonStart;           //  start time of high noon in seconds since midnight
int nightTimeStart;          //  start time of night time in seconds since midnight
int dawnDurationSec;         //  duration of dawn in seconds
int sunriseDurationSec;      //  duration of sunrise in seconds
int highNoonDurationSec;     //  duration of high noon in seconds
int sunsetDurationSec;       //  duration of sunset in seconds
int duskDurationSec;         //  duration of dusk in seconds
int nightTimeDurationSec;    //  duration of night time in seconds
int MIDNIGHT = 86400;        //  24 hours in seconds

//  floats for the max intensities and min values from (data / 100) * MAX_DAC
float whiteMinValue_float;
float blueMinValue_float;
float white_1_MaxIntensity_float;
float white_2_MaxIntensity_float;
float blue_1_MaxIntensity_float;
float blue_2_MaxIntensity_float;
float overrideWhite_1_Intensity_float;
float overrideWhite_2_Intensity_float;
float overrideBlue_1_Intensity_float;
float overrideBlue_2_Intensity_float;

// Node controller core object
NodeControllerCore core;

//--------------------------------------------- put function declarations here:----------------------------------------------------

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

// Function to send LED intensities to the App
void SendLEDIntensities(void *parameters);

// Function to Adjust fade the LED intensities through the daylight cycles
void LightCycles(void *parameters);

// Function to check the manual override switch
void chkmanualOverrideSwitch();

// Function to update the times and schedules of the light cycles
void updateTimes();

//-------------------------------------------------------- setup -------------------------------------------------------------------

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Initializing Serial Communication");
  Serial.println("");

  // Initialize the OneWire communication
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Initializing I2C Communication");
  Serial.println("");

  //  Initialize the DAC
  MCP12bitDAC.begin();
  while (!MCP12bitDAC.begin())
  {
    delay(100); // will pause Zero, Leonardo, etc until serial console opens
    Serial.println("Adafruit MCP4728 initializing...");

    // Try to initialize!
    if (!MCP12bitDAC.begin())
    {
      Serial.println("Failed to find MCP4728 chip");
      Serial.println("");
      while (1)
      {
        MCP12bitDAC.begin();
        delay(10);
      }
    }
    Serial.println("MCP4728 Found!");
    Serial.println("");
  }
  /*
   * @param channel The channel to update
   * @param new_value The new value to assign
   * @param new_vref Optional vref setting - Defaults to `MCP4728_VREF_VDD`
   * @param new_gain Optional gain setting - Defaults to `MCP4728_GAIN_1X`
   * @param new_pd_mode Optional power down mode setting - Defaults to
   * `MCP4728_PD_MOOE_NORMAL`
   * @param udac Optional UDAC setting - Defaults to `false`, latching (nearly).
   * Set to `true` to latch when the UDAC pin is pulled low
   *
   */

  //  Setup the 4 DAC channels with the initial values of 0
  MCP12bitDAC.setChannelValue(
      WHITE_1_DAC,           //  channel
      0,                     //  value to write 0-4095
      MCP4728_VREF_INTERNAL, //  internal Vref = 2.048V
      MCP4728_GAIN_2X);      //  gain = 2X

  MCP12bitDAC.setChannelValue(
      WHITE_2_DAC,
      0,
      MCP4728_VREF_INTERNAL,
      MCP4728_GAIN_2X);

  MCP12bitDAC.setChannelValue(
      BLUE_1_DAC,
      0,
      MCP4728_VREF_INTERNAL,
      MCP4728_GAIN_2X);

  MCP12bitDAC.setChannelValue(
      BLUE_2_DAC,
      0,
      MCP4728_VREF_INTERNAL,
      MCP4728_GAIN_2X);

  MCP12bitDAC.saveToEEPROM();

  // Setup RELAY pins
  pinMode(BLUE_RELAY, OUTPUT);
  pinMode(WHITE_RELAY, OUTPUT);
  digitalWrite(WHITE_RELAY, RELAY_ON); // turn on the relay
  digitalWrite(BLUE_RELAY, RELAY_ON);  // turn on the relay

  // Create the node controller core object
  core = NodeControllerCore();
  Serial.println("test");
  // Initialize the node controller core object
  if (core.Init(receive_message, NODE_ID))
  {
    Serial.println("Driver device initialized");
    Serial.println("");
  }
  else
  {
    Serial.println("Failed to initialize driver");
    Serial.println("");
  }

  xTaskCreate(
      SendLEDIntensities,   /* Task function. */
      "SendLEDIntensities", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */
                            /* pin task to core 0 */

  xTaskCreate(
      LightCycles,   /* Task function. */
      "LightCycles", /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      NULL);         /* Task handle to keep track of created task */
                     /* pin task to core 0 */
}

//  --------------------------------------------------------------  Loop  -------------------------------------------------------------------

void loop()
{
}

////////////////////////////////////////////// put function definitions here:  /////////////////////////////////////////////////////////

void LightCycles(void *parameters)
{
  while (1)
  {
    delay(MessageGap);
    updateTimes();

    // Set the LEDs to off in no Light cycles are valid times
    MCP12bitDAC.setChannelValue(WHITE_1_DAC, DAC_OFF); //  set the WHITE_1_DAC to DAC_OFF
    MCP12bitDAC.setChannelValue(WHITE_2_DAC, DAC_OFF); //  set the WHITE_2_DAC to DAC_OFF
    MCP12bitDAC.setChannelValue(BLUE_1_DAC, DAC_OFF);  //  set the BLUE_1_DAC to DAC_OFF
    MCP12bitDAC.setChannelValue(BLUE_2_DAC, DAC_OFF);  //  set the BLUE_2_DAC to DAC_OFF
    digitalWrite(BLUE_RELAY, RELAY_OFF);               // turn off the blue relay
    digitalWrite(WHITE_RELAY, RELAY_OFF);              // turn off the white relay

#ifdef debuging
    Serial.println("Blue LED off");
    Serial.println("White LED off");
    Serial.println("");
#endif

    //-------------------------------------------------- "Dawn" -------------------------------------------------------------------

    while (curTimeSec >= dawnStart && curTimeSec < sunriseStart) // check if the current time is between dawnStart and sunriseStart
    {
      delay(updateLEDs);
      chkmanualOverrideSwitch();                                                                                 //  check if the manual override switch is on
      updateTimes();                                                                                             //  check if the manual override switch is on
      blue_1_MaxIntensity = (int)blue_1_MaxIntensity_float;                                                      //  cast the float to an int
      blue_2_MaxIntensity = (int)blue_2_MaxIntensity_float;                                                      //  cast the float to an int
      currentBlue_1_Intensity = map(curTimeSec, dawnStart, dawnStart + dawnDurationSec, 0, blue_1_MaxIntensity); //  map the current time to the start time and the duration of the dawnDurationSec
      currentBlue_2_Intensity = map(curTimeSec, dawnStart, dawnStart + dawnDurationSec, 0, blue_2_MaxIntensity); //  map the current time to the start time and the duration of the dawnDurationSec
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);                                //  set the BLUE_1_DAC to the mapped intensity
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);                                //  set the BLUE_2_DAC to the mapped intensity
      if (currentBlue_1_Intensity <= minBlueValue)
      {
        digitalWrite(BLUE_RELAY, RELAY_OFF); //  turn off the blue relay
        delay(updateLEDs);
      }
      else
      {
        digitalWrite(BLUE_RELAY, RELAY_ON); //  turn on the blue relay
      }

#ifdef debuging
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.print("curTimeSec = " + String(curTimeSec));
      Serial.print("dawnDurationSec = " + String(dawnDurationSec));
      Serial.print("dawnStart = " + String(dawnStart));
      Serial.print("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.print("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.print("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.print("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.print("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }
    //----------------------------------- "Sunrise" Fade blue and white to highNoon  --------------------------------------------------
    while (curTimeSec >= sunriseStart && curTimeSec < highNoonStart)
    {
      chkmanualOverrideSwitch();
      updateTimes();
      delay(updateLEDs);
      blue_1_MaxIntensity = (int)blue_1_MaxIntensity_float;   //  cast the float to an int
      blue_2_MaxIntensity = (int)blue_2_MaxIntensity_float;   //  cast the float to an int
      white_1_MaxIntensity = (int)white_1_MaxIntensity_float; //  cast the float to an int
      white_2_MaxIntensity = (int)white_2_MaxIntensity_float; //  cast the float to an int
      //                     map( inputValue, low range input, high range input, low range output, high range output);
      currentBlue_1_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, blue_1_MaxIntensity);
      currentBlue_2_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, blue_2_MaxIntensity);
      currentWhite_1_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, white_1_MaxIntensity);
      currentWhite_2_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, white_2_MaxIntensity);
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - currentWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - currentWhite_2_Intensity);

      if (currentWhite_1_Intensity <= minWhiteValue)
      {
        digitalWrite(WHITE_RELAY, RELAY_OFF);
        delay(updateLEDs);
      }
      else
      {
        digitalWrite(WHITE_RELAY, RELAY_ON);
      }

#ifdef debuging
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.print("curTimeSec = " + String(curTimeSec));
      Serial.print("sunriseDurationSec = " + String(sunriseDurationSec));
      Serial.print("sunriseStart = " + String(sunriseStart));
      Serial.print("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.print("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.print("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.print("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.print("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //----------------------------------------------------- "HighNoon" ---------------------------------------------------------------

    while (curTimeSec >= highNoonStart && curTimeSec < sunriseStart)
    {
      chkmanualOverrideSwitch();
      updateTimes();
      delay(updateLEDs);
      blue_1_MaxIntensity = (int)blue_1_MaxIntensity_float;   //  cast the float to an int
      blue_2_MaxIntensity = (int)blue_2_MaxIntensity_float;   //  cast the float to an int
      white_1_MaxIntensity = (int)white_1_MaxIntensity_float; //  cast the float to an int
      white_2_MaxIntensity = (int)white_2_MaxIntensity_float; //  cast the float to an int
      currentBlue_1_Intensity = blue_1_MaxIntensity;
      currentBlue_2_Intensity = blue_2_MaxIntensity;
      currentWhite_1_Intensity = white_1_MaxIntensity;
      currentWhite_2_Intensity = white_2_MaxIntensity;
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - currentWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - currentWhite_2_Intensity);

#ifdef debuging
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.print("curTimeSec = " + String(curTimeSec));
      Serial.print("highNoonDurationSec = " + String(highNoonDurationSec));
      Serial.print("highNoonStart = " + String(highNoonStart));
      Serial.print("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.print("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.print("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.print("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.print("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //------------------------------------------------"Sunset" Fade blue and white to  --------------------------------------------------

    while (curTimeSec >= sunsetStart && curTimeSec < duskStart)
    {
      chkmanualOverrideSwitch();
      updateTimes();
      delay(updateLEDs);
      blue_1_MaxIntensity = (int)blue_1_MaxIntensity_float;   //  cast the float to an int
      blue_2_MaxIntensity = (int)blue_2_MaxIntensity_float;   //  cast the float to an int
      white_1_MaxIntensity = (int)white_1_MaxIntensity_float; //  cast the float to an int
      white_2_MaxIntensity = (int)white_2_MaxIntensity_float; //  cast the float to an int
      currentBlue_1_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, blue_1_MaxIntensity, minBlueValue);
      currentBlue_2_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, blue_2_MaxIntensity, minBlueValue);
      currentWhite_1_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, white_1_MaxIntensity, minWhiteValue);
      currentWhite_2_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, white_2_MaxIntensity, minWhiteValue);
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - currentWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - currentWhite_2_Intensity);

      if (currentWhite_1_Intensity <= minWhiteValue)
      {
        digitalWrite(WHITE_RELAY, RELAY_OFF);
        delay(updateLEDs);
      }
      else
      {
        digitalWrite(WHITE_RELAY, RELAY_ON);
      }

#ifdef debuging
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.print("curTimeSec = " + String(curTimeSec));
      Serial.print("sunsetDurationSec = " + String(sunsetDurationSec));
      Serial.print("sunsetStart = " + String(sunsetStart));
      Serial.print("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.print("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.print("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.print("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.print("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //------------------------------------------------------- Dusk fade blue down -----------------------------------------------------------------------

    while (duskStart <= curTimeSec && curTimeSec < nightTimeStart)
    {
      chkmanualOverrideSwitch();
      updateTimes();
      delay(updateLEDs);
      blue_1_MaxIntensity = (int)blue_1_MaxIntensity_float; //  cast the float to an int
      blue_2_MaxIntensity = (int)blue_2_MaxIntensity_float; //  cast the float to an int
      currentBlue_1_Intensity = map(curTimeSec, duskStart, duskStart + duskDurationSec, minBlueValue, blue_1_MaxIntensity);
      currentBlue_2_Intensity = map(curTimeSec, duskStart, duskStart + duskDurationSec, minBlueValue, blue_2_MaxIntensity);
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);

      if (currentBlue_1_Intensity <= minBlueValue)
      {
        digitalWrite(BLUE_RELAY, RELAY_OFF);
        delay(updateLEDs);
      }
      else
      {
        digitalWrite(BLUE_RELAY, RELAY_ON);
      }
    }

#ifdef debuging
    Serial.println("UNIXtime = " + String(UNIXtime));
    Serial.print("curTimeSec = " + String(curTimeSec));
    Serial.print("duskDurationSec = " + String(duskDurationSec));
    Serial.print("duskStart = " + String(duskStart));
    Serial.print("currentBlue_1_Intensity = ");
    Serial.println(currentBlue_1_Intensity);
    Serial.print("currentWhite_1_Intensity = ");
    Serial.println(currentWhite_1_Intensity);
    Serial.print("Blue relay = ");
    Serial.println(digitalRead(BLUE_RELAY));
    Serial.print("White relay = ");
    Serial.println(digitalRead(WHITE_RELAY));
    Serial.println("");
#endif
  }
  //------------------------------------------------------- nightTime -----------------------------------------------------------------------

  while (curTimeSec >= nightTimeStart || curTimeSec < dawnStart)
  {
    chkmanualOverrideSwitch();
    updateTimes();
    delay(updateLEDs);
    MCP12bitDAC.setChannelValue(WHITE_1_DAC, DAC_OFF);
    MCP12bitDAC.setChannelValue(WHITE_2_DAC, DAC_OFF);
    MCP12bitDAC.setChannelValue(BLUE_1_DAC, DAC_OFF);
    MCP12bitDAC.setChannelValue(BLUE_2_DAC, DAC_OFF);
    digitalWrite(BLUE_RELAY, RELAY_OFF);
    digitalWrite(WHITE_RELAY, RELAY_OFF);
    {

#ifdef debuging
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.print("curTimeSec = " + String(curTimeSec));
      Serial.print("nightTimeDurationSec = " + String(nightTimeDurationSec));
      Serial.print("nightTimeStart = " + String(nightTimeStart));
      Serial.print("currentBlue_1_Intensity = ");
      Serial.println(currentBlue_1_Intensity);
      Serial.print("currentWhite_1_Intensity = ");
      Serial.println(currentWhite_1_Intensity);
      Serial.print("Blue relay = ");
      Serial.println(digitalRead(BLUE_RELAY));
      Serial.print("White relay = ");
      Serial.println(digitalRead(WHITE_RELAY));
      Serial.println("");
#endif
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
  Serial.println("Message received callback");
  Serial.println("");

  // Check if the message is for this node
  if (nodeID == NODE_ID)
  {
    Serial.println("Message received to self");
    // Check the message ID for the LED control messages
    switch (messageID)
    {
      // ---------------------Min max and override values control messages-------------------------

    case MIN_WHITE_VALUE_MESSAGE_ID:
      minWhiteValue = (int)(data / 100) * MAX_DAC;
      Serial.println("Min White Value data =  " + String(data));
      Serial.println("Min White Value set to " + String(minWhiteValue));
      Serial.println("");
      break;

    case MIN_BLUE_VALUE_MESSAGE_ID:
      minBlueValue = (int)(data / 100) * MAX_DAC;
      Serial.println("Min Blue Value data =  " + String(data));
      Serial.println("Min Blue Value set to " + String(minBlueValue));
      Serial.println("");
      break;

    case BLUE_1_MAX_INTENSITY_MESSAGE_ID:
      blue_1_MaxIntensity_float = (data / 100) * MAX_DAC;
      Serial.println("Blue 1 Max Intensity set to " + String(data));
      Serial.println("Blue 1 Max Intensity set to " + String(blue_1_MaxIntensity_float));
      Serial.println("");
      break;

    case BLUE_2_MAX_INTENSITY_MESSAGE_ID:
      blue_2_MaxIntensity_float = (data / 100) * MAX_DAC;
      Serial.println("Blue 2 Max Intensity set to " + String(data));
      Serial.println("Blue 2 Max Intensity set to " + String(blue_2_MaxIntensity_float));
      Serial.println("");
      break;

    case WHITE_1_MAX_INTENSITY_MESSAGE_ID:
      white_1_MaxIntensity_float = (data / 100) * MAX_DAC;
      Serial.println("White 1 Max Intensity set to " + String(data));
      Serial.println("White 1 Max Intensity set to " + String(white_1_MaxIntensity_float));
      Serial.println("");
      break;

    case WHITE_2_MAX_INTENSITY_MESSAGE_ID:
      white_2_MaxIntensity_float = (data / 100) * MAX_DAC;
      Serial.println("White 2 Max Intensity set to " + String(data));
      Serial.println("White 2 Max Intensity set to " + String(white_2_MaxIntensity_float));
      Serial.println("");
      break;

    case MANUAL_OVERRIDE_SWITCH_MESSAGE_ID:
      manualOverrideSwitch = data;
      Serial.println("Manual Override Switch set to " + String(data));
      Serial.println("");
      break;

    case OVERRIDE_WHITE_1_INTENSITY_MESSAGE_ID:
      overrideWhite_1_Intensity = (int)(data / 100) * MAX_DAC;
      Serial.println("Override White 1 Intensity set to " + String(data));
      Serial.println("Override White 1 Intensity set to " + String(overrideWhite_1_Intensity));
      Serial.println("");
      break;

    case OVERRIDE_WHITE_2_INTENSITY_MESSAGE_ID:
      overrideWhite_2_Intensity = (int)(data / 100) * MAX_DAC;
      Serial.println("Override White 2 Intensity set to " + String(data));
      Serial.println("Override White 2 Intensity set to " + String(overrideWhite_2_Intensity));
      Serial.println("");
      break;

    case OVERRIDE_BLUE_1_INTENSITY_MESSAGE_ID:
      overrideBlue_1_Intensity = (int)(data / 100) * MAX_DAC;
      Serial.println("Override Blue 1 Intensity set to " + String(data));
      Serial.println("Override Blue 1 Intensity set to " + String(overrideBlue_1_Intensity));
      Serial.println("");
      break;

    case OVERRIDE_BLUE_2_INTENSITY_MESSAGE_ID:
      overrideBlue_2_Intensity = (int)(data / 100) * MAX_DAC;
      Serial.println("Override Blue 2 Intensity set to " + String(data));
      Serial.println("Override Blue 2 Intensity set to " + String(overrideBlue_2_Intensity));
      Serial.println("");
      break;

      //  ----------------- lightcycle times in hours and mins messages--------------------------------

    case DAWN_MINUTES_MESSAGE_ID:
      dawnMinutes = data;
      Serial.println("Dawn duration set to " + String(data));
      break;

    case DAWN_HOURS_MESSAGE_ID:
      dawnHours = data;
      Serial.println("Dawn duration set to " + String(data));
      break;

    case DUSK_MINUTES_MESSAGE_ID:
      duskMinutes = data;
      Serial.println("Dusk duration set to " + String(data));
      break;

    case DUSK_HOURS_MESSAGE_ID:
      duskHours = data;
      Serial.println("Dusk duration set to " + String(data));
      break;

    case SUNRISE_MINUTES_MESSAGE_ID:
      sunriseMinutes = data;
      Serial.println("Sunrise duration set to " + String(data));
      break;

    case SUNRISE_HOURS_MESSAGE_ID:
      sunriseHours = data;
      Serial.println("Sunrise duration set to " + String(data));
      break;

    case SUNSET_MINUTES_MESSAGE_ID:
      sunsetMinutes = data;
      Serial.println("Sunset duration set to " + String(data));
      break;

    case SUNSET_HOURS_MESSAGE_ID:
      sunsetHours = data;
      Serial.println("Sunset duration set to " + String(data));
      break;

    case HIGH_NOON_MINUTES_MESSAGE_ID:
      highNoonMinutes = data;
      Serial.println("High Noon duration set to " + String(data));
      break;

    case HIGH_NOON_HOURS_MESSAGE_ID:
      highNoonHours = data;
      Serial.println("High Noon duration set to " + String(data));
      break;

    default:
      Serial.println("");
      break;
    }
  }
  // Receive the time info from the base station ----------------------------------------------------------------------------------------------------

  if (nodeID == BASESTATION_ID)
  {
    delay(MessageGap);
    Serial.println("Message received from Base Station");
    Serial.println("");
    getLocalTime(&timeinfo); // Get the current time from the ESP32 RTC

    /*                                           Globals for time keeping
    char localTime[64];                                                                // Local time string
    String localTimeZone;                                                              // "UTC+" + localTimeZoneOffset
    int localTimeZoneOffset = 8;                                                       // Timezone offset
    char buf_localTimeZone[8];                                                         // Char array Buffer for timezone string
    struct tm timeinfo;
    time_t UNIXtime;
    */

    if (messageID == UNIX_TIME_MESSAGE_ID)
    // Time keeping
    {
      delay(MessageGap);
      UNIXtime = data; // Get the UNIX time from the message
      localTimeZone = "UTC+" + String(localTimeZoneOffset);
      localTimeZone.toCharArray(buf_localTimeZone, 8);
      setenv("TZ", buf_localTimeZone, 1);
      tzset();
      localtime_r(&UNIXtime, &timeinfo); // Get the local time
      strftime(localTime, sizeof(localTime), "%c", &timeinfo);
      getLocalTime(&timeinfo); // Get the current time from the ESP32 RTC

#ifdef debugingTime
      Serial.println("Message ID is for UNIX time keeping");
      Serial.println("The local time zone is = " + String(localTimeZone));
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("data = " + String(data));
      Serial.println("The current RTC date/time is: localTime = " + String(localTime));
      Serial.println("buf_localTimeZone = " + String(buf_localTimeZone));
      Serial.println("Time Zone Offset = " + String(localTimeZoneOffset));
      Serial.println("Local Time Zone = " + String(localTimeZone));
      Serial.println("UNIX TIMESTAMP = " + String(UNIXtime));
      Serial.println("");
#endif
    }

    if (messageID == UPDATE_TIMEZONE_OFFSET_MESSAGE_ID)
    {
      localTimeZoneOffset = (int)data;
      localTimeZone = "UTC+" + String(localTimeZoneOffset);
      localTimeZone.toCharArray(buf_localTimeZone, 8);
      setenv("TZ", buf_localTimeZone, 1);
      tzset();
      localtime_r(&UNIXtime, &timeinfo); // Get the local time
      strftime(localTime, sizeof(localTime), "%c", &timeinfo);
      getLocalTime(&timeinfo); // Get the current time from the ESP32 RTC

#ifdef debugingTime
      Serial.println("Message ID is for updating the time zone offset");
      Serial.println("The local time zone is = " + String(localTimeZoneOffset));
      Serial.println("data = " + String(data));
      Serial.println("");
#endif
    }
  }
  else
  {
    Serial.println("Message ID is not for time keeping");
    Serial.println("");
  }
}

// ----------------------------Send the LED intensities to App-------------------------

void SendLEDIntensities(void *parameters)
{
  while (1)
  {
    if (!manualOverrideSwitch)
    {
      uint64_t White_1_Intensity;
      uint64_t White_2_Intensity;
      uint64_t Blue_1_Intensity;
      uint64_t Blue_2_Intensity;
      float White_1_Intensity_float = (currentWhite_1_Intensity / MAX_DAC) * 100;
      float White_2_Intensity_float = (currentWhite_2_Intensity / MAX_DAC) * 100;
      float Blue_1_Intensity_float = (currentBlue_1_Intensity / MAX_DAC) * 100;
      float Blue_2_Intensity_float = (currentBlue_2_Intensity / MAX_DAC) * 100;
      White_1_Intensity = (uint64_t)White_1_Intensity_float;
      White_2_Intensity = (uint64_t)White_2_Intensity_float;
      Blue_1_Intensity = (uint64_t)Blue_1_Intensity_float;
      Blue_2_Intensity = (uint64_t)Blue_2_Intensity_float;
      core.sendMessage(CURRENT_WHITE_1_MESSAGE_ID, &White_1_Intensity, false); // Send the white LED intensity on the Canbus
      delay(updateLEDs);
      core.sendMessage(CURRENT_WHITE_2_MESSAGE_ID, &White_2_Intensity, false); // Send the white LED intensity on the Canbus
      delay(updateLEDs);
      core.sendMessage(CURRENT_BLUE_1_MESSAGE_ID, &Blue_1_Intensity, false); // Send the blue LED intensity on the Canbus
      delay(updateLEDs);
      core.sendMessage(CURRENT_BLUE_2_MESSAGE_ID, &Blue_2_Intensity, false); // Send the blue LED intensity on the Canbus
      delay(sendMqttMessageUpdateUI);
    }
    else
    {
      uint16_t CurrentWhite_1_MessageID = CURRENT_WHITE_1_MESSAGE_ID;
      uint16_t CurrentWhite_2_MessageID = CURRENT_WHITE_2_MESSAGE_ID;
      uint16_t CurrentBlue_1_MessageID = CURRENT_BLUE_1_MESSAGE_ID;
      uint16_t CurrentBlue_2_MessageID = CURRENT_BLUE_2_MESSAGE_ID;
      uint64_t White_1_Intensity;
      uint64_t White_2_Intensity;
      uint64_t Blue_1_Intensity;
      uint64_t Blue_2_Intensity;
      White_1_Intensity = overrideWhite_1_Intensity;
      White_2_Intensity = overrideWhite_2_Intensity;
      Blue_1_Intensity = overrideBlue_1_Intensity;
      Blue_2_Intensity = overrideBlue_2_Intensity;
      core.sendMessage(CurrentWhite_1_MessageID, &White_1_Intensity, false); // Send the white LED intensity on the Canbus
      delay(updateLEDs);
      core.sendMessage(CurrentWhite_2_MessageID, &White_2_Intensity, false); // Send the white LED intensity on the Canbus
      delay(updateLEDs);
      core.sendMessage(CurrentBlue_1_MessageID, &Blue_1_Intensity, false); // Send the blue LED intensity on the Canbus
      delay(updateLEDs);
      core.sendMessage(CurrentBlue_2_MessageID, &Blue_2_Intensity, false); // Send the blue LED intensity on the Canbus
      delay(sendMqttMessageUpdateUI);
    }
  }
}

void chkmanualOverrideSwitch()
{
  while (manualOverrideSwitch)
  {
    if (overrideWhite_1_Intensity < minWhiteValue)
    {
      digitalWrite(WHITE_RELAY, RELAY_ON);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, DAC_OFF);
    }
    else
    {
      digitalWrite(WHITE_RELAY, RELAY_OFF);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - overrideWhite_1_Intensity);
    }

    if (overrideWhite_2_Intensity < minWhiteValue)
    {
      digitalWrite(WHITE_RELAY, RELAY_ON);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, DAC_OFF);
    }
    else
    {
      digitalWrite(WHITE_RELAY, RELAY_OFF);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - overrideWhite_2_Intensity);
    }

    if (overrideBlue_1_Intensity < minBlueValue)
    {
      digitalWrite(BLUE_RELAY, RELAY_OFF);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, DAC_OFF);
    }
    else
    {
      digitalWrite(BLUE_RELAY, RELAY_ON);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - overrideBlue_1_Intensity);
    }

    if (overrideBlue_2_Intensity < minBlueValue)
    {
      digitalWrite(BLUE_RELAY, RELAY_OFF);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, DAC_OFF);
    }
    else
    {
      digitalWrite(BLUE_RELAY, RELAY_ON);
      delay(updateLEDs);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - overrideBlue_2_Intensity);
    }

#ifdef debuging
    Serial.println("Manual LED control override switch is = " + String(manualOverrideSwitch));
    Serial.println("Blue relay = " + String(digitalRead(BLUE_RELAY)));
    Serial.println("White relay = " + String(digitalRead(WHITE_RELAY)));
    Serial.println("Override white intensity = " + String(overrideWhite_1_Intensity));
    Serial.println("Override blue intensity = " + String(overrideBlue_1_Intensity));
    Serial.println("");
#endif
  }
}

// Function to update the times and schedules of the light cycles

void updateTimes()
{
  while (1)
  {
    delay(MessageGap);
    curTimeSec = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;                                              //  get the start time in seconds since midnight
    dawnDurationSec = (((sunriseHours * 3600) + (sunriseMinutes * 60)) - ((dawnHours * 3600) + (dawnMinutes * 60)));            //  calculate the duration of the dawn cycle in seconds
    sunriseDurationSec = (((highNoonHours * 3600) + (highNoonMinutes * 60)) - ((sunriseHours * 3600) + (sunriseMinutes * 60))); //  calculate the duration of the sunrise cycle in seconds
    highNoonDurationSec = (((sunsetHours * 3600) + (sunsetMinutes * 60)) - ((highNoonHours * 3600) + (highNoonMinutes * 60)));  //  calculate the duration of the highNoon cycle in seconds
    sunsetDurationSec = (((duskHours * 3600) + (duskMinutes * 60)) - ((sunsetHours * 3600) + (sunsetMinutes * 60)));            //  calculate the duration of the sunset cycle in seconds
    duskDurationSec = (((nightTimeHours * 3600) + (nightTimeMinutes * 60)) - ((duskHours * 3600) + (duskMinutes * 60)));        //  calculate the duration of the dusk cycle in seconds
    nightTimeDurationSec = ((MIDNIGHT + dawnStart) - ((nightTimeHours * 3600) + (nightTimeMinutes * 60)));                      //  calculate the duration of the nightTime cycle in seconds
    dawnStart = (dawnHours * 3600) + (dawnMinutes * 60);                                                                        //  calculate the start time of dawn in seconds since midnight
    sunriseStart = (sunriseHours * 3600) + (sunriseMinutes * 60);                                                               //  calculate the start time of sunrise in seconds since midnight
    highNoonStart = (highNoonHours * 3600) + (highNoonMinutes * 60);                                                            //  calculate the start time of high noon in seconds since midnight
    sunsetStart = (sunsetHours * 3600) + (sunsetMinutes * 60);                                                                  //  calculate the start time of sunset in seconds since midnight
    duskStart = (duskHours * 3600) + (duskMinutes * 60);                                                                        //  calculate the start time of dusk in seconds since midnight
    nightTimeStart = (nightTimeHours * 3600) + (nightTimeMinutes * 60);                                                         //  calculate the start time of night time in seconds since midnight

#ifdef debuging
    Serial.println("Running updateTimes function");
    Serial.println("timeinfo = " + String());
    Serial.println("UNIXtime = " + String(UNIXtime));
    Serial.println("24 hour time = " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));
    Serial.println("curTimeSec in seconds  = " + String(curTimeSec));
    Serial.println("dawnDurationSec = " + String(dawnDurationSec));
    Serial.println("sunriseDurationSec = " + String(sunriseDurationSec));
    Serial.println("highNoonDurationSec = " + String(highNoonDurationSec));
    Serial.println("sunsetDurationSec = " + String(sunsetDurationSec));
    Serial.println("duskDurationSec = " + String(duskDurationSec));
    Serial.println("nightTimeDurationSec = " + String(nightTimeDurationSec));
    Serial.println("dawnStart = " + String(dawnStart));
    Serial.println("sunriseStart = " + String(sunriseStart));
    Serial.println("highNoonStart = " + String(highNoonStart));
    Serial.println("sunsetStart = " + String(sunsetStart));
    Serial.println("duskStart = " + String(duskStart));
    Serial.println("nightTimeStart = " + String(nightTimeStart));
    Serial.println("");
#endif
  }
}