#include <Arduino.h>
#include <NodeControllerCore.h>
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <Adafruit_MCP4728.h>
#include "DFRobot_GP8403.h"

// Insert UNIX Time below when flashing firmware
// What is current UNIX time for testing Jan 1, 2026 10:09:00 PM UTC?
#define TimeSet 1767305485
//--------------------------------------------- put #define statements here: ----------------------------------------------------

// #define debuging
// #define debugingTime // comment out to remove time debuging

// #define usingAdafruit_MCP4728
// #define usingDFRobot_GP8403
// #define using_2X_DFRobot_GP8403
#define usingPWM_to_0to10V

//  RELAY logic, change if inverted logic
#define RELAY_ON 1
#define RELAY_OFF 0

#ifdef usingDFRobot_GP8403
// create an instance DFRobot_GP8403 of DAC using the Wire library
DFRobot_GP8403 DFRobot_GP8403_1(&Wire, 0x5F); // instance of the DAC at I2C address 0x5F
DFRobot_GP8403 DFRobot_GP8403_2(&Wire, 0x60); // instance of the DAC at I2C address 0x60
#define WHITE_1_DAC_DFRobot 0                 // DAC channel for WHITE_1
#define WHITE_2_DAC_DFRobot 0                 // DAC channel for WHITE_2
#define BLUE_1_DAC_DFRobot 1                  // DAC channel for BLUE_1
#define BLUE_2_DAC_DFRobot 1                  // DAC channel for BLUE_2
int MAX_DAC = 10000;                          // maximum DAC output value for 10V range
int DAC_OFF = 0;                              // value to turn off the DAC output
#endif

#ifdef usingAdafruit_MCP4728
// Adafruit_MCP4728 DAC object
Adafruit_MCP4728 MCP12bitDAC; //  create the MCP4728 object 12 bits of resolution = 4096 steps, so MAX_DAC = 4095
int MAX_DAC = 4095;           //  this can be changed if there is a different resolution than 12bit
int DAC_OFF = MAX_DAC;        //  change this to 0 if the DAC is not inverted
#endif

#ifdef usingPWM_to_0to10V
// PWM to 0-10V variables
#define WHITE_1_PWM_PIN 0 // PWM pin for WHITE_1
#define WHITE_2_PWM_PIN 1 // PWM pin for WHITE_2
#define BLUE_1_PWM_PIN 2  // PWM pin for BLUE_1
#define BLUE_2_PWM_PIN 3  // PWM pin for BLUE_2

#define PWM_FREQUENCY 1000                // PWM frequency IN HZ
#define PWM_RESOLUTION 10                 // PWM resolution in bits
int MAX_DAC = pow(2, PWM_RESOLUTION) - 1; // maximum PWM value based on resolution
int DAC_OFF = 0;                          // value to turn off the PWM output

#define WHITE_CHANNEL_1 0
#define WHITE_CHANNEL_2 1
#define BLUE_CHANNEL_1 2
#define BLUE_CHANNEL_2 3
#endif

// USE int FOR I2C PIN DEFINITIONS
int I2C_SDA = 2;
int I2C_SCL = 3;

// define the node ID and message IDs
#define BASESTATION_ID 0x00                         // 0
#define NODE_ID 0xA1                                // 161
#define UNIX_TIME_MESSAGE_ID 2000                   // 2000
#define UPDATE_TIMEZONE_OFFSET_MESSAGE_ID 2001      // 2001
#define DAWN_MINUTES_MESSAGE_ID 25060               // 0xA00
uint64_t dawnMinutes = 30;                          // variable to store the dawn minutes message data
#define DAWN_HOURS_MESSAGE_ID 25061                 // 0xA01
uint64_t dawnHours = 9;                             // variable to store the dawn hours message data
#define DUSK_MINUTES_MESSAGE_ID 25062               // 0xA02
uint64_t duskMinutes = 30;                          // variable to store the dusk minutes message data
#define DUSK_HOURS_MESSAGE_ID 25063                 // 0xA03
uint64_t duskHours = 21;                            // variable to store the dusk hours message data
#define SUNRISE_MINUTES_MESSAGE_ID 25064            // 0xA04
uint64_t sunriseMinutes;                            // variable to store the sunrise minutes message data
#define SUNRISE_HOURS_MESSAGE_ID 25065              // 0xA05
uint64_t sunriseHours = 11;                         // variable to store the sunrise hours message data
#define SUNSET_MINUTES_MESSAGE_ID 25066             // 0xA06
uint64_t sunsetMinutes;                             // variable to store the sunset minutes message data
#define SUNSET_HOURS_MESSAGE_ID 25067               // 0xA07
uint64_t sunsetHours = 19;                          // variable to store the sunset hours message data
#define HIGH_NOON_MINUTES_MESSAGE_ID 25068          // 0xA08
uint64_t highNoonMinutes;                           // variable to store the high noon minutes message data
#define HIGH_NOON_HOURS_MESSAGE_ID 25069            // 0xA09
uint64_t highNoonHours = 13;                        // variable to store the high noon hours message data
#define NIGHT_TIME_MINUTES_MESSAGE_ID 25070         // 0xA0A
uint64_t nightTimeMinutes = 30;                     // variable to store the night time minutes message data
#define NIGHT_TIME_HOURS_MESSAGE_ID 25071           // 0xA0B
uint64_t nightTimeHours = 22;                       // variable to store the night time hours message data
#define BLUE_1_MAX_INTENSITY_MESSAGE_ID 25072       // 0xA0C
uint64_t blue_1_MaxIntensity = 75;                  // variable to store the blue_1 max intensity message data
float blue_1_MaxIntensity_float;                    // variable to store the blue_1 max intensity message data
#define BLUE_2_MAX_INTENSITY_MESSAGE_ID 25073       // 0xA0D
uint64_t blue_2_MaxIntensity = 75;                  // variable to store the blue_2 max intensity message data
float blue_2_MaxIntensity_float;                    // variable to store the blue_2 max intensity message data
#define WHITE_1_MAX_INTENSITY_MESSAGE_ID 25074      // 0xA0E
uint64_t white_1_MaxIntensity = 75;                 // variable to store the white_1 max intensity message data
float white_1_MaxIntensity_float;                   // variable to store the white_1 max intensity message data
#define WHITE_2_MAX_INTENSITY_MESSAGE_ID 25075      // 0xA0F
uint64_t white_2_MaxIntensity = 75;                 // variable to store the white_2 max intensity message data
float white_2_MaxIntensity_float;                   // variable to store the white_2 max intensity message data
#define CURRENT_WHITE_1_MESSAGE_ID 25076            // 0xA10
uint64_t currentWhite_1_Intensity;                  // variable to store the current white_1 intensity message data
float currentWhite_1_Intensity_float;               // variable to store the current white_1 intensity message data
#define CURRENT_WHITE_2_MESSAGE_ID 25077            // 0xA11
uint64_t currentWhite_2_Intensity;                  // variable to store the current white_2 intensity message data
float currentWhite_2_Intensity_float;               // variable to store the current white_2 intensity message data
#define CURRENT_BLUE_1_MESSAGE_ID 25078             // 0xA12
uint64_t currentBlue_1_Intensity;                   // variable to store the current blue_1 intensity message data
float currentBlue_1_Intensity_float;                // variable to store the current blue_1 intensity message data
#define CURRENT_BLUE_2_MESSAGE_ID 25079             // 0xA13
uint64_t currentBlue_2_Intensity;                   // variable to store the current blue_2 intensity message data
float currentBlue_2_Intensity_float;                // variable to store the current blue_2 intensity message data
#define MANUAL_OVERRIDE_SWITCH_MESSAGE_ID 25080     // 0xA14
bool manualOverrideSwitch = false;                  // variable to store the manual override switch message data
#define OVERRIDE_WHITE_1_INTENSITY_MESSAGE_ID 25081 // 0xA15
uint64_t overrideWhite_1_Intensity;                 // variable to store the override white_1 intensity message data
float overrideWhite_1_Intensity_float;              // variable to store the override white_1 intensity message data
#define OVERRIDE_WHITE_2_INTENSITY_MESSAGE_ID 25082 // 0xA16
uint64_t overrideWhite_2_Intensity;                 // variable to store the override white_2 intensity message data
float overrideWhite_2_Intensity_float;              // variable to store the override white_2 intensity message data
#define OVERRIDE_BLUE_1_INTENSITY_MESSAGE_ID 25083  // 0xA17
uint64_t overrideBlue_1_Intensity;                  // variable to store the override blue_1 intensity message data
float overrideBlue_1_Intensity_float;               // variable to store the override blue_1 intensity message data
#define OVERRIDE_BLUE_2_INTENSITY_MESSAGE_ID 25084  // 0xA18
uint64_t overrideBlue_2_Intensity;                  // variable to store the override blue_2 intensity message data
float overrideBlue_2_Intensity_float;               // variable to store the override blue_2 intensity message data
#define MIN_WHITE_VALUE_MESSAGE_ID 25085            // 0xA19
u_int64_t minWhiteValue = 0;                        // variable to store the min white value message data
float minWhiteValue_float;                          // variable to store the min white value message data
#define MIN_BLUE_VALUE_MESSAGE_ID 25086             // 0xA1A
uint64_t minBlueValue = 0;                          // variable to store the min blue value message data
float minBlueValue_float;                           // variable to store the min blue value message data

#ifdef usingAdafruit_MCP4728
// colored and numbered names for the DAC channels to take values from 0-4095
#define WHITE_1_DAC MCP4728_CHANNEL_A
#define WHITE_2_DAC MCP4728_CHANNEL_B
#define BLUE_1_DAC MCP4728_CHANNEL_C
#define BLUE_2_DAC MCP4728_CHANNEL_D
#endif

// RELAY pins
#define BLUE_RELAY 3
#define WHITE_RELAY 2

#define sendMqttMessageUpdateUI 5000 // frequency of sending LED intensity messages to the App
#define updateLEDs 100               // Delay time in milliseconds
#define MessageGap 1000              // Delay time in milliseconds

// Globals for time keeping
char localTime[64];          // Local time string
String localTimeZone;        // "UTC+" + localTimeZoneOffset
int localTimeZoneOffset = 8; // Timezone offset
char buf_localTimeZone[8];   // Char array Buffer for timezone string
struct tm timeinfo;          // Time structure
struct timeval tv;           // Time value structure
time_t UNIXtime;             // UNIX time
int curTimeSec;              //  get the start time in seconds since midnight
int dawnStart;               //  start time of dawn in seconds since midnight
int sunriseStart;            //  start time of sunrise in seconds since midnight
int highNoonStart;           //  start time of high noon in seconds since midnight
int sunsetStart;             //  start time of sunset in seconds since midnight
int duskStart;               //  start time of dusk in seconds since midnight
int nightTimeStart;          //  start time of night time in seconds since midnight
int dawnDurationSec;         //  duration of dawn in seconds
int sunriseDurationSec;      //  duration of sunrise in seconds
int highNoonDurationSec;     //  duration of high noon in seconds
int sunsetDurationSec;       //  duration of sunset in seconds
int duskDurationSec;         //  duration of dusk in seconds
int nightTimeDurationSec;    //  duration of night time in seconds
int MIDNIGHT = 86400;        //  24 hours in seconds

// Node controller core object
NodeControllerCore core;

//--------------------------------------------- put function declarations here:----------------------------------------------------

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

// Function to set the PWM value for a given channel
void setPWM(int channel, int value);

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

  // Initialize the time keeping
  tv.tv_sec = TimeSet; // Set the time in seconds since epoch
  localTimeZone = "UTC+" + String(localTimeZoneOffset);
  localTimeZone.toCharArray(buf_localTimeZone, 8);
  setenv("TZ", buf_localTimeZone, 1);
  tzset();
  settimeofday(&tv, NULL);
  // getLocalTime(&timeinfo);

  // Initialize the OneWire communication
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Initializing I2C Communication");
  Serial.println("");

#ifdef usingPWM_to_0to10V
  //  Initialize the LEDC channelS for PWM to 0-10V
  ledcSetup(WHITE_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION); //  setup WHITE_CHANNEL_1
  ledcSetup(WHITE_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION); //  setup WHITE_CHANNEL_2
  ledcSetup(BLUE_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);  //  setup BLUE_CHANNEL_1
  ledcSetup(BLUE_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);  //  setup BLUE_CHANNEL_2

  //  Attach the LEDC channels to the PWM pins
  ledcAttachPin(WHITE_1_PWM_PIN, WHITE_CHANNEL_1); //  attach WHITE_1_PWM_PIN to WHITE_CHANNEL_1
  ledcAttachPin(WHITE_2_PWM_PIN, WHITE_CHANNEL_2); //  attach WHITE_2_PWM_PIN to WHITE_CHANNEL_2
  ledcAttachPin(BLUE_1_PWM_PIN, BLUE_CHANNEL_1);   //  attach BLUE_1_PWM_PIN to BLUE_CHANNEL_1
  ledcAttachPin(BLUE_2_PWM_PIN, BLUE_CHANNEL_2);   //  attach BLUE_2_PWM_PIN to BLUE_CHANNEL_2

  //  Set all PWM channels to 0 (off)
  setPWM(WHITE_CHANNEL_1, 0);
  setPWM(WHITE_CHANNEL_2, 0);
  setPWM(BLUE_CHANNEL_1, 0);
  setPWM(BLUE_CHANNEL_2, 0);
#endif

#ifdef usingDFRobot_GP8403
  //  Initialize the DAC
  Serial.println("Initializing DFRobot GP8403 DAC");
  Serial.println("");
  if (!DFRobot_GP8403_1.begin())
  {
    Serial.println("Error initializing DFRobot GP8403 DAC");
    Serial.println("Please check your connections");
    Serial.println("");
  }
  else
  {
    Serial.println("DFRobot GP8403 DAC Initialized");
    Serial.println("");
  }
#endif

#ifdef using_2X_DFRobot_GP8403
  if (!DFRobot_GP8403_2.begin())
  {
    Serial.println("Error initializing 2nd DFRobot GP8403 DAC");
    Serial.println("Please check your connections");
    Serial.println("");
  }
  else
  {
    Serial.println("2nd DFRobot GP8403 DAC Initialized");
    Serial.println("");
  }
#endif

#ifdef usingAdafruit_MCP4728
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
#endif

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
      SendLEDIntensities,   // Task function
      "SendLEDIntensities", // name of task
      10000,                // Stack size of task
      NULL,                 // parameter of the task
      1,                    // priority of the task
      NULL);                // Task handle to keep track of created task
                            // pin task to core 0

  xTaskCreate(
      LightCycles,   /* Task function. */
      "LightCycles", /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      NULL);         /* Task handle to keep track of created task */
                     /* pin task to core 0 */

/*      PWM Test and Calabration Code
while (1)
{
setPWM(WHITE_CHANNEL_1, MAX_DAC);
setPWM(WHITE_CHANNEL_2, MAX_DAC);
setPWM(BLUE_CHANNEL_1, MAX_DAC);
setPWM(BLUE_CHANNEL_2, MAX_DAC);
}

while (1)
{
for (int dutyCycle = 0; dutyCycle <= MAX_DAC; dutyCycle += 10)
{
setPWM(WHITE_CHANNEL_1, dutyCycle);
delay(100);
}
for (int dutyCycle = MAX_DAC; dutyCycle >= 0; dutyCycle -= 10)
{
setPWM(WHITE_CHANNEL_1, dutyCycle);
delay(100);
}
}
*/
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

    Serial.println("");
    Serial.println("Starting Light Cycles Function " + String(curTimeSec));
    Serial.println("");

    // Set the LEDs to off in no Light cycles are valid times

#ifdef usingDFRobot_GP8403
    DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, DAC_OFF); //  set the WHITE_1_DAC to 0V
    DFRobot_GP8403_2.setDACOutVoltage(WHITE_2_DAC_DFRobot, DAC_OFF); //  set the WHITE_2_DAC to 0V
    DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, DAC_OFF);  //  set the BLUE_1_DAC to 0V
    DFRobot_GP8403_2.setDACOutVoltage(BLUE_2_DAC_DFRobot, DAC_OFF);  //  set the BLUE_2_DAC to 0V
#endif

#ifdef usingAdafruit_MCP4728
    MCP12bitDAC.setChannelValue(WHITE_1_DAC, DAC_OFF); //  set the WHITE_1_DAC to DAC_OFF
    MCP12bitDAC.setChannelValue(WHITE_2_DAC, DAC_OFF); //  set the WHITE_2_DAC to DAC_OFF
    MCP12bitDAC.setChannelValue(BLUE_1_DAC, DAC_OFF);  //  set the BLUE_1_DAC to DAC_OFF
    MCP12bitDAC.setChannelValue(BLUE_2_DAC, DAC_OFF);  //  set the BLUE_2_DAC to DAC_OFF
#endif

#ifdef usingPWM_to_0to10V
    // Turn off all PWM channels
    setPWM(WHITE_CHANNEL_1, 0);
    setPWM(WHITE_CHANNEL_2, 0);
    setPWM(BLUE_CHANNEL_1, 0);
    setPWM(BLUE_CHANNEL_2, 0);
#endif

    digitalWrite(BLUE_RELAY, RELAY_OFF);  // turn off the blue relay
    digitalWrite(WHITE_RELAY, RELAY_OFF); // turn off the white relay

#ifdef debuging
    Serial.println("Blue LED off");
    Serial.println("White LED off");
    Serial.println("curTimeSec 2 = " + String(curTimeSec));
    Serial.println("");
#endif

    //-------------------------------------------------- "Dawn" -------------------------------------------------------------------

    while (curTimeSec >= dawnStart && curTimeSec < sunriseStart) // check if the current time is between dawnStart and sunriseStart
    {
      Serial.println("In Dawn Loop curTimeSec = " + String(curTimeSec));
      Serial.println("");
      
      delay(updateLEDs);
      updateTimes();
      chkmanualOverrideSwitch();    //  check if the manual override switch is on
      
      currentBlue_1_Intensity = map(curTimeSec, dawnStart, dawnStart + dawnDurationSec, 0, blue_1_MaxIntensity); //  map the current time to the start time and the duration of the dawnDurationSec
      currentBlue_2_Intensity = map(curTimeSec, dawnStart, dawnStart + dawnDurationSec, 0, blue_2_MaxIntensity); //  map the current time to the start time and the duration of the dawnDurationSec

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, currentBlue_1_Intensity); //  set the BLUE_1_DAC to the mapped intensity
      DFRobot_GP8403_2.setDACOutVoltage(BLUE_2_DAC_DFRobot, currentBlue_2_Intensity); //  set the BLUE_2_DAC to the mapped intensity
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity); //  set the BLUE_1_DAC to the mapped intensity
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity); //  set the BLUE_2_DAC to the mapped intensity
#endif

#ifdef usingPWM_to_0to10V
      // Set the PWM value for each channel
      setPWM(BLUE_CHANNEL_1, currentBlue_1_Intensity);
      setPWM(BLUE_CHANNEL_2, currentBlue_2_Intensity);
#endif

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
      Serial.println("while (curTimeSec >= dawnStart && curTimeSec < sunriseStart)");
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("curTimeSec = " + String(curTimeSec));
      Serial.println("dawnDurationSec = " + String(dawnDurationSec));
      Serial.println("dawnStart = " + String(dawnStart));
      Serial.println("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.println("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.println("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.println("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.println("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //----------------------------------- "Sunrise" Fade blue and white to highNoon  --------------------------------------------------
    while (curTimeSec >= sunriseStart && curTimeSec < highNoonStart)
    {
      Serial.println("In Sunrise Loop curTimeSec = " + String(curTimeSec));
      Serial.println("");
      
      delay(updateLEDs);
      updateTimes();
      chkmanualOverrideSwitch();

      //                     map( inputValue, low range input, high range input, low range output, high range output);
      currentBlue_1_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, blue_1_MaxIntensity);
      currentBlue_2_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, blue_2_MaxIntensity);
      currentWhite_1_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, white_1_MaxIntensity);
      currentWhite_2_Intensity = map(curTimeSec, sunriseStart, sunriseStart + sunriseDurationSec, minBlueValue, white_2_MaxIntensity);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, currentBlue_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_2_DAC_DFRobot, currentBlue_2_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, currentWhite_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_2_DAC_DFRobot, currentWhite_2_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - currentWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - currentWhite_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, currentBlue_1_Intensity);
      setPWM(BLUE_CHANNEL_2, currentBlue_2_Intensity);
      setPWM(WHITE_CHANNEL_1, currentWhite_1_Intensity);
      setPWM(WHITE_CHANNEL_2, currentWhite_2_Intensity);
#endif

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
      Serial.println("while (curTimeSec >= sunriseStart && curTimeSec < highNoonStart)");
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("curTimeSec = " + String(curTimeSec));
      Serial.println("sunriseDurationSec = " + String(sunriseDurationSec));
      Serial.println("sunriseStart = " + String(sunriseStart));
      Serial.println("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.println("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.println("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.println("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.println("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //----------------------------------------------------- "HighNoon" ---------------------------------------------------------------

    while (curTimeSec >= highNoonStart && curTimeSec < sunsetStart)
    {
      Serial.println("In HighNoon Loop curTimeSec = " + String(curTimeSec));
      Serial.println("");

      delay(updateLEDs);
      updateTimes();
      chkmanualOverrideSwitch();

      currentBlue_1_Intensity = blue_1_MaxIntensity;
      currentBlue_2_Intensity = blue_2_MaxIntensity;
      currentWhite_1_Intensity = white_1_MaxIntensity;
      currentWhite_2_Intensity = white_2_MaxIntensity;

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, currentBlue_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_2_DAC_DFRobot, currentBlue_2_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, currentWhite_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_2_DAC_DFRobot, currentWhite_2_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - currentWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - currentWhite_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, currentBlue_1_Intensity);
      setPWM(BLUE_CHANNEL_2, currentBlue_2_Intensity);
      setPWM(WHITE_CHANNEL_1, currentWhite_1_Intensity);
      setPWM(WHITE_CHANNEL_2, currentWhite_2_Intensity);
#endif

#ifdef debuging
      Serial.println("while (curTimeSec >= highNoonStart && curTimeSec < sunsetStart)");
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("curTimeSec = " + String(curTimeSec));
      Serial.println("highNoonDurationSec = " + String(highNoonDurationSec));
      Serial.println("highNoonStart = " + String(highNoonStart));
      Serial.println("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.println("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.println("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.println("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.println("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //------------------------------------------------"Sunset" Fade blue and white to  --------------------------------------------------

    while (curTimeSec >= sunsetStart && curTimeSec < duskStart)
    {
      Serial.println("In Sunset Loop curTimeSec = " + String(curTimeSec));
      Serial.println("");
      
      delay(updateLEDs);
      updateTimes();
      chkmanualOverrideSwitch();

      currentBlue_1_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, blue_1_MaxIntensity, minBlueValue);
      currentBlue_2_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, blue_2_MaxIntensity, minBlueValue);
      currentWhite_1_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, white_1_MaxIntensity, minWhiteValue);
      currentWhite_2_Intensity = map(curTimeSec, sunsetStart, sunsetStart + sunsetDurationSec, white_2_MaxIntensity, minWhiteValue);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, currentBlue_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_2_DAC_DFRobot, currentBlue_2_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, currentWhite_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_2_DAC_DFRobot, currentWhite_2_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - currentWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - currentWhite_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, currentBlue_1_Intensity);
      setPWM(BLUE_CHANNEL_2, currentBlue_2_Intensity);
      setPWM(WHITE_CHANNEL_1, currentWhite_1_Intensity);
      setPWM(WHITE_CHANNEL_2, currentWhite_2_Intensity);
#endif

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
      Serial.println("while (curTimeSec >= sunsetStart && curTimeSec < duskStart)");
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("curTimeSec = " + String(curTimeSec));
      Serial.println("sunsetDurationSec = " + String(sunsetDurationSec));
      Serial.println("sunsetStart = " + String(sunsetStart));
      Serial.println("blue_1_MaxIntensity = " + String(blue_1_MaxIntensity));
      Serial.println("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
      Serial.println("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
      Serial.println("Blue relay = " + String(digitalRead(BLUE_RELAY)));
      Serial.println("White relay = " + String(digitalRead(WHITE_RELAY)));
      Serial.println("");
#endif
    }

    //------------------------------------------------------- Dusk fade blue down -----------------------------------------------------------------------

    while (duskStart <= curTimeSec && curTimeSec < nightTimeStart)
    {
      Serial.println("In Dusk Loop curTimeSec = " + String(curTimeSec));
      Serial.println("");
      
      delay(updateLEDs);
      updateTimes();
      chkmanualOverrideSwitch();

      currentBlue_1_Intensity = map(curTimeSec, duskStart, duskStart + duskDurationSec, minBlueValue, blue_1_MaxIntensity);
      currentBlue_2_Intensity = map(curTimeSec, duskStart, duskStart + duskDurationSec, minBlueValue, blue_2_MaxIntensity);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, currentBlue_1_Intensity);
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_2_DAC_DFRobot, currentBlue_2_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - currentBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - currentBlue_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, currentBlue_1_Intensity);
      setPWM(BLUE_CHANNEL_2, currentBlue_2_Intensity);
#endif

      if (currentBlue_1_Intensity <= minBlueValue)
      {
        digitalWrite(BLUE_RELAY, RELAY_OFF);
        delay(updateLEDs);
      }
      else
      {
        digitalWrite(BLUE_RELAY, RELAY_ON);
      }

#ifdef debuging
      Serial.println("while (duskStart <= curTimeSec && curTimeSec < nightTimeStart)");
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("curTimeSec = " + String(curTimeSec));
      Serial.println("duskDurationSec = " + String(duskDurationSec));
      Serial.println("duskStart = " + String(duskStart));
      Serial.println("currentBlue_1_Intensity = ");
      Serial.println(currentBlue_1_Intensity);
      Serial.println("currentWhite_1_Intensity = ");
      Serial.println(currentWhite_1_Intensity);
      Serial.println("Blue relay = ");
      Serial.println(digitalRead(BLUE_RELAY));
      Serial.println("White relay = ");
      Serial.println(digitalRead(WHITE_RELAY));
      Serial.println("");
#endif
    }
    //------------------------------------------------------- nightTime -----------------------------------------------------------------------

    while (curTimeSec >= nightTimeStart || curTimeSec < dawnStart)
    {
      Serial.println("In Night Loop curTimeSec = " + String(curTimeSec));
      Serial.println("");
      
      delay(updateLEDs);
      updateTimes();
      chkmanualOverrideSwitch();

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, DAC_OFF); //  set the WHITE_1_DAC to 0V
      DFRobot_GP8403_2.setDACOutVoltage(WHITE_2_DAC_DFRobot, DAC_OFF); //  set the WHITE_2_DAC to 0V
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, DAC_OFF);  //  set the BLUE_1_DAC to 0V
      DFRobot_GP8403_2.setDACOutVoltage(BLUE_2_DAC_DFRobot, DAC_OFF);  //  set the BLUE_2_DAC to 0V
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, DAC_OFF);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, DAC_OFF);
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, DAC_OFF);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, DAC_OFF);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(WHITE_CHANNEL_1, 0);
      setPWM(WHITE_CHANNEL_2, 0);
      setPWM(BLUE_CHANNEL_1, 0);
      setPWM(BLUE_CHANNEL_2, 0);
#endif

      digitalWrite(BLUE_RELAY, RELAY_OFF);
      digitalWrite(WHITE_RELAY, RELAY_OFF);

#ifdef debuging
      Serial.println("while (curTimeSec >= nightTimeStart || curTimeSec < dawnStart)");
      Serial.println("UNIXtime = " + String(UNIXtime));
      Serial.println("curTimeSec = " + String(curTimeSec));
      Serial.println("nightTimeDurationSec = " + String(nightTimeDurationSec));
      Serial.println("nightTimeStart = " + String(nightTimeStart));
      Serial.println("currentBlue_1_Intensity = ");
      Serial.println(currentBlue_1_Intensity);
      Serial.println("currentWhite_1_Intensity = ");
      Serial.println(currentWhite_1_Intensity);
      Serial.println("Blue relay = ");
      Serial.println(digitalRead(BLUE_RELAY));
      Serial.println("White relay = ");
      Serial.println(digitalRead(WHITE_RELAY));
      Serial.println("");
#endif
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to set the PWM value for a given channel
void setPWM(int channel, int value)
{
  ledcWrite(channel, value); // value: 0–255 for 8-bit
}

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
  Serial.println("CanBus Message received");
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
      minWhiteValue = (data * MAX_DAC) / 100;
      Serial.println("Min White Value data =  " + String(data));
      Serial.println("Min White Value set to " + String(minWhiteValue));
      Serial.println("");
      break;

    case MIN_BLUE_VALUE_MESSAGE_ID:
      minBlueValue = (data * MAX_DAC) / 100;
      Serial.println("Min Blue Value data =  " + String(data));
      Serial.println("Min Blue Value set to " + String(minBlueValue));
      Serial.println("");
      break;

    case BLUE_1_MAX_INTENSITY_MESSAGE_ID:
      blue_1_MaxIntensity = (data * MAX_DAC) / 100;
      Serial.println("Blue 1 Max Intensity set to " + String(data));
      Serial.println("Blue 1 Max Intensity set to " + String(blue_1_MaxIntensity_float));
      Serial.println("");
      break;

    case BLUE_2_MAX_INTENSITY_MESSAGE_ID:
      blue_2_MaxIntensity = (data * MAX_DAC) / 100;
      Serial.println("Blue 2 Max Intensity set to " + String(data));
      Serial.println("Blue 2 Max Intensity set to " + String(blue_2_MaxIntensity_float));
      Serial.println("");
      break;

    case WHITE_1_MAX_INTENSITY_MESSAGE_ID:
      white_1_MaxIntensity = (data * MAX_DAC) / 100;
      Serial.println("White 1 Max Intensity set to " + String(data));
      Serial.println("White 1 Max Intensity set to " + String(white_1_MaxIntensity_float));
      Serial.println("");
      break;

    case WHITE_2_MAX_INTENSITY_MESSAGE_ID:
      white_2_MaxIntensity = (data * MAX_DAC) / 100;
      Serial.println("White 2 Max Intensity set to " + String(data));
      Serial.println("White 2 Max Intensity set to " + String(white_2_MaxIntensity_float));
      Serial.println("");
      break;

    case MANUAL_OVERRIDE_SWITCH_MESSAGE_ID:
      manualOverrideSwitch = (data != 0);
      Serial.println("Manual Override Switch set to " + String(data));
      Serial.println("");
      break;

    case OVERRIDE_WHITE_1_INTENSITY_MESSAGE_ID:
      overrideWhite_1_Intensity = (data * MAX_DAC) / 100;
      Serial.println("Override White 1 Intensity set to " + String(data));
      Serial.println("Override White 1 Intensity set to " + String(overrideWhite_1_Intensity));
      Serial.println("");
      break;

    case OVERRIDE_WHITE_2_INTENSITY_MESSAGE_ID:
      overrideWhite_2_Intensity = (data * MAX_DAC) / 100;
      Serial.println("Override White 2 Intensity set to " + String(data));
      Serial.println("Override White 2 Intensity set to " + String(overrideWhite_2_Intensity));
      Serial.println("");
      break;

    case OVERRIDE_BLUE_1_INTENSITY_MESSAGE_ID:
      overrideBlue_1_Intensity = (data * MAX_DAC) / 100;
      Serial.println("Override Blue 1 Intensity set to " + String(data));
      Serial.println("Override Blue 1 Intensity set to " + String(overrideBlue_1_Intensity));
      Serial.println("");
      break;

    case OVERRIDE_BLUE_2_INTENSITY_MESSAGE_ID:
      overrideBlue_2_Intensity = (data * MAX_DAC) / 100;
      Serial.println("Override Blue 2 Intensity set to " + String(data));
      Serial.println("Override Blue 2 Intensity set to " + String(overrideBlue_2_Intensity));
      Serial.println("");
      break;

      //  ----------------- lightcycle times in hours and mins messages--------------------------------

    case DAWN_MINUTES_MESSAGE_ID:
      dawnMinutes = data;
      Serial.println("Dawn minutes set to " + String(data));
      break;

    case DAWN_HOURS_MESSAGE_ID:
      dawnHours = data;
      Serial.println("Dawn hours set to " + String(data));
      break;

    case DUSK_MINUTES_MESSAGE_ID:
      duskMinutes = data;
      Serial.println("Dusk minutes set to " + String(data));
      break;

    case DUSK_HOURS_MESSAGE_ID:
      duskHours = data;
      Serial.println("Dusk hours set to " + String(data));
      break;

    case SUNRISE_MINUTES_MESSAGE_ID:
      sunriseMinutes = data;
      Serial.println("Sunrise minutes set to " + String(data));
      break;

    case SUNRISE_HOURS_MESSAGE_ID:
      sunriseHours = data;
      Serial.println("Sunrise hours set to " + String(data));
      break;

    case SUNSET_MINUTES_MESSAGE_ID:
      sunsetMinutes = data;
      Serial.println("Sunset minutes set to " + String(data));
      break;

    case SUNSET_HOURS_MESSAGE_ID:
      sunsetHours = data;
      Serial.println("Sunset hours set to " + String(data));
      break;

    case HIGH_NOON_MINUTES_MESSAGE_ID:
      highNoonMinutes = data;
      Serial.println("High Noon minutes set to " + String(data));
      break;

    case HIGH_NOON_HOURS_MESSAGE_ID:
      highNoonHours = data;
      Serial.println("High Noon hours set to " + String(data));
      break;

    case NIGHT_TIME_MINUTES_MESSAGE_ID:
      nightTimeMinutes = data;
      Serial.println("Night Time minutes set to " + String(data));
      break;

    case NIGHT_TIME_HOURS_MESSAGE_ID:
      nightTimeHours = data;
      Serial.println("Night Time hours set to " + String(data));
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
      tv.tv_sec = UNIXtime;
      settimeofday(&tv, NULL);
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
      localTimeZoneOffset = data;
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
  while (true)
  {
    if (!manualOverrideSwitch)
    {
      // Send raw current intensities over CAN
      core.sendMessage(CURRENT_WHITE_1_MESSAGE_ID, currentWhite_1_Intensity, false);
      delay(updateLEDs);
      Serial.println("Sent CURRENT_WHITE_1_MESSAGE_ID with intensity " + String(CURRENT_WHITE_1_MESSAGE_ID) + " " + String(currentWhite_1_Intensity));

      core.sendMessage(CURRENT_WHITE_2_MESSAGE_ID, currentWhite_2_Intensity, false);
      delay(updateLEDs);
      Serial.println("Sent CURRENT_WHITE_2_MESSAGE_ID with intensity " + String(CURRENT_WHITE_2_MESSAGE_ID) + " " + String(currentWhite_2_Intensity));

      core.sendMessage(CURRENT_BLUE_1_MESSAGE_ID, currentBlue_1_Intensity, false);
      delay(updateLEDs);
      Serial.println("Sent CURRENT_BLUE_1_MESSAGE_ID with intensity " + String(CURRENT_BLUE_1_MESSAGE_ID) + " " + String(currentBlue_1_Intensity));

      core.sendMessage(CURRENT_BLUE_2_MESSAGE_ID, currentBlue_2_Intensity, false);
      delay(sendMqttMessageUpdateUI);
      Serial.println("Sent CURRENT_BLUE_2_MESSAGE_ID with intensity " + String(CURRENT_BLUE_2_MESSAGE_ID) + " " + String(currentBlue_2_Intensity));
    }
    else
    {
      // Apply override values to current intensities
      currentWhite_1_Intensity = overrideWhite_1_Intensity;
      currentWhite_2_Intensity = overrideWhite_2_Intensity;
      currentBlue_1_Intensity = overrideBlue_1_Intensity;
      currentBlue_2_Intensity = overrideBlue_2_Intensity;

      // White channel logic
      bool whiteTooLow = (overrideWhite_1_Intensity < minWhiteValue) || (overrideWhite_2_Intensity < minWhiteValue);
      digitalWrite(WHITE_RELAY, whiteTooLow ? RELAY_OFF : RELAY_ON);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, whiteTooLow ? DAC_OFF : overrideWhite_1_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, whiteTooLow ? DAC_OFF : MAX_DAC - overrideWhite_1_Intensity);
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, whiteTooLow ? DAC_OFF : MAX_DAC - overrideWhite_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(WHITE_CHANNEL_1, whiteTooLow ? 0 : overrideWhite_1_Intensity);
      setPWM(WHITE_CHANNEL_2, whiteTooLow ? 0 : overrideWhite_2_Intensity);
#endif

      // Blue channel logic
      bool blueTooLow = (overrideBlue_1_Intensity < minBlueValue) || (overrideBlue_2_Intensity < minBlueValue);
      digitalWrite(BLUE_RELAY, blueTooLow ? RELAY_OFF : RELAY_ON);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, blueTooLow ? DAC_OFF : overrideBlue_1_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, blueTooLow ? DAC_OFF : MAX_DAC - overrideBlue_1_Intensity);
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, blueTooLow ? DAC_OFF : MAX_DAC - overrideBlue_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, blueTooLow ? 0 : overrideBlue_1_Intensity);
      setPWM(BLUE_CHANNEL_2, blueTooLow ? 0 : overrideBlue_2_Intensity);
#endif

      delay(sendMqttMessageUpdateUI);
    }

#ifdef debuging
    Serial.println("Manual LED control override switch is = " + String(manualOverrideSwitch));
    Serial.println("Blue relay = " + String(digitalRead(BLUE_RELAY)));
    Serial.println("White relay = " + String(digitalRead(WHITE_RELAY)));
    Serial.println("overrideWhite_1_Intensity = " + String(overrideWhite_1_Intensity));
    Serial.println("overrideWhite_2_Intensity = " + String(overrideWhite_2_Intensity));
    Serial.println("overrideBlue_1_Intensity = " + String(overrideBlue_1_Intensity));
    Serial.println("overrideBlue_2_Intensity = " + String(overrideBlue_2_Intensity));
    Serial.println("currentWhite_1_Intensity = " + String(currentWhite_1_Intensity));
    Serial.println("currentWhite_2_Intensity = " + String(currentWhite_2_Intensity));
    Serial.println("currentBlue_1_Intensity = " + String(currentBlue_1_Intensity));
    Serial.println("currentBlue_2_Intensity = " + String(currentBlue_2_Intensity));
    Serial.println("");
#endif
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

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, DAC_OFF);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, DAC_OFF);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(WHITE_CHANNEL_1, 0);
#endif
    }
    else
    {
      digitalWrite(WHITE_RELAY, RELAY_OFF);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_1_DAC_DFRobot, overrideWhite_1_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(WHITE_1_DAC, MAX_DAC - overrideWhite_1_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(WHITE_CHANNEL_1, overrideWhite_1_Intensity);
#endif
    }

    if (overrideWhite_2_Intensity < minWhiteValue)
    {
      digitalWrite(WHITE_RELAY, RELAY_ON);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_2_DAC_DFRobot, DAC_OFF);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, DAC_OFF);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(WHITE_CHANNEL_2, 0);
#endif
    }
    else
    {
      digitalWrite(WHITE_RELAY, RELAY_OFF);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(WHITE_2_DAC_DFRobot, overrideWhite_2_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(WHITE_2_DAC, MAX_DAC - overrideWhite_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(WHITE_CHANNEL_2, overrideWhite_2_Intensity);
#endif
    }

    if (overrideBlue_1_Intensity < minBlueValue)
    {
      digitalWrite(BLUE_RELAY, RELAY_OFF);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, DAC_OFF);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, DAC_OFF);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, 0);
#endif
    }
    else
    {
      digitalWrite(BLUE_RELAY, RELAY_ON);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_1_DAC_DFRobot, overrideBlue_1_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_1_DAC, MAX_DAC - overrideBlue_1_Intensity);
#endif
#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_1, overrideBlue_1_Intensity);
#endif
    }

    if (overrideBlue_2_Intensity < minBlueValue)
    {
      digitalWrite(BLUE_RELAY, RELAY_OFF);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_2_DAC_DFRobot, DAC_OFF);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, DAC_OFF);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_2, 0);
#endif
    }
    else
    {
      digitalWrite(BLUE_RELAY, RELAY_ON);
      delay(updateLEDs);

#ifdef usingDFRobot_GP8403
      DFRobot_GP8403_1.setDACOutVoltage(BLUE_2_DAC_DFRobot, overrideBlue_2_Intensity);
#endif

#ifdef usingAdafruit_MCP4728
      MCP12bitDAC.setChannelValue(BLUE_2_DAC, MAX_DAC - overrideBlue_2_Intensity);
#endif

#ifdef usingPWM_to_0to10V
      setPWM(BLUE_CHANNEL_2, overrideBlue_2_Intensity);
#endif
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
  delay(MessageGap);

  Serial.println("Running updateTimes function\n");

  getLocalTime(&timeinfo); // Get the current time from the ESP32 RTC
  strftime(localTime, sizeof(localTime), "%c", &timeinfo);
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
  nightTimeStart = (nightTimeHours * 3600) + (nightTimeMinutes * 60);

  Serial.println("Times updated in updateTimes function\n");

#ifdef debuging
  delay(5000);
  Serial.println("Running updateTimes function");
  Serial.println("UNIXtime = " + String(UNIXtime));
  Serial.println("localTimeZone = " + String(localTimeZone));
  Serial.println("localTime = " + String(localTime));
  Serial.println("24 hour time = " + String(timeinfo.tm_year) + "," + String(timeinfo.tm_mon) + " " + String(timeinfo.tm_wday) + " " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));
  Serial.println("curTimeSec in seconds  = " + String(curTimeSec));
  Serial.println("dawnHours = " + String(dawnHours));
  Serial.println("dawnMinutes = " + String(dawnMinutes));
  Serial.println("dawnDurationSec = " + String(dawnDurationSec));
  Serial.println("sunriseHours = " + String(sunriseHours));
  Serial.println("sunriseMinutes = " + String(sunriseMinutes));
  Serial.println("sunriseDurationSec = " + String(sunriseDurationSec));
  Serial.println("highNoonHours = " + String(highNoonHours));
  Serial.println("highNoonMinutes = " + String(highNoonMinutes));
  Serial.println("highNoonDurationSec = " + String(highNoonDurationSec));
  Serial.println("sunsetHours = " + String(sunsetHours));
  Serial.println("sunsetMinutes = " + String(sunsetMinutes));
  Serial.println("sunsetDurationSec = " + String(sunsetDurationSec));
  Serial.println("duskHours = " + String(duskHours));
  Serial.println("duskMinutes = " + String(duskMinutes));
  Serial.println("duskDurationSec = " + String(duskDurationSec));
  Serial.println("nightTimeHours = " + String(nightTimeHours));
  Serial.println("nightTimeMinutes = " + String(nightTimeMinutes));
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