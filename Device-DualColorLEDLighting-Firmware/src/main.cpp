#include <Arduino.h>
#include <NodeControllerCore.h>

#define NODE_ID 0xA1            // 161
#define WHITE_MESSAGE_ID 0x0A00 // 2560
#define BLUE_MESSAGE_ID 0x0A01  // 2561

// LED pins
#define WHITE_PWM_PIN 3
#define BLUE_PWM_PIN 2
// RELAY pins
#define BLUE_RELAY 4
#define WHITE_RELAY 5

// Node controller core object
NodeControllerCore core;

// put function declarations here:
// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);

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
}

void loop()
{
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