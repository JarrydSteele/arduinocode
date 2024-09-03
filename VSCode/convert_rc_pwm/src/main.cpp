#pragma region includes

#include <Arduino.h>
#include "makros.h"
#include "debug.h"
#include "RCReceive.h"

#pragma endregion includes
#pragma region constants

const byte PIN_RC = A5;
const byte PIN_FC_1 = 2;
const byte PIN_FC_2 = 3;

const bool ENABLE_SERIAL = false;

#pragma endregion constants
#pragma region globalvariables

RCReceive rcReceiver;

#pragma endregion globalvariables
#pragma region helpermethods

// Activate Primary Flight Controller
void activatePrimary()
{
  // Set the appropriate outputs
  digitalWrite(PIN_FC_1, LOW);
  digitalWrite(PIN_FC_2, HIGH);
    
  // Optionally output status
  if (ENABLE_SERIAL)
    Serial.println("System: 1");
}

// Activate Secondary Flight Controller
void activateSecondary()
{
  // Set the appropriate outputs
  digitalWrite(PIN_FC_1, HIGH);
  digitalWrite(PIN_FC_2, LOW);
    
  // Optionally output status
  if (ENABLE_SERIAL)
    Serial.println("System: 2");
}

// Get the value of the RC Pin, and set the two outputs appropriately
void process()
{
  // Get the value of the RC Pin. Using the getValue method automatically applies a 10 value rolling average in order to mitigate any transmission errors.
  byte value = rcReceiver.getValue();
  
  // This will be a value in the range 0-255
  if (value < 150)
    activatePrimary();
  else
    activateSecondary();
}

#pragma endregion helpermethods

// Initialise setup variables
void setup()
{
  // Optionally begin serial communications
  if (ENABLE_SERIAL)
    Serial.begin(9600);

  // Attach the RC input pin
  rcReceiver.attach(PIN_RC);

  // Initialise the two output pins to drive the inputs of The Switch, and set their initial states
  pinMode(PIN_FC_1, OUTPUT);
  pinMode(PIN_FC_2, OUTPUT);
  activatePrimary();
}

void loop()
{
  // Read current RC value 
  rcReceiver.poll();

  // Determination of zero point? 
  if (rcReceiver.hasNP() && !rcReceiver.hasError()) 
  {
    // Read the RC input and update The Switch outputs
    process();
  } else if (rcReceiver.hasError()) 
  {
    // This is where we would do some error handling, but YOLO.
  }
}