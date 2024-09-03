#include <Arduino.h>
#include "makros.h"
#include "debug.h"
#include "RCReceive.h"

const byte PIN_RC = A5;
const byte PIN_FC_1 = 2;
const byte PIN_FC_2 = 3;

const bool ENABLE_SERIAL = false;

RCReceive rcReceiver;

// Initialise setup variables
void setup() 
{
  // Attach the RC input pin
  rcReceiver.attach(PIN_RC);

  // Initialise the two output pins to drive the inputs of The Switch, and set their initial states
  pinMode(PIN_FC_1, OUTPUT);
  pinMode(PIN_FC_2, OUTPUT);
  digitalWrite(PIN_FC_1, LOW);
  digitalWrite(PIN_FC_2, HIGH);

  // Optionally begin serial communications
  if (ENABLE_SERIAL)
    Serial.begin(9600);
}

// Get the value of the RC Pin, and set the two outputs appropriately
void mainloop() 
{
  // Get the value of the RC Pin. Using the getValue method automatically applies a 10 value rolling average in order to mitigate any transmission errors.
  byte value = rcReceiver.getValue();
  
  // This will be a value in the range 0-255
  if (value < 150)
  {
    // Activate Primary Flight Controller
    digitalWrite(PIN_FC_1, LOW);
    digitalWrite(PIN_FC_2, HIGH);
    
    // Optionally output status
    if (ENABLE_SERIAL)
      Serial.println("System: 1");
  }
  else
  {
    // Activate Secondary Flight Controller
    digitalWrite(PIN_FC_1, HIGH);
    digitalWrite(PIN_FC_2, LOW);

    // Optionally output status
    if (ENABLE_SERIAL)
      Serial.println("System: 2");
  }
}

void loop() 
{
  // Read current RC value 
  rcReceiver.poll();

  // Determination of zero point? 
  if (rcReceiver.hasNP() && !rcReceiver.hasError()) 
  {
    // Run the main loop
    mainloop();
  } else if (rcReceiver.hasError()) 
  {
    // This is where we would do some error handling, but YOLO.
  }
}