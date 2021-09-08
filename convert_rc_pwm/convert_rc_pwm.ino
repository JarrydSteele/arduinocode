#include "makros.h"
#include "debug.h"
#include "RCReceive.h"
#include <DigisparkRGB.h>

const byte PIN_RC = A0;
const byte PIN_FC_1 = 0;
const byte PIN_FC_2 = 1;

RCReceive rcReceiver;

void setup() {
  rcReceiver.attach(PIN_RC);

  // put your setup code here, to run once:
  
  Serial.begin(9600);
  pinMode(PIN_FC_1, OUTPUT);
  pinMode(PIN_FC_2, OUTPUT);
  digitalWrite(PIN_FC_1, LOW);
  digitalWrite(PIN_FC_2, HIGH);
}

void loop() {
  rcReceiver.poll();

  if (rcReceiver.hasNP() && !rcReceiver.hasError()) {
    mainloop();
  } else if (rcReceiver.hasError()) {
  }
}

void mainloop() {
  byte value = rcReceiver.getValue();
  
  // put your main code here, to run repeatedly:

  // Serial.println(String(value));
  if (value >= 150)
  {
    Serial.println("System: 2");
    digitalWrite(PIN_FC_2, LOW);
    digitalWrite(PIN_FC_1, HIGH);
  }
  else
  {
    Serial.println("System: 1");
    digitalWrite(PIN_FC_1, LOW);
    digitalWrite(PIN_FC_2, HIGH);
  }

}
