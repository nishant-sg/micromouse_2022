#include "allFunctions.h"
#include <Arduino.h>

void powerOnLED(byte pin)
{
  digitalWrite(pin, HIGH);
  return;
}

void poweroffLED(byte pin)
{
  digitalWrite(pin, LOW);
  return;
}

void setupLED(byte pin)
{
  pinMode(pin, OUTPUT);
  return;
}