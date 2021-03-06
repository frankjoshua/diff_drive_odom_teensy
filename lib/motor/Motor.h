#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
public:
  Motor(int motorNumber);
  void adjust(double delta);

private:
  int power = 0;
  String motorCommand;
};

#endif