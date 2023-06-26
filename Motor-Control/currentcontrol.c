#include "NU32.h"          // constants, functions for startup and UART
#include "currentcontrol.h"
#include <stdio.h>

volatile float CURRENT_GAINS[2] = {0,0};

void set_current_gains(float Kp, float Ki){
  CURRENT_GAINS[0] = Kp;
  CURRENT_GAINS[1] = Ki;
}

float * get_current_gains(void){
  return CURRENT_GAINS;
}
