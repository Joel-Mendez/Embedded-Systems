#include "NU32.h"          // constants, functions for startup and UART
#include "positioncontrol.h"
#include <stdio.h>

volatile float POSITION_GAINS[3] = {0,0,0};


void pos_control_init(void){
  PR3 = 49999;            //Period
  TMR3 = 0;               //Initial Count
  T3CONbits.TCKPS = 3;    //Setting prescaler to 8
  IPC3bits.T3IP = 5;      //Priority
  IPC3bits.T3IS = 0;      //Sub-Priority
  IFS0bits.T3IF = 0;      //flag
  IEC0bits.T3IE = 1;
  T3CONbits.ON = 1;       //Turning on Timer 3
  }

void set_position_gains(float Kp, float Ki, float Kd){
  POSITION_GAINS[0] = Kp;
  POSITION_GAINS[1] = Ki;
  POSITION_GAINS[2] = Kd;
}

float * get_position_gains(void){
  return POSITION_GAINS;
}
