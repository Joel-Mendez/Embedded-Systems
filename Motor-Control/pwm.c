#include "NU32.h"          // constants, functions for startup and UART
#include "currentcontrol.h"
#include <stdio.h>



volatile int MOTOR_PWM = 0;
volatile int DIRECTION_BIT = 1;
char buffer[200];


int saturate_PWM(int PWM_input){
  if (PWM_input < -100){
    return -100;
  }
  else if (PWM_input > 100){
    return 100;
  }
  else{
    return PWM_input;
  }
}
void set_PWM(int PWM_input){
  PWM_input = saturate_PWM(PWM_input);
  if (PWM_input >= 0){
    MOTOR_PWM=PWM_input;
  }
  else{
    MOTOR_PWM= -1* PWM_input;
  }

}
int get_PWM(void){
  return MOTOR_PWM;
};
void set_direction_bit(PWM_input){
  if (PWM_input>=0){
    DIRECTION_BIT = 1;
  }
  else {
    DIRECTION_BIT = 0;
  }
}
int get_direction_bit(void){
  return DIRECTION_BIT;
}
void invert_direction_bit(void){
  if (DIRECTION_BIT == 1){
    DIRECTION_BIT = 0;
  }
  else{
    DIRECTION_BIT = 1;
  }
}
void interrupt_init(void){
  _CP0_SET_COMPARE(40000000*.0002);
  IPC0bits.CTIP = 6;
  IPC0bits.CTIS = 0;
  IFS0bits.CTIF = 0;
  IEC0bits.CTIE = 1;
}
void pwm_init(void){
  T2CONbits.TCKPS = 0;     // N = 1
  PR2 = 3999;
  TMR2 = 0;
  OC1CONbits.OCM = 0b110;
  OC1RS = .01*MOTOR_PWM*(PR2+1);             // duty cycle = OC1RS/(PR2+1) = 25%
  OC1R = .01*MOTOR_PWM*(PR2+1);
  T2CONbits.ON = 1;        // turn on Timer2
  OC1CONbits.ON = 1;       // turn on OC1
}
void digital_output_init(void){
  TRISCbits.TRISC13=0; // Using RC13 as my digital output
  LATCbits.LATC13=0;
}
