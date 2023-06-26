#include "utilities.h"
#include <xc.h>

static enum mode_t{IDLE,PWM,ITEST,HOLD,TRACK};
enum mode_t PIC32_MODE;

int get_mode(void){
  return PIC32_MODE;
}

void set_mode(int i){
  PIC32_MODE = i;
}
