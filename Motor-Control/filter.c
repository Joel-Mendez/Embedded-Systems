#include "filter.h"
#include <xc.h>
#include "NU32.h"

int ORDER =2;
//int b[3] = {.33,.33,.33};
volatile int current_count = 0;
volatile int position_count = 0;
volatile float current[5]={0,0,0,0,0};
volatile float position[5]={0,0,0,0,0};

float current_filter(float x){
  float z;

  current_count++;
  current[4] = current[3];
  current[3] = current[2];
  current[2] = current[1];
  current[1] = current[0];
  current[0] = x;

  if (current_count == 1){
    z = 1*current[0];
  }
  else if (current_count = 2){
    z = .6*current[0] + .4*current[1];
  }
  else if (current_count = 3){
    z = .5*current[0] + .3*current[1] + .2*current[2];
  }
  else if (current_count = 4){
    z = .4*current[0] + .3*current[1] + .2*current[2] +.1*current[3];
  }
  else{
    z = .3*current[0] + .3*current[1] + .2*current[2] +.1*current[3] +.1*current[4];
  }
  return z;
}

float position_filter(float x){
  float z;

  position_count++;
  position[4] = position[3];
  position[3] = position[2];
  position[2] = position[1];
  position[1] = position[0];
  position[0] = x;

  if (position_count == 1){
    z = 1*position[0];
  }
  else if (position_count = 2){
    z = .6*position[0] + .4*position[1];
  }
  else if (position_count = 3){
    z = .5*position[0] + .3*position[1] + .2*position[2];
  }
  else if (current_count = 4){
    z = .4*position[0] + .3*position[1] + .2*position[2] +.1*position[3];
  }
  else{
    z = .3*position[0] + .3*position[1] + .2*position[2] +.2*position[3] + .1*position[4];
  }

  return z;
}


void reset_current_filter(void){
  current_count = 0;
  current[0]=0;
  current[1]=0;
  current[2]=0;
  current[3]=0;
  current[4]=0;
}

void reset_position_filter(void){
  position_count = 0;
  position[0]=0;
  position[1]=0;
  position[2]=0;
  position[3]=0;
  position[4]=0;
}
