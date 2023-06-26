#include "NU32.h"          // constants, functions for startup and UART
#include "trajectory.h"
#include <stdio.h>


volatile int N;
volatile float trajectory[1000];

void set_NSamples(int n){
  N=n;
}

int get_NSamples(void){
  return N;
}

void create_trajectory(void){
  int j = 0;
  float point = 0;
  char buffer[200];
  for(j=0;j<N;j++){
    NU32_ReadUART3(buffer,200);
    sscanf(buffer, "%f \r\n",&point);
    trajectory[j]=point;
  }
}

float traj_element(int n){
  return trajectory[n];
}
