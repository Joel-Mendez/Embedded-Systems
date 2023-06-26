#include "encoder.h"
#include <xc.h>

void check_value_int(int i){
  char msg1[200];
  sprintf(msg1,"%d \r\n",i); 
  NU32_WriteUART3(msg1);

}
