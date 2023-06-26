#ifndef CURRENTCONTROL__H__
#define CURRENTCONTROL__H__
#define TICKS_PER_SEC 40000000

void set_current_gains(float Kp, float Ki);
float * get_current_gains(void);
void ITEST (void);

#endif
