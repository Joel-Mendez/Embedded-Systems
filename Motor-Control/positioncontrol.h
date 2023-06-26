#ifndef POSITIONCONTROL__H__
#define POSITIONCONTROL__H__
#define TICKS_PER_SEC 40000000

void set_position_gains(float Kp, float Ki, float Kd);
float * get_position_gains(void);
void pos_control_init(void);

#endif
