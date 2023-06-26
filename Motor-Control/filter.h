#ifndef FILTER__H__
#define FILTER__H__

void reset_current_filter(void);
void reset_position_filter(void);
float current_filter(float x);
float position_filter(float x);

#endif
