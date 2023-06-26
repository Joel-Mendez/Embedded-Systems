#ifndef PWM__H__
#define PWM__H__

#define TICKS_PER_SEC 40000000

void pwm_init(void);
void digital_output_init(void);
void interrupt_init(void);

int saturate_PWM(int PWM_input);
void set_PWM(int PWM_input);
int get_PWM(void);

void set_direction_bit(int PWM_input);
int get_direction_bit(void);
void invert_direction_bit(void);
#endif
