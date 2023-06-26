#ifndef ISENSE__H__
#define ISENSE__H__

void adc_init(void);
unsigned int adc_sample_convert(int pin); //reads current in counts
float adc_sample_convert_current(int pin); // reads current in mA


#endif
