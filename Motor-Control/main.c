#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include <stdio.h>
#include <xc.h>
#include "utilities.h" //Misc. Functions
#include "encoder.h" // Encoder Functions
#include "isense.h" // Current Sensor Functions
#include "currentcontrol.h" // Current Control Functions
#include "positioncontrol.h" // Position Control Functions
#include "pwm.h"
#include "filter.h"
#include "trajectory.h"

#define BUF_SIZE 200
#define CT  40000000

float ACTUAL_CURRENT[100];
float DESIRED_CURRENT[100];
float POS_D=0;
float POS=0;
float DESIRED_POSITION[2000];
float ACTUAL_POSITION[2000];
float REF_CURRENT=0;
float e_prev = 0;
float ei_prev = 0;
float e_p_prev = 0;
float ei_p_prev = 0;
float ei_max = 10;
float Ud_prev = 0;
int FILTER_ON = 1;
static int i = 0;
static int N = 1;

void __ISR(_CORE_TIMER_VECTOR, IPL6SRS) CoreTimerISR(void) {
  IFS0bits.CTIF = 0;
  switch (get_mode()) {
    case 0: //IDLE Mode
    {
      reset_current_filter();
      reset_position_filter();
      i = 0;
      N = 1;
      e_prev=0;
      ei_prev=0;
      e_p_prev = 0;
      ei_p_prev = 0;
      Ud_prev = 0;
      LATCbits.LATC13=0;
      OC1RS=0;
      break;
    }
    case 1: //PWM Mode
    {
      LATCbits.LATC13=get_direction_bit();
      OC1RS=.01*get_PWM()*(PR2+1);
      break;
    }
    case 2: //ITEST Mode
    {
      float dt = .0002 ;
      float * gains = get_current_gains();
      float Kp = gains[0];
      float Ki = gains[1];
      float e;
      float ei;
      float P;
      float I;

      if( i>=100){
        //Setting MODE back to IDLE MODE
        set_mode(0);

        //reset_current_control();
        i = 0;
        e_prev = 0;
        ei_prev = 0;
        //Letting MATLAB know that ITEST is done
        char buffer[BUF_SIZE];
        sprintf(buffer,"%d\r\n", 1);
        NU32_WriteUART3(buffer);

        //Sending MATLAB number of samples for read_plot_matrix
        sprintf(buffer,"%d\r\n", 100);
        NU32_WriteUART3(buffer);

        //Sending saved arrays
        int n = 0;
        for(n=0;n<100;n++){
          sprintf(buffer,"%f %f\r\n", DESIRED_CURRENT[n], ACTUAL_CURRENT[n]); //get_array_element()
          NU32_WriteUART3(buffer);
        }
      }
      else{

        //Reference Current
        if(i<25){
          DESIRED_CURRENT[i]= 200;
        }
        else if(i<50){
          DESIRED_CURRENT[i]= -200;
        }
        else if(i<75){
          DESIRED_CURRENT[i]= 200;
        }
        else{
          DESIRED_CURRENT[i]= -200;
        }

        //Actual Current
        if (FILTER_ON==1){
          ACTUAL_CURRENT[i]= current_filter(adc_sample_convert_current(15));
        }
        else{
          ACTUAL_CURRENT[i]= adc_sample_convert_current(15);
        }

        //PI Control
        ///error
        e = DESIRED_CURRENT[i]-ACTUAL_CURRENT[i];
        if(e<25){
          e=e*2.5;
        }
        ///P Control
        P = Kp*e;
        ///I Control
        ei = (ei_prev + e)*dt;
        if (ei <= -ei_max){
          ei= -ei_max;
        }
        else if (ei >= ei_max){
          ei= ei_max;
        }
        I = -Ki*ei;

        ///Sending PWM_Effort
        float U = P+I;
        int Effort = (int)U;
        set_direction_bit(Effort);
        set_PWM(Effort);
        LATCbits.LATC13=get_direction_bit();
        OC1RS=.01*get_PWM()*(PR2+1);
      }
      ei_prev = ei;
      e_prev = e;
      i = i+1;
      break;
    }
    case 3: //HOLD Mode
    {
      float dt = .0002 ;
      float * gains = get_current_gains();
      float Kp = gains[0];
      float Ki = gains[1];
      float e;
      float ei;
      float P;
      float I;

      //PI Control
      ////error
      if(FILTER_ON==1){
        e = REF_CURRENT-current_filter(adc_sample_convert_current(15));
      }
      else{
        e = REF_CURRENT-adc_sample_convert_current(15);
      }
      ////P Control
      P = Kp*e;
      ///I Control
      ei = (ei_prev + e)*dt;
      if (ei <= -ei_max){
        ei= -ei_max;
      }
      else if (ei >= ei_max){
        ei= ei_max;
      }
      I = Ki*ei;

      ///Sending PWM_Effort
      float U = P+I;
      int Effort = (int)U;
      set_direction_bit(Effort);
      set_PWM(Effort);
      LATCbits.LATC13=get_direction_bit();
      OC1RS=.01*get_PWM()*(PR2+1);

      ei_prev = ei;
      e_prev = e;
      break;
    }
    case 4: //TRACK Mode
    {
      float dt = .0002 ;
      float * gains = get_current_gains();
      float Kp = gains[0];
      float Ki = gains[1];
      float e;
      float ei;
      float P;
      float I;

      //PI Control
      ////error
      if(FILTER_ON==1){
        e = REF_CURRENT-current_filter(adc_sample_convert_current(15));
      }
      else{
        e = REF_CURRENT-adc_sample_convert_current(15);
      }
      ////P Control
      P = Kp*e;
      ///I Control
      ei = (ei_prev + e)*dt;
      if (ei <= -ei_max){
        ei= -ei_max;
      }
      else if (ei >= ei_max){
        ei= ei_max;
      }
      I = Ki*ei;

      ///Sending PWM_Effort
      float U = P+I;
      int Effort = (int)U;
      set_direction_bit(Effort);
      set_PWM(Effort);
      LATCbits.LATC13=get_direction_bit();
      OC1RS=.01*get_PWM()*(PR2+1);

      ei_prev = ei;
      e_prev = e;
      break;
    }
  }
  _CP0_SET_COUNT(0);                // set core timer counter to 0
  _CP0_SET_COMPARE(TICKS_PER_SEC*.0002);     // must set CP0_COMPARE again after interrupt
}

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Timer3ISR(void) {
  if (get_mode()==3){
    float dt = .005;
    float * Pos_Gains = get_position_gains();
    float Kp = Pos_Gains[0];
    float Ki = Pos_Gains[1];
    float Kd = Pos_Gains[2];
    float e, ei, Up, Ui, Ud;

    //POS = position_filter(encoder_degrees());
    POS = encoder_degrees();
    e = POS_D - POS;

    //PID control
    ////P Term
    Up = Kp*e;
    ////I Term
    ei = (ei_p_prev + e)*dt;
    if (ei <= -ei_max){
      ei= -ei_max;
    }
    else if (ei >= ei_max){
      ei= ei_max;
    }
    Ui = Ki*ei;
    ////D Term
    Ud = Kd*(50)*(e-e_p_prev)+Ud_prev*(1-(50)*dt);
    ////Setting Current Effort
    REF_CURRENT = Up+Ui+Ud;
    ei_p_prev = ei;
    e_p_prev = e;
    Ud_prev = Ud;
  }

  if (get_mode()==4){
    float dt = .005;
    float * Pos_Gains = get_position_gains();
    float Kp = Pos_Gains[0];
    float Ki = Pos_Gains[1];
    float Kd = Pos_Gains[2];
    float e, ei, Up, Ui, Ud;
    int Samples = get_NSamples();
    if( i>=Samples){
      POS_D=traj_element(Samples);
      set_mode(3);

      i = 0;
      e_prev = 0;
      ei_prev = 0;
      //Letting MATLAB know that Trajectory is done
      char buffer[BUF_SIZE];
      sprintf(buffer,"%d\r\n", 1);
      NU32_WriteUART3(buffer);

      //Sending MATLAB number of samples for read_plot_matrix
      sprintf(buffer,"%d\r\n", Samples);
      NU32_WriteUART3(buffer);

      //Sending saved arrays
      int n = 0;
      for(n=0;n<Samples;n++){
        sprintf(buffer,"%f %f\r\n", traj_element(n), ACTUAL_POSITION[n]);
        NU32_WriteUART3(buffer);
      }
    }
    else{
      if(FILTER_ON==1){
        ACTUAL_POSITION[i] = position_filter(encoder_degrees());
      }
      else{
        ACTUAL_POSITION[i] = encoder_degrees();
      }
      e = traj_element(i) - ACTUAL_POSITION[i];

      //PID control
      ////P Term
      Up = Kp*e;
      ////I Term
      ei = (ei_p_prev + e)*dt;
      if (ei <= -ei_max){
        ei= -ei_max;
      }
      else if (ei >= ei_max){
        ei= ei_max;
      }
      Ui = Ki*ei;
      ////D Term
      Ud = Kd*(50)*(e-e_p_prev)+Ud_prev*(1-(50)*dt);
      ////Setting Current Effort
      REF_CURRENT = Up+Ui+Ud;
    }

    ei_p_prev = ei;
    e_p_prev = e;
    Ud_prev = Ud;
    i=i+1;
  }
  IFS0bits.T3IF = 0;              // clear interrupt flag
}


int main()
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;

  __builtin_disable_interrupts();
  set_mode(0);
  encoder_init();
  adc_init();
  interrupt_init();
  pwm_init();
  digital_output_init();
  pos_control_init();
  __builtin_enable_interrupts();

  _CP0_SET_COUNT(0);
  _CP0_SET_COMPARE(TICKS_PER_SEC*.0002);

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      //Defining Cases/////////////////////////////////////////////////////////
      case 'a': //Read curent sensor (ADC counts)
      {
        sprintf(buffer,"%d\r\n", adc_sample_convert(15));
        NU32_WriteUART3(buffer);
        break;
      }
      case 'b': //Read curent sensor (mA)
      {
        sprintf(buffer,"%f\r\n", adc_sample_convert_current(15));
        NU32_WriteUART3(buffer);
        break;
      }
      case 'c': //Read encoder (counts)
      {
        sprintf(buffer,"%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'd': //Read encoder (deg)
      {
        sprintf(buffer,"%f\r\n", encoder_degrees());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'e': //Reset Encoder
      {
        encoder_reset();
        break;
      }
      case 'f': //Set PWM (-100 to 100)
      {
        //Getting user input from MATLAB
        int PWM_input = 0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d\r\n", &PWM_input);

        //Setting PWM and Direction Variables according to input
        set_direction_bit(PWM_input);
        set_PWM(PWM_input);

        //Writing to MATLAB
        sprintf(buffer,"%d\r\n", get_PWM());
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%d\r\n", get_direction_bit());
        NU32_WriteUART3(buffer);

        //Switching to PWM Mode
        set_mode(1);

        break;
      }
      case 'g': //Set current gains
      {

        float Kp = 0;
        float Ki = 0;

        //Reading from MATLAB
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f \r\n", &Kp);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f \r\n", &Ki);

        //Setting Current Gains
        set_current_gains(Kp,Ki);

        break;
      }
      case 'h': //Get current gains
      {
        float * gains;
        gains=get_current_gains();

        //Sending gains to MATLAB
        sprintf(buffer,"%f\r\n", gains[0]);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", gains[1]);
        NU32_WriteUART3(buffer);

        break;
      }
      case 'i': //Set position gains
      {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;

        //Reading from MATLAB
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f \r\n", &Kp);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f \r\n", &Ki);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f \r\n", &Kd);

        //Setting Position Gains
        set_position_gains(Kp,Ki,Kd);

        break;
      }
      case 'j': //Get position gains
      {
        float * gains;
        gains=get_position_gains();

        //Sending gains to MATLAB
        sprintf(buffer,"%f\r\n", gains[0]);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", gains[1]);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", gains[2]);
        NU32_WriteUART3(buffer);

        break;
      }
      case 'k': //Test current control
      {
        set_mode(2);
        break;
      }
      case 'l': //Go to angle (deg)
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d\r\n", &DESIRED_POSITION);
        set_mode(3);
        break;
      }
      case 'm': //Load step trajectory
      {
        int N_Samples = 0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d \r\n", &N_Samples);
        set_NSamples(N_Samples);
        create_trajectory();
        break;
      }
      case 'n': //Load cubic trajectory
      {
        int N_Samples = 0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d \r\n", &N_Samples);
        set_NSamples(N_Samples);
        create_trajectory();
        break;
      }
      case 'o': //Execute trajectory
      {
        set_mode(4);
        break;
      }
      case 'p': //Unpower the motor
      {
        set_mode(0);
        break;
      }
      case 'q': //Quit client
      {
        set_mode(0);
        break;
      }
      case 'r': //Get mode
      {
        sprintf(buffer,"%d\r\n", get_mode());
        NU32_WriteUART3(buffer); // send encoder count to client
        break;
      }
      case 'z':
      {
        FILTER_ON = -1*FILTER_ON;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}
