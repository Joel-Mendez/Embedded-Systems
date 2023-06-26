Motor control via a PIC32 Microcontroller

- main.c - main control loop
- NU32.c - startup and UART 
- isense.c - reading and interpreting current from ADC
- encoder.c - reading and interpreting position from encoder
- currentcontrol.c - handles parameters and PID gains of current control loop
- positioncontrol.c - handles parameters and PID gains of position control loop
- pwm.c - handles PWM
- filter.c - handles filtering of current and position
- trajectory.c - handles trajectory generation
- utilities.c - misc. functions
