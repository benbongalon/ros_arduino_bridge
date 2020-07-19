/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

#ifdef PARALLAX_HB25
  #include <Servo.h>
  #define LEFT_MOTOR_PWM  8
  #define RIGHT_MOTOR_PWM 9

  // Pulse width (1 ms = 1000 microseconds) will control the HB-25 as follows:
  //      1.0 ms Full Reverse
  //      1.5 ms Neutral (STOP)
  //      2.0 ms Full Forward
  const unsigned int NeutralPulseWidth = 1500,
                     FullReversePulseWidth = 1000,
                     FullForwardPulseWidth = 2000;
  extern Servo leftServo, rightServo;
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
