/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(PARALLAX_HB25)
  #define UNDEFINED  -1
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  volatile int8_t L_read = UNDEFINED,
                  L_read_prev = UNDEFINED,
                  R_read = UNDEFINED,
                  R_read_prev = UNDEFINED;
                  
  ISR(PCINT0_vect){
    // Gets called when any of the encoder pins toggle
    
    static const int8_t ENC_STATES[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};  // X4-encoding lookup table
    int dir;
    
    L_read = 2*digitalRead(LEFT_ENC_PIN_A) + digitalRead(LEFT_ENC_PIN_B);
    R_read = 2*digitalRead(RIGHT_ENC_PIN_A) + digitalRead(RIGHT_ENC_PIN_B);
  
    // Convert quadrature encoder values to tick counts.
    // Swap the A and B connectors if the direction value is opposite of what you desire.
    // https://www.scribd.com/document/235161074/How-to-Use-a-Quadrature-Encoder-Let-s-Make-Robots
    if (L_read_prev != UNDEFINED) {
      dir = ENC_STATES[4*L_read_prev + L_read];
      left_enc_pos += dir;
    }
    L_read_prev = L_read;
    if (R_read_prev != UNDEFINED) {
      dir = ENC_STATES[4*R_read_prev + R_read];
      right_enc_pos += dir;
    }
    R_read_prev = R_read;    
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

