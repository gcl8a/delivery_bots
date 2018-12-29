/*
 * For managing quadrature encoders
 */

#ifndef __ENCODER_H
#define __ENCODER_H

//motor:channel
const uint8_t ENCODER_1A = A2;
const uint8_t ENCODER_1B = A3;
const uint8_t ENCODER_2A = A0;
const uint8_t ENCODER_2B = A1;

class Encoder
{
  volatile int16_t currTicks = 0; //the current encoder value
  volatile int16_t snapTicks = 0; //frozen in an interrupt for speed calculations later
  volatile int16_t prevTicks = 0; //previous value for calculating speed

  const uint8_t pinIntA;
  const uint8_t pinIntB;  

public:
  Encoder(int pinA, int pinB) 
  : pinIntA(pinA), pinIntB(pinB)
  {}

  void ProcessInterrupt(uint8_t pinInt) //volatile
  {
    int8_t del = -1;           //default for pin A
    if (pinInt == pinIntB) del = 1; //if it's pin B, reverse everything

    if (digitalRead(pinIntA) == digitalRead(pinIntB)) 
    { 
      del *= -1; //turning the other way
    }

    currTicks += del;
  }

  int16_t TakeSnapshot(void) //volatile
  {
    snapTicks = currTicks;
    return snapTicks;
  }

  int16_t CalcDelta(void) //volatile
  {
    int16_t delta = snapTicks - prevTicks;
    prevTicks = snapTicks;

    return delta;
  }
};

Encoder* encoder1 = NULL;
Encoder* encoder2 = NULL;

void EncoderHandler1A(void) {if(encoder1) encoder1->ProcessInterrupt(ENCODER_1A);}
void EncoderHandler1B(void) {if(encoder1) encoder1->ProcessInterrupt(ENCODER_1B);}
void EncoderHandler2A(void) {if(encoder2) encoder2->ProcessInterrupt(ENCODER_2A);}
void EncoderHandler2B(void) {if(encoder2) encoder2->ProcessInterrupt(ENCODER_2B);}

void SetupEncoders(void)
{
  encoder1 = new Encoder(ENCODER_1A, ENCODER_1B);
  encoder2 = new Encoder(ENCODER_2A, ENCODER_2B);
  
  //(almost) every pin is an interrupt on the SAMD, so use attachInterrupt()
  attachInterrupt(ENCODER_1A, EncoderHandler1A, CHANGE);
  attachInterrupt(ENCODER_1B, EncoderHandler1B, CHANGE);
  attachInterrupt(ENCODER_2A, EncoderHandler2A, CHANGE);
  attachInterrupt(ENCODER_2B, EncoderHandler2B, CHANGE);
}

#endif
