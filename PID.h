#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "math.h"

class PID
{
  public:
	void setup(float Kp, float Ki, float Kd, float tau, float Ts);
	void setStates(float state_int, float state_der);
	void reset(void);
    float getControl(float e, float uMin, float uMax);
	float getControl(float e);
  private:
   float getPControl(float e);
   float getIControl(float e, float uMin, float uMax);
   float getIControl(float e);
   float getDControl(float e);
   /************************
   PID Parameters
   ************************/
   float Kp;
   float Ki;
   float Kd;
   float tau;
   float Ts;
   /************************
   PID states
   ************************/
	float _prev_e = 0;
	float _state_int;
	float _state_int_delay;	
	float _state_der;
	float _state_der_delay;	
};

#endif
