/******************************************************************************
 * Includes
 ******************************************************************************/

#include "Arduino.h"
#include "PID.h"
/***********************************************************************************************
The setup function needs to be called first 
Kp- proportional gain term of PID
Ki- Integral gain term of PID
Kd- Differential gain term of PID
tau - Time Constant doe the Derivative pseudo-pole 
Ts- Sampling time 
  
************************************************************************************************/
void PID::setup(float Kp, float Ki, float Kd, float tau, float Ts)
{
	this->Kp=Kp;
	this->Ki=Ki;
	this->Kd=Kd;
	this->tau=tau;
	this->Ts=Ts;

	setStates(0,0); //Setting Initial States to 0
}

/***********************************************************************************************
This function sets the inital states of the controller to the desired values, if required
************************************************************************************************/
void PID::setStates(float state_int, float state_der)
{
	_state_int = state_int;
	_state_der = state_der;
}

/***********************************************************************************************
This function resets the whole PID controller
************************************************************************************************/
void PID::reset(void)
{
	_state_int = 0;
	_state_int_delay = 0;
	_state_der = 0;
	_state_der_delay = 0;
	_prev_e = 0;
}
/************************************************************************************
This function computes PID control value at a given time instance  (between uMin and uMax if no limits on the actuator are present) 
If limits are provided, it handles anti wind-up with the clamping technique (Integration is stopped when saturations are hit) 
e - Tracking error (r-y)

uMin - Minimum control value 
uMax - Maximum control value 

**************************************************************************************/
float PID::getControl(float e, float uMin, float uMax)
{
float pidControlValue=getPControl(e) + getIControl(e,uMin,uMax)+getDControl(e);
_prev_e = e;

//Check for input saturation
if(pidControlValue<uMin)
pidControlValue=uMin;
if(pidControlValue>uMax)
pidControlValue=uMax;

return pidControlValue;
}

float PID::getControl(float e)
{
float pidControlValue=getPControl(e) + getIControl(e)+getDControl(e);
_prev_e = e;
return pidControlValue;
}
/**********************************************************************************
This function is used to computer proportional control value 
e - tracking error 
***********************************************************************************/
float PID::getPControl(float e)
{
  return Kp*e;
}
/***************************************************************************************
This function is used to computer Integral control value. It has implementation of a discrete integrator  (Tustin/Bi-linear/Trapezoidal)
with the option of clamping for anti-windup, if limits are passed as arguments
e - tracking error 
uMin - Minimum control value 
uMax - Maximum control value 
*************************************************************************************/
float PID::getIControl(float e, float uMin, float uMax)
{	
	_state_int_delay = _state_int;
	_state_int = _state_int_delay + (Ki*Ts/2)*(e + _prev_e);	
	_state_int = max(min(_state_int, uMax), uMin); //Neat way to implement Anti-Windup
	return _state_int;
}

float PID::getIControl(float e)
{	
	_state_int_delay = _state_int;
	_state_int = _state_int_delay + (Ki*Ts/2)*(e + _prev_e);	
	return _state_int;
}
/********************************************************************************
This function is used to compute the differential control value using a discrete differentiator (Tustin/Bi-linear/Trapezoidal) 
e - tracking error 
**********************************************************************************/
float PID::getDControl(float e)
{
  _state_der_delay = _state_der;
  _state_der = (2*Kd/(Ts + 2*tau))*(e - _prev_e) - ((Ts - 2*tau)/(Ts + 2*tau))*_state_der_delay;
  return _state_der;
}
