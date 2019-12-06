#include "PID.h"
#include "math.h"

/*OPEN SERIAL PLOTTER TO SEE OUTPUT */

/********************************************************
*************PID controller library example**************

  Plant =  0.009901
          ----------  ======> y(t)=0.009901 u(t-1)+0.9802 y(t-1)
          z - 0.9802

          Kp=7.5827
          Ki=82.7956
          Kd=0.1552
          Ts=0.01 sec
          tau = T/2 = 0.005 sec
          Reference is sin wave of amplitude 1 at 5 rad/sec


******************************************/
PID myPID;

float delayInput = 0; //Plant delayed input value
float delayOutput = 0; //Plant delayed output value
float F = 5; //frequency of reference Sine Wave
float ref = 0; //Initializing Reference to zero
float Ts = 0.01; //Sample Time
uint32_t loopTimer; //Variable for implentation of Control Loop

void setup() {
  myPID.setup(7.5827, 82.7956, 0.1552, 0.005, 0.01); // setup(Kp, Ki, Kd, tau, Ts)
  myPID.setStates(0, 0); //Not required in this case, but possible
  Serial.begin(19200);
  loopTimer = millis(); // Initializing time keeping variable to current millis() clock
}

void loop() {

  //myPID.reset(); //Reset PID function to 0 inital states and input

  ref = sin(F * loopTimer / float(1000));

  //Plant Implementation
  float Output = 0.009901 * delayInput + 0.9802 * delayOutput;
  delayOutput = Output;

  //delayInput=myPID.getControl(ref-Output,-100,100); //Computation of control signal with limits of -100 and 100
  delayInput = myPID.getControl(ref - Output); //Computation of control signal with no saturations

  Serial.print(ref);
  Serial.print(", ");
  Serial.println(Output);
  while (millis() - loopTimer < Ts * 1000); // Waitinf ro the the rest of the sample time to pass
  loopTimer = millis(); //resetting time-keeping variable to current millis() clock
}
