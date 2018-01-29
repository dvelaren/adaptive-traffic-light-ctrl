//Name: adaptiveTrafficLight
//Author: David Velasquez (dvelas25@eafit.edu.co)
//        Raul Mazo (rimazop@eafit.edu.co)
//Date: 29/01/2018
//Description: This program controls a traffic light that adapts to Day/Night context.
//             If the sensor detects that the system is in the Day State, it works as
//             a normal traffic light. In other case (Night State) it starts blinking
//             the red light. It is programmed as a Finite State Machine (FSM) algorithm.
//             At the day, the traffic lights control the total illumination

//Library definitions
#include <PID_v1.h> //PID Library

//I/O Pin labeling
//->Inputs
#define daySensor 0 //Light Dependant Resistor (LDR) daySensor on pin A0 (Analog)
#define lightSensor 1 //Light Dependant Resistor (LDR) lightSensor on pin A1 (Analog)
//->Outputs
#define LR 2 //Red Light on pin 2
#define LY 3 //Yellow Light on pin 3
#define LG 4 //Green Light on pin 4

//FSM names labeling
//->Main FSM
#define EDAY 0  //State Day
#define ENIGHT 1  //State Night
//->Normal Traffic Light FSM
#define ER 0  //Red state
#define EG 1  //Green state
#define EY 2  //Yellow state
//->Blinking red light FSM
#define EROFF 0 //Red off state
#define ERON 1  //Red on state

//Constant definitions
const unsigned long LDRVAL = 300; //Constant ADC Value of LDR for changing from Day to Night (400 ADC)
const unsigned long TRG = 4000; //Constant time from red to green (4000 msecs)
const unsigned long TGY = 2000; //Constant time from green to yellow (2000 msecs)
const unsigned long TYR = 1000; //Constant time from yellow to red (1000 msecs)
const unsigned long TILT = 500;  //Constant time to tilt red light in night (500 msecs)
double consKp = 0.032, consKi = 3.2, consKd = 0.0016; //PID Control constants Proportional Kp, Integral Ki y Derivative Kd
const unsigned int numReadings = 2;  //Samples for recursive moving average

//Variables definition
//->Analog input vars
unsigned int sensorVal = 0; //Variable to store sensor LDR value in ADC (10 bits - Number between 0- 1023)
double lightVal = 0;  //Variable to store sensor LDR total light in ADC (10 bits - Number between 0-1023)
//->Controler Variable
double Setpoint = 600, Output = 0; //Controller vars (SetPoint or desired value, Output of the PID Controller)
PID myPID(&lightVal, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);  //PID Library object
//->Variables for moving average
unsigned int readings[numReadings] = {0};
unsigned int readIndex = 0;
unsigned int total = 0;
//->FSM Vars
unsigned int mainState = EDAY;  //Variable to store current State for FSM Day/Night (Main FSM) initialized on EDAY
unsigned int trafficState = ER; //Variable to store current State for FSM Normal Traffic Light (Main FSM) initialized on ER
unsigned int blinkState = EROFF; //Variable to store current State for FSM blinking red light  initialized on EROFF
//->FSM Timing Vars
unsigned long tact = 0; //Variable to store actual time for all the timers
unsigned long tiniTraffic = 0;  //Variable to store initial time for Normal Traffic Light FSM
unsigned long trelTraffic = 0;  //Variable to store relative time for Normal Traffic Light FSM
unsigned long tiniBlink = 0;  //Variable to store relative time for Blinking red light FSM
unsigned long trelBlink = 0;  //Variable to store relative time for Blinking red light FSM
unsigned long tprev = 0;

//Subroutines & Functions
unsigned int smooth() { //Moving Average Subroutine
  // subtract the last reading
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(lightSensor);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  return total / numReadings;
}

//FSM normal traffic light
void fsmTraffic() {
  switch (trafficState) {
    case ER:
      //Physical Output state
      analogWrite(LR, Output); //Turn red light on with PID Output
      digitalWrite(LY, LOW);  //Turn yellow light off
      digitalWrite(LG, LOW);  //Turn green light off

      //Internal Variable state calculation
      trelTraffic = tact - tiniTraffic; //Calculate relative time for Traffic timer

      //Transition conditionals
      if (trelTraffic >= TRG) {
        trafficState = EG;  //Change to Green State
        Serial.println("trafficState: EG"); //Debug print current state
        tiniTraffic = millis(); //Assign new starting time for next state
      }
      break;
    case EG:
      //Physical Output state
      digitalWrite(LR, LOW); //Turn red light off
      digitalWrite(LY, LOW);  //Turn yellow light off
      analogWrite(LG, Output);  //Turn green light on with PID Output

      //Internal Variable state calculation
      trelTraffic = tact - tiniTraffic; //Calculate relative time for Traffic timer

      //Transition conditionals
      if (trelTraffic >= TGY) {
        trafficState = EY;  //Change to Yellow State
        Serial.println("trafficState: EY"); //Debug print current state
        tiniTraffic = millis(); //Assign new starting time for next state
      }
      break;
    case EY:
      //Physical Output state
      digitalWrite(LR, LOW); //Turn red light off
      analogWrite(LY, Output);  //Turn yellow light on with PID Output
      digitalWrite(LG, LOW);  //Turn green light off

      //Internal Variable state calculation
      trelTraffic = tact - tiniTraffic; //Calculate relative time for Traffic timer

      //Transition conditionals
      if (trelTraffic >= TYR) {
        trafficState = ER;  //Change to Red State
        Serial.println("trafficState: ER"); //Debug print current state
        tiniTraffic = millis(); //Assign new starting time for next state
      }
      break;
  }
}

//FSM normal traffic light
void fsmBlink() {
  switch (blinkState) {
    case EROFF:
      //Physical Output state
      digitalWrite(LR, HIGH); //Turn red light on
      digitalWrite(LY, LOW);  //Turn yellow light off
      digitalWrite(LG, LOW);  //Turn green light off

      //Internal Variable state calculation
      trelBlink = tact - tiniBlink; //Calculate relative time for Blinking timer

      //Transition conditionals
      if (trelBlink >= TILT) {
        blinkState = ERON;  //Change to Red on State
        Serial.println("blinkState: ERON"); //Debug print current state
        tiniBlink = millis(); //Assign new starting time for next state
      }
      break;
    case ERON:
      //Physical Output state
      digitalWrite(LR, LOW); //Turn red light off
      digitalWrite(LY, LOW);  //Turn yellow light off
      digitalWrite(LG, LOW);  //Turn green light off

      //Internal Variable state calculation
      trelBlink = tact - tiniBlink; //Calculate relative time for Blinking timer

      //Transition conditionals
      if (trelBlink >= TILT) {
        blinkState = EROFF;  //Change to Red off State
        Serial.println("blinkState: EROFF"); //Debug print current state
        tiniBlink = millis(); //Assign new starting time for next state
      }
      break;
  }
}

//Configuration
void setup() {
  //I/O configuration
  pinMode(LR, OUTPUT);  //Red light as Digital Output
  pinMode(LY, OUTPUT);  //Yellow light as Digital Output
  pinMode(LG, OUTPUT);  //Green light as Digital Output

  //Physical Output initial cleaning
  digitalWrite(LR, LOW); //Turn red light off
  digitalWrite(LY, LOW);  //Turn yellow light off
  digitalWrite(LG, LOW);  //Turn green light off

  //Communications
  Serial.begin(9600); //Start serial comms to debug with Serial Monitor PC (9600 bauds)
  myPID.SetMode(AUTOMATIC); //Set PID in automatic mode
  Serial.println("mainState: EDAY");  //Debug print current state
  Serial.println("trafficState: ER");  //Debug print current state
  Serial.println("blinkState: EROFF");  //Debug print current state
  //Initial time cleaning
  tiniTraffic = millis();
  tiniBlink = millis();
}

//Run-time
void loop() {
  tact = millis();  //Take actual time for all FSM relative timing calculations
  //Main FSM
  switch (mainState) {
    case EDAY:
      //Physical Output state
      //->Controlled by FSM traffic
      fsmTraffic(); //Call FSM traffic light

      //Internal Variable state calculation
      sensorVal = analogRead(daySensor);  //Read analog value of LDR as ADC (10 bits- 0 to 1023 number)
      lightVal = smooth();  //Acquire readings from sensor and smooth it through Recursive Moving Average
      myPID.Compute();  //Compute PID Output

      //Transition conditionals
      if (sensorVal <= LDRVAL) {
        mainState = ENIGHT;  //Change to Night State
        Serial.println("mainState: ENIGHT"); //Debug print current state
        tiniBlink = millis(); //Assign new starting time for next state
      }
      break;

    case ENIGHT:
      //Physical Output state
      //->Controlled by FSM traffic
      fsmBlink(); //Call FSM Blink

      //Internal Variable state calculation
      sensorVal = analogRead(daySensor);  //Read analog value of LDR as ADC (10 bits- 0 to 1023 number)

      //Transition conditionals
      if (sensorVal > LDRVAL) {
        mainState = EDAY;  //Change to Day State
        Serial.println("mainState: EDAY"); //Debug print current state
        tiniTraffic = millis(); //Assign new starting time for next state
      }
      break;
  }
  if(millis() - tprev >= 2000) {
    Serial.println("daySensor: " + String(sensorVal) + " Setpoint: " + String(Setpoint) + " lightSensor: " + String(lightSensor) + " Error: " + String(Setpoint-lightSensor) + " Output: " + String(Output));
    tprev = millis();
  }
}

