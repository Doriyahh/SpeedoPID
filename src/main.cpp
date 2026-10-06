#include <Arduino.h>

//Pin definitions
const int pcmPin = 4;             //3.3V signal from the octocoupler
const int encoderPin = 5;         //3.3V signal from the motor encoder
const int pwmPin = 18;            //3.3V PWM output pin to MOSFET transistor gate

//Pulse counters for square signals. Must be volatile so they can be increased independently from main loop
volatile int pcmCount = 0;        //Increases every pulse from PCM
volatile int encoderCount = 0;    //Increases every motor encoder pulse

//Time variables
unsigned long previousMS = 0; 
const int interval = 100;

//PID Perameters
float Kp = 2.0;   //Proportional term: Multiplies error for output
float Ki = 0.5;   //Looks at previous outputs to adjust if outside resistance exists
float Kd = 1.0;   //Reduces output when getting close to target to reduce overshoot

//Other useful values
float integral = 0;
int previousError = 0;


//Hardware interrupts to monitor pulses outside of software loop
void IRAM_ATTR countPCM() {
  pcmCount++;
}

void IRAM_ATTR countEncoder() {
  encoderCount++;
}


void setup() {
  pinMode(pcmPin, INPUT_PULLUP);
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);

  //Attach the hardware interrupts to count up whenever corresponding signal goes from low to high
  attachInterrupt(digitalPinToInterrupt(pcmPin), countPCM, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderCount), countEncoder, RISING);
}

void loop() {
  unsigned long currentMS = millis();

  if(currentMS - previousMS >= interval) {
    previousMS = currentMS;

    //Grab the current counts without risking them being changed during calculation
    noInterrupts();   //Prevents the interrupts from activating
    int currentPCM = pcmCount;
    int currentEncoder = encoderCount;
    pcmCount = 0;
    encoderCount = 0;
    interrupts(); //Resumes interrupts

    //Set the target encoder count based on the current PCM and adjusted for difference in pulse rate
    //The encoder has 28 pulses per revolution, and at 60MPH we want 1000 rev/min, so 28,000 pulses/minute.
    //And we would get 4,000 PCM pulses per minute at 60MPH, so 28,000/ 4,000 = 7 encoder pulses per PCM pulse
    int targetEncoderCount = currentPCM * 7;

    //Calculate the error between PCM and encoder counts
    int error = targetEncoderCount - currentEncoder;

    integral += error; //Add the error to the integral for future reference
    int derivative = error - previousError; //Calculate the difference between the current and previous error

    //Calculate the output based on tuning factors and the calculated variables
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    //Constrain the output to be between 0 and 255 for 8-bit PWM output
    int pwmValue = constrain(output, 0, 255);

    if(pwmValue == 255 || pwmValue == 0) {
      integral -= error; //Reduce the integral if the output is at min or max to prevent windup and burnout
    }

    //Drive the MOSFETs gate pin
    analogWrite(pwmPin, pwmValue);

    //Store the current error for the next loop
    previousError = error;
  }
}