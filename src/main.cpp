#include "utilities.h"
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// --------- Global Variables ---------

// rotary stage gear ratio
const uint8_t ROT_GEAR = 5;
// rotary stage steps
int32_t ROT_STEPS;

// linear stage ball screw lead [mm]
const uint8_t LINEAR_LEAD = 5;
// linear stage steps
int32_t LIN_STEPS;

// microstep resolution
const uint16_t MICROSTEP = 6400;

const float GUIDE_RAD = 11.44;

const float LIM_SWITCH_OFFSET = 4.1;

enum OperatingState {
  IDLE,
  HOMING,
  CALIBRATE,
  SETUP,
  INITIALIZING,
  WINDING,
  ERROR
};

OperatingState CURR_STATE = IDLE;
bool IS_HOMED = false;
bool IS_SETUP = false;

// homing distance to travel [mm]
const float LIN_HOME_CMD = -500.0;

// winding length of fiber pump [mm]
float WIND_AMNT = 50.0;
float HELIX_ANGLE;

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

// --------- Arduino Pin Initialization ---------

// limit switch pin
const uint8_t limswitchPin = 6;

// photomicrosensor pin
const uint8_t optsensorPin = 18;

// button switch pin
const uint8_t buttonPin = 7;

// onboard LED
const uint8_t ledPin = 13;

// linear stage motor connections                         
const uint8_t dirPinM1 = 2;
const uint8_t stepPinM1 = 3;

// rotary stage motor connections
const uint8_t dirPinM2 = 4;
const uint8_t stepPinM2 = 5;

// --------- Stepper Motor Initialization ---------

// individual stepper objects
AccelStepper linearStepper(AccelStepper::DRIVER, stepPinM1, dirPinM1);
AccelStepper rotStepper(AccelStepper::DRIVER, stepPinM2, dirPinM2);

// multistepper coordination object
MultiStepper steppers;

int32_t millimetersToSteps(float);
void resetStepperDynamics();

void onPulse() {
  unsigned long currTime = micros();
  pulseInterval = currTime - lastPulseTime;
  lastPulseTime = currTime;
  newPulse = true;
}

void setup() {

  pinMode(limswitchPin, INPUT_PULLUP);
  pinMode(optsensorPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // microstep resolution
  uint8_t microstep_res = (int)(MICROSTEP / 200);

  // stepper angle resolution (deg)
  float stepper_res = 1.8;

  // mandrell diameter
  float d_man = 1.0;

  // filament diameter
  float d_fil = 0.4;

  // number of filaments
  float num_fil = 6;

  Utilities linearStage(num_fil, d_fil, d_man, stepper_res, microstep_res);
  Utilities rotaryStage(num_fil, d_fil, d_man, stepper_res, microstep_res);

  float helix_pitch = linearStage.getPitch();
  HELIX_ANGLE = linearStage.getHelixAngle();

  ROT_STEPS = static_cast<int32_t>(rotaryStage.getSteps(ROT_GEAR));
  LIN_STEPS = static_cast<int32_t>(linearStage.getSteps(helix_pitch / LINEAR_LEAD));

  attachInterrupt(digitalPinToInterrupt(optsensorPin), onPulse, FALLING);

  Serial.begin(115200);

  resetStepperDynamics();

  steppers.addStepper(linearStepper);
  steppers.addStepper(rotStepper);
}
int32_t millimetersToSteps(float mm){
 
  return (int32_t)(((float)MICROSTEP / LINEAR_LEAD) * mm);

}

void resetStepperDynamics(){ 

  int accelSteps = millimetersToSteps(100.0);

  linearStepper.setMaxSpeed(LIN_STEPS);
  linearStepper.setSpeed((int32_t)LIN_STEPS/5.6);
  linearStepper.setAcceleration(accelSteps);

  ROT_STEPS = (int32_t)(ROT_STEPS * 1.01);
  rotStepper.setMaxSpeed(ROT_STEPS);
  rotStepper.setSpeed((int32_t)(ROT_STEPS/4));
  rotStepper.setAcceleration(accelSteps);

}

void homeLinear() {

  linearStepper.setMaxSpeed(millimetersToSteps(12.0));

  Serial.println("Homing Linear Stage...");
  
  linearStepper.moveTo(millimetersToSteps(LIN_HOME_CMD));
  
  while(digitalRead(limswitchPin)) {
    linearStepper.run();
  }

  linearStepper.stop();
  
  linearStepper.setCurrentPosition(0);

  linearStepper.move(millimetersToSteps(5.0));

  while(linearStepper.currentPosition() != millimetersToSteps(5.0)) {
    linearStepper.run();
  }

  linearStepper.setCurrentPosition(0);

}

void homeRotary() {

  rotStepper.setMaxSpeed(millimetersToSteps(8.0));

  Serial.println("Homing Rotary Stage...");

  if (!digitalRead(optsensorPin)) {

    rotStepper.setCurrentPosition(0);
    rotStepper.move(millimetersToSteps(1.0));
    while(rotStepper.currentPosition() != millimetersToSteps(1.0)) {
      rotStepper.run();
    }
  } 

  rotStepper.move(millimetersToSteps(40.0));
  while(digitalRead(optsensorPin)) {
    rotStepper.run();
  }

  rotStepper.setCurrentPosition(0);
  rotStepper.setMaxSpeed(millimetersToSteps(0.25));
  rotStepper.move(millimetersToSteps(1.0));
  while(!digitalRead(optsensorPin)) {
    rotStepper.run();
  }
  rotStepper.setCurrentPosition(0);

  rotStepper.move(-millimetersToSteps(1.0));
  while(digitalRead(optsensorPin)) {
    rotStepper.run();
  }
  
  rotStepper.setCurrentPosition(0);
  rotStepper.move(-millimetersToSteps(1.0));
  while(!digitalRead(optsensorPin)) {
    rotStepper.run();
  }

  int rot_home_offset = rotStepper.currentPosition() / 2;
  rotStepper.moveTo(rot_home_offset);
  while(rotStepper.currentPosition() != rot_home_offset) {
    rotStepper.run();
  }
  rotStepper.setCurrentPosition(0);

  }

void homingSequence(){

  Serial.println("Homing Stages...");

  homeLinear();

  homeRotary();

  IS_HOMED = true;

}

void loop() {

  switch (CURR_STATE) {

    case IDLE:

      delay(2000);

      if (!IS_HOMED) {
        CURR_STATE = HOMING;
      }

      else if (!digitalRead(buttonPin)) {
        if (!IS_SETUP) {
          Serial.println("Starting Setup");
          CURR_STATE = SETUP;
        }
        else {
          WIND_AMNT += 50.0;
          CURR_STATE = WINDING;
        }
      }

      break;

    case HOMING:

      if (!IS_HOMED) {
        homingSequence();
      } else {
        Serial.println("Stages Are Homed!");
        CURR_STATE = IDLE;
      }

      break;

    case CALIBRATE: {
    
      resetStepperDynamics();

      int interrupt_count = 0;
      unsigned long time_running_avg = 0;

      while(interrupt_count != 8){
          if (newPulse) {
              noInterrupts();
              unsigned long interval = pulseInterval;
              newPulse = false;
              interrupts();

              if (interrupt_count < 2) {
                  time_running_avg = 0;
              } else {
                  time_running_avg += interval;
              }
              interrupt_count++;

              Serial.print("Interval: ");
              Serial.print(interval);
              Serial.println(" us");

              if (interval > 0) {
                  float frequency = 1000000.0 / (interval);
                  Serial.print("Frequency: ");
                  Serial.print(frequency);
                  Serial.println(" Hz");
              }
          }
          rotStepper.runSpeed();
      } ;

      CURR_STATE = IDLE;
      break;  
  }      

    case SETUP:

      linearStepper.moveTo(millimetersToSteps(50.0));

      while(linearStepper.currentPosition() != millimetersToSteps(50.0)) {
        linearStepper.run();
      }

      while (digitalRead(buttonPin)) {
        Serial.println("Setting Up...");
        delay(1000);
      }

      linearStepper.moveTo(0);

      while(linearStepper.currentPosition() != 0) {
        linearStepper.run();
      }

      CURR_STATE = INITIALIZING;

      break;

    
    case INITIALIZING: {
      
      resetStepperDynamics();

      float init_height = ((GUIDE_RAD) / tan(HELIX_ANGLE)) - LIM_SWITCH_OFFSET;
  
      linearStepper.moveTo(millimetersToSteps(init_height));

      while(linearStepper.currentPosition() != millimetersToSteps(init_height)) {
        linearStepper.run();
      }

      rotStepper.setMaxSpeed(millimetersToSteps(10.0));
      rotStepper.moveTo(millimetersToSteps(25.0));

      while (rotStepper.currentPosition() != millimetersToSteps(25.0))
      {
        rotStepper.run();  
      }
      
      linearStepper.move(-millimetersToSteps(init_height + 3.5));

      while(linearStepper.currentPosition() != -millimetersToSteps(3.5)) {
        linearStepper.run();
      }

      delay(1000);

      CURR_STATE = WINDING;

      break;
    }

    case WINDING:

      resetStepperDynamics();

      while(linearStepper.currentPosition() != millimetersToSteps(WIND_AMNT)) {
        linearStepper.runSpeed();
        rotStepper.runSpeed();
      }
      
      Serial.println("Done");

      IS_SETUP = true;
      CURR_STATE = IDLE;
      
      break;

    case ERROR:
      break; 
  }

}


