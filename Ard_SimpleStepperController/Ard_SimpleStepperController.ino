#include <AccelStepper.h>

// Define the step and direction pins for the X, Y, Z, and A axes
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7
#define A_STEP_PIN 12
#define A_DIR_PIN 13

// Create instances of AccelStepper for X, Y, Z, and A axes
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperA(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);

float GlobalMaxSpeed = 250.0; 
float GlobalAcceleration = 25.0;
float ContinuousSpeed = 250.0;
int stepsToMove = 2500;

unsigned long previousMillis = 0;
const long interval = 1000;
bool movingForward = true;
bool xPaused = false;

void setup() 
{
  // Set max speed and acceleration for the X stepper
  stepperX.setMaxSpeed(GlobalMaxSpeed);
  stepperX.setAcceleration(GlobalAcceleration);

  // Set speed for continuous movement of Y, Z, and A steppers
  stepperY.setMaxSpeed(GlobalMaxSpeed);
  stepperY.setSpeed(ContinuousSpeed);
  stepperZ.setMaxSpeed(GlobalMaxSpeed);
  stepperZ.setSpeed(ContinuousSpeed);
  stepperA.setMaxSpeed(GlobalMaxSpeed);
  stepperA.setSpeed(ContinuousSpeed);
}

void loop() 
{
  // Get the current time
  unsigned long currentMillis = millis();

  // Manage the X stepper movement with non-blocking delay
  if (xPaused) {
    if (currentMillis - previousMillis >= interval) {
      xPaused = false;
      previousMillis = currentMillis;
      if (movingForward) {
        stepperX.moveTo(stepsToMove);
        movingForward = false;
      } else {
        stepperX.moveTo(0);
        movingForward = true;
      }
    }
  } else {
    if (stepperX.distanceToGo() == 0) {
      xPaused = true;
      previousMillis = currentMillis;
    }
    stepperX.run();
  }

  // Run Y, Z, and A steppers continuously
  stepperY.runSpeed();
  stepperZ.runSpeed();
  stepperA.runSpeed();
}
