#include <AccelStepper.h>

// Define the step and direction pins for the X, Y, Z, and A axes
#define STEP_PIN_A 2
#define DIR_PIN_A 3
#define STEP_PIN_B 4
#define DIR_PIN_B 5
#define STEP_PIN_C 6
#define DIR_PIN_C 7
#define STEP_PIN_D 8
#define DIR_PIN_D 9

// Define the analog pin for the optical sensors
#define SENSOR_PIN_A1 A1
#define SENSOR_PIN_A2 A2
#define SENSOR_PIN_C A3
#define SENSOR_PIN_D A4

// Create instances of AccelStepper for X, Y, Z, and A axes
AccelStepper stepperA(AccelStepper::DRIVER, STEP_PIN_A, DIR_PIN_A);
const int stepsPerRevolution_A = 6400 * 5;
const int maxHomingSteps_A = (stepsPerRevolution_A / 360.0) * 180; // Steps for 180 degrees

AccelStepper stepperB(AccelStepper::DRIVER, STEP_PIN_B, DIR_PIN_B);
AccelStepper stepperC(AccelStepper::DRIVER, STEP_PIN_C, DIR_PIN_C);
AccelStepper stepperD(AccelStepper::DRIVER, STEP_PIN_D, DIR_PIN_D);

float GlobalMaxSpeed = 10000.0;
float GlobalAcceleration = 1000.0;
float ContinuousSpeed = 500.0;
int stepsToMove = stepsPerRevolution_A;

unsigned long previousMillis = 0;
const long interval = 1000;
bool movingForward = true;
bool xPaused = false;

// -----------------------------------------------------------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------------------------------------------------------

void setup() 
{
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set max speed and acceleration for the X stepper
  stepperA.setMaxSpeed(GlobalMaxSpeed);
  stepperA.setAcceleration(GlobalAcceleration);

  // Set speed for continuous movement of Y, Z, and A steppers
  stepperB.setMaxSpeed(GlobalMaxSpeed);
  stepperB.setSpeed(ContinuousSpeed);
  stepperC.setMaxSpeed(GlobalMaxSpeed);
  stepperC.setSpeed(ContinuousSpeed);
  stepperD.setMaxSpeed(GlobalMaxSpeed);
  stepperD.setSpeed(ContinuousSpeed);
  
  // Perform homing for stepperA
  homeStepperA();
}

// -----------------------------------------------------------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------------------------------------------------------

void loop() 
{
  // Get the current time
  unsigned long currentMillis = millis();

  // Manage the A stepper movement with non-blocking delay
  if (xPaused) {
    if (currentMillis - previousMillis >= interval) {
      xPaused = false;
      previousMillis = currentMillis;
      if (movingForward) {
        stepperA.moveTo(stepsToMove);
        movingForward = false;
      } else {
        stepperA.moveTo(0);
        movingForward = true;
      }
    }
  } else {
    if (stepperA.distanceToGo() == 0) {
      xPaused = true;
      previousMillis = currentMillis;
    }
    stepperA.run();
  }
  
  // Run Y, Z, and A steppers continuously
  stepperB.runSpeed();
  stepperC.runSpeed();
  stepperD.runSpeed();
}

// -----------------------------------------------------------------------------------------------------------------------------
// Homing
// -----------------------------------------------------------------------------------------------------------------------------
void homeStepperA() {
  Serial.println("Starting homing procedure...");
  stepperA.setSpeed(GlobalMaxSpeed / 10);

  int DegreesToMove = 0;
  while (DegreesToMove == 0)
  {
    int sensorValueA1 = analogRead(SENSOR_PIN_A1);
    int sensorValueA2 = analogRead(SENSOR_PIN_A2);
    
    stepperA.runSpeed();
    if (sensorValueA1 <= 100) {   
      DegreesToMove = 80;
      break;
    }
    if (sensorValueA2 <= 100) {
      DegreesToMove = -80;
      break;
    }
  }
  
  stepperA.move(degreesToSteps(DegreesToMove, stepsPerRevolution_A));
  while (stepperA.distanceToGo() != 0) {
    stepperA.run();
  }

  stepperA.setCurrentPosition(0);
  Serial.println("Homing procedure complete.");
  return;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------------------------------------------------------

// Function to calculate the steps needed for a given amount of degrees
int degreesToSteps(float degrees, int stepsPerRevolution) {
  return (degrees / 360.0) * stepsPerRevolution;
}
