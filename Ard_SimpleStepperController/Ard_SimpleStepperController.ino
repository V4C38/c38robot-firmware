#include <AccelStepper.h>


// -----------------------------------------------------------------------------------------------------------------------------
// Stepper Definitions
// -----------------------------------------------------------------------------------------------------------------------------
#define PIN_STEP_A 2
#define PIN_DIR_A 3
AccelStepper Stepper_A(AccelStepper::DRIVER, PIN_STEP_A, PIN_DIR_A);
const int Stpr_A_GearRatio = 0;
const int Stpr_A_RevSteps = Stpr_A_GearRatio * 800 ; // Microstepping value
float Stpr_A_MaxSpeed = 5000.0;
float Stpr_A_Acceleration = 500.0;

#define PIN_STEP_B 52
#define PIN_DIR_B 53
AccelStepper Stepper_B(AccelStepper::DRIVER, PIN_STEP_B, PIN_DIR_B);
const int Stpr_B_GearRatio = 27;
const int Stpr_B_RevSteps = Stpr_B_GearRatio * 800 ; // Microstepping value
float Stpr_B_MaxSpeed = 2000.0;
float Stpr_B_Acceleration = 250.0;

#define PIN_STEP_C 6
#define PIN_DIR_C 7
AccelStepper Stepper_C(AccelStepper::DRIVER, PIN_STEP_C, PIN_DIR_C);
const int Stpr_C_GearRatio = 5.0;
const int Stpr_C_RevSteps = Stpr_C_GearRatio * 825 ; // Microstepping value
float Stpr_C_MaxSpeed = 5000.0;
float Stpr_C_Acceleration = 25000.0;

#define PIN_STEP_D 8
#define PIN_DIR_D 9
AccelStepper Stepper_D(AccelStepper::DRIVER, PIN_STEP_D, PIN_DIR_D);
const int Stpr_D_GearRatio = 1;
const int Stpr_D_RevSteps = Stpr_D_GearRatio * 800 ; // Microstepping value
float Stpr_D_MaxSpeed = 5000.0;
float Stpr_D_Acceleration = 500.0;


// -----------------------------------------------------------------------------------------------------------------------------
// Sensor Definitions
// -----------------------------------------------------------------------------------------------------------------------------
#define PIN_LIMITSWITCH_A1 A1
#define PIN_LIMITSWITCH_A2 A2
#define PIN_LIMITSWITCH_B1 A3
#define PIN_LIMITSWITCH_C1 A4


// =============================================================================================================================
// Setup
// =============================================================================================================================
void setup() 
{
  Serial.begin(9600);

  // Set the limit switch pins as INPUT_PULLUP
  pinMode(PIN_LIMITSWITCH_A1, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_A2, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_B1, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_C1, INPUT_PULLUP);

  // Init Stepper settings
  // Stepper_A.setMaxSpeed(Stpr_A_MaxSpeed);
  // Stepper_A.setAcceleration(Stpr_A_Acceleration);
  Stepper_B.setMaxSpeed(Stpr_B_MaxSpeed);
  Stepper_B.setAcceleration(Stpr_B_Acceleration);
  // Stepper_C.setMaxSpeed(Stpr_C_MaxSpeed);
  // Stepper_C.setAcceleration(Stpr_C_Acceleration);
  // Stepper_D.setMaxSpeed(Stpr_D_MaxSpeed);
  // Stepper_D.setAcceleration(Stpr_D_Acceleration);

  
  // Home Steppers in order
  // HomeStepper_D();
  // HomeStepper_C();
  // HomeStepper_B();
  // HomeStepper_A();
}

// =============================================================================================================================
// Loop
// =============================================================================================================================
void loop() 
{

  // Move motor forward by 1000 steps
  Stepper_B.move(5000);  // Adjust this value for more/less movement
  Stepper_B.runToPosition();  // Block until move completes

  delay(1000);  // Pause for 1 second

  // Move motor backward by 1000 steps
  Stepper_B.move(-1000);  // Adjust this value for more/less movement
  Stepper_B.runToPosition();  // Block until move completes

  delay(1000);  // Pause for 1 second
}





// -----------------------------------------------------------------------------------------------------------------------------
// Homing
// -----------------------------------------------------------------------------------------------------------------------------

void HomeStepper_A()
{
  return;
}

void HomeStepper_B()
{
  /*
  Serial.println("Homing Stepper_B.");
  float HomingSpeed = Stpr_B_MaxSpeed / 10;
  Stepper_B.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
    int OptSensorVal_pos = 0; // analogRead(PIN_OptSensor_B_pos);
    int OptSensorVal_neg = 0; //analogRead(PIN_OptSensor_B_neg);

    Stepper_B.runSpeed();
    if (OptSensorVal_pos <= 0) //OptSensor_B_Treshold) 
    {   
      Stepper_B.stop();
      DegreesToHomePos = -20;
      break;
    }
    if (OptSensorVal_neg <= 0) //OptSensor_B_Treshold) 
    {   
      Stepper_B.stop();
      DegreesToHomePos = 20;
      break;
    }
  }
  // Then move to the axis center location and assume that as positon 0.
  Stepper_B.move(degreesToSteps(DegreesToHomePos, Stpr_B_RevSteps));
  while (Stepper_B.distanceToGo() != 0) 
  {
    Stepper_B.run();
  }
  Stepper_B.setCurrentPosition(0);
  Stepper_B.setSpeed(Stpr_B_MaxSpeed);
  Serial.println("Completed: Stepper_B homing.");
  */
  return;
}

void HomeStepper_C()
{
  /*
  Serial.println("Homing Stepper_C.");
  float HomingSpeed = Stpr_C_MaxSpeed / 3;
  Stepper_C.setSpeed(HomingSpeed);
  
  // Run Stepper_C until clockwise until it triggers OptSensor_C. 
  while (true)
  {
    int OptSensorVal = 0; //analogRead(PIN_OptSensor_C);
    Stepper_C.runSpeed();
    if (OptSensorVal <= 0) //OptSensor_C_Treshold) {   
      Stepper_C.stop();
      break;
    }
  }
  // Then move to the axis center location and assume that as positon 0.
  Stepper_C.move(degreesToSteps(77, Stpr_C_RevSteps));
  while (Stepper_C.distanceToGo() != 0) 
  {
    Stepper_C.run();
  }
  Stepper_C.setCurrentPosition(0);
  Stepper_C.setSpeed(Stpr_C_MaxSpeed);
  Serial.println("Completed: Stepper_C homing.");

  */
  return;
}

void HomeStepper_D()
{
  return;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------------------------------------------------------

// Function to calculate the steps needed for a given amount of degrees
int degreesToSteps(float degrees, int stepsPerRevolution) {
  return (degrees / 360.0) * stepsPerRevolution;
}
