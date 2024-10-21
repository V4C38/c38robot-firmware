#include <AccelStepper.h>


// -----------------------------------------------------------------------------------------------------------------------------
// Stepper Definitions
// -----------------------------------------------------------------------------------------------------------------------------
#define PIN_STEPPER_PUL_0 52
#define PIN_STEPPER_DIR_0 53
AccelStepper Stepper_0(AccelStepper::DRIVER, PIN_STEPPER_PUL_0, PIN_STEPPER_DIR_0);
const int Stepper_0_GearRatio = 1;
const int Stepper_0_StepsPerRevoloution = Stepper_0_GearRatio * 400 ; // Microstepping value
float Stepper_0_MaxSpeed = 5000.0;
float Stepper_0_Acceleration = 10000.0;
#define PIN_LIMITSWITCH_0A A0

#define PIN_STEPPER_PUL_1 50
#define PIN_STEPPER_DIR_1 51
AccelStepper Stepper_1(AccelStepper::DRIVER, PIN_STEPPER_PUL_1, PIN_STEPPER_DIR_1);
const int Stepper_1_GearRatio = 27;
const int Stepper_1_StepsPerRevoloution = Stepper_1_GearRatio * 400 ; // Microstepping value
float Stepper_1_MaxSpeed = 5000.0;
float Stepper_1_Acceleration = 10000.0;
#define PIN_LIMITSWITCH_1A A1
#define PIN_LIMITSWITCH_1B A2

#define PIN_STEPPER_PUL_2 48
#define PIN_STEPPER_DIR_2 49
AccelStepper Stepper_2(AccelStepper::DRIVER, PIN_STEPPER_PUL_2, PIN_STEPPER_DIR_2);
const int Stepper_2_GearRatio = 5;
const int Stepper_2_StepsPerRevoloution = Stepper_2_GearRatio * 200 ; // Microstepping value
float Stepper_2_MaxSpeed = 5000.0;
float Stepper_2_Acceleration = 10000.0;
#define PIN_LIMITSWITCH_2A A3



// =============================================================================================================================
// Setup
// =============================================================================================================================
void setup() 
{
  Serial.begin(9600);

  // Set the limit switch pins as INPUT_PULLUP
  pinMode(PIN_LIMITSWITCH_0A, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_1A, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_1B, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_2A, INPUT_PULLUP);

  // Init Stepper settings
  Stepper_0.setMaxSpeed(Stepper_0_MaxSpeed);
  Stepper_0.setAcceleration(Stepper_0_Acceleration);
  Stepper_1.setMaxSpeed(Stepper_1_MaxSpeed);
  Stepper_1.setAcceleration(Stepper_1_Acceleration);
  Stepper_2.setMaxSpeed(Stepper_2_MaxSpeed);
  Stepper_2.setAcceleration(Stepper_2_Acceleration);
  // Stepper_C.setMaxSpeed(Stpr_C_MaxSpeed);
  // Stepper_C.setAcceleration(Stpr_C_Acceleration);
  // Stepper_D.setMaxSpeed(Stpr_D_MaxSpeed);
  // Stepper_D.setAcceleration(Stpr_D_Acceleration);

}


// -----------------------------------------------------------------------------------------------------------------------------
// Testing Mode
// -----------------------------------------------------------------------------------------------------------------------------
void RunTest(int InTestID)
{
    switch (InTestID) {
      case 0:
        Serial.println("Running test for Stepper 0.");
        Stepper_0.move(1000);
        Stepper_0.runToPosition();
        Serial.println("Test for Stepper 0 completed.");
        break;

      case 1:
        Serial.println("Running test for Stepper 1.");
        Stepper_1.move(1000);
        Stepper_1.runToPosition();
        Serial.println("Test for Stepper 1 completed.");
        break;

      case 2:
        Serial.println("Running test for Stepper 2.");
        Stepper_2.move(1000);
        Stepper_2.runToPosition();
        Serial.println("Test for Stepper 0 completed.");
        break;

      default:
        Serial.println("Invalid stepper index provided.");
        break;
    }
    return;
}



// -----------------------------------------------------------------------------------------------------------------------------
// Homing
// -----------------------------------------------------------------------------------------------------------------------------
void RunHomingSequence(int InStepperIndex, bool InHomeAll = false)
{
    // If InHomeAll is true or InStepperIndex is -1, home all steppers in sequence
    if (InHomeAll || InStepperIndex == -1) {
        Serial.println("Homing all steppers.");
        HomeStepper_2();
        HomeStepper_1();
        HomeStepper_0();
        Serial.println("All steppers homed.");
    }
    else {
        // Home only the specified stepper based on InStepperIndex
        switch (InStepperIndex) {
            case 0:
                HomeStepper_0();
                break;

            case 1:
                HomeStepper_1();
                break;

            case 2:
                HomeStepper_2();
                break;

            default:
                Serial.println("Invalid stepper index provided.");
                break;
        }
    }
}


// =============================================================================================================================
// Loop
// =============================================================================================================================
void loop() 
{
    // Read Serial Commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');  // Read incoming command
        command.trim();  // Remove any leading/trailing whitespace

        if (command.startsWith("RunTest")) {
            int testID = command.substring(8).toInt();  // Extract the test ID
            RunTest(testID);
        }
        else if (command.startsWith("RunHomingSequence")) {
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex != -1) {
                String argument = command.substring(spaceIndex + 1);
                argument.trim();

                int stepperIndex = argument.toInt();
                bool homeAll = (stepperIndex == -1);  // Home all if stepperIndex is -1

                RunHomingSequence(stepperIndex, homeAll);
            } else {
                Serial.println("Invalid RunHomingSequence command format.");
            }
        } else {
            Serial.println("Unknown command received.");
        }
    }
}




void HomeStepper_0()
{
  Serial.println("Homing Stepper_0.");
  float HomingSpeed = Stepper_0_MaxSpeed / 5;
  Stepper_0.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
    int Val_LimitSwitch_0A = digitalRead(PIN_LIMITSWITCH_0A);

    Stepper_0.runSpeed();
    if (Val_LimitSwitch_0A == LOW)
    {   
      Stepper_0.stop();
      DegreesToHomePos = -90;
      break;
    }
  }
  // Then move to the axis center location and assume that as positon 0.
  Stepper_0.move(degreesToSteps(DegreesToHomePos, Stepper_0_StepsPerRevoloution));
  while (Stepper_0.distanceToGo() != 0) 
  {
    Stepper_0.run();
  }
  Stepper_0.setCurrentPosition(0);
  Stepper_0.setSpeed(Stepper_0_MaxSpeed);
  Serial.println("Completed: Stepper_0 homing.");
  return;
}

void HomeStepper_1()
{
  Serial.println("Homing Stepper_1.");
  float HomingSpeed = Stepper_1_MaxSpeed / 10;
  Stepper_1.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
    int Val_LimitSwitch_1A = digitalRead(PIN_LIMITSWITCH_1A);
    int Val_LimitSwitch_1B = digitalRead(PIN_LIMITSWITCH_1B);

    Stepper_1.runSpeed();
    if (Val_LimitSwitch_1A == LOW)
    {   
      Stepper_1.stop();
      DegreesToHomePos = -90;
      break;
    }
    if (Val_LimitSwitch_1B == LOW)
    {   
      Stepper_1.stop();
      DegreesToHomePos = 90;
      break;
    }
  }
  // Then move to the axis center location and assume that as positon 0.
  Stepper_1.move(degreesToSteps(DegreesToHomePos, Stepper_1_StepsPerRevoloution));
  while (Stepper_1.distanceToGo() != 0) 
  {
    Stepper_1.run();
  }
  Stepper_1.setCurrentPosition(0);
  Stepper_1.setSpeed(Stepper_1_MaxSpeed);
  Serial.println("Completed: Stepper_1 homing.");
  return;
}

void HomeStepper_2()
{
  Serial.println("Homing Stepper_2.");
  float HomingSpeed = Stepper_2_MaxSpeed / 5;
  Stepper_2.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
    int Val_LimitSwitch_2A = digitalRead(PIN_LIMITSWITCH_2A);

    Stepper_2.runSpeed();
    if (Val_LimitSwitch_2A == LOW)
    {   
      Stepper_2.stop();
      DegreesToHomePos = -180;
      break;
    }
  }
  // Then move to the axis center location and assume that as positon 0.
  Stepper_2.move(degreesToSteps(DegreesToHomePos, Stepper_2_StepsPerRevoloution));
  while (Stepper_2.distanceToGo() != 0) 
  {
    Stepper_2.run();
  }
  Stepper_2.setCurrentPosition(0);
  Stepper_2.setSpeed(Stepper_2_MaxSpeed);
  Serial.println("Completed: Stepper_2 homing.");
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
