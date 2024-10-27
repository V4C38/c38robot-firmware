#include <AccelStepper.h>


// -----------------------------------------------------------------------------------------------------------------------------
// Stepper Definitions
// -----------------------------------------------------------------------------------------------------------------------------
#define PIN_STEPPER_PUL_0 52
#define PIN_STEPPER_DIR_0 53
AccelStepper Stepper_0(AccelStepper::DRIVER, PIN_STEPPER_PUL_0, PIN_STEPPER_DIR_0);
const int Stepper_0_Microstepping = 400;
const int Stepper_0_GearRatio = 1;
const int Stepper_0_StepsPerRevoloution = Stepper_0_GearRatio * Stepper_0_Microstepping;
float Stepper_0_MaxSpeed = 5000.0;
float Stepper_0_Acceleration = 10000.0;
bool Stepper_0_IsHoming = false;
float Stepper_0_StepsToMove = 0.0;
#define PIN_LIMITSWITCH_0A A0

#define PIN_STEPPER_PUL_1 50
#define PIN_STEPPER_DIR_1 51
AccelStepper Stepper_1(AccelStepper::DRIVER, PIN_STEPPER_PUL_1, PIN_STEPPER_DIR_1);
const int Stepper_1_Microstepping = 400;
const int Stepper_1_GearRatio = 27;
const int Stepper_1_StepsPerRevoloution = Stepper_1_GearRatio * Stepper_1_Microstepping;
float Stepper_1_MaxSpeed = 2600.0;
float Stepper_1_Acceleration = 10000.0;
bool Stepper_1_IsHoming = false;
float Stepper_1_StepsToMove = 0.0;
#define PIN_LIMITSWITCH_1A A1
#define PIN_LIMITSWITCH_1B A2

#define PIN_STEPPER_PUL_2 48
#define PIN_STEPPER_DIR_2 49
AccelStepper Stepper_2(AccelStepper::DRIVER, PIN_STEPPER_PUL_2, PIN_STEPPER_DIR_2);
const int Stepper_2_Microstepping = 400;
const int Stepper_2_GearRatio = 5;
const int Stepper_2_StepsPerRevoloution = Stepper_2_GearRatio * Stepper_2_Microstepping;
float Stepper_2_MaxSpeed = 5000.0;
float Stepper_2_Acceleration = 15000.0;
bool Stepper_2_IsHoming = false;
float Stepper_2_StepsToMove = 0.0;
#define PIN_LIMITSWITCH_2A A3
#define PIN_LIMITSWITCH_2B A4

#define PIN_STEPPER_PUL_3 46
#define PIN_STEPPER_DIR_3 47
AccelStepper Stepper_3(AccelStepper::DRIVER, PIN_STEPPER_PUL_3, PIN_STEPPER_DIR_3);
const int Stepper_3_Microstepping = 400;
const int Stepper_3_GearRatio = 1;
const int Stepper_3_StepsPerRevoloution = Stepper_3_GearRatio * Stepper_3_Microstepping;
float Stepper_3_MaxSpeed = 5000.0;
float Stepper_3_Acceleration = 10000.0;
bool Stepper_3_IsHoming = false;
float Stepper_3_StepsToMove = 0.0;
#define PIN_LIMITSWITCH_3A A5


// -----------------------------------------------------------------------------------------------------------------------------
// Function forward declares
// -----------------------------------------------------------------------------------------------------------------------------
void SetAxisAngle(int InAxisID, float InAngle);
float GetAxisAngle(int InAxisID);

void runActiveSteppers();
bool isAnyMotorMoving();
void waitForMovementsComplete();

void RunHomingSequence(int InStepperIndex, bool InHomeAll = false);
void HomeStepper_0();
void HomeStepper_1();
void HomeStepper_2();
void HomeStepper_3();
void HomeStepper_4();

void RunTest(int InTestID);


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
  pinMode(PIN_LIMITSWITCH_2B, INPUT_PULLUP);
  pinMode(PIN_LIMITSWITCH_3A, INPUT_PULLUP);

  // Init Stepper settings
  Stepper_0.setMaxSpeed(Stepper_0_MaxSpeed);
  Stepper_0.setAcceleration(Stepper_0_Acceleration);
  Stepper_1.setMaxSpeed(Stepper_1_MaxSpeed);
  Stepper_1.setAcceleration(Stepper_1_Acceleration);
  Stepper_2.setMaxSpeed(Stepper_2_MaxSpeed);
  Stepper_2.setAcceleration(Stepper_2_Acceleration);
  Stepper_3.setMaxSpeed(Stepper_3_MaxSpeed);
  Stepper_3.setAcceleration(Stepper_3_Acceleration);

}


// =============================================================================================================================
// Loop
// =============================================================================================================================

void loop() 
{
  // ---- TESTING ---- (Limit Switches)
    if (digitalRead(PIN_LIMITSWITCH_0A) == LOW){ Serial.println("Limit reached: 0A");}
    if (digitalRead(PIN_LIMITSWITCH_1A) == LOW){ Serial.println("Limit reached: 1A");}
    if (digitalRead(PIN_LIMITSWITCH_1B) == LOW){ Serial.println("Limit reached: 1B");}
    if (digitalRead(PIN_LIMITSWITCH_2A) == LOW){ Serial.println("Limit reached: 2A");}
    if (digitalRead(PIN_LIMITSWITCH_2B) == LOW){ Serial.println("Limit reached: 2B");}
    if (digitalRead(PIN_LIMITSWITCH_3A) == LOW){ Serial.println("Limit reached: 3A");}

  runActiveSteppers();
  
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

// -----------------------------------------------------------------------------------------------------------------------------
// Axis Angle
// -----------------------------------------------------------------------------------------------------------------------------
void SetAxisAngle(int InAxisID, float InAngle)
{
  return;
}

float GetAxisAngle(int InAxisID)
{
  return;
}

// -----------------------------------------------------------------------------------------------------------------------------
// Movement
// -----------------------------------------------------------------------------------------------------------------------------

// Check if limit switch is hit for a specific stepper
bool isLimitSwitchHit(int stepperIndex) {
    if (stepperIndex == 0 && !Stepper_0_IsHoming) {
        if (digitalRead(PIN_LIMITSWITCH_0A) == LOW) {
            Stepper_0.stop();
            Stepper_0.setCurrentPosition(Stepper_0.currentPosition());
            Stepper_0.moveTo(Stepper_0.currentPosition());
            Serial.println("Hall effect sensor hit for Stepper 0");
            return true;
        }
    }
    else if (stepperIndex == 1 && !Stepper_1_IsHoming) {
        if (digitalRead(PIN_LIMITSWITCH_1A) == LOW || digitalRead(PIN_LIMITSWITCH_1B) == LOW) {
            Stepper_1.stop();
            Stepper_1.setCurrentPosition(Stepper_1.currentPosition());
            Stepper_1.moveTo(Stepper_1.currentPosition());
            Serial.println("Limit switch hit for Stepper 1");
            return true;
        }
    }
    else if (stepperIndex == 2 && !Stepper_2_IsHoming) {
        if (digitalRead(PIN_LIMITSWITCH_2A) == LOW || digitalRead(PIN_LIMITSWITCH_2B) == LOW) {
            Stepper_2.stop();
            Stepper_2.setCurrentPosition(Stepper_2.currentPosition());
            Stepper_2.moveTo(Stepper_2.currentPosition());
            Serial.println("Limit switch hit for Stepper 2");
            return true;
        }
    }
    else if (stepperIndex == 3 && !Stepper_3_IsHoming) {
        if (digitalRead(PIN_LIMITSWITCH_3A) == LOW ) {
            Stepper_3.stop();
            Stepper_3.setCurrentPosition(Stepper_3.currentPosition());
            Stepper_3.moveTo(Stepper_3.currentPosition());
            Serial.println("Hall effect sensor hit for Stepper 3");
            return true;
        }
    }
    return false;
}

// Safe movement function using steps
void SafeMoveSteps(int stepperIndex, float steps) {
    if (isLimitSwitchHit(stepperIndex)) return;

    switch (stepperIndex) {
        case 0:
            Stepper_0_StepsToMove = steps;
            Stepper_0.move(steps);
            break;
        case 1:
            Stepper_1_StepsToMove = steps;
            Stepper_1.move(steps);
            break;
        case 2:
            Stepper_2_StepsToMove = steps;
            Stepper_2.move(steps);
            break;
        case 3:
            Stepper_3_StepsToMove = steps;
            Stepper_3.move(steps);
            break;
    }
}

// Safe movement function using degrees
void SafeMoveDegrees(int stepperIndex, float degrees) {
    long steps;
    switch (stepperIndex) {
        case 0:
            steps = degreesToSteps(degrees, Stepper_0_StepsPerRevoloution);
            break;
        case 1:
            steps = degreesToSteps(degrees, Stepper_1_StepsPerRevoloution);
            break;
        case 2:
            steps = degreesToSteps(degrees, Stepper_2_StepsPerRevoloution);
            break;
        case 3:
            steps = degreesToSteps(degrees, Stepper_3_StepsPerRevoloution);
            break;
        default:
            Serial.println("Invalid stepper index");
            return;
    }
    SafeMoveSteps(stepperIndex, steps);
}

void runActiveSteppers() {
    if (Stepper_0_StepsToMove != 0.0) 
    {
        if (isLimitSwitchHit(0)) 
        {
          Stepper_0_StepsToMove = 0.0;
        }
        else
        {
          Stepper_0.run();
        }
    }

    if (Stepper_1_StepsToMove != 0.0) 
    {
        if (isLimitSwitchHit(1)) 
        {
          Stepper_1_StepsToMove = 0.0;
        }
        else
        {
          Stepper_1.run();
        }
    }

    if (Stepper_2_StepsToMove != 0.0) 
    {
        if (isLimitSwitchHit(2)) 
        {
          Stepper_2_StepsToMove = 0.0;
        }
        else
        {
          Stepper_2.run();
        }
    }

    if (Stepper_3_StepsToMove != 0.0) 
    {
        if (isLimitSwitchHit(3)) 
        {
          Stepper_3_StepsToMove = 0.0;
        }
        else
        {
          Stepper_3.run();
        }
    }
}

bool isMotorMoving(int stepperIndex) {
    switch (stepperIndex) {
        case 0:
            return Stepper_0.distanceToGo() != 0;
        case 1:
            return Stepper_1.distanceToGo() != 0;
        case 2:
            return Stepper_2.distanceToGo() != 0;
        case 3:
            return Stepper_3.distanceToGo() != 0;
        default:
            return false;
    }
}

void waitForMotors(int InMotors[], int InArraySize) {
    bool anyMotorMoving;
    do {
        anyMotorMoving = false;
        for(int i = 0; i < InArraySize; i++) {
            if(isMotorMoving(InMotors[i])) {
                anyMotorMoving = true;
            }
        }
        runActiveSteppers();
    } while(anyMotorMoving);
}

// Function to calculate the steps needed for a given amount of degrees
int degreesToSteps(float degrees, int stepsPerRevolution) {
  return (degrees / 360.0) * stepsPerRevolution;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Homing
// -----------------------------------------------------------------------------------------------------------------------------
void RunHomingSequence(int InStepperIndex, bool InHomeAll = false)
{
    // If InHomeAll is true or InStepperIndex is -1, home all steppers in sequence
    if (InHomeAll || InStepperIndex == -1) {
        Serial.println("Homing all steppers.");
        HomeStepper_4();
        HomeStepper_3();
        HomeStepper_2();
        HomeStepper_1();
        // HomeStepper_0();
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

            case 3:
                HomeStepper_3();
                break;

            case 4:
                HomeStepper_4();
                break;

            default:
                Serial.println("Invalid stepper index provided.");
                break;
        }
    }
}

void HomeStepper_0()
{
  Serial.println("Homing Stepper_0.");
  Stepper_0_IsHoming = true;
  float HomingSpeed = 2000.0;
  Stepper_0.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
    Stepper_0.runSpeed();
    if (digitalRead(PIN_LIMITSWITCH_0A) == LOW)
    {   
      Stepper_0.stop();
      DegreesToHomePos = -155;
      Serial.println("Homing Stepper_0: found limit A");
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
  Stepper_0_IsHoming = false;
  Serial.println("Completed: Stepper_0 homing.");
  return;
}

void HomeStepper_1()
{
  Serial.println("Homing Stepper_1.");

  Stepper_1_IsHoming = true;
  float HomingSpeed = 3000.0;
  Stepper_1.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
      Stepper_1.runSpeed();
      if (digitalRead(PIN_LIMITSWITCH_1A) == LOW)
      {   
        Stepper_1.stop();
        DegreesToHomePos = -97;
        Serial.println("Homing Stepper_1: found limit A");
        break;
      }
      if (digitalRead(PIN_LIMITSWITCH_1B) == LOW)
      {   
        Stepper_1.stop();
        DegreesToHomePos = 102;
        Serial.println("Homing Stepper_1: found limit B");
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
  Stepper_1_IsHoming = false;
  Serial.println("Completed: Stepper_1 homing.");
  return;
}

void HomeStepper_2()
{
   Serial.println("Homing Stepper_2.");

       // Check if Stepper 1 is at a limit
    if (digitalRead(PIN_LIMITSWITCH_1A) == LOW || digitalRead(PIN_LIMITSWITCH_1B) == LOW) {
        Serial.println("Stepper_1 at limit; homing Stepper_1 first.");
        HomeStepper_1(); // Home Stepper 1 before proceeding with Stepper 2
    }

   Stepper_2_IsHoming = true;
   float HomingSpeed = 1500.0;
   Stepper_2.setSpeed(HomingSpeed);
   int DegreesToHomePos = 0;
   while (true)
   {
      Stepper_2.runSpeed();
      if (digitalRead(PIN_LIMITSWITCH_2A) == LOW)
      {
          Stepper_2.stop();
          DegreesToHomePos = -133;
          Serial.println("Homing Stepper_2: found limit A");
          break;
      }
      if (digitalRead(PIN_LIMITSWITCH_2B) == LOW)
      {
          Stepper_2.stop();
          DegreesToHomePos = 133;
          Serial.println("Homing Stepper_2: found limit B");
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
   Stepper_2_IsHoming = false;
   Serial.println("Completed: Stepper_2 homing.");
   return;
}

void HomeStepper_3()
{
  Serial.println("Homing Stepper_3.");
  Stepper_3_IsHoming = true;
  float HomingSpeed = 2000.0;
  Stepper_3.setSpeed(HomingSpeed);

  int DegreesToHomePos = 0;
  while (true)
  {
      Stepper_3.runSpeed();
      if (digitalRead(PIN_LIMITSWITCH_3A) == LOW)
      {   
        Stepper_3.stop();
        DegreesToHomePos = -170;
        Serial.println("Homing Stepper_3: found limit A");
        break;
      }
  }
  // Then move to the axis center location and assume that as positon 0.
  Stepper_3.move(degreesToSteps(DegreesToHomePos, Stepper_3_StepsPerRevoloution));
  while (Stepper_3.distanceToGo() != 0) 
  {
    Stepper_3.run();
  }
  Stepper_3.setCurrentPosition(0);
  Stepper_3.setSpeed(Stepper_3_MaxSpeed);
  Stepper_3_IsHoming = false;
  Serial.println("Completed: Stepper_3 homing.");
  return;
}

void HomeStepper_4()
{
  Serial.println("Homing Stepper_4.");
  Serial.println("Completed: Stepper_4 homing.");
  return;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Testing Mode
// -----------------------------------------------------------------------------------------------------------------------------
void RunTest(int InTestID)
{
    switch (InTestID) {
      case 0:
        Serial.println("Running test for Stepper 0.");
        SafeMoveDegrees(0, 45);
        SafeMoveDegrees(0, -45);
        Serial.println("Test for Stepper 0 completed.");
        break;

      case 1:
        Serial.println("Running test for Stepper 1.");
        SafeMoveDegrees(1, 45);
        SafeMoveDegrees(1, -45);
        Serial.println("Test for Stepper 1 completed.");
        break;

      case 2:
        Serial.println("Running test for Stepper 2.");
        SafeMoveDegrees(2, 45);
        SafeMoveDegrees(2, -45);
        Serial.println("Test for Stepper 2 completed.");
        break;

      case 3:
        Serial.println("Running test for Stepper 3.");
        SafeMoveDegrees(3, 45);
        SafeMoveDegrees(3, -45);
        Serial.println("Test for Stepper 3 completed.");
        break;

      case 4:
        Serial.println("Running test for Stepper 4.");
        Serial.println("Test for Stepper 4 completed.");
        break;

      case 5:
        Serial.println("Running test ID 4 - Jogging 1");

        int Motors_Jog1[] = {1, 2, 3};
        SafeMoveDegrees(1, 45);
        SafeMoveDegrees(2, 45);
        SafeMoveDegrees(3, 45);
        waitForMotors(Motors_Jog1, 3);
        
        SafeMoveDegrees(1, -45);
        SafeMoveDegrees(2, -45);
        SafeMoveDegrees(3, -45);
        waitForMotors(Motors_Jog1, 3);
        
        SafeMoveDegrees(1, 65);
        SafeMoveDegrees(2, -85);
        SafeMoveDegrees(3, 35);
        waitForMotors(Motors_Jog1, 3);


        SafeMoveDegrees(1, 15);
        SafeMoveDegrees(2, -55);
        SafeMoveDegrees(3, 25);
        waitForMotors(Motors_Jog1, 3);

        SafeMoveDegrees(1, -65);
        SafeMoveDegrees(2, 85);
        SafeMoveDegrees(3, -35);
        waitForMotors(Motors_Jog1, 3);

        SafeMoveDegrees(1, -15);
        SafeMoveDegrees(2, 55);
        SafeMoveDegrees(3, -25);
        waitForMotors(Motors_Jog1, 3);

        SafeMoveDegrees(1, 5);
        SafeMoveDegrees(2, 55);
        SafeMoveDegrees(3, -25);
        waitForMotors(Motors_Jog1, 3);

        SafeMoveDegrees(1, -5);
        SafeMoveDegrees(2, -55);
        SafeMoveDegrees(3, 25);
        waitForMotors(Motors_Jog1, 3);

        RunHomingSequence(0, true);


        Serial.println("Completed test ID 5 - Jogging 1");
        break;

      case 6:
        Serial.println("Running test ID 6 - Jogging 2");
        Serial.println("Completed test ID 6 - Jogging 2");
        break;

      default:
        Serial.println("Invalid stepper index provided.");
        break;
    }
    return;
}

