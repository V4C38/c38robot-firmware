#include <AccelStepper.h>

// -----------------------------------------------------------------------------------------------------------------------------
// Stepper Motor Data
// Explain
// -----------------------------------------------------------------------------------------------------------------------------
class StepperMotor 
{
public:
    AccelStepper stepper;
    int microstepping;
    int gearRatio;
    int stepsPerRevolution;
    float maxSpeed;
    float acceleration;

    bool isHoming;
    float stepsToMove;

    int* limitPositions;
    int numLimitPositions;
    uint8_t* digitalSensorPins;
    int numDigitalSensors;
    
    // Constructor
    StepperMotor(int stepPin, int dirPin, int microstepping, int gearRatio, float maxSpeed, float acceleration,
                 uint8_t* digitalSensorPins = nullptr, int numDigitalSensors = 0,
                 int* limitPositions = nullptr, int numLimitPositions = 0)
        : stepper(AccelStepper::DRIVER, stepPin, dirPin),
          microstepping(microstepping),
          gearRatio(gearRatio),
          maxSpeed(maxSpeed),
          acceleration(acceleration),
          numDigitalSensors(numDigitalSensors),
          numLimitPositions(numLimitPositions)
    {
        stepsPerRevolution = microstepping * gearRatio;
        isHoming = false;
        stepsToMove = 0.0;

        // Allocate memory for digital sensor pins
        if (numDigitalSensors > 0 && digitalSensorPins != nullptr)
        {
            this->digitalSensorPins = new uint8_t[numDigitalSensors];
            for (int i = 0; i < numDigitalSensors; ++i)
            {
                this->digitalSensorPins[i] = digitalSensorPins[i];
            }
        }
        else
        {
            this->digitalSensorPins = nullptr;
        }

        // Allocate memory for limit positions
        if (numLimitPositions > 0 && limitPositions != nullptr)
        {
            this->limitPositions = new int[numLimitPositions];
            for (int i = 0; i < numLimitPositions; ++i)
            {
                this->limitPositions[i] = limitPositions[i];
            }
        }
        else
        {
            this->limitPositions = nullptr;
        }
    }

    // Destructor to clean up dynamically allocated memory
    ~StepperMotor() 
    {
        delete[] digitalSensorPins;
        delete[] limitPositions;
    }
};

// -----------------------------------------------------------------------------------------------------------------------------
// Stepper Definitions
// Explain
// -----------------------------------------------------------------------------------------------------------------------------
#define NUM_STEPPERS 6
StepperMotor steppers[NUM_STEPPERS] = 
{
    // Stepper Axis 0
    StepperMotor(
        52,             // stepPin
        53,             // dirPin
        800,            // microstepping
        1,              // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){50, 51}, 2,    // Digital Sensor Pins
        (int[]){-45, 45}, 2   // Limit positions
    ),

    // Stepper Axis 1
    StepperMotor(
        48,             // stepPin
        49,             // dirPin
        800,            // microstepping
        1,             // gearRatio
        5000.0,        // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){46, 47}, 2,    // Digital Sensor Pins
        (int[]){-45, 45}, 2   // Limit positions
    ),

    // Stepper Axis 2
    StepperMotor(
        44,             // stepPin
        45,             // dirPin
        800,            // microstepping
        1,              // gearRatio
        5000.0,        // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){42, 43}, 2,    // Digital Sensor Pins
        (int[]){-45, 45}, 2   // Limit positions
    ),

    // Stepper Axis 3
    StepperMotor(
        40,             // stepPin
        41,             // dirPin
        1600,            // microstepping
        5,              // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){38, 39}, 2,    // Digital Sensor Pins
        (int[]){-45, 45}, 2   // Limit positions
    ),

    // Stepper Axis 4
    StepperMotor(
        36,             // stepPin
        37,             // dirPin
        800,           // microstepping
        26.85,          // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){34, 35}, 2,    // Digital Sensor Pins
        (int[]){-45, 45}, 2   // Limit positions
    ),

    // Stepper Axis 5
    StepperMotor(
        32,             // stepPin
        33,             // dirPin
        800,           // microstepping
        50,             // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){30, 31}, 2,    // Digital Sensor Pins
        (int[]){-25, 25}, 2   // Limit positions
    )
};

// -----------------------------------------------------------------------------------------------------------------------------
// Function forward declares
// -----------------------------------------------------------------------------------------------------------------------------
void SetAxisAngle(int InAxisID, float InAngle);
float GetAxisAngle(int InAxisID);
float GetAxisAngleInRadians(int InAxisID);

void runActiveSteppers();
bool isAnyMotorMoving();
void waitForMovementsComplete();
void AbortAllCommands();

void ExecuteHomingCommand(int InAxisIndex, bool InHomeAll = false);
void HomeAxis(int InAxisIndex);

void ExecuteTestCommand(int InTestID);
void DefaultTest(int InAxis);

// =============================================================================================================================
// Setup
// =============================================================================================================================
void setup() 
{
    Serial.begin(9600);

    // Initialize each stepper motor
    for (int i = 0; i < NUM_STEPPERS; i++) 
    {
        StepperMotor &motor = steppers[i];

        // Set digital sensor pins as INPUT_PULLUP (if there are any)
        for (int j = 0; j < motor.numDigitalSensors; j++) 
        {
            pinMode(motor.digitalSensorPins[j], INPUT_PULLUP);
        }

        // Set maximum speed and acceleration for each stepper
        motor.stepper.setMaxSpeed(motor.maxSpeed);
        motor.stepper.setAcceleration(motor.acceleration);
    }
}

// =============================================================================================================================
// Loop
// Explain
// =============================================================================================================================
void loop() 
{

    // Explain
    for (int i = 0; i < NUM_STEPPERS; i++) 
    {
        isStepperAtLimit(i);
    }
    runActiveSteppers();

    // Read Serial Commands
    if (Serial.available()) 
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.print("Received command: ");
        Serial.println(command);

        if (command == "EmergencyStop") 
        {
            AbortAllCommands();
        }
        else if (command.startsWith("RunTest")) 
        {
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex != -1) 
            {
                String argument = command.substring(spaceIndex + 1);
                argument.trim();
                int testID = argument.toInt();
                ExecuteTestCommand(testID);
            } 
            else 
            {
                Serial.println("Error: Invalid RunTest command format.");
            }
        }
        else if (command.startsWith("RunHomingSequence")) 
        {
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex != -1) 
            {
                String argument = command.substring(spaceIndex + 1);
                argument.trim();

                int stepperIndex = argument.toInt();
                bool homeAll = (stepperIndex == -1);

                ExecuteHomingCommand(stepperIndex, homeAll);
            } 
            else 
            {
                Serial.println("Error: Invalid RunHomingSequence command format.");
            }
        }

        else 
        {
            Serial.println("Error: Unknown command received.");
        }
    }
}

// =============================================================================================================================
// API
// Explain
// =============================================================================================================================

// Explain
void SetAxisAngle(int InAxisID, float InAngle) 
{
    if (InAxisID < 0 || InAxisID >= NUM_STEPPERS) 
    {
        Serial.println("Invalid axis ID");
        return;
    }

    StepperMotor &motor = steppers[InAxisID];
    float currentAngle = GetAxisAngle(InAxisID);
    float angleDifference = InAngle - currentAngle;

    // Normalize the angle difference to be within -180 to +180 degrees
    while (angleDifference > 180.0) 
        angleDifference -= 360.0;
    while (angleDifference < -180.0) 
        angleDifference += 360.0;

    // Convert angle difference to steps and set stepsToMove
    long steps = degreesToSteps(angleDifference, motor.stepsPerRevolution);
    motor.stepsToMove = steps;
    motor.stepper.move(steps);
}

void SetAxisAngleInRadians(int InAxisID, float InAngleRadians)
{
    float angleDegrees = InAngleRadians * (180.0 / 3.14159265358979323846);
    SetAxisAngle(InAxisID, angleDegrees);
}

// Explain
float GetAxisAngle(int InAxisID) 
{
    if (InAxisID < 0 || InAxisID >= NUM_STEPPERS) 
    {
        Serial.println("Invalid axis ID");
        return 0.0;
    }
    StepperMotor &motor = steppers[InAxisID];
    float angle = (float)motor.stepper.currentPosition() / motor.stepsPerRevolution * 360.0;

    // Normalize angle to -180 to +180
    if (angle > 180.0) angle -= 360.0;
    else if (angle < -180.0) angle += 360.0;

    return angle;
}

float GetAxisAngleInRadians(int InAxisID)
{
    float angleDegrees = GetAxisAngle(InAxisID);
    return angleDegrees * (3.14159265358979323846 / 180.0);
}

// Explain
void AbortAllCommands() 
{
    Serial.println("Aborting all commands.");

    for (int i = 0; i < NUM_STEPPERS; i++) 
    {
        StepperMotor &motor = steppers[i];
        
        // Stop the motor immediately
        motor.stepper.stop();
        
        // Reset any movement tracking variables
        motor.stepsToMove = 0.0;
        motor.isHoming = false;
        
        // Set the motor's current position as the target to prevent further movement
        motor.stepper.setCurrentPosition(motor.stepper.currentPosition());
    }

    Serial.println("All commands aborted - all motor positions have been reset, homing sequence required.");
}

// -----------------------------------------------------------------------------------------------------------------------------
// Step movement
// -----------------------------------------------------------------------------------------------------------------------------

// Explain
void runActiveSteppers() 
{
    for (int i = 0; i < NUM_STEPPERS; i++) 
    {
        StepperMotor &motor = steppers[i];

        if (motor.stepsToMove != 0.0) 
        {
            if (isStepperAtLimit(i)) 
            {
                motor.stepsToMove = 0.0;
            } 
            else 
            {
                motor.stepper.run();
            }
        }
    }
}

// Explain
void SafeMoveSteps(int stepperIndex, float steps) 
{
    if (stepperIndex < 0 || stepperIndex >= NUM_STEPPERS) return;
    StepperMotor &motor = steppers[stepperIndex];
    if (isStepperAtLimit(stepperIndex)) return;

    motor.stepsToMove = steps;
    motor.stepper.move(steps);
}

// Explain
void SafeMoveDegrees(int stepperIndex, float degrees) 
{
    StepperMotor &motor = steppers[stepperIndex];
    long steps = degreesToSteps(degrees, motor.stepsPerRevolution);
    SafeMoveSteps(stepperIndex, steps);
}

bool isMotorMoving(int stepperIndex) 
{
    if (stepperIndex < 0 || stepperIndex >= NUM_STEPPERS) return false;
    StepperMotor &motor = steppers[stepperIndex];
    return motor.stepper.distanceToGo() != 0;
}

// Explain
void waitForMotors(int InMotors[], int InArraySize) 
{
    bool anyMotorMoving;
    do 
    {
        anyMotorMoving = false;
        for (int i = 0; i < InArraySize; i++) 
        {
            if (isMotorMoving(InMotors[i])) 
            {
                anyMotorMoving = true;
            }
        }
        runActiveSteppers();
    } 
    while (anyMotorMoving);
}

bool isStepperAtLimit(int stepperIndex) 
{
    if (stepperIndex < 0 || stepperIndex >= NUM_STEPPERS) return false;
    StepperMotor &motor = steppers[stepperIndex];

    if (!motor.isHoming) 
    {
        for (int i = 0; i < motor.numDigitalSensors; i++) 
        {
            if (digitalRead(motor.digitalSensorPins[i]) == LOW) 
            {
                motor.stepper.stop();
                motor.stepsToMove = 0.0;
                Serial.print("Digital limit switch hit for Stepper ");
                Serial.println(stepperIndex);
                return true;
            }
        }
    }
    return false;
}

int degreesToSteps(float degrees, int stepsPerRevolution) 
{
    return (int)((degrees / 360.0) * stepsPerRevolution);
}

// -----------------------------------------------------------------------------------------------------------------------------
// Homing Functions
// -----------------------------------------------------------------------------------------------------------------------------

// Explain
void ExecuteHomingCommand(int InAxisIndex, bool InHomeAll)
{
    if (InHomeAll || InAxisIndex == -1) 
    {
        Serial.println("Homing all Axes.");
        HomeAxis(0);
        HomeAxis(1);
        HomeAxis(2);
        HomeAxis(3);
        HomeAxis(4);
        HomeAxis(5);
        Serial.println("All Axes homed.");
    } 
    else 
    {
        if (InAxisIndex < -1 || InAxisIndex > NUM_STEPPERS - 1)
        {
          Serial.println("Invalid Axis index.");
          return;
        }
        HomeAxis(InAxisIndex);
    }
}

// HomeAxis: Moves the motor until a limit switch is triggered or timeout occurs.
// If limit is hit, the motor moves to it´s origin (limit is position defined in the stepper class)
void HomeAxis(int InAxisIndex) 
{
    Serial.println("Homing Axis " + String(InAxisIndex) + ": starting...");

    StepperMotor &motor = steppers[InAxisIndex];
    motor.isHoming = true;
    bool foundLimit = false;
    motor.stepper.setSpeed(motor.maxSpeed / 2);

    int DegreesToHomePos = 0;

    unsigned long startTime = millis();
    const unsigned long timeout = 15000;

    while (true) 
    {
        motor.stepper.runSpeed();

        // Check if any sensor is triggered
        for (int i = 0; i < motor.numDigitalSensors; i++) 
        {
            if (digitalRead(motor.digitalSensorPins[i]) == LOW) 
            {
                if (i < motor.numLimitPositions) 
                {
                    DegreesToHomePos = motor.limitPositions[i];
                }
                Serial.println("Homing Axis " + String(InAxisIndex) + ": found limit index " + String(i));
                foundLimit = true;
                break;
            }
        }
        if (foundLimit) 
        {
          motor.stepper.stop();
          break;
        }
        
        // Check for timeout
        if (millis() - startTime > timeout) 
        {
            Serial.println("Homing Axis " + String(InAxisIndex) + ": Error - timeout after " + String(timeout / 1000.0, 2) + " seconds.");
            motor.stepper.stop();
            motor.isHoming = false;
            return;
        }
    }

    motor.stepper.move(degreesToSteps(DegreesToHomePos, motor.stepsPerRevolution));
    while (motor.stepper.distanceToGo() != 0) 
    {
        motor.stepper.run();
    }
    motor.stepper.setCurrentPosition(0);
    motor.stepper.setSpeed(motor.maxSpeed);
    motor.isHoming = false;

    Serial.println("Homing Axis " + String(InAxisIndex) + ": completed.");
}

// -----------------------------------------------------------------------------------------------------------------------------
// Testing Mode
// -----------------------------------------------------------------------------------------------------------------------------

// Explain
void ExecuteTestCommand(int InTestID) 
{
    Serial.print("Requested TestID: ");
    Serial.println(InTestID);

    switch (InTestID) 
    {
        case 6:
        {
            Serial.println("Running test ID 6 - Jogging 1");
            int Motors_Jog1[] = {3, 4, 5};
            
            SetAxisAngle(3, 25.0);
            SetAxisAngle(4, 45.0);
            SetAxisAngle(5, 45.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(3, 35.0);
            SetAxisAngle(4, -60.0);
            SetAxisAngle(5, -45.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(3, 55.0);
            SetAxisAngle(4, -105.0);
            SetAxisAngle(5, -30.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, 0.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, 0.0);
            waitForMotors(Motors_Jog1, 3);

            Serial.println("Completed test ID 6 - Jogging 1");
            break;
        }

        case 7:
        {
            Serial.println("Running test ID 7 - Jogging 2");
            int Motors_Jog2[] = {1, 2, 3};

            SetAxisAngle(1, 45.0);
            SetAxisAngle(2, 90.0);
            SetAxisAngle(3, 90.0);
            waitForMotors(Motors_Jog2, 3);

            delay(1000);

            SetAxisAngle(1, -45.0);
            SetAxisAngle(2, -90.0);
            SetAxisAngle(3, -90.0);
            waitForMotors(Motors_Jog2, 3);

            delay(1000);

            SetAxisAngle(1, 0.0);
            SetAxisAngle(2, 0.0);
            SetAxisAngle(3, 0.0);
            waitForMotors(Motors_Jog2, 3);

            Serial.println("Completed test ID 7 - Jogging 2");
            break;
        }

        default:
            if (InTestID < 0 || InTestID > 7) 
            {
              Serial.println("Invalid TestID " + String(InTestID));
            }
            else
            {
              DefaultTest(InTestID);
            }
            break;
    }
}

// Explain
void DefaultTest(int InAxis)
{
    Serial.println("Test Axis " + String(InAxis) + " started...");
    int testMotors[] = {InAxis};

    SetAxisAngle(InAxis, 45.0);
    waitForMotors(testMotors, 1);
    SetAxisAngle(InAxis, 0.0);
    waitForMotors(testMotors, 1);
    Serial.println("Running test for Axis " + String(InAxis) + " completed.");
}
