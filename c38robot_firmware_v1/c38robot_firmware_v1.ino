
#include <AccelStepper.h>

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
    
    uint8_t* analogSensorPins;
    uint8_t* digitalSensorPins;
    int numAnalogSensors;
    int numDigitalSensors;

    // Constructor
    StepperMotor(int stepPin, int dirPin, int microstepping, int gearRatio, float maxSpeed, float acceleration,
                 uint8_t* analogSensorPins = nullptr, int numAnalogSensors = 0,
                 uint8_t* digitalSensorPins = nullptr, int numDigitalSensors = 0)
        : stepper(AccelStepper::DRIVER, stepPin, dirPin),
          microstepping(microstepping),
          gearRatio(gearRatio),
          maxSpeed(maxSpeed),
          acceleration(acceleration),
          numAnalogSensors(numAnalogSensors),
          numDigitalSensors(numDigitalSensors)
    {
        stepsPerRevolution = microstepping * gearRatio;
        isHoming = false;
        stepsToMove = 0.0;

        // Allocate memory for analog sensor pins
        if (numAnalogSensors > 0 && analogSensorPins != nullptr)
        {
            this->analogSensorPins = new uint8_t[numAnalogSensors];
            for (int i = 0; i < numAnalogSensors; ++i)
            {
                this->analogSensorPins[i] = analogSensorPins[i];
            }
        }
        else
        {
            this->analogSensorPins = nullptr;
        }

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
    }

    // Destructor to clean up dynamically allocated memory
    ~StepperMotor() 
    {
        delete[] analogSensorPins;
        delete[] digitalSensorPins;
    }
};

// -----------------------------------------------------------------------------------------------------------------------------
// Stepper Definitions
// -----------------------------------------------------------------------------------------------------------------------------
#define NUM_STEPPERS 5
StepperMotor steppers[NUM_STEPPERS] = 
{
    // Stepper 0 
    StepperMotor(
        52,             // stepPin
        53,             // dirPin
        400,            // microstepping
        1,              // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){A0}, 1,    // Analog Limit Switch pins
        nullptr, 0      // No digital sensors
    ),

    // Stepper 1
    StepperMotor(
        50,             // stepPin
        51,             // dirPin
        400,            // microstepping
        27,             // gearRatio
        10000.0,        // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){A1, A2}, 2, // Analog Limit Switch pins
        nullptr, 0      // No digital sensors
    ),

    // Stepper 2
    StepperMotor(
        48,             // stepPin
        49,             // dirPin
        400,            // microstepping
        5,              // gearRatio
        10000.0,        // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){A3, A4}, 2,    // Analog Limit Switch pins
        nullptr, 0      // No digital sensors
    ),

    // Stepper 3
    StepperMotor(
        46,             // stepPin
        47,             // dirPin
        400,            // microstepping
        1,              // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        (uint8_t[]){A5}, 1,    // Analog Limit Switch pins
        nullptr, 0      // No digital sensors
    ),

    // Stepper 4
    StepperMotor(
        44,             // stepPin
        45,             // dirPin
        1600,           // microstepping
        1,              // gearRatio
        5000.0,         // maxSpeed
        10000.0,        // acceleration
        nullptr, 0,     // Analog Limit Switch pins
        nullptr, 0      // No digital sensors
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

    // Initialize each stepper motor
    for (int i = 0; i < NUM_STEPPERS; i++) 
    {
        StepperMotor &motor = steppers[i];

        // Set analog sensor pins as INPUT_PULLUP
        for (int j = 0; j < motor.numAnalogSensors; j++) 
        {
            pinMode(motor.analogSensorPins[j], INPUT_PULLUP);
        }

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
// =============================================================================================================================
void loop() 
{
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
        if (command == "Abort") 
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
                RunTest(testID);
            } 
            else 
            {
                Serial.println("Invalid RunTest command format.");
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

                RunHomingSequence(stepperIndex, homeAll);
            } 
            else 
            {
                Serial.println("Invalid RunHomingSequence command format.");
            }
        } 
        else 
        {
            Serial.println("Unknown command received.");
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------
// Axis Angle API
// -----------------------------------------------------------------------------------------------------------------------------
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

    Serial.println("All commands aborted.");
}

// -----------------------------------------------------------------------------------------------------------------------------
// Step movement
// -----------------------------------------------------------------------------------------------------------------------------
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

void SafeMoveSteps(int stepperIndex, float steps) 
{
    if (stepperIndex < 0 || stepperIndex >= NUM_STEPPERS) return;
    StepperMotor &motor = steppers[stepperIndex];
    if (isStepperAtLimit(stepperIndex)) return;

    motor.stepsToMove = steps;
    motor.stepper.move(steps);
}

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
        for (int i = 0; i < motor.numAnalogSensors; i++) 
        {
            if (digitalRead(motor.analogSensorPins[i]) == LOW) 
            {
                motor.stepper.stop();
                motor.stepsToMove = 0.0;
                Serial.print("Analog limit switch hit for Stepper ");
                Serial.println(stepperIndex);
                return true;
            }
        }

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

void RunHomingSequence(int InStepperIndex, bool InHomeAll)
{
    if (InHomeAll || InStepperIndex == -1) 
    {
        Serial.println("Homing all steppers.");
        HomeStepper_0();
        HomeStepper_1();
        HomeStepper_2();
        HomeStepper_3();
        HomeStepper_4();
        Serial.println("All steppers homed.");
    } 
    else 
    {
        switch (InStepperIndex) 
        {
            case 0: HomeStepper_0(); break;
            case 1: HomeStepper_1(); break;
            case 2: HomeStepper_2(); break;
            case 3: HomeStepper_3(); break;
            case 4: HomeStepper_4(); break;
            default: Serial.println("Invalid stepper index."); break;
        }
    }
}

void HomeStepper_0() 
{
    Serial.println("Homing Stepper 0 - Skipped");
    return;

    Serial.println("Homing Stepper 0.");
    StepperMotor &motor = steppers[0];
    motor.isHoming = true;
    float HomingSpeed = motor.maxSpeed / 2;
    motor.stepper.setSpeed(HomingSpeed);

    int DegreesToHomePos = 0;
    while (true) 
    {
        motor.stepper.runSpeed();
        if (digitalRead(motor.analogSensorPins[0]) == LOW) 
        {
            motor.stepper.stop();
            DegreesToHomePos = -155;
            Serial.println("Homing Stepper 0: found limit A");
            break;
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
    Serial.println("Completed: Stepper 0 homing.");
}

void HomeStepper_1() 
{
    Serial.println("Homing Stepper 1.");
    StepperMotor &motor = steppers[1];
    motor.isHoming = true;
    float HomingSpeed = motor.maxSpeed / 3.5;
    motor.stepper.setSpeed(HomingSpeed);

    int DegreesToHomePos = 0;
    while (true) 
    {
        motor.stepper.runSpeed();
        if (digitalRead(motor.analogSensorPins[0]) == LOW) 
        {
            motor.stepper.stop();
            DegreesToHomePos = -97;
            Serial.println("Homing Stepper 1: found limit A");
            break;
        }
        if (digitalRead(motor.analogSensorPins[1]) == LOW) 
        {
            motor.stepper.stop();
            DegreesToHomePos = 102;
            Serial.println("Homing Stepper 1: found limit B");
            break;
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
    Serial.println("Completed: Stepper 1 homing.");
}

// Homing function for Stepper 2
void HomeStepper_2() 
{
    Serial.println("Homing Stepper 2.");
    StepperMotor &motor = steppers[2];
    motor.isHoming = true;
    float HomingSpeed = motor.maxSpeed / 8;
    motor.stepper.setSpeed(HomingSpeed);

    int DegreesToHomePos = 0;
    while (true) 
    {
        motor.stepper.runSpeed();
        if (digitalRead(motor.analogSensorPins[0]) == LOW) 
        {
            motor.stepper.stop();
            DegreesToHomePos = -133;
            Serial.println("Homing Stepper 2: found limit A");
            break;
        }
        if (digitalRead(motor.analogSensorPins[1]) == LOW) 
        {
            motor.stepper.stop();
            DegreesToHomePos = 133;
            Serial.println("Homing Stepper 2: found limit B");
            break;
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
    Serial.println("Completed: Stepper 2 homing.");
}

// Homing function for Stepper 3
void HomeStepper_3() 
{
    Serial.println("Homing Stepper 3.");
    StepperMotor &motor = steppers[3];
    motor.isHoming = true;
    float HomingSpeed = motor.maxSpeed / 2;
    motor.stepper.setSpeed(HomingSpeed);

    int DegreesToHomePos = 0;
    while (true) 
    {
        motor.stepper.runSpeed();
        if (digitalRead(motor.analogSensorPins[0]) == LOW) 
        {
            motor.stepper.stop();
            DegreesToHomePos = -170;
            Serial.println("Homing Stepper 3: found limit A");
            break;
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
    Serial.println("Completed: Stepper 3 homing.");
}

// Homing function for Stepper 4
void HomeStepper_4() 
{
    Serial.println("Homing Stepper 4.");
    Serial.println("Completed: Stepper 4 homing.");
}

// -----------------------------------------------------------------------------------------------------------------------------
// Testing Mode
// -----------------------------------------------------------------------------------------------------------------------------
void RunTest(int InTestID) 
{
    Serial.print("Requested TestID: ");
    Serial.println(InTestID);

    switch (InTestID) 
    {
        case 0:
        {
            Serial.println("Running test for Stepper 0.");
            int Motors_Test_0[] = {0};

            SetAxisAngle(0, 45.0);
            waitForMotors(Motors_Test_0, 1);
            SetAxisAngle(0, 0.0);
            waitForMotors(Motors_Test_0, 1);
            Serial.println("Test for Stepper 0 completed.");
            break;
        }

        case 1:
        {
            Serial.println("Running test for Stepper 1.");
            int Motors_Test_1[] = {1};

            SetAxisAngle(1, 45.0);
            waitForMotors(Motors_Test_1, 1);
            SetAxisAngle(1, 0.0);
            waitForMotors(Motors_Test_1, 1);
            Serial.println("Test for Stepper 1 completed.");
            break;
        }

        case 2:
        {
            Serial.println("Running test for Stepper 2.");
            int Motors_Test_2[] = {2};

            SetAxisAngle(2, 45.0);
            waitForMotors(Motors_Test_2, 1);
            SetAxisAngle(2, 0.0);
            waitForMotors(Motors_Test_2, 1);
            Serial.println("Test for Stepper 2 completed.");
            break;
        }

        case 3:
        {
            Serial.println("Running test for Stepper 3.");
            int Motors_Test_3[] = {3};

            SetAxisAngle(3, 45.0);
            waitForMotors(Motors_Test_3, 1);
            SetAxisAngle(3, 0.0);
            waitForMotors(Motors_Test_3, 1);
            Serial.println("Test for Stepper 3 completed.");
            break;
        }

        case 4:
        {
            Serial.println("Running test for Stepper 4.");
            int Motors_Test_4[] = {4};

            SetAxisAngle(4, 45.0);
            waitForMotors(Motors_Test_4, 1);
            SetAxisAngle(4, 0.0);
            waitForMotors(Motors_Test_4, 1);
            Serial.println("Test for Stepper 4 completed.");
            break;
        }

        case 5:
        {
            Serial.println("Running test ID 4 - Jogging 1");
            int Motors_Jog1[] = {1, 2, 3};
            
            SetAxisAngle(1, 25.0);
            SetAxisAngle(2, 45.0);
            SetAxisAngle(3, 45.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(1, 35.0);
            SetAxisAngle(2, -60.0);
            SetAxisAngle(3, -45.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(1, 55.0);
            SetAxisAngle(2, -105.0);
            SetAxisAngle(3, -30.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(1, 0.0);
            SetAxisAngle(2, 0.0);
            SetAxisAngle(3, 0.0);
            waitForMotors(Motors_Jog1, 3);

            SetAxisAngle(1, 0.0);
            SetAxisAngle(2, 0.0);
            SetAxisAngle(3, 0.0);
            waitForMotors(Motors_Jog1, 3);

            Serial.println("Completed test ID 5 - Jogging 1");
            break;
        }

        case 6:
        {
            Serial.println("Running test ID 6 - Jogging 2");
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

            Serial.println("Completed test ID 6 - Jogging 2");
            break;
        }

        default:
            Serial.println("Invalid stepper index provided.");
            break;
    }
}

