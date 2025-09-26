#include <AccelStepper.h>
#include <ArduinoJson.h>


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
    float stepAngle;
    int stepsPerRevolution;
    float maxSpeed;
    float acceleration;

    bool isHoming;
    float stepsToMove;

    int* limitPositions;
    int numLimitPositions;
    uint8_t* digitalSensorPins;
    int numDigitalSensors;
    bool atLimit;
    int lastLimitIndex;
    
    // Constructor
    StepperMotor(int stepPin, int dirPin, int microstepping, int gearRatio, float stepAngle, 
                float maxSpeed, float acceleration, uint8_t* digitalSensorPins = nullptr, 
                int numDigitalSensors = 0, int* limitPositions = nullptr, int numLimitPositions = 0)
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
        atLimit = false;
        lastLimitIndex = -1;

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
        10000,            // microstepping - this one has 0.094 deg per step so it is x19.14
        19.203,           // gearRatio
        0.094,            // stepAngle
        7500.0,         // maxSpeed
        500.0,        // acceleration
        (uint8_t[]){50, 51}, 2,    // Digital Sensor Pins
        (int[]){-49, 49}, 2   // Limit positions
    ),

    // Stepper Axis 1
    StepperMotor(
        48,             // stepPin
        49,             // dirPin
        800,            // microstepping
        50,             // gearRatio
        1.8,            // stepAngle
        50000.0,        // maxSpeed
        2000.0,        // acceleration
        (uint8_t[]){46, 47}, 2,    // Digital Sensor Pins
        (int[]){-90, 90}, 2   // Limit positions
    ),

    // Stepper Axis 2
    StepperMotor(
        44,             // stepPin
        45,             // dirPin
        400,            // microstepping
        26.85,          // gearRatio
        1.8,            // stepAngle
        25000.0,        // maxSpeed
        2000.0,        // acceleration
        (uint8_t[]){43, 42}, 2,    // Digital Sensor Pins
        (int[]){-110, 100}, 2   // Limit positions
    ),

    // Stepper Axis 3
    StepperMotor(
        32,             // stepPin
        33,             // dirPin
        800,            // microstepping
        5,              // gearRatio
        1.8,            // stepAngle
        10000.0,         // maxSpeed
        1500.0,        // acceleration
        (uint8_t[]){35, 34}, 2,    // Digital Sensor Pins
        (int[]){-110, 110}, 2   // Limit positions
    ),

    // Stepper Axis 4 
    StepperMotor(
        36,             // stepPin
        37,             // dirPin
        2800,           // microstepping
        13.73,          // gearRatio
        0.131,            // stepAngle
        15000.0,         // maxSpeed
        1750.0,        // acceleration
        (uint8_t[]){38}, 1,    // Digital Sensor Pins
        (int[]){-90, 90}, 2   // Limit positions
    ),

    // Stepper Axis 5
    StepperMotor(
        40,             // stepPin
        41,             // dirPin
        800,           // microstepping
        1.0,             // gearRatio
        1.8,            // stepAngle
        20000.0,         // maxSpeed
        5000.0,        // acceleration
        (uint8_t[]){}, 0,    // Digital Sensor Pins
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

void handleCommand(const String& commandType, const JsonObject& parameters, const JsonObject& responseFormat, const String& topLevelUuid);
void sendSerialMessage(const char* type, const char* uuid = nullptr, const char* command = nullptr, const char* status = nullptr, const char* message = nullptr, const JsonObject* stateUpdate = nullptr);
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
    Serial.begin(115200);

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
        if (command.length() == 0) return; // Skip empty messages

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, command);
        if (error)
        {
            sendSerialMessage("response", nullptr, "unknown", "error", "Invalid JSON received.");
            return;
        }

        // Process the command
        String commandType = doc["command"] | "";
        String topLevelUuid = doc["uuid"] | "";  // <--- Added
        JsonObject parameters = doc["parameters"];
        JsonObject expectedResponse = doc["expectedResponse"];
        handleCommand(commandType, parameters, expectedResponse, topLevelUuid); // <--- Added arg
    }
}


// =============================================================================================================================
// Command handling
// =============================================================================================================================
void handleCommand(const String& commandType, const JsonObject& parameters, const JsonObject& responseFormat, const String& topLevelUuid)
{
    String embeddedUuid = responseFormat["uuid"] | "";
    String uuid = embeddedUuid.length() > 0 ? embeddedUuid : topLevelUuid;
    String status = "success";
    String message = "";
    StaticJsonDocument<1024> stateUpdateDoc;
    JsonObject stateUpdate = stateUpdateDoc.to<JsonObject>();

    if (commandType == "emergencyStop")
    {
        AbortAllCommands();
        message = "Emergency stop executed.";
    }
    else if (commandType == "runTest")
    {
        int testID = parameters["testIndex"] | -1;
        if (testID != -1)
        {
            ExecuteTestCommand(testID);
            message = "Test " + String(testID) + " completed.";
        }
        else
        {
            status = "error";
            message = "Invalid 'testIndex' parameter: " + String(testID);
        }
    }
    else if (commandType == "setAxisAngle")
    {
        int axis = parameters["axis"] | -1;
        float angle = parameters["angle"] | 0.0;
        if (axis >= 0 && axis < NUM_STEPPERS)
        {
            SetAxisAngle(axis, angle);
            message = "Axis " + String(axis) + " angle set to " + String(angle) + " degrees.";

            JsonObject axes = stateUpdate.createNestedObject("axes");
            axes[String(axis)] = angle;
        }
        else
        {
            status = "error";
            message = "Invalid axis ID: " + String(axis);
        }
    }
    else if (commandType == "getState")
    {
        JsonObject axes = stateUpdate.createNestedObject("axes");
        for (int i = 0; i < NUM_STEPPERS; i++)
        {
            axes[String(i)] = GetAxisAngle(i);
        }
        // Include limit information for each axis
        JsonObject limits = stateUpdate.createNestedObject("limits");
        for (int i = 0; i < NUM_STEPPERS; i++)
        {
            JsonObject axisLimit = limits.createNestedObject(String(i));
            axisLimit["isAtLimit"] = steppers[i].atLimit;
            axisLimit["limitIndex"] = steppers[i].atLimit ? steppers[i].lastLimitIndex : -1;
        }
        message = "State retrieved successfully.";
    }
    else if (commandType == "homingSequence")
    {
        int axis = parameters["axis"] | -1;
        if (axis < -1 || axis >= NUM_STEPPERS)
        {
            status = "error";
            message = "Invalid axis ID for homing: " + String(axis);
        }
        else
        {
            ExecuteHomingCommand(axis, axis == -1);
            message = (axis == -1) ? "Homing all axes." : ("Homing axis " + String(axis));
        }
    }
    else
    {
        status = "error";
        message = "Unknown command type: " + commandType;
    }

    sendSerialMessage("response",
                      uuid.length() > 0 ? uuid.c_str() : nullptr,
                      commandType.c_str(),
                      status.c_str(),
                      message.c_str(),
                      stateUpdate.isNull() ? nullptr : &stateUpdate);
}


// =============================================================================================================================
// sendSerialMessage
// =============================================================================================================================
void sendSerialMessage(const char* type, const char* uuid, const char* command, const char* status, const char* message, const JsonObject* stateUpdate)
{
    StaticJsonDocument<1024> doc;
    doc["type"] = type;

    if (uuid != nullptr && strlen(uuid) > 0)
    {
        doc["uuid"] = uuid;
    }

    if (command != nullptr)
    {
        doc["command"] = command;
    }

    if (status != nullptr)
    {
        doc["status"] = status;
    }

    if (message != nullptr)
    {
        doc["message"] = message;
    }

    if (stateUpdate != nullptr && !stateUpdate->isNull())
    {
        doc["stateUpdate"] = *stateUpdate;
    }

    String serializedMessage;
    serializeJson(doc, serializedMessage);
    Serial.println(serializedMessage);
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

    // If currently at a limit, prevent further motion into that limit direction
    if (motor.atLimit)
    {
        // Left limit (index 0) => disallow negative movement; Right limit (index 1) => disallow positive movement
        if ((motor.lastLimitIndex == 0 && angleDifference < 0) ||
            (motor.lastLimitIndex == 1 && angleDifference > 0))
        {
            motor.stepsToMove = 0.0;
            return;
        }
    }

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

    // If currently at a limit, prevent further motion into that limit direction
    if (motor.atLimit)
    {
        if ((motor.lastLimitIndex == 0 && steps < 0) ||
            (motor.lastLimitIndex == 1 && steps > 0))
        {
            return;
        }
    }

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
                motor.atLimit = true;
                motor.lastLimitIndex = i;

                // Determine intended movement direction by remaining distance
                long distanceToGo = motor.stepper.distanceToGo();
                bool movingNegative = distanceToGo < 0 || motor.stepsToMove < 0;
                bool movingPositive = distanceToGo > 0 || motor.stepsToMove > 0;

                // If moving further into the triggered limit, stop and block
                if ((i == 0 && movingNegative) || (i == 1 && movingPositive))
                {
                    motor.stepper.stop();
                    motor.stepsToMove = 0.0;
                    Serial.print("Digital limit (" );
                    Serial.print(i);
                    Serial.print(") blocking inward move for Stepper ");
                    Serial.println(stepperIndex);
                    return true;
                }
                
                // Otherwise allow movement away from the limit
                return false;
            }
        }
    }
    motor.atLimit = false;
    return false;
}

int degreesToSteps(float degrees, float stepsPerRev) {
    return (int)((degrees / 360.0f) * stepsPerRev);
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
        HomeAxis(3);
        HomeAxis(4);
        HomeAxis(2);
        HomeAxis(1);
        HomeAxis(0);
        Serial.println("All Axes homed.");
    } 
    else 
    {
        if (InAxisIndex < 0 || InAxisIndex > NUM_STEPPERS - 1)
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

    int DegreesToHomePos = 0;
    unsigned long startTime = millis();
    const unsigned long timeout = 15000;

    // Start moving slowly in one direction to find limit switch
    // Use the same movement system as SetAxisAngle
    float homingSpeed = motor.maxSpeed * 0.3;  // Slower speed for homing

    switch (InAxisIndex) {
        case 0:
        homingSpeed = homingSpeed * 0.2;
            SetAxisAngle(InAxisIndex, 180.0);
            break;
        case 1:
            homingSpeed = homingSpeed * 0.15;
            SetAxisAngle(InAxisIndex, 180.0);
            break;

        case 2:
            homingSpeed = homingSpeed * 0.35;
            SetAxisAngle(InAxisIndex, -180.0);
            break;

        case 3:
            SetAxisAngle(InAxisIndex, -180.0);
            break;
            
        case 4:
            SetAxisAngle(3, -45.0);
            int axis3Motors[] = {3};
            waitForMotors(axis3Motors, 1);
            
            SetAxisAngle(InAxisIndex, 180.0); 
            break;
            
        default:
            break;
    }
    motor.stepper.setMaxSpeed(homingSpeed);

    while (!foundLimit) 
    {
        // Use the same movement system as normal commands
        runActiveSteppers();

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
                
                // Stop the motor immediately
                motor.stepper.stop();
                motor.stepsToMove = 0.0;
                break;
            }
        }
        
        // Check for timeout
        if (millis() - startTime > timeout) 
        {
            Serial.println("Homing Axis " + String(InAxisIndex) + ": Error - timeout after " + String(timeout / 1000.0, 2) + " seconds.");
            motor.stepper.stop();
            motor.stepsToMove = 0.0;
            motor.isHoming = false;
            motor.stepper.setMaxSpeed(motor.maxSpeed);  // Restore original speed
            return;
        }

        // Check if motor stopped moving (might have hit mechanical limit)
        if (motor.stepper.distanceToGo() == 0 && motor.stepsToMove == 0.0) 
        {
            Serial.println("Homing Axis " + String(InAxisIndex) + ": Warning - motor stopped without hitting sensor.");
            break;
        }
    }

    // Move to the home position using the same movement system
    motor.stepper.setCurrentPosition(0);  // Set current position as 0 degrees
    motor.stepper.setMaxSpeed(motor.maxSpeed);  // Restore original speed
    
    // Now move to the actual home position
    SetAxisAngle(InAxisIndex, DegreesToHomePos);
    
    // Wait for movement to complete using the same system
    int homeMotors[] = {InAxisIndex};
    waitForMotors(homeMotors, 1);
    
    // Set this position as the new zero
    motor.stepper.setCurrentPosition(0);

    if (InAxisIndex == 4){
        SetAxisAngle(3, 0.0);
        int axis3Motors[] = {3};
        waitForMotors(axis3Motors, 1);
    }

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
        case 1:
        {
            Serial.println("Running test ID 1 - Constrained Test Axis 1");
            int Motors_Test1[] = {1};

            SetAxisAngle(1, 15.0);
            waitForMotors(Motors_Test1, 1);
            
            SetAxisAngle(1, 0.0);
            waitForMotors(Motors_Test1, 1);

            Serial.println("Completed test ID 1 - Constrained Test Axis 1");
            break;
        }
        case 6:
        {
            Serial.println("Running test ID 6 - Jogging 1");
            int Motors_Jog1[] = {0, 1, 2, 3, 4, 5};
            
            SetAxisAngle(0, 25.0);
            SetAxisAngle(1, 25.0);
            SetAxisAngle(2, 25.0);
            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, 45.0);
            waitForMotors(Motors_Jog1, 6);

            SetAxisAngle(0, -25.0);
            SetAxisAngle(1, -25.0);
            SetAxisAngle(2, -25.0);
            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, -45.0);
            waitForMotors(Motors_Jog1, 6);

            SetAxisAngle(0, 45.0);
            SetAxisAngle(1, -35.0);
            SetAxisAngle(2, 45.0);
            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, -30.0);
            waitForMotors(Motors_Jog1, 6);

            SetAxisAngle(0, 0.0);
            SetAxisAngle(1, 0.0);
            SetAxisAngle(2, 0.0);
            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, 0.0);
            waitForMotors(Motors_Jog1, 6);

            Serial.println("Completed test ID 6 - Jogging 1");
            break;
        }

        case 7:
        {
            Serial.println("Running test ID 7 - Jogging 2");
            int Motors_Jog2[] = {0, 1, 2, 3, 4, 5};
            
            SetAxisAngle(0, 5.0);
            SetAxisAngle(1, -25.0);
            SetAxisAngle(2, -40.0);
            SetAxisAngle(3, 25.0);
            SetAxisAngle(4, -15.0);
            SetAxisAngle(5, 15.0);
            waitForMotors(Motors_Jog2, 6);

            SetAxisAngle(0, 15.0);
            SetAxisAngle(1, -45.0);
            SetAxisAngle(2, -65.0);
            SetAxisAngle(3, 45.0);
            SetAxisAngle(4, -35.0);
            SetAxisAngle(5, -15.0);
            waitForMotors(Motors_Jog2, 6);

            SetAxisAngle(0, -10.0);
            SetAxisAngle(1, -15.0);
            SetAxisAngle(2, 35.0);
            SetAxisAngle(3, -45.0);
            SetAxisAngle(4, 35.0);
            SetAxisAngle(5, 50.0);
            waitForMotors(Motors_Jog2, 6);

            SetAxisAngle(0, 0.0);
            SetAxisAngle(1, 0.0);
            SetAxisAngle(2, 0.0);
            SetAxisAngle(3, 0.0);
            SetAxisAngle(4, 0.0);
            SetAxisAngle(5, 0.0);
            waitForMotors(Motors_Jog2, 6);

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
