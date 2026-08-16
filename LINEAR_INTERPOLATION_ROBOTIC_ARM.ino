#include <AccelStepper.h>
#include <Servo.h> 
#include <math.h>

// ------------------------------ DIMENSIONS ------------------------------ //
const float L1_LENGTH = 120.0; //mm 
const float L2_LENGTH = 100.0; //mm 
const float L1_SQ = 14400.0;   // Pre-calculated L1^2 for fast math
const float L2_SQ = 10000.0;   // Pre-calculated L2^2 for fast math

// ------------------------------ IO ------------------------------ //
const int WRIST_SERVO_PIN = 5; 
const int SHOULDER_LIMIT_SWITCH = 7; 
const int ELBOW_LIMIT_SWITCH = 6; 
const int BASE_PUL_PIN = 10; 
const int BASE_DIR_PIN = 11; 
const int SHOULDER_PUL_PIN = 12; 
const int SHOULDER_DIR_PIN = 13; 
const int ELBOW_PUL_PIN = 8; 
const int ELBOW_DIR_PIN = 9; 

// ------------------------------ STEPPER SETUP ------------------------------ //
AccelStepper BASE_STEPPER(1, BASE_PUL_PIN, BASE_DIR_PIN); 
AccelStepper SHOULDER_STEPPER(1, SHOULDER_PUL_PIN, SHOULDER_DIR_PIN); 
AccelStepper ELBOW_STEPPER(1, ELBOW_PUL_PIN, ELBOW_DIR_PIN); 

const int BASE_STEPS_PER_REV = 1600; 
const int SHOULDER_STEPS_PER_REV = 800; 
const int ELBOW_STEPS_PER_REV = 800; 

bool baseMotorMoving = false;
bool baseMotorReached = true;
bool shoulderMotorMoving = false;
bool shoulderMotorReached = true;
bool elbowMotorMoving = false;
bool elbowMotorReached = true;

const int SHOULDER_HOME_OFFSET = -395;
const int SHOULDER_HOME_RETRACT = 200; 
const int ELBOW_HOME_OFFSET = 300;

// ------------------------------ SERVO SETUP ------------------------------ //
Servo WRIST_SERVO; 
int WRIST_POSITION = 0; 

// ------------------------------ STATE MACHINES ------------------------------ //
enum RobotHomeState { HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, ROBOT_HOMED }; 
enum ShoulderHomeState { SEEK_SHOULDER_SWITCH, BACK_SHOULDER_OFF, SET_SHOULDER_ZERO, SHOULDER_HOMED }; 
enum ElbowHomeState { SEEK_ELBOW_SWITCH, BACK_ELBOW_OFF, ELBOW_HOMED }; 

RobotHomeState robotHomeState = HOME_BASE; 
ShoulderHomeState shoulderHomeState = SEEK_SHOULDER_SWITCH;
ElbowHomeState elbowHomeState = SEEK_ELBOW_SWITCH;

// ------------------------------ WAYPOINT QUEUE & UI ------------------------------ //
bool promptShown = false;
const int MAX_POINTS = 30; // 30 point limit to save RAM

struct TargetPoint {
    float x; float y; float z;
};

TargetPoint pointQueue[MAX_POINTS];
int pointsInQueue = 0;       
int currentPointIndex = 0;   
bool isExecuting = false;    

// TRACKING POSITION FOR LINES (Matched to your resting vertical position)
float lastX = 0; 
float lastY = 0;
float lastZ = 220;

// ------------------------------ IK STRUCTS ------------------------------ //
struct IKResult { double baseAngle; double shoulderAngle; double elbowAngle; double wristAngle; };
struct stepResult { long baseSteps; long shoulderSteps; long elbowSteps; };

// ------------------------------ SETUP & LOOP ------------------------------ //
void setup() {
    Serial.begin(9600); 
    BASE_STEPPER.setMaxSpeed(1500); BASE_STEPPER.setAcceleration(800);
    SHOULDER_STEPPER.setMaxSpeed(500); SHOULDER_STEPPER.setAcceleration(500);
    ELBOW_STEPPER.setMaxSpeed(500); ELBOW_STEPPER.setAcceleration(500);
  
    WRIST_SERVO.attach(WRIST_SERVO_PIN);   
    pinMode(SHOULDER_LIMIT_SWITCH, INPUT_PULLUP); 
    pinMode(ELBOW_LIMIT_SWITCH, INPUT_PULLUP); 

    Serial.println(F("STARTING HOMING...")); 
    delay(1000); 
    BASE_STEPPER.setCurrentPosition(0);
}

void loop() {
    if(robotHomeState != ROBOT_HOMED) {
        homeRobot(); 
    } else {
        updateBaseMotor();
        updateShoulderMotor();
        updateElbowMotor();
        WRIST_SERVO.write(WRIST_POSITION); 
        
        getCoordinateInput(); 
        processQueue();       
    }    
}

// ------------------------------ HOMING FUNCTIONS ------------------------------ //
void homeShoulderMotor() {
    switch(shoulderHomeState) {
        case SEEK_SHOULDER_SWITCH:
            if (digitalRead(SHOULDER_LIMIT_SWITCH) == 0) {       
                SHOULDER_STEPPER.setSpeed(400); 
                SHOULDER_STEPPER.runSpeed();
            } else {   
                SHOULDER_STEPPER.setSpeed(0);
                SHOULDER_STEPPER.runSpeed();
                delay(250); 
                SHOULDER_STEPPER.setCurrentPosition(0);
                SHOULDER_STEPPER.move(SHOULDER_HOME_OFFSET);
                shoulderHomeState = SET_SHOULDER_ZERO;
            }
            break;
        case SET_SHOULDER_ZERO:
            SHOULDER_STEPPER.run();
            if (SHOULDER_STEPPER.distanceToGo() == 0) {
                SHOULDER_STEPPER.setCurrentPosition(0);
                shoulderHomeState = BACK_SHOULDER_OFF;
                SHOULDER_STEPPER.move(SHOULDER_HOME_RETRACT);
            }
            break;
        case BACK_SHOULDER_OFF:
            SHOULDER_STEPPER.run();
            if (SHOULDER_STEPPER.distanceToGo() == 0) shoulderHomeState = SHOULDER_HOMED;
            break;
        case SHOULDER_HOMED: break;
    }
}

void homeElbowMotor() {
    switch(elbowHomeState) {
        case SEEK_ELBOW_SWITCH:
            if (digitalRead(ELBOW_LIMIT_SWITCH) == 1) {             
                ELBOW_STEPPER.setSpeed(-1000); 
                ELBOW_STEPPER.runSpeed();
            } else { 
                ELBOW_STEPPER.setSpeed(0);
                ELBOW_STEPPER.runSpeed();
                delay(250);
                ELBOW_STEPPER.setCurrentPosition(0); 
                ELBOW_STEPPER.move(ELBOW_HOME_OFFSET);
                elbowHomeState = BACK_ELBOW_OFF;
            }
            break;
        case BACK_ELBOW_OFF:
            ELBOW_STEPPER.run();
            if (ELBOW_STEPPER.distanceToGo() == 0) {
                ELBOW_STEPPER.setCurrentPosition(0);
                elbowHomeState = ELBOW_HOMED;
            }
            break;
        case ELBOW_HOMED: break;
    }
}

void homeRobot() {
    switch(robotHomeState) {
        case HOME_BASE:
            BASE_STEPPER.setCurrentPosition(0);
            robotHomeState = HOME_SHOULDER;
            delay(500); 
            break; 
        case HOME_SHOULDER:
            homeShoulderMotor(); 
            if(shoulderHomeState == SHOULDER_HOMED){ robotHomeState = HOME_ELBOW; delay(500); }
            break;
        case HOME_ELBOW:
            homeElbowMotor(); 
            if(elbowHomeState == ELBOW_HOMED){ robotHomeState = HOME_WRIST; delay(500); }
            break;
        case HOME_WRIST:
            WRIST_SERVO.write(90); 
            robotHomeState = ROBOT_HOMED;
            Serial.println(F("HOMING COMPLETE!")); 
            delay(500); 
            break;
        case ROBOT_HOMED: break; 
    }
}

// ------------------------------ MATH CHECKS & IK ------------------------------ //
bool isReachable(float X, float Y, float Z) {
    // Fast math: avoided double sqrt() logic by calculating direct 3D distance
    double R = sqrt((X*X) + (Y*Y) + (Z*Z));
    if (R > (L1_LENGTH + L2_LENGTH + 0.5) || R < (abs(L1_LENGTH - L2_LENGTH) - 0.5)) return false;
    return true;
}

IKResult jointAngles(float X, float Y, float Z){
    IKResult result; 
    double baseAngle = degrees(atan2(Y, X));  
    
    // Fast math: Using (X*X) instead of pow()
    double XY = sqrt((X*X) + (Y*Y)); 
    double R_SQ = (XY*XY) + (Z*Z);
    double R = sqrt(R_SQ); 

    double thetaOne = degrees(atan2(Z, XY)); 
    double thetaTwo = degrees(acos((L1_SQ + R_SQ - L2_SQ) / (2.0 * L1_LENGTH * R)));
    double thetaThree = degrees(acos((L1_SQ + L2_SQ - R_SQ) / (2.0 * L1_LENGTH * L2_LENGTH)));

    result.baseAngle = baseAngle; 
    result.shoulderAngle = thetaOne + thetaTwo;
    result.elbowAngle = -(180 - thetaThree); 
    result.wristAngle = -(90 + thetaOne);  

    WRIST_POSITION = 90 - (result.shoulderAngle + result.elbowAngle); 
    return result; 
}

stepResult jointSteps(float X, float Y, float Z){
    IKResult ik = jointAngles(X, Y, Z);
    stepResult steps; 

    // Calculate PURE Absolute Joint Steps from Angles
    steps.baseSteps = (ik.baseAngle * BASE_STEPS_PER_REV) / 360.0; 
    steps.shoulderSteps = (ik.shoulderAngle * SHOULDER_STEPS_PER_REV) / 360.0; 
    long pureElbowSteps = (ik.elbowAngle * ELBOW_STEPS_PER_REV) / 360.0; 

    // --- MECHANICAL COUPLING COMPENSATION ---
    // Counteracts the physical belt drift caused by the coaxial layout.
    steps.elbowSteps = pureElbowSteps + steps.shoulderSteps; 

    return steps; 
}

void moveMotors(float xCord, float yCord, float zCord) {
    stepResult steps = jointSteps(xCord, yCord, zCord); 
    
    BASE_STEPPER.moveTo(steps.baseSteps);
    SHOULDER_STEPPER.moveTo(steps.shoulderSteps);
    ELBOW_STEPPER.moveTo(steps.elbowSteps); // Now pure absolute positioning
   
    baseMotorMoving = true; baseMotorReached = false;
    shoulderMotorMoving = true; shoulderMotorReached = false;
    elbowMotorMoving = true; elbowMotorReached = false;
}

// ------------------------------ MOTOR UPDATES ------------------------------ //
void updateBaseMotor() {
    BASE_STEPPER.run();
}

void updateShoulderMotor() {
    SHOULDER_STEPPER.run();
}

void updateElbowMotor() {
    ELBOW_STEPPER.run();
}

// ------------------------------ LINE GENERATOR ------------------------------ //
void generateLine(float targetX, float targetY, float targetZ, int numSegments) {
    Serial.println(F("Calculating line..."));
    
    float stepX = (targetX - lastX) / numSegments;
    float stepY = (targetY - lastY) / numSegments;
    float stepZ = (targetZ - lastZ) / numSegments;

    for (int i = 1; i <= numSegments; i++) {
        float nextX = lastX + (stepX * i);
        float nextY = lastY + (stepY * i);
        float nextZ = lastZ + (stepZ * i);

        if (!isReachable(nextX, nextY, nextZ)) {
            Serial.println(F("ERROR: Out of reach!"));
            return; 
        }
        if (pointsInQueue < MAX_POINTS) {
            pointQueue[pointsInQueue] = {nextX, nextY, nextZ};
            pointsInQueue++;
        } else {
            Serial.println(F("ERROR: Queue full!"));
            break;
        }
    }
    lastX = targetX; lastY = targetY; lastZ = targetZ;
    Serial.println(F("Line generated!"));
}

// ------------------------------ BATCH QUEUE & UI ------------------------------ //
void getCoordinateInput() {
    
    // Check if arm is moving FIRST
    if (isExecuting) {
        if (Serial.available()) {
            while(Serial.available()) Serial.read(); // Flush buffer
            Serial.println(F("IGNORED: Arm moving."));
        }
        return; 
    }

    // Print the prompt BEFORE waiting for input
    if (!promptShown) {
        Serial.println(F("\n--- WAITING FOR COMMANDS ---"));
        Serial.println(F("Type GO, CLEAR, X,Y,Z or LINE,X,Y,Z,Seg"));
        promptShown = true;
    }

    // Now wait for the user to type something
    if (!Serial.available()) return;

    // Lightweight C-string reading
    char input[32];
    memset(input, 0, 32);
    Serial.readBytesUntil('\n', input, 31);
    
    // Capitalize and strip return characters
    for(byte i=0; i<strlen(input); i++) {
        if (input[i] == '\r') input[i] = '\0';
        input[i] = toupper(input[i]);
    }
    
    if (strlen(input) == 0) return; 

    if (strcmp(input, "GO") == 0) {
        if (pointsInQueue > 0) {
            Serial.println(F("\n>>> STARTING <<<"));
            isExecuting = true; currentPointIndex = 0;
            TargetPoint p = pointQueue[currentPointIndex];
            moveMotors(p.x, p.y, p.z);
        } else { Serial.println(F("Queue empty!")); }
        return;
    }

    if (strcmp(input, "CLEAR") == 0) {
        pointsInQueue = 0; Serial.println(F("Queue cleared."));
        return;
    }

    // Parse LINE command
    if (strncmp(input, "LINE,", 5) == 0) {
        char* token = strtok(input, ",");
        token = strtok(NULL, ","); if(!token) return; float tX = atof(token);
        token = strtok(NULL, ","); if(!token) return; float tY = atof(token);
        token = strtok(NULL, ","); if(!token) return; float tZ = atof(token);
        token = strtok(NULL, ","); if(!token) return; int segs = atoi(token);
        generateLine(tX, tY, tZ, segs);
        return;
    }

    // Parse XYZ coordinate
    if (strchr(input, ',') != NULL) {
        char* token = strtok(input, ","); if(!token) return; float tX = atof(token);
        token = strtok(NULL, ","); if(!token) return; float tY = atof(token);
        token = strtok(NULL, ","); if(!token) return; float tZ = atof(token);

        if (isReachable(tX, tY, tZ)) {
            if (pointsInQueue < MAX_POINTS) {
                pointQueue[pointsInQueue] = {tX, tY, tZ};
                pointsInQueue++;
                lastX = tX; lastY = tY; lastZ = tZ;
                Serial.print(F("Point saved: ")); Serial.println(pointsInQueue);
            } else { Serial.println(F("ERROR: Queue full.")); }
        } else { Serial.println(F("ERROR: Unreachable!")); }
        return;
    }
    
    Serial.println(F("Unknown format."));
}

// ------------------------------ BATCH QUEUE PROCESSOR ------------------------------ //
const int BLEND_ZONE = 15; // Blend radius in steps

void processQueue() {
    if (!isExecuting) return;

    // Check if motors are inside the blend zone
    bool readyForNext = (abs(BASE_STEPPER.distanceToGo()) <= BLEND_ZONE) && 
                        (abs(SHOULDER_STEPPER.distanceToGo()) <= BLEND_ZONE) && 
                        (abs(ELBOW_STEPPER.distanceToGo()) <= BLEND_ZONE);

    if (readyForNext) {
        
        if (currentPointIndex < pointsInQueue - 1) {
            // Load next point instantly to maintain speed
            currentPointIndex++;
            TargetPoint nextPoint = pointQueue[currentPointIndex];
            moveMotors(nextPoint.x, nextPoint.y, nextPoint.z);
        } 
        else {
            // Very last point: Wait for distanceToGo to hit exactly 0 to stop
            if (BASE_STEPPER.distanceToGo() == 0 && 
                SHOULDER_STEPPER.distanceToGo() == 0 && 
                ELBOW_STEPPER.distanceToGo() == 0) 
            {
                Serial.println(F("\n>>> COMPLETE <<<"));
                isExecuting = false; 
                pointsInQueue = 0; 
                promptShown = false; 
            }
        }
    }
}