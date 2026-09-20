#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// ------------------------------ PHYSICAL DIMENSIONS ------------------------------ //
const float L1_LENGTH = 120.0; // mm
const float L2_LENGTH = 100.0; // mm
const float L1_SQ = 14400.0;   // L1^2
const float L2_SQ = 10000.0;   // L2^2

// ------------------------------ IO PIN ASSIGNMENTS ------------------------------ //
const int WRIST_SERVO_PIN = 5;
const int SHOULDER_LIMIT_SWITCH = 7;
const int ELBOW_LIMIT_SWITCH = 6;
const int BASE_PUL_PIN = 11;
const int BASE_DIR_PIN = 10;
const int SHOULDER_PUL_PIN = 12;
const int SHOULDER_DIR_PIN = 13;
const int ELBOW_PUL_PIN = 8;
const int ELBOW_DIR_PIN = 9;

// ------------------------------ STEPPERS SETUP ------------------------------ //
AccelStepper BASE_STEPPER(1, BASE_PUL_PIN, BASE_DIR_PIN);
AccelStepper SHOULDER_STEPPER(1, SHOULDER_PUL_PIN, SHOULDER_DIR_PIN);
AccelStepper ELBOW_STEPPER(1, ELBOW_PUL_PIN, ELBOW_DIR_PIN);

// 3200 microsteps * 5.067 gearbox ratio = 16214.4 steps/rev
const float BASE_STEPS_PER_REV     = 3200.0 * 5.067;
const float SHOULDER_STEPS_PER_REV = 3200.0 * 5.067;
const float ELBOW_STEPS_PER_REV    = 3200.0 * 5.067;

const long SHOULDER_HOME_OFFSET  = lround(-(SHOULDER_STEPS_PER_REV / 2.0));
const long SHOULDER_HOME_RETRACT = lround(SHOULDER_STEPS_PER_REV * 0.275);
const long ELBOW_HOME_OFFSET     = lround(ELBOW_STEPS_PER_REV * 0.375);

const float DRAWING_BASE_SPEED   = 1200.0; // steps/sec
const float DEFAULT_ACCELERATION = 900.0;

long ELBOW_OFFSET = 0;

// ------------------------------ SERVO SETUP ------------------------------ //
Servo WRIST_SERVO;
int WRIST_POSITION = 90;

// ------------------------------ STATE MACHINE ------------------------------ //
enum RobotHomeState {
    WAIT_FOR_HOME_CMD,
    HOME_BASE,
    HOME_SHOULDER,
    HOME_ELBOW,
    HOME_WRIST,
    ROBOT_HOMED
};

enum ShoulderHomeState {
    SEEK_SHOULDER_SWITCH,
    BACK_SHOULDER_OFF,
    SET_SHOULDER_ZERO,
    SHOULDER_HOMED
};

enum ElbowHomeState {
    SEEK_ELBOW_SWITCH,
    BACK_ELBOW_OFF,
    ELBOW_HOMED
};

RobotHomeState robotHomeState = WAIT_FOR_HOME_CMD;
ShoulderHomeState shoulderHomeState = SEEK_SHOULDER_SWITCH;
ElbowHomeState elbowHomeState = SEEK_ELBOW_SWITCH;

bool isMoving = false;
bool pointReported = false;
const int MAX_BLEND_STEPS = 30;
int dynamicBlendZone = 0;

// Non-blocking serial buffer
char serialBuffer[48];
byte bufferIdx = 0;

// ------------------------------ KINEMATIC STRUCTS ------------------------------ //
struct IKResult {
    double baseAngle;
    double shoulderAngle;
    double elbowAngle;
    double wristAngle;
};

struct StepResult {
    long baseSteps;
    long shoulderSteps;
    long elbowSteps;
};

// ------------------------------ FUNCTION PROTOTYPES ------------------------------ //
IKResult jointAngles(float X, float Y, float Z);
StepResult jointSteps(float X, float Y, float Z);
bool isReachable(float X, float Y, float Z);
void moveMotorsSynchronized(float targetX, float targetY, float targetZ);
void homeRobot();
void homeShoulderMotor();
void homeElbowMotor();
void readSerialNonBlocking();
void handleCommand(char* buffer);

// ------------------------------ SETUP ------------------------------ //
void setup() {
    Serial.begin(115200);

    BASE_STEPPER.setMaxSpeed(DRAWING_BASE_SPEED);
    BASE_STEPPER.setAcceleration(DEFAULT_ACCELERATION);

    SHOULDER_STEPPER.setMaxSpeed(DRAWING_BASE_SPEED);
    SHOULDER_STEPPER.setAcceleration(DEFAULT_ACCELERATION);

    ELBOW_STEPPER.setMaxSpeed(DRAWING_BASE_SPEED);
    ELBOW_STEPPER.setAcceleration(DEFAULT_ACCELERATION);

    WRIST_SERVO.attach(WRIST_SERVO_PIN);
    WRIST_SERVO.write(90);

    pinMode(SHOULDER_LIMIT_SWITCH, INPUT_PULLUP);
    pinMode(ELBOW_LIMIT_SWITCH, INPUT_PULLUP);

    Serial.println(F("STATUS:IDLE_UNHOMED"));
}

// ------------------------------ MAIN LOOP ------------------------------ //
void loop() {
    if (robotHomeState != ROBOT_HOMED) {
        if (robotHomeState != WAIT_FOR_HOME_CMD) {
            homeRobot();
        }
    } else {
        BASE_STEPPER.run();
        SHOULDER_STEPPER.run();
        ELBOW_STEPPER.run();

        if (isMoving) {
            long remBase = abs(BASE_STEPPER.distanceToGo());
            long remShoulder = abs(SHOULDER_STEPPER.distanceToGo());
            long remElbow = abs(ELBOW_STEPPER.distanceToGo());
            long maxRem = max(remBase, max(remShoulder, remElbow));

            // Early lookahead blend trigger
            if (!pointReported && dynamicBlendZone > 0 && maxRem <= dynamicBlendZone) {
                pointReported = true;
                Serial.println(F("POINT_REACHED"));
            }

            // Target reached
            if (remBase == 0 && remShoulder == 0 && remElbow == 0) {
                isMoving = false;
                WRIST_SERVO.write(constrain(WRIST_POSITION, 0, 180));
                if (!pointReported) {
                    pointReported = true;
                    Serial.println(F("POINT_REACHED"));
                }
            }
        }
    }

    readSerialNonBlocking();
}

// ------------------------------ SERIAL PROTOCOL ------------------------------ //
void readSerialNonBlocking() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            serialBuffer[bufferIdx] = '\0';
            handleCommand(serialBuffer);
            bufferIdx = 0;
        } else {
            if (bufferIdx < sizeof(serialBuffer) - 1) {
                serialBuffer[bufferIdx++] = c;
            }
        }
    }
}

void handleCommand(char* buffer) {
    if (strlen(buffer) == 0) return;

    if (strcmp(buffer, "HOME") == 0) {
        if (robotHomeState == ROBOT_HOMED) {
            Serial.println(F("HOMING_COMPLETE"));
            return;
        }
        Serial.println(F("HOMING_STARTED"));
        shoulderHomeState = SEEK_SHOULDER_SWITCH;
        elbowHomeState = SEEK_ELBOW_SWITCH;
        robotHomeState = HOME_BASE;
        return;
    }

    if (strncmp(buffer, "GOTO,", 5) == 0) {
        if (robotHomeState != ROBOT_HOMED) {
            Serial.println(F("ERR_UNHOMED"));
            return;
        }

        char* token = strtok(buffer, ",");
        token = strtok(NULL, ","); if (!token) return; float targetX = atof(token);
        token = strtok(NULL, ","); if (!token) return; float targetY = atof(token);
        token = strtok(NULL, ","); if (!token) return; float targetZ = atof(token);

        if (!isReachable(targetX, targetY, targetZ)) {
            Serial.println(F("ERR_UNREACHABLE"));
            return;
        }

        moveMotorsSynchronized(targetX, targetY, targetZ);
        return;
    }

    if (strcmp(buffer, "STATUS") == 0) {
        if (robotHomeState == ROBOT_HOMED) {
            Serial.println(isMoving ? F("STATUS:MOVING") : F("STATUS:READY"));
        } else {
            Serial.println(F("STATUS:UNHOMED"));
        }
    }
}

// ------------------------------ MOTION ENGINE ------------------------------ //
void moveMotorsSynchronized(float targetX, float targetY, float targetZ) {
    StepResult targetSteps = jointSteps(targetX, targetY, targetZ);

    long deltaBase = abs(targetSteps.baseSteps - BASE_STEPPER.currentPosition());
    long deltaShoulder = abs(targetSteps.shoulderSteps - SHOULDER_STEPPER.currentPosition());
    long deltaElbow = abs(targetSteps.elbowSteps - ELBOW_STEPPER.currentPosition());

    long maxDelta = max(deltaBase, max(deltaShoulder, deltaElbow));

    if (maxDelta > 0) {
        dynamicBlendZone = min((long)MAX_BLEND_STEPS, maxDelta / 2);

        float speedBase = DRAWING_BASE_SPEED * ((float)deltaBase / (float)maxDelta);
        float speedShoulder = DRAWING_BASE_SPEED * ((float)deltaShoulder / (float)maxDelta);
        float speedElbow = DRAWING_BASE_SPEED * ((float)deltaElbow / (float)maxDelta);

        BASE_STEPPER.setMaxSpeed(max(speedBase, 20.0f));
        SHOULDER_STEPPER.setMaxSpeed(max(speedShoulder, 20.0f));
        ELBOW_STEPPER.setMaxSpeed(max(speedElbow, 20.0f));

        BASE_STEPPER.moveTo(targetSteps.baseSteps);
        SHOULDER_STEPPER.moveTo(targetSteps.shoulderSteps);
        ELBOW_STEPPER.moveTo(targetSteps.elbowSteps);

        isMoving = true;
        pointReported = false;
    } else {
        Serial.println(F("POINT_REACHED"));
    }
}

// ------------------------------ KINEMATICS ------------------------------ //
bool isReachable(float X, float Y, float Z) {
    double R = sqrt((X * X) + (Y * Y) + (Z * Z));
    return (R <= (L1_LENGTH + L2_LENGTH + 0.5) && R >= (abs(L1_LENGTH - L2_LENGTH) - 0.5));
}

IKResult jointAngles(float X, float Y, float Z) {
    IKResult result;
    double baseAngle = degrees(atan2(Y, X));

    double XY = sqrt((X * X) + (Y * Y));
    double R_SQ = (XY * XY) + (Z * Z);
    double R = sqrt(R_SQ);
    if (R == 0) R = 0.0001;

    double cosTheta2 = (L1_SQ + R_SQ - L2_SQ) / (2.0 * L1_LENGTH * R);
    double cosTheta3 = (L1_SQ + L2_SQ - R_SQ) / (2.0 * L1_LENGTH * L2_LENGTH);

    cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
    cosTheta3 = constrain(cosTheta3, -1.0, 1.0);

    double thetaOne = degrees(atan2(Z, XY));
    double thetaTwo = degrees(acos(cosTheta2));
    double thetaThree = degrees(acos(cosTheta3));

    result.baseAngle = baseAngle;
    result.shoulderAngle = thetaOne + thetaTwo;
    result.elbowAngle = -(180.0 - thetaThree);
    result.wristAngle = -(90.0 + thetaOne);

    WRIST_POSITION = lround(90.0 + (result.shoulderAngle + result.elbowAngle));
    return result;
}

StepResult jointSteps(float X, float Y, float Z) {
    IKResult ik = jointAngles(X, Y, Z);
    StepResult steps;

    steps.baseSteps = lround((ik.baseAngle * BASE_STEPS_PER_REV) / 360.0);
    steps.shoulderSteps = lround((ik.shoulderAngle * SHOULDER_STEPS_PER_REV) / 360.0);
    long pureElbowSteps = lround((ik.elbowAngle * ELBOW_STEPS_PER_REV) / 360.0);

    // Mechanical coupling compensation
    steps.elbowSteps = pureElbowSteps + steps.shoulderSteps + ELBOW_OFFSET;
    return steps;
}

// ------------------------------ HOMING ROUTINES ------------------------------ //
void homeShoulderMotor() {
    switch (shoulderHomeState) {
        case SEEK_SHOULDER_SWITCH:
            if (digitalRead(SHOULDER_LIMIT_SWITCH) == 1) {
                SHOULDER_STEPPER.setSpeed(2500);
                SHOULDER_STEPPER.runSpeed();
            } else {
                SHOULDER_STEPPER.setSpeed(0);
                SHOULDER_STEPPER.runSpeed();
                delay(150);
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
            if (SHOULDER_STEPPER.distanceToGo() == 0) {
                shoulderHomeState = SHOULDER_HOMED;
            }
            break;

        case SHOULDER_HOMED:
            break;
    }
}

void homeElbowMotor() {
    switch (elbowHomeState) {
        case SEEK_ELBOW_SWITCH:
            if (digitalRead(ELBOW_LIMIT_SWITCH) == 1) {
                ELBOW_STEPPER.setSpeed(-2500);
                ELBOW_STEPPER.runSpeed();
            } else {
                ELBOW_STEPPER.setSpeed(0);
                ELBOW_STEPPER.runSpeed();
                delay(150);
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

        case ELBOW_HOMED:
            break;
    }
}

void homeRobot() {
    switch (robotHomeState) {
        case HOME_BASE:
            BASE_STEPPER.setCurrentPosition(0);
            robotHomeState = HOME_SHOULDER;
            break;

        case HOME_SHOULDER:
            homeShoulderMotor();
            if (shoulderHomeState == SHOULDER_HOMED) robotHomeState = HOME_ELBOW;
            break;

        case HOME_ELBOW:
            homeElbowMotor();
            if (elbowHomeState == ELBOW_HOMED) robotHomeState = HOME_WRIST;
            break;

        case HOME_WRIST:
            WRIST_SERVO.write(90);
            {
                IKResult homeIK = jointAngles(0, 0, 220);
                long homeShoulderSteps = lround((homeIK.shoulderAngle * SHOULDER_STEPS_PER_REV) / 360.0);
                long homePureElbowSteps = lround((homeIK.elbowAngle * ELBOW_STEPS_PER_REV) / 360.0);
                ELBOW_OFFSET = 0 - homeShoulderSteps - homePureElbowSteps;
            }
            robotHomeState = ROBOT_HOMED;
            Serial.println(F("HOMING_COMPLETE"));
            break;

        case ROBOT_HOMED:
        case WAIT_FOR_HOME_CMD:
            break;
    }
}