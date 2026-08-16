#include <AccelStepper.h>
#include <Servo.h> 
#include <math.h>


// ------------------------------ DIMENSIONS ------------------------------ //
const int L1_LENGTH = 120; //mm 
const int L2_LENGTH = 100; //mm 


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
AccelStepper SHOULDER_STEPPER(1, SHOULDER_PUL_PIN, SHOULDER_DIR_PIN);  // Driver mode, STEP pin 2, DIR pin 3
AccelStepper ELBOW_STEPPER(1, ELBOW_PUL_PIN, ELBOW_DIR_PIN); 


// MICROSTEPPING VALUES 
const int BASE_STEPS_PER_REV = 1600; 
const int SHOULDER_STEPS_PER_REV = 800; 
const int ELBOW_STEPS_PER_REV = 800; 

// STEPPER FLAGS
bool baseMotorMoving = false;
bool baseMotorReached = true;

bool shoulderMotorMoving = false;
bool shoulderMotorReached = true;

bool elbowMotorMoving = false;
bool elbowMotorReached = true;


//HOMING CONSTANTS 
const int SHOULDER_HOME_OFFSET = -395;
const int SHOULDER_HOME_RETRACT = 200; 
const int ELBOW_HOME_OFFSET = 300;

int ELBOW_EXPECTED_STEPS = 0; 
/// ------------------------------ SERVO SETUP ------------------------------ //
Servo WRIST_SERVO; 
int WRIST_POSITION = 0; 

// ------------------------------ STATE MACHINE ------------------------------ //

//GLOBAL HOMING STATE MACHINE 
enum RobotHomeState
{
    HOME_BASE, 
    HOME_SHOULDER,
    HOME_ELBOW,   
    HOME_WRIST, 
    ROBOT_HOMED
}; 

//SHOULDER HOMING STATE MACHINE 
enum ShoulderHomeState
{
    SEEK_SHOULDER_SWITCH,
    BACK_SHOULDER_OFF,
    SET_SHOULDER_ZERO, 
    SHOULDER_HOMED
}; 

//ELBOW HOMING STATE MACHINE
enum ElbowHomeState
{
    SEEK_ELBOW_SWITCH,
    BACK_ELBOW_OFF,
    ELBOW_HOMED
}; 


RobotHomeState robotHomeState = HOME_BASE; 
ShoulderHomeState shoulderHomeState = SEEK_SHOULDER_SWITCH;
ElbowHomeState elbowHomeState = SEEK_ELBOW_SWITCH;

// ------------------------------ UI VARIABLES ------------------------------ //
float targetX;
float targetY;

bool promptShown = false;



/// IK RESUULTS 

struct IKResult
{
    double baseAngle;
    double shoulderAngle;
    double elbowAngle;
    double wristAngle; 
};

struct stepResult
{
    long baseSteps;
    long shoulderSteps;
    long elbowSteps;
};



void setup() {
  Serial.begin(9600); 

  BASE_STEPPER.setMaxSpeed(1500);
  BASE_STEPPER.setAcceleration(800);

  SHOULDER_STEPPER.setMaxSpeed(500);
  SHOULDER_STEPPER.setAcceleration(500);

  ELBOW_STEPPER.setMaxSpeed(500);
  ELBOW_STEPPER.setAcceleration(500);
  
  WRIST_SERVO.attach(WRIST_SERVO_PIN);   

  pinMode(SHOULDER_LIMIT_SWITCH, INPUT_PULLUP); 
  pinMode(ELBOW_LIMIT_SWITCH, INPUT_PULLUP); 

  Serial.println("STARTING HOMING PROTOCOL"); 
  delay(1000); 

  BASE_STEPPER.setCurrentPosition(0);

}

void loop() {

    if(robotHomeState != ROBOT_HOMED){
 
        homeRobot(); 
    }else{
        updateBaseMotor();
        updateShoulderMotor();
        updateElbowMotor();
        WRIST_SERVO.write(WRIST_POSITION); 
        getCoordinateInput();     
    }    
}

void homeShoulderMotor()
{
    switch(shoulderHomeState)
    {
        case SEEK_SHOULDER_SWITCH:

            if (digitalRead(SHOULDER_LIMIT_SWITCH) == 0)
            {
                Serial.println("Moving backward!");           
                SHOULDER_STEPPER.setSpeed(800);
                SHOULDER_STEPPER.runSpeed();
            }
            else if(digitalRead(SHOULDER_LIMIT_SWITCH) == 1)
            {   
                Serial.println("I got clicked!"); 
                SHOULDER_STEPPER.setCurrentPosition(0);
                SHOULDER_STEPPER.move(SHOULDER_HOME_OFFSET);
                shoulderHomeState = SET_SHOULDER_ZERO;
            }
            break;

        case SET_SHOULDER_ZERO:

            SHOULDER_STEPPER.run();
            Serial.println("Moving forward"); 

            if (SHOULDER_STEPPER.distanceToGo() == 0)
            {
                SHOULDER_STEPPER.setCurrentPosition(0);
                shoulderHomeState = BACK_SHOULDER_OFF;
                Serial.println("Made it home");
                SHOULDER_STEPPER.move(SHOULDER_HOME_RETRACT);
            }
            break;
        
        case BACK_SHOULDER_OFF:
            
            SHOULDER_STEPPER.run();
            Serial.println("Moving backward"); 

            if (SHOULDER_STEPPER.distanceToGo() == 0)
                {
                    shoulderHomeState = SHOULDER_HOMED;
                    Serial.println("Made it to starting position");
                }
            
            break;
        

        case SHOULDER_HOMED:
            break;
    }
}

void homeElbowMotor()
{
    switch(elbowHomeState)
    {
        case SEEK_ELBOW_SWITCH:

            if (digitalRead(ELBOW_LIMIT_SWITCH) == 1)
            {
                Serial.println("Moving Foward!");               
                ELBOW_STEPPER.setSpeed(-5000);
                ELBOW_STEPPER.runSpeed();
            }
            else
            { 
                Serial.println("I got clicked!"); 
                ELBOW_STEPPER.move(ELBOW_HOME_OFFSET);
                elbowHomeState = BACK_ELBOW_OFF;
            }
            break;

        case BACK_ELBOW_OFF:

            ELBOW_STEPPER.run();
            Serial.println("Moving Backward"); 

            if (ELBOW_STEPPER.distanceToGo() == 0)
            {
                ELBOW_STEPPER.setCurrentPosition(0);
                elbowHomeState = ELBOW_HOMED;
                Serial.println("Made it home"); 
            }
            break;

        case ELBOW_HOMED:
            break;
    }
}

void homeRobot()
{
    switch(robotHomeState)
    {
        case HOME_BASE:

            BASE_STEPPER.setCurrentPosition(0);
            robotHomeState = HOME_SHOULDER;
            Serial.println("BASE HOMING COMPLETE!"); 
            delay(1000); 
            break; 
        
        case HOME_SHOULDER:

            homeShoulderMotor(); 

            if(shoulderHomeState == SHOULDER_HOMED){
                Serial.println("SHOULDER HOMING COMPLETE"); 
                delay(1000); 
                robotHomeState = HOME_ELBOW; 
            }

            break;
        
        case HOME_ELBOW:

            homeElbowMotor(); 
            if(elbowHomeState == ELBOW_HOMED){
                Serial.println("ELBOW HOMING COMPLETE"); 
                delay(1000); 
                robotHomeState = HOME_WRIST; 
            }
            break;

        
        case HOME_WRIST:
            WRIST_SERVO.write(90); 
            Serial.println("WRIST HOMING COMPLETE"); 
            delay(1000); 
            robotHomeState = ROBOT_HOMED;
            break;
        case ROBOT_HOMED: 
            Serial.println("HOMING OPERATION COMPLETED!"); 
            break; 
        
    }
}



// ------------------------------ IK FUNCTIONALITY ------------------------------ //

IKResult jointAngles(float X, float Y, float Z){

    IKResult result; 

    double baseAngle = degrees(atan2(Y, X));  // handles all 4 quadrants
    
    double XY = sqrt(pow(X, 2) + pow(Y, 2)); // CORRECT LOGIC 
    double R = sqrt(pow(XY, 2) + pow(Z, 2)); // CORRECT LOGIC 

    double thetaOne = degrees(atan2(Z, XY)); // CORRECT LOGIC 
    
    double thetaTwo = degrees( // CORRECT LOGIC 
        acos(
            (pow(L1_LENGTH, 2) + pow(R, 2) - pow(L2_LENGTH, 2))
            / (2.0 * L1_LENGTH * R)
        )
    );
        
    double thetaThree = degrees( // CORRECT LOGIC 
        acos(
            (pow(L1_LENGTH, 2) + pow(L2_LENGTH, 2) - pow(R, 2))
            / (2.0 * L1_LENGTH * L2_LENGTH)
        )
    );

    double thetaFour = 180 - thetaTwo - thetaThree; 
    double thetaFive = 180 - thetaThree; 

    result.baseAngle = baseAngle; 
    result.shoulderAngle = thetaOne + thetaTwo;
    result.elbowAngle = -thetaFive; 
    result.wristAngle = -(90 + thetaOne);  

    WRIST_POSITION = 90 - (result.shoulderAngle + result.elbowAngle); 

    Serial.print("Angles: ");
    Serial.print(result.baseAngle); 
    Serial.print(" "); 
    Serial.print(result.shoulderAngle); 
    Serial.print(" "); 
    Serial.print(result.elbowAngle);
    Serial.print(" "); 
    Serial.println(result.wristAngle);


    return result; 
}

stepResult jointSteps(float X, float Y, float Z){
    IKResult ik = jointAngles(X, Y, Z);

    stepResult steps; 

    steps.baseSteps = ((ik.baseAngle)*BASE_STEPS_PER_REV)/360; 
    steps.shoulderSteps = ((ik.shoulderAngle)*SHOULDER_STEPS_PER_REV)/360; 
    steps.elbowSteps = (((ik.elbowAngle)*ELBOW_STEPS_PER_REV)/360); 

    long shoulderCurrentPos = SHOULDER_STEPPER.currentPosition();
    long shoulderTravel =  steps.shoulderSteps - shoulderCurrentPos; 

    long elbowTravel = steps.elbowSteps - ELBOW_STEPPER.currentPosition(); 

    // if(shoulderCurrentPos > SHOULDER_STEPS_PER_REV/2){
    //     steps.elbowSteps += shoulderTravel*4/5; 
    // }else{
    //     steps.elbowSteps -= shoulderTravel*4/5; 
    // }

    Serial.print("Shoulder Travel: "); 
    Serial.println(shoulderTravel); 
    //steps.elbowSteps +=  (shoulderTravel + ELBOW_STEPPER.currentPosition()); 
    
    steps.elbowSteps = (shoulderTravel) + elbowTravel; 
    ELBOW_EXPECTED_STEPS = (ik.elbowAngle * ELBOW_STEPS_PER_REV)/360; 
    

    Serial.print("Steps: ");
    Serial.print(steps.baseSteps); 
    Serial.print(" "); 
    Serial.print(steps.shoulderSteps); 
    Serial.print(" "); 
    Serial.println(steps.elbowSteps); 

    return steps; 
}

void moveMotors(float xCord, float yCord, float zCord)
{
    stepResult steps = jointSteps(xCord, yCord, zCord); 

    BASE_STEPPER.moveTo(steps.baseSteps);
    SHOULDER_STEPPER.moveTo(steps.shoulderSteps);
    ELBOW_STEPPER.move(steps.elbowSteps);
   
    
    baseMotorMoving = true;
    baseMotorReached = false;

    shoulderMotorMoving = true;
    shoulderMotorReached = false;

    elbowMotorMoving = true;
    elbowMotorReached = false;
}

// ------------------------------ BASE FUNCTINALITY ------------------------------ //


void updateBaseMotor()
{
    if (!baseMotorMoving)
        return;

    BASE_STEPPER.run();

    if (BASE_STEPPER.distanceToGo() == 0)
    {
        baseMotorMoving = false;
        baseMotorReached = true;

        Serial.println("Base motor reached target.");
    }
}



// ------------------------------ SHOULDER FUNCTINALITY ------------------------------ //

void updateShoulderMotor()
{
    if (!shoulderMotorMoving)
        return;

    SHOULDER_STEPPER.run();

    if (SHOULDER_STEPPER.distanceToGo() == 0)
    {
        shoulderMotorMoving = false;
        shoulderMotorReached = true;

        Serial.println("Shoulder motor reached target.");
    }
}


// ------------------------------ SHOULDER FUNCTINALITY ------------------------------ //


void updateElbowMotor()
{
    if (!elbowMotorMoving)
        return;

    ELBOW_STEPPER.run();

    if (ELBOW_STEPPER.distanceToGo() == 0)
    {
        elbowMotorMoving = false;
        elbowMotorReached = true;
        ELBOW_STEPPER.setCurrentPosition(ELBOW_EXPECTED_STEPS); 
        Serial.println("Elbow motor reached target.");
    }
}


// ------------------------------ UI FUNCTINALITY ------------------------------ //

void getCoordinateInput()
{
    // Block input while robot is moving
    if (!baseMotorReached)
    {
        promptShown = false;
        return;
    }

    // Show prompt only once per cycle
    if (!promptShown)
    {
        Serial.println();
        Serial.println("Enter X,Y,Z coordinate:");
        Serial.println("Example: 100,50,25");
        promptShown = true;
    }

    // Wait for input
    if (!Serial.available())
        return;

    String input = Serial.readStringUntil('\n');
    input.trim();

    // Find commas
    int c1 = input.indexOf(',');
    int c2 = input.indexOf(',', c1 + 1);

    // Validate format
    if (c1 == -1 || c2 == -1)
    {
        Serial.println("Invalid format. Use X,Y,Z");
        return;
    }

    float x = input.substring(0, c1).toFloat();
    float y = input.substring(c1 + 1, c2).toFloat();
    float z = input.substring(c2 + 1).toFloat();

    Serial.print("Target X: ");
    Serial.print(x);
    Serial.print("  Y: ");
    Serial.print(y);
    Serial.print("  Z: ");
    Serial.println(z);

    // Send to IK + motion system
    moveMotors(x, y, z);
}