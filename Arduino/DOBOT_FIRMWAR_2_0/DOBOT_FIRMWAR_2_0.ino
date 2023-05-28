//---------------------------- library prototypes ---------------------------------------------------------------
#include <AccelStepper.h>
#include <Arduino.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>     // math functions
#include <string.h>   // string functions
#include <ctype.h>    // character functions
#include <stdbool.h>  // bool definitions

//---------------------------- Program Constants ----------------------------------------------------------------------

const double L1 = 135.0;  // Length of the shoulder arm
const double L2 = 160.0;  // Length of the elbow arm
const double ABS_THETA1_DEG_MAX = 90;  // Maximum magnitude of shoulder angle in degrees
const double ABS_THETA2_DEG_MAX = 148.76;  // Maximum magnitude of elbow angle in degrees
const double LMAX = L1 + L2;  // Maximum reach of the robot
const double LMIN = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(PI - ABS_THETA2_DEG_MAX * PI / 180.0));  // Minimum L value

const int COMMAND_INDEX_NOT_FOUND = -1;  // Index value indicating command not found

const double STEP_RESOLUTION[] = { 1.0, 0.5, 0.25, 0.125, 0.0625 };  // Possible resolutions for the driver
const double STEP_PER_REV = (2000.0 / STEP_RESOLUTION[4]);  // Steps per rotation
const double DEG_PER_STEP = 360.0 / STEP_PER_REV;  // Degrees per step

const long shoulderHomeAngle = 19.9;  // Home angle for the shoulder joint
const long elbowHomeAngle = ABS_THETA2_DEG_MAX;  // Home angle for the elbow joint
const long shoulderHomePos = (long)round(shoulderHomeAngle / DEG_PER_STEP);  // Home position for the shoulder joint
const long elbowHomePos = (long)round(elbowHomeAngle / DEG_PER_STEP);  // Home position for the elbow joint

const char* seps = "\t,\n ;:";  // Delimiters for tokenizing the line string

const double ERROR_VALUE = -1;  // Error value

const double AfectorOffSet[2] = { 193.05, 35.67 };  // Offset values for the arm's factors



// number of points on path for every 500 units of arc length
const int LOW_RESOLUTION_POINTS_PER_10_UNITS = 5;
const int MEDIUM_RESOLUTION_POINTS_PER_10_UNITS = 10;
const int HIGH_RESOLUTION_POINTS_PER_10_UNITS = 20;

#define COMMAND_STRING_ARRAY_SIZE 502  // size of array to store commands written by sprintf_s for robot. 
                                       // NOTE: 2 elements must be reserved for trailing '\n' and '\0'

#define MAX_LINE_SIZE 1002  // size of array to store a line from a file. 
                            // NOTE: 2 elements must be reserved for trailing '\n' and '\0'


enum MOTOR_SPEED { MOTOR_SPEED_LOW,
                   MOTOR_SPEED_MEDIUM,
                   MOTOR_SPEED_HIGH };  // motor speed

enum RESOLUTION { RESOLUTION_LOW,
                  RESOLUTION_MEDIUM,
                  RESOLUTION_HIGH };  // line resolution

enum STEP_RESOLUTION { RESOLUTION_FULL,
                       RESOLUTION_HALF,
                       RESOLUTION_QUARTER,
                       RESOLUTION_EIHGTH,
                       RESOLUTION_SIXTEENTHS };  // motor step resolition

enum CURRENT_ANGLES { GET_CURRENT_ANGLES,
                      UPDATE_CURRENT_ANGLES };  // used to get/update current SCARA angles

enum COMMAND_INDEX  // list of all command indexes
{
  ROTATE_JOINT,
  MOTOR_SPEED,
  GRIPPER_ON,
  GRIPPER_ANGLE,
  PRECISION,
  END,
  HOME,
  NUM_COMMANDS
};

//---------------------------- Program Global ----------------------------------------------------------------------

// Defines pins for rotation
#define baceStepPin 4
#define baceDirPin 3

// Defines pins for shoulder
#define shoulderStepPin 6
#define shoulderDirPin 5

// Defines pins for elbow
#define elbowStepPin 8
#define elbowDirPin 7

// Defines pin for suction
#define suctionPin 52

// Defines pins for PRECISION
#define MS1_Pin 9
#define MS2_Pin 10
#define MS3_Pin 11

char strLine[MAX_LINE_SIZE] = { '\0' };  // Define input buffer
int commandIndex = -1;  // Command index
char *token = NULL, *next_token = NULL;  // Token pointers for command processing


// structure to map command keyword string to a command index
typedef struct COMMAND {
  const int index;
  const char* strCommand;
} COMMAND;

// SCARA tooltip coordinates
typedef struct TOOL_POSITION {
  double x, y;
} TOOL_POSITION;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES {
  double theta1Deg, theta2Deg;
} JOINT_ANGLES;

// pen state
typedef struct SUCTION_STATE {
  JOINT_ANGLES sucAngle;
  int sucPos;
} PEN_STATE;

// forward kinematics solution data
typedef struct FORWARD_SOLUTION {
  TOOL_POSITION toolPos;  // tool tip coordinates
  bool bCanReach;         // true if robot can reach, false if not
} FORWARD_SOLUTION;

// inverse kinematics solution data
typedef struct INVERSE_SOLUTION {
  JOINT_ANGLES jointAngles;  // joint angles (in degrees).  Left and Right arm solutions
  bool bCanReach;            // true if robot can reach, false if not.  Left and right arm configurations
} INVERSE_SOLUTION;

//----------------------------- Globals -------------------------------------------------------------------------------
// global array of command keyword string to command index associations
const COMMAND m_Commands[NUM_COMMANDS] = { { ROTATE_JOINT, "ROTATE_JOINT" }, { MOTOR_SPEED, "MOTOR_SPEED" }, { GRIPPER_ON, "GRIPPER_ON" }, { GRIPPER_ANGLE, "GRIPPER_ANGLE" }, { PRECISION, "PRECISION" }, { END, "END" }, { HOME, "HOME" } };

//----------------------------- Function Prototypes -------------------------------------------------------------------

// Computes the nearest integer from a double value
int nint(double);

// Returns angle in radians from input angle in degrees
double degToRad(double);

// Returns angle in degrees from input angle in radians
double radToDeg(double);

// Make sure inverseKinematic angles are mapped in range the robot understands
double mapAngle(double);

// Makes an input string all uppercase
void makeStringUpperCase(char*);

// Gets or updates the current SCARA angles
void robotAngles(JOINT_ANGLES*, int);

// Get left arm joint angles from x,y position using inverse kinematics
INVERSE_SOLUTION inverseKinematics(TOOL_POSITION);

// Get the x,y position from the joint angles using forward kinematics
FORWARD_SOLUTION forwardKinematics(JOINT_ANGLES);

// Set the angles of the joints based on the input command string
bool setJointAngles(char*);

// Set the motor speed based on the input command string
bool setMotorSpeed(char*);

// Turn the gripper on or off based on the input command string
bool setGripperState(char*);

// Set the gripper angle based on the input command string
bool setGripperAngle(char*);

// Processes a command string from the Serial input
void processCommand(int commandIndex, char*);

// Set the angles of the robot's joints and run the stepper motors accordingly
void runStepperAngles(JOINT_ANGLES);

// Setup everything for the stepper motors
void stepperMotorSetup();

// Setup the servo motors
void ServoSetup();

// Get the command keyword index from a string
int getCommandIndex(char*);

// Define the stepper motor and the pins that are connected to
AccelStepper bace(1, baceStepPin, baceDirPin);  // Type of driver: with 2 pins (STEP, DIR)
AccelStepper shoulder(1, shoulderStepPin, shoulderDirPin);
AccelStepper elbow(1, elbowStepPin, elbowDirPin);

Servo Gripper;  // Gripper servo
Servo GripperAngle;  // Gripper angle servo
MultiStepper steppersControl;  // Create instance of MultiStepper

void setup() {
  JOINT_ANGLES Home = { shoulderHomeAngle, elbowHomeAngle };  // Define home angles for the robot
  robotAngles(&Home, UPDATE_CURRENT_ANGLES);  // Update the robot's current angles to the home angles
  stepperMotorSetup();  // Setup the stepper motor
  ServoSetup();  // Setup the servo motor

  Serial.begin(9600);  // Start serial communication at 9600 baud
  while (!Serial) {  // Wait for the serial port to connect
    delay(100);
  }
}

void loop() {
  if (Serial.available() > 0) {  // If there is data available
    char incoming = Serial.read();  // Read the incoming character
    if (incoming == '\n') {  // If the incoming character is a newline
      strLine[strnlen(strLine, MAX_LINE_SIZE)] = '\0';  // Terminate the input string
      
      // Get the command index and process it
      makeStringUpperCase(strLine);  // Make line string all uppercase (makes commands case-insensitive)
      commandIndex = getCommandIndex(strLine);
      token = strtok(strLine, seps);
      token = strtok(NULL, "");  // Get the rest of the line as the next token
      processCommand(commandIndex, token);
      
      // Reset command variable to wait for the next command
      memset(strLine, '\0', sizeof(strLine));  // Clear the input buffer
    } else {  // If the incoming character is not a newline
      if (strnlen(strLine, MAX_LINE_SIZE - 1) < MAX_LINE_SIZE - 2) {  // If there is room in the input buffer
        strncat(strLine, &incoming, 1);  // Add the incoming character to the input buffer
      } else {  // If the input buffer is full
        Serial.println("Input buffer full, command ignored.");
      }
    }
  }
}


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  processes a command referenced by the commandIndex.  Parses the command string from the file and
//               packages up the command to be sent to the robot if no errors found.
// ARGUMENTS:    commandIndex:  index of the command keyword string
//               strCommandLine: command line from the file in the form of a string
// RETURN VALUE: none
void processCommand(int commandIndex, char* strCommandLine) {
  bool bSuccess = true;

  switch (commandIndex) {
    case ROTATE_JOINT:
      Serial.println("ROTATE_JOINT command selected");
      bSuccess = setJointAngles(strCommandLine);
      break;

    case MOTOR_SPEED:
      Serial.println("MOTOR_SPEED command selected");
      bSuccess = setMotorSpeed(strCommandLine);
      break;

    case GRIPPER_ON:
      Serial.println("GRIPPER_ON command selected");
      bSuccess = setGripperState(strCommandLine);
      break;

    case GRIPPER_ANGLE:
      Serial.println("GRIPPER_ANGLE command selected");
      bSuccess = setGripperAngle(strCommandLine);
      Serial.println(strCommandLine);
      break;

    case PRECISION:
      Serial.println("PRECISION command selected, N.A");
      Serial.println(strCommandLine);
      break;

    case END:
      Serial.println("END command selected, N.");
      Serial.println(strCommandLine);
      break;

    case HOME:
      Serial.println("HOME command selected, N.");
      Serial.println(strCommandLine);
      break;

    default:
      Serial.println("unknown command!\n");
   }
  if(bSuccess) Serial.println("Command sent to robot!\n\n");
}

void RunStepperAngles(JOINT_ANGLES angles) {

  //long stepCountShoulder = (long)round((angles.theta1Deg - currentAngles.theta1Deg) / DEG_PER_STEP);
  long stepCountShoulder = -map(angles.theta1Deg - shoulderHomeAngle, 0, ABS_THETA1_DEG_MAX, 0, STEP_PER_REV / (360/ABS_THETA1_DEG_MAX));
  long stepCountElboe =    map(angles.theta2Deg + (180 - ABS_THETA2_DEG_MAX) + shoulderHomeAngle, elbowHomeAngle, 0, 0, STEP_PER_REV / (360/ABS_THETA2_DEG_MAX));
  long gotoposition[2] = { stepCountShoulder, stepCountElboe };

  // print out the extracted values
  Serial.print("Deg1: ");
  Serial.print(angles.theta1Deg);
  Serial.print(" Step pos: ");
  Serial.println(gotoposition[0]);

  Serial.print("Deg2: ");
  Serial.print(angles.theta2Deg);
  Serial.print(" Step pos: ");
  Serial.print(gotoposition[1]);
  Serial.print("\n\n");

  // Serial.print("Deg3: ");
  // Serial.print(angles[2]);
  // Serial.print(" Step Count: ");
  // Serial.println(gotoposition[2]);

  steppersControl.moveTo(gotoposition);  // Calculates the required speed for all motors
  steppersControl.runSpeedToPosition();  // Blocks until all steppers are in position

  robotAngles(&angles, UPDATE_CURRENT_ANGLES);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: will fined what index the command is
// ARGUMENTS:    strLine:  A file line string.
// RETURN VALUE: the index of the command
int getCommandIndex(char* strLine) {

  char strLine2[MAX_LINE_SIZE];
  strcpy(strLine2, strLine);

  // find which command matches the known commands
  for (int n = 0; n <= NUM_COMMANDS; n++) {
    // is the given string the same as the N'th command
    if (strstr(strLine2, m_Commands[n].strCommand) != NULL) {
      return m_Commands[n].index;
    }
  }

  // if the command was not found
  return -1;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  will set the joints to the given angles
// ARGUMENTS:    the string wiht the info for seting the angles
// RETURN VALUE: true if the command ran, false if not
bool setJointAngles(char* strCommandLine) {
  const int paramaterNnumber = 2;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  JOINT_ANGLES angles = { 0, 0 };  // holds the angles of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i == 0) angles.theta1Deg = (double)strtod(token, &token);  // set the first token to the theta1Deg
      if (i == 1) angles.theta2Deg = (double)strtod(token, &token);  // set the second token to the theta2Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    RunStepperAngles(angles);
  }
  return goodCom;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  will set the Gripper angle to the given angle
// ARGUMENTS:    the string wiht the info for seting the angle
// RETURN VALUE: true if the command ran, false if not
bool setMotorSpeed(char* strCommandLine) {
  const int paramaterNnumber = 1;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  double motorSpeed = 0;           // holds the angle of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i == 0) motorSpeed = (double)strtod(token, &token);  // set the first token to the theta1Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    shoulder.setMaxSpeed(motorSpeed);
    elbow.setMaxSpeed(motorSpeed);
  }
  return goodCom;
}

bool setGripperState(char* strCommandLine) {
  const int paramaterNnumber = 1;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  long GripperState = 0;           // holds the angle of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i == 0) GripperState = (double)strtol(token, &token, 10);  // set the first token to the theta1Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    if (GripperState > 0) {
      Gripper.write(500);
    } else if (GripperState <= 0) {
      Gripper.write(800);
    }
  }
  return goodCom;
}

bool setGripperAngle(char* strCommandLine) {
  const int paramaterNnumber = 1;  // the number of paramaters to deal with
  bool goodCom = true;             // is the command good or not
  double angle = 0;           // holds the angle of the command
  char* token = NULL;              // if there is any garbage

  for (int i = 0; i < paramaterNnumber; i++) {
    token = strtok(i == 0 ? strCommandLine : NULL, seps);

    if (token != NULL) {
      if (i == 0) angle = (double)strtod(token, &token);  // set the first token to the theta1Deg

      if (token[0] != '\0')  // theres was garbace traling the current string
      {
        Serial.print("Garbage found in command\n");
        Serial.print("Please fix command!");
        goodCom = false;
        break;
      }
    } else  // theres was nothing in the token string
    {
      Serial.println("Token was NULL");
      goodCom = false;
      break;
    }
  }

  if (goodCom)  // run if the command was good, can be run
  {
    angle = map(angle, -90, 90, 230, 720);
    GripperAngle.write((long)angle);
  }
  return goodCom;
}

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Find shoulder and elbow angles for both left/right arm configurations for a x,y coordinate
// ARGUMENTS:    tp: the x,y coordinates of a tool tip position
// RETURN VALUE: a structure that contains the left/right arm joint angles corresponding to tp and a true/false
//               value for each arm indicating if the coordinate is reachable.
INVERSE_SOLUTION inverseKinematics(TOOL_POSITION tp) {
  INVERSE_SOLUTION isol = { 0 };  // solution values

  double thata1 = -1.0, thata2 = -1.0, thata2a = -1.0;  // Declare theta1, theta2, RIGHT
  double X = tp.x;                                      // get the X pos
  double Y = tp.y;                                      // get the Y pos
  double magnitude = sqrt(X * X + Y * Y);               // Calculate the magnitude of the pos
  double bata = atan2(Y, X);
  double alpha = acos((pow(L2, 2) - pow(magnitude, 2) - pow(L1, 2)) / (-2.0 * magnitude * L1));

  // Calculate theta1
  thata1 = bata + alpha;

  // Calculate theta2a
  thata2a = atan2((Y - L1 * sin(thata1)), (X - L1 * cos(thata1)));

  //Calculate theta2
  thata2 = thata2a - thata1;

  // Convert joint angles from radians to degrees and map them to the range [-180, 180] degrees
  thata1 = radToDeg(mapAngle(thata1));
  thata2 = radToDeg(mapAngle(thata2));

  // assine the tem varibles to the solushion strut
  isol.jointAngles.theta1Deg = thata1;
  isol.jointAngles.theta2Deg = thata2;

  // Check if the calculated joint angles are within the allowable range for the RIGHT arm
  isol.bCanReach = true;
  if (fabs(thata1) > ABS_THETA1_DEG_MAX || fabs(thata2) >= ABS_THETA2_DEG_MAX) {
    isol.bCanReach = false;
  }

  //Check if the calculated magnitude is within the allowable range
  if (magnitude > LMAX && magnitude < LMIN) {
    isol.bCanReach = false;
  }

  return isol;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle understood by the robot (-PI <= ang <= +PI)
// ARGUMENTS:    ang: the angle in radians
// RETURN VALUE: the mapped angle in radians
double mapAngle(double angRad) {
  angRad = fmod(angRad, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

  // map into range -PI <= ang <= +PI
  if (angRad > PI)
    angRad -= 2.0 * PI;
  else if (angRad < -PI)
    angRad += 2.0 * PI;

  return angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg) {
  return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad) {
  return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  computes nearest integer to given double
// ARGUMENTS:    d: double value
// RETURN VALUE: nearest int
int nint(double d) {
  return (int)floor(d + 0.5);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  makes a string all upper case characters
// ARGUMENTS:    str:  the string memory address
// RETURN VALUE: none
void makeStringUpperCase(char* str) {
  if (str == NULL) return;  // safety!

  for (size_t i = 0; i < strlen(str); i++) str[i] = (char)toupper(str[i]);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  get or update current robot shoulder and elbow angles
// ARGUMENTS:    pAngles:  shoulder/joint angles.
//               getOrUpdate:  set to UPDATE_CURRENT_ANGLES to update the current angles
//                             set to GET_CURRENT_ANGLES to retrieve the current angles
// RETURN VALUE: none
void robotAngles(JOINT_ANGLES* pAngles, int getOrUpdate) {
  static JOINT_ANGLES currentAngles = { shoulderHomePos, elbowHomePos };  // NOTE:  robot must be in home position when program starts!

  if (pAngles == NULL)  // safety
  {
    Serial.println("NULL JOINT_ANGLES pointer! (robotAngles)");
    return;
  }

  if (getOrUpdate == UPDATE_CURRENT_ANGLES)
    currentAngles = *pAngles;
  else if (getOrUpdate == GET_CURRENT_ANGLES)
    *pAngles = currentAngles;
  else
    Serial.println("Unknown value for getOrUpdate (robotAngles)");
}

void stepperMotorSetup() {
  Serial.begin(9600);  // Set up serial communication

  pinMode(suctionPin, OUTPUT);  //Setup suction

  pinMode(shoulderStepPin, OUTPUT);
  pinMode(elbowStepPin, OUTPUT);
  pinMode(baceStepPin, OUTPUT);

  pinMode(shoulderStepPin, OUTPUT);
  pinMode(elbowDirPin, OUTPUT);
  pinMode(shoulderDirPin, OUTPUT);

  pinMode(MS1_Pin, OUTPUT);  // Microstepping 1
  pinMode(MS2_Pin, OUTPUT);  // Microstepping 2
  pinMode(MS3_Pin, OUTPUT);  // Microstepping 3

  shoulder.setMaxSpeed(2000);
  elbow.setMaxSpeed(2000);  // Set maximum speed value for the stepper

  steppersControl.addStepper(shoulder);  // Set acceleration value for the stepper
  steppersControl.addStepper(elbow);     // Set the current position to 0 steps
}

void ServoSetup() {
  Gripper.attach(9);
  GripperAngle.attach(10);

  // Gripper.write(500);
  // GripperAngle.write(480);
}

size_t strnlen(const char * s, size_t len) {
    size_t i = 0;
    for ( ; i < len && s[i] != '\0'; ++i);
    return i;
}
