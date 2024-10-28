#include <Servo.h>
#include <AFMotor.h>

#define LINE_BUFFER_LENGTH 512

char STEP = MICROSTEP ;

// Servo position for Up and Down 
const int penZUp = 130;
const int penZDown = 83;

// Servo on PWM pin 10
const int penServoPin = 10;

// Should be right for DVD steppers, but is not too important here
const int stepsPerRevolution = 50;

// create servo object to control a servo 
Servo penServo;  

// Initialize steppers for X- and Y-axis using these Arduino pins for the L293D H-bridge
AF_Stepper myStepperY(stepsPerRevolution, 1);            
AF_Stepper myStepperX(stepsPerRevolution, 2);  

/* Structures, global variables */
struct point { 
  float x; 
  float y; 
  float z; 
};

// Current position of plothead
struct point actuatorPos;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelay = 1;
int LineDelay = 2;
int penDelay = 50;

// Motor steps to go 1 millimeter.
// Use a test sketch to go 100 steps. Measure the length of the line. 
// Calculate steps per mm. Enter here.
float StepsPerMillimeterX = 64;
float StepsPerMillimeterY = 100.0;

// Drawing robot limits, in mm
// OK to start with. Could go up to 50 mm if calibrated well. 
float Xmin = 0;
float Xmax = 40;
float Ymin = 0;
float Ymax = 40;
float Zmin = 0;
float Zmax = 1;

float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmax; 

// Set to true to get debug output.
boolean verbose = false;

//  Needs to interpret 
//  G1 for moving
//  G4 P300 (wait 150ms)
//  M300 S30 (pen down)
//  M300 S50 (pen up)
//  Discard anything with a (
//  Discard any other command!

/********************
 * void setup() - Initializations
 ***********************/
void setup() {
  //  Setup
  Serial.begin(9600);
  
  penServo.attach(penServoPin);
  penServo.write(penZUp);
  delay(100);

  // Decrease if necessary
  myStepperX.setSpeed(600);
  myStepperY.setSpeed(600);  

  //  Set & move to initial default position
  // TBD

  //  Notifications!!!
  Serial.println("Mini CNC Plotter alive and kicking!");
  Serial.print("X range is from "); 
  Serial.print(Xmin); 
  Serial.print(" to "); 
  Serial.print(Xmax); 
  Serial.println(" mm."); 
  Serial.print("Y range is from "); 
  Serial.print(Ymin); 
  Serial.print(" to "); 
  Serial.print(Ymax); 
  Serial.println(" mm."); 
}

/********************
 * void loop() - Main loop
 ***********************/
void loop() {
  delay(100);
  char line[LINE_BUFFER_LENGTH];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (1) {

    // Serial reception - Mostly from Grbl, which is "OK" for the mini plotter's purposes. 
    if (Serial.available()) {

      c = Serial.read();

      if ((c == '\n') || (lineIndex >= LINE_BUFFER_LENGTH - 1)) {

        line[lineIndex] = '\0';  // Keep string NULL terminated

        // Execute gcode interpreter
        if (lineIndex > 0) {
          if (!lineIsComment && !lineSemiColon) {
            gcode_interpreter(line);
          }
        }

        // Reset the line since we're done here
        lineIndex = 0;
        lineSemiColon = false;
        lineIsComment = false;

      } 
      else {

        // Check to see if the the comment flag was set
        if (lineIsComment) {

          // Nope, just a new line character... ignore it
          continue;

        } 
        else {

          // If we're at the start of a new line, reset the line comment flag
          if (lineIndex == 0) {
            lineIsComment = false;
          }

          // Check to see if the current character is a comment
          if (c == '(') {
            lineIsComment = true;
            continue;
          }

          // Only check for a semicolon if we're not in a comment
          if (!lineIsComment && (c == ';')) {
            lineSemiColon = true;
            continue;
          }

          // Add the character to the line buffer
          line[lineIndex++] = c;
        }
      }
    }
  }
}

/********************
 * Gcode interpreter
 * void gcode_interpreter(char *line)
 ***********************/
void gcode_interpreter(char *line) {
  float px = Xpos;
  float py = Ypos;
  float pz = Zpos;

  // Discard block delete [not yet]
  if (strchr(line, '*')) {
    return;
  }

  // G1: Linear Movement
  if (strstr(line, "G1")) {
    // XYZ
    if (strchr(line, 'X')) {
      px = getNumber(line, 'X', px);
    }
    if (strchr(line, 'Y')) {
      py = getNumber(line, 'Y', py);
    }
    if (strchr(line, 'Z')) {
      pz = getNumber(line, 'Z', pz);
    }
    move(px, py, pz);
    return;
  }

  // G4: Dwell
  if (strstr(line, "G4")) {
    // P milliseconds
    if (strchr(line, 'P')) {
      delay(getNumber(line, 'P', 0));
    }
    return;
  }

  // Pen up
  if (strstr(line, "M300 S50")) {
    penUp();
    return;
  }

  // Pen down
  if (strstr(line, "M300 S30")) {
    penDown();
    return;
  }
}

/********************
 * float getNumber (char *line, char c, float oldNumber)
 * Get a number from the gcode line
 * Very basic, assumes that there is only one number per line
 * Also assumes that the gcode is well-formed
 ***********************/
float getNumber(char *line, char c, float oldNumber) {
  char *ptr;
  float number;

  number = strtod(strchr(line, c) + 1, &ptr);

  if (ptr == line + 1) {
    return oldNumber;
  } 
  else {
    return number;
  }
}

/********************
 * void move (float px, float py, float pz)
 * Move to position in px, py and pz
 ***********************/
void move(float px, float py, float pz) {
  float dx, dy, dz;
  int i;
  int numberOfStepsX, numberOfStepsY;

  if (verbose) {
    Serial.print("Moving to ");
    Serial.print(px);
    Serial.print(" ");
    Serial.print(py);
    Serial.print(" ");
    Serial.print(pz);
    Serial.println(" ");
  }

  dx = (px - Xpos) * StepsPerMillimeterX;
  dy = (py - Ypos) * StepsPerMillimeterY;
  dz = (pz - Zpos);

  // Decide which motor has to run forward or backward
  // And run both motors at a time

  if (dx < 0) {
    myStepperX.step(-dx, BACKWARD, MICROSTEP);
  } 
  else {
    myStepperX.step(dx, FORWARD, MICROSTEP);
  }

  if (dy < 0) {
    myStepperY.step(-dy, BACKWARD, MICROSTEP);
  } 
  else {
    myStepperY.step(dy, FORWARD, MICROSTEP);
  }

  delay(100);

  Xpos = px;
  Ypos = py;
  Zpos = pz;
}

/********************
 * void penUp()
 * Put the pen up
 ***********************/
void penUp() {
  if (verbose) {
    Serial.println("Pen up");
  }
  penServo.write(penZUp);
  delay(penDelay);
}

/********************
 * void penDown()
 * Put the pen down
 ***********************/
void penDown() {
  if (verbose) {
    Serial.println("Pen down");
  }
  penServo.write(penZDown);
  delay(penDelay);
}
