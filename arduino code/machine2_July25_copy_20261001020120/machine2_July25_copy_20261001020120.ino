// THIS VERSION:
// HOMING: Fast not reaching 1mm; + FINE ADJUST. DELTA IS ONLY ABOUT 1-3 ENCODER COUNTS/ 0.2mm
// SERIAL COMM WITH RPI ESTABLISHED
// > ENCODER FUNCTIONALIZED AT 12 BIT. ENCODERCORR ACHIEVES 0.06MM ERROR FROM EXPECTED/CALC POSIT - DEALS WITH CONSIDERABLE BACKLASH
//  > ENCODER BRINGS +4/-4 PIXELS FROM 0 POS.
// cameraAdjust: CAMERA: gets +1/-2 PIXELS, often zero, after  2nd adjustment

#include <AccelStepper.h>  // to control stepper drivers
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <Notecard.h>  // to control lte cellular modem attached to the arduino
#include <Wire.h>      // to use i2c
#include <math.h>

#define LED_PIN A1          // Pin connected to the data line of the LED strip
#define NUM_LEDS 12 //20         // Number of LEDs in the strip
CRGB leds[NUM_LEDS];        // Define an array to hold the LED data
#define AS5600_ADDR 0x36    // AS5600 magnetic absolute encoder on x axis motor
#define ANGLE_REG_MSB 0x0E  // angle register address

// interrupt pins
int resetPin = 51;  // pin51
// int yErrorPin = 18;
int dataInterruptPin = 19;
int zStop = 11;
int yStop = 10;
int xStop = 9;
int setupModePin = 18;
int setupPin = A3;
int buzzer = 45;

// stepper motor control pins and instantiate AccelStepper instances
int motorEnable = 8;  // pin 4 to enable/disable all motors
int zDirPin = 7;      // pin 5 controls direction of vertical axis
int zStepPin = 4;     // pin 6 steps vertical axis
int xDirPin = 5;
int xStepPin = 2;
int yDirPin = 6;   // pin 9 controls plunger in or out direction
int yStepPin = 3;  // assuming a stepper motor used for plunger, steps the motor
// laser control
int laserPin = 31;  // on or off laser (visual fault finder)
// cue light pin
int cuePin = 12;

// stepper control
AccelStepper yStepper(1, yStepPin, yDirPin);
AccelStepper xStepper(1, xStepPin, xDirPin);
AccelStepper zStepper(1, zStepPin, zDirPin);
int xStepperDefaultMaxSpeed = 13000;
int xStepperDefaultSpeed = 1400;

// define notecard and log-in credentials with the Blues Wireless hub
Notecard notecard;
#define myProductID "com.gmail.yakhyoev:lte_board_tester"
String testwifiCredentials = "AT+CWJAP=\"MyHome\",\"GoogleHome265\"";
String AustinWifi = "AT+CWJAP=\"MyOptimum 2cba85\",\"8475-turquoise-98\"";

String phoneWifiCredentials = "AT+CWJAP=\"Saids phone15\",\"dushanbe\"";
String wifiCredentials = "AT+CWJAP=\"Turbo_5270\",\"45eQF7PA\"";
String RPiCommand = "";  // flag, indicates that RaspPi sends a serial message and it is command type for execution

// global constants  and flags
bool verbose = true;      // whether to print outputs from functions
bool createServer = false;
bool interactive = false;  // whether load in interative mode to send ATcommands manually f
bool attn = false;         // modem data attention pin, HIGH or LOW
bool lte_connected = true;
int requestedPort = 0;
int previousPort = 0;
int currentPort = 0;
int requestedCommandType = 2;
int requestedValue = 0;  // generic value passed via tcp
bool zClear = true;
int extraRetract = 200 * 0.29;   // steps
int plungeDepth = -3500;         // for 1/2 step drivers -350; // -650 //-2800*0.29;  negative number. distance to insert connector into cassette
int releaseDepth = 2500 * 0.29;  // 2500; // probe release
int acceptDepth = 2000 * 0.29;   // 2000;s
int maxAllowedPort = 288;        // max allowed por number to connect
int returnStepCount = 0;         // number of steps it took to return to home position, for the current given axis
bool tcpLink = false;            // flag, whether wifi connection is established to the user comp
bool tcpConnection = false;
bool Sta = false;     // any stations joined the hotspot?
bool isWifi = false;  // whether machine joined a wifi hotspot
int deltaX = 0;       // micro adj by camera, in pixels
int deltaY = 0;
int lastPortAdj = 40;  // 40 steps ~ 1 mm. How much to travel short of 12th port so that camera can better see the edge

int ExpectedEncoderPosition = 0;

String ipAddress;

// Stepper config
float yStartPosition = -20.5; // -21mm. offset in mm from limit switch. -21 for cassette adaptor use
float xStartPosition = 1.3;      // 2.3 offset in mm from home position to align with the first cassette; 2.3 for 288 cabinet
int yStepsPerMM = 200;          // 50  for 1/2 steps if driver is tmc2208
int xStepsPerMM = 40;           // meaning 1/8 microsteps. 10 for tmc2208 at 1/2 steps; 40 for 2209
int xStepperPrevPos = 0;        // last step count of x stepper - test
int xBackupSteps = xStepsPerMM * 3;          // standard number of steps to back off end stop = 1mmm

float xPixelsPerMM = 12.8;
float yPixelsPerMM = 13.5;

// 1mm = 10px
// 1mm = 40 xmotor steps => 1px=4 xmotor steps
// 1mm = 10px = 200 y steps
// 1mm = 102.4 12-bit encoder angle ct
// 1 stepper Fullstep = 2.56 counts; 1/8step = 2.56 of Encoder at 12 bit

// 1mm = 102.4 encoder angle counts

// Sensor config
float encoderZero = 0;  // absolute encoder's angle (int, 0-1023) as voltage at endStop trigger point
float encoderPrevDeg = 0;
float xStepperDistanceToGoPrev = 0;
float encoderCurrentDeg = 0;
float encoderCurrentPos = 0;
///////// TEMP VARIABLES ///////////
int st = 0;

// machine dimensions - adjustments
float rowInterval = 20.63;  // vertical distance between casettes - varies between 20mm and 20.7mm
float colInterval = 12.4;   // interval between ports on a cassette

// counters
int portsToRapid = 0;

// flags
bool homed = false;
bool yStopTriggered = false;
bool xStopTriggered = false;
bool cueFlag = false;
bool received = false;

// modes of operation
bool cueMode = false;
bool errorMode = false;
bool setupMode = false;
bool rapidMode = false;
char requestedMode;

// timers
unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
const long everyMinute = 15000;
int resetPeriod = 14;
unsigned long cueCounter = 0;
unsigned long cueTimer = 0;
int cueWaitCycles = 100;  // how long to wait to move after cue is low, in CYCLES not sec
int minCounter = 0;       // send pings to server to maintain connection watchdog. groups of 100

int distwentx = 0;
int distwenty = 0;

bool tcpFunc = false;
bool parsedContent = false;

// Strings
// const char page[] PROGMEM = {}};
String tcpPayload;
String wifiStation;  // stores mac address of the device currently connected to the hotspot

// lists
int unavailablePorts[88] = { 0 };

//   //    //
void (*resetMachine)(void) = 0;  // reset machine

void positionProbe(int requestedPort) {  // interpret port number to row and column; determines steps and direction to requested port;

  homed = false;
  if (requestedPort > 0 && requestedPort < maxAllowedPort) {  // if legit port maneuver requested, convert port to  row and port column.
    // interpret port number in terms of row and columns
    int row = requestedPort / 12;  // e.g. requested port is 15, row = 1 or second cassette
    int col = requestedPort % 12;  // e.g. 15 %12 = 3, 3rd port
    // Serial.println("col originally %"+String(col));

    if (col > 0) {  // keep column as is, but increment to next cassette below
      row++;
      // row = row;
    } else if (col == 0) {  // this means column falls exactly into 12th port of the cassette
      col = 12;
    }

    // port positions in term of steps; absolute
    // yStartPosition and xStartPosition are distances from homed position to the first port
    long yRequestedPosition = (yStartPosition + (row * rowInterval)) * yStepsPerMM;        // distance in steps from home pos to requested cassette, y axis; 20mm vert interval
    long xRequestedPosition = (xStartPosition + ((col - 1) * colInterval)) * xStepsPerMM;  // col-1 because probe is homed already nearly over port 1; 12.7mm hor interval between ports

    /// test
    lastPortAdj = col == 12 ? 40 : 0;  // if col is 12, last port Adj is set to -1mm

    /// end test

    // yRequestedPosition -= deltaY;  // substruct micro adjustmnet made from prev port
    xRequestedPosition -= deltaX;
    xRequestedPosition -= lastPortAdj;

    // special case: last ports e.g. 12 - replaced with test above
    // if(col == 12){
    //   xRequestedPosition -= lastPortAdj; // undershoot by 40 steps or 1mm so camera can better see the edge
    // }

    // float predPos = (xRequestedPosition * 0.223) / 9;
    // float predAngle = (int(encoderPrevDeg) + int(xRequestedPosition * 0.223)) % 360;

    // Serial.println("y start: "+String(yStartPosition)+"row and interval: "+String((row * rowInterval)));
    // Serial.println("yStepsPerMM: "+String(yStepsPerMM));
    // Serial.println("target before yStepPerMM: "+String((yStartPosition + (row * rowInterval))));
    // Serial.println("yrequestedPos: "+String(yRequestedPosition));

    // run steppers
    // digitalWrite(motorEnable, LOW);
    yStepper.moveTo(yRequestedPosition);
    xStepper.moveTo(xRequestedPosition);
    // Serial.println("xRequestedPosition to port "+String(requestedPort)+": "+String(xRequestedPosition)+", dis2go: "+String(xStepper.distanceToGo()) + ", encoder should be:"+String(xExpectedEncoder));

    // Serial.println();

    // compare to encoder

    currentPort = requestedPort;

    // print
    // ////Serial.print("yRequested pos:");
    // ////Serial.println(yRequestedPosition);

    // ////Serial.print("moving to cassette: ");
    // ////Serial.println(row);
    // ////Serial.print("positioning port no: ");
    // ////Serial.println(col);

    // Serial.print("current port: ");
    // Serial.println(currentPort);

    // Serial.println("xStepper speed: " + String(xStepper.speed()));
  }
}

int xPos() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_REG_MSB);  // Start from MSB
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);  // Request 2 bytes
  if (Wire.available() == 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int angle = (highByte << 8) | lowByte;
    int angle10Bit = angle >> 2;
    // return angle10Bit;
    return angle;
    // float degrees = (angle * 360.0) / 4096.0;

    // // Serial.println("unadj degree: " + String(degrees));

    // //wrap around 360
    // int quar = int(degrees) / 90; // determine quarter
    // int prevQuar = int(encoderPrevDeg) / 90;
    // int flip = 0;
    // float deltaDeg = 0.0;

    // // int flip = prevQuar - quar < -2 ? -1: 0;
    // // flip = prevQuar - quar > 2 ? 1:0;

    // if ((prevQuar - quar) < -1){flip = -1;}
    // else if((prevQuar-quar) > 2){flip = 1;}
    // else{flip = 0;}
    // // Serial.println("flip: " + String(flip));

    // //compare magnitude of change - if more than 180, we dont know which way magnet turned..
    // // requires more frequent updating
    // // float delta = encoderPrevDeg - degrees;

    // // change in angle
    // if(flip > 0){
    //   deltaDeg = (abs(encoderPrevDeg - 360) + abs(degrees)) * flip;
    // }
    // else if(flip<0){
    //   deltaDeg = (encoderPrevDeg + (360-degrees)) * flip;
    // }
    // else{
    //   deltaDeg = degrees - encoderPrevDeg;
    // }
    // // Serial.println("deg: " + String(degrees) + ", deltaDeg: " + String(deltaDeg));

    // float translationMM = -1.00 * (deltaDeg / 9.00);
    // encoderCurrentPos  = encoderCurrentPos + translationMM;
    // Serial.print(String(st) + " tr: " + String(translationMM) + ", curpos: " + String(encoderCurrentPos));
    // Serial.println("  degrees: " + String(degrees));
    // Serial.println();
    // encoderPrevDeg = degrees;
    // st = st + 1;
    // delay(20);
  }
  delay(100);
}

void flushNotes() {
  // Serial.println("flushing queue");
  for (int i = 8; i > 0; i--) {
    J *req = notecard.newRequest("note.get");
    JAddStringToObject(req, "file", "data.qi");
    JAddBoolToObject(req, "delete", true);
    J *resp = notecard.requestAndResponse(req);
    if (resp != NULL) {
      if (notecard.responseError(resp)) {
        break;
      }
    }
  }
  Serial.println("old commands flushed");
}

void readModem() {
  // informs arduino of port and command type
  J *req = notecard.newRequest("note.get");    // arduino compiling a request for notecard modem to find a new note
  JAddStringToObject(req, "file", "data.qi");  // add an additoinal line to request, specifying which file to look in
  JAddBoolToObject(req, "delete", true);       // add also "delete":"true" line to delete the note upon reading
  // notecard.sendRequest(req); // finally, submit the request to the notecard modem

  J *rsp = notecard.requestAndResponse(req);  // submit the prepared request and await a response/confirmation. place it into rsp variable

  if (rsp != NULL) {  // make sure notecard didn't return and empty response...
    // see if response is of error type..
    if (notecard.responseError(rsp)) {
      Serial.print("error: ");
      String error = JGetString(rsp, "err");
      Serial.println(error);
      attnArm();
      return;
    }

    // retreieve response's {body} object, placed it into body variable
    J *body = JGetObject(rsp, "body");
    if (body != NULL) {
      // interpret action requested
      int commandType = JGetNumber(body, "commandType");
      if (commandType != NULL) {
        // Serial.print("command type: ");
        // Serial.println(commandType);
        requestedCommandType = commandType;  // assign to global variable
      }
      // // interpret action requested
      // int commandType = JGetBool(body, "mode");
      // if (commandType != NULL) {
      //   // Serial.print("command type: ");
      //   // Serial.println(commandType);
      //   requestedCommandType = commandType;  // assign to global variable
      // }

      // interpret port requested
      int port = JGetNumber(body, "port");
      if (port != NULL) {
        // ////Serial.print("requested port on modem: ");
        requestedPort = port;  // assign a valid port to requestedPort so it is not 0
      }

      // OTA parameter setting

      float updateXStartPos = JGetNumber(body, "xStart");
      if (updateXStartPos != NULL) {
        xStartPosition = updateXStartPos;
        // Serial.print("xStartPosition has been updated: ");
        // Serial.println(xStartPosition);
        // Serial.println();
      }

      float updateYStartPos = JGetNumber(body, "yStart");
      if (updateYStartPos != NULL) {
        yStartPosition = updateYStartPos;
        Serial.print("yStartPosition has been updated: ");
        // Serial.println(yStartPosition);
        // Serial.println();
      }
    }
  }
  attnArm();
  notecard.deleteResponse(rsp);
}

void zeroRow() {
  yStepper.move(-(12 * yStepsPerMM));
  yStepper.runToPosition();
  yStepper.setCurrentPosition(0);
  xStepper.setCurrentPosition(12 * xStepsPerMM);
}

bool reportReady() {
  if (digitalRead(zStop) == LOW) {  // if plunger is protruding, stop; home position is HIGH signal

    return false;
  }
  if (!lte_connected) {  // if no network connectoin (no sessoin), dont start
    return false;
  }
  // ////Serial.println("Machine is ready");
  return true;  // else if all ok, return true
}

void errorModeToggle(String msg = "!ERROR") {  // turn off motors;freeze as is and wait for help
  errorMode = true;
  msg = msg + "\n\r";
  Serial.print(msg);
  conv(msg);
  digitalWrite(motorEnable, HIGH);
  confirmByLTE(-1, -1);
  while (1)
    ;
}

void connect_to_LTE() {
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", "com.gmail.yakhyoev:lte_board_tester");
  JAddStringToObject(req, "mode", "continuous");
  JAddBoolToObject(req, "sync", "true");
  notecard.sendRequest(req);
  // ////Serial.println("LTE connection request sent");
  delay(1000);  // give time to establish connection
                // place code to confirm connection; expected an empty JSON object {} from the hub
  J *req_wireless = notecard.newRequest("card.wireless");
  notecard.sendRequest(req_wireless);
  tone(buzzer, 1000, 50);
  delay(100);
  tone(buzzer, 2000, 50);
  delay(400);
  tone(buzzer, 1000, 50);
  delay(100);
  tone(buzzer, 2000, 50);
}

void confirmByLTE(int port, int commandType) {
  delay(50);
  // send by HTTP POST a JSON confirmation > notehub.io Routes > Firebase Cloud Message
  // confirm that port and commandType are avail
  // Serial.print("port and command:");
  // Serial.print(String(port)+" "+String(commandType));

  J *conf = notecard.newRequest("note.add");
  JAddStringToObject(conf, "file", "confirm.qo");  // update this file; a confirmation file
  JAddBoolToObject(conf, "live", true);
  JAddBoolToObject(conf, "sync", true);

  J *body = JCreateObject();               // create a "body":{} json sub-object
  JAddNumberToObject(body, "port", port);  // include inside body, "port": port
  JAddNumberToObject(body, "commandType", commandType);
  JAddItemToObject(conf, "body", body);  // add "body":{} to the main json message
  // NoteRequest(req);
  // notecard.sendRequest(conf);
  J *rsp = notecard.requestAndResponse(conf);  // submit the prepared request and await a response/confirmation. place it into rsp variable
  if (rsp != NULL) {                           // make sure notecard didn't return and empty response...
    if (notecard.responseError(rsp)) {
      Serial.println("        RESEND REQ. io error");
    }
    // J *body = JGetObject(rsp, "body");
    // String respBody = JGetString(body, "err");
    // if(respBody != NULL){
    //   Serial.println(" _ _ ERROR in BODY");
    //   return;
    // }
  }
  notecard.deleteResponse(rsp);
}

void establish_data_interrupt() {  // assign pin interrupt upon data availability
                                   // Disarm ATTN To clear any previous state before rearming
  J *reqq = notecard.newRequest("card.attn");
  JAddStringToObject(reqq, "mode", "disarm,-files");
  notecard.sendRequest(reqq);

  // create and arm an interrupt service at the notecard
  J *req = notecard.newRequest("card.attn");
  JAddStringToObject(req, "mode", "files");
  const char *filesToWatch[] = { "data.qi" };
  int numFilesToWatch = sizeof(filesToWatch) / sizeof(const char *);
  J *filesArray = JCreateStringArray(filesToWatch, numFilesToWatch);
  JAddItemToObject(req, "files", filesArray);
  // JAddItemToObject(req, "files", ["data.qi"]);

  notecard.sendRequest(req);
}

void attnISR() {  // interrupts that signifies that data arrived and available at the modem, or connection has been established
  attn = true;
}

void attnArm() {
  // Make sure that we pick up the next RISING edge of the interrupt
  attn = false;
  // Set the ATTN pin low, and wait for the earlier of file modification or a timeout
  J *req = notecard.newRequest("card.attn");
  JAddStringToObject(req, "mode", "reset");
  JAddNumberToObject(req, "seconds", resetPeriod);
  notecard.sendRequest(req);
  // Serial.println(" attn armed");
}

void xStopISR() {
  yStopTriggered = true;
  ////Serial.println("x endStop triggered");
}

void yStopISR() {
  yStopTriggered = true;
}

void yErrorISR() {
  // Serial.println("y zeroed");
}

void setupISR() {
  setupMode = true;
}

void stepTo(AccelStepper &stepToStepper, int steps = 100) {  // manual steps command for a generic stepper
  stepToStepper.move(steps);
  while (stepToStepper.distanceToGo() != 0) {
    stepToStepper.run();
  }
}

int xAngle(int stepperPos = 0) {
  long encoderDegsPerMM = 25.64 * 4;  // encoder counts (unmapped degrees) per 1mm x-axis travel @ 12.7mm motor pulley
  int encoderResolution = 4096;       // 1024; // 1024 if 10 bit, or 4096 if 12 bit
  int portIntervalDeg = 318 * 4;
  int curExpectedPortAngle = 0;  // absolute
  int curExpectedPortAngleWrapped = 0;
  int encoderRevs = 0;
  int rawEncoder = xPos();            // analogRead(A9); // actual abs value at currently measure pos
  int portColumn = currentPort % 12;  // current port needs to be 0 for idle state. but port % 12 also yeilds 0, hence -1
  if (portColumn == 0) {
    portColumn = 12;
  }  // distance of 11 ports -1mm. 318 steps per mm ~ 25.6 encoder deg per mm

  int col1 = encoderZero - int(abs(xStartPosition) * encoderDegsPerMM);  // 62-84 encoder ticks is distance from to the left from zero (xstop trigger) to the port 1 pos
  int posError = 0;

  // //absolute tracking
  if (!homed) {
    curExpectedPortAngle = encoderZero - int(abs(xStartPosition) * encoderDegsPerMM) + ((portColumn - 1) * portIntervalDeg);
  } 
  else {
    curExpectedPortAngle = int(encoderZero + ((xBackupSteps / xStepsPerMM) * encoderDegsPerMM));
  }
  curExpectedPortAngleWrapped = curExpectedPortAngle >= 0 ? int(curExpectedPortAngle) % encoderResolution : encoderResolution - (abs(int(curExpectedPortAngle)) % encoderResolution);
  encoderRevs = floor(abs(curExpectedPortAngle / encoderResolution));  // Serial.println("revs: " + String(encoderRevs));

  posError = rawEncoder - curExpectedPortAngleWrapped;  //
  if(verbose){Serial.println("xAngle():  pos error: " + String(posError) + ", raw encoder angle: " + String(rawEncoder));}  return posError;
}

bool homeAxis(AccelStepper &stepper, int endStop, bool homingSteps = false) {  // home a given stepper based on stepper and endStop provided. returns true if successful or false if timesout
  int timeOutCounter = 0;
  int safetyRun = 0;
  int backupSteps = 10;

  if (endStop == yStop) {  // parameters for y axis homing
    safetyRun = -25000;
    backupSteps = yStepsPerMM;  // backup from y axis down by 4 mm, shoul be over port 1
    timeOutCounter = 25000;

  } else if (endStop == xStop) {  // home x axis parameters
    safetyRun = -5500;
    backupSteps = xBackupSteps;                        // xStepsPerMM;  // backup from X endstop
    stepper.setMaxSpeed(xStepperDefaultMaxSpeed / 4);
    stepper.setSpeed(xStepperDefaultSpeed / 4);  // home gently for fine homing

  } else if (endStop == zStop) {
    zStepper.setSpeed(500);
    safetyRun = -plungeDepth + 100;  // 6000; 
    backupSteps = 30; // lower the probe to about 1th row of cassettes
  }

  // how much to run back to zero?
  if (stepper.currentPosition() == 0) {          // if no info, go back safetyRun steps
    stepper.setCurrentPosition(-1 * safetyRun);  // 5500
    stepper.moveTo(5);                           // starting with zero down
  } else {  // if pos known, go back to -5
    
    stepper.moveTo(-5);
  }

  // if z Axis
  if (endStop == zStop) {
    while (zStepper.distanceToGo() != 0) {
      if (digitalRead(endStop) == LOW) {  // move for hall sensor endstop to triggers
        break;
      }
      stepper.run();
    }
    if (digitalRead(endStop) == HIGH) {  // if probe is still inserted after safety run (according to end stop) - raise an error
      zClear = false;
      errorModeToggle("Probe is pulled out or stuck");
    } else {  // if probe is retracted
      zClear = true;
    }
    delay(50);
  } 
  // X or Y axis
  else {                            
    if (digitalRead(zStop) == LOW) {  // dont home unless z is home
      // 1st, high speed, rough homing
      while (digitalRead(endStop) == HIGH) {  // while endstop not triggered
        stepper.run();
      }
    } else {
      errorModeToggle("Z Axis failed to home before moving other axis. Either probe is stuck or it overshoot and pulled out");
    }
    // homing stops here

  }

  if (endStop == xStop) {
    homed = true;
    encoderZero = xPos();
    encoderPrevDeg = encoderZero;
    encoderCurrentPos = 0;
  }

  returnStepCount = stepper.currentPosition();
  stepper.stop();
  stepper.setCurrentPosition(0);  // set end-stop trigger position as zero

  //  >> // step back if x or y
  delay(100);
  if (endStop != zStop) {
    stepTo(stepper, backupSteps);
  }
  

  // if (endStop == xStop) {  // 2nd, fine homing
  //   Serial.println("fine homing..");
  //   stepper.move(-backupSteps * 4);  // 40 x2, about 8mm
  //   while(digitalRead(endStop) == HIGH){
  //     stepper.runSpeed();
  //   }
  //   stepper.stop();
  //   stepper.setCurrentPosition(0);         // set end-stop trigger position as zero
  //   encoderZero = xPos();
  //   Serial.println("homed at: " + String(encoderZero));

  //   stepTo(stepper, backupSteps); // finaly, backup again from the hall sensor to turn off endStop
  //   xStepper.setMaxSpeed(xStepperDefaultMaxSpeed);  // restore to full default max speed
  //   xStepper.setSpeed(xStepperDefaultSpeed); // restore relative default speed
  // }
  if(verbose){Serial.println("homeAxis("+String(endStop)+"):  homed at: "+ String(encoderZero));}
  currentPort = 0;
  previousPort = 0;
  return true;
}

void setupModeToggle() {  //  method to adj port1 position
  Serial.println("SETUP MODE");
  tone(buzzer, 300, 50);
  delay(100);
  tone(buzzer, 300, 50);
  delay(100);
  tone(buzzer, 300, 50);
  delay(100);
  tone(buzzer, 300, 50);

  Serial.println("setup pin " + String(digitalRead(setupPin)));

  while (digitalRead(setupPin) == LOW) {  // wait until user releases the buttom;
    continue;
  }
  // Serial.println("motors disabled; safe to position manually");

  setupMode = true;
  digitalWrite(motorEnable, HIGH);  // disable motors to allow for manual positioning

  // manually insert probe to PORT 1, from there it calculates offset
  Serial.println("manually insert the probe into port 1");
  Serial.println("press ADJ button to set this position as port 1");

  while (1) {
    if (digitalRead(setupPin) == LOW) {  // once user pushes the adj button again, this position becomes port 1
      tone(buzzer, 300, 50);
      delay(100);
      tone(buzzer, 300, 50);
      delay(100);
      tone(buzzer, 300, 50);
      delay(100);
      tone(buzzer, 300, 50);
      break;
    }
    continue;
  }
  // once button is pushed..
  yStepper.setCurrentPosition(0);  // zero this position, in order to be able to count distance to home position
  xStepper.setCurrentPosition(0);

  Serial.println("This position is now PORT 1");
  // Serial.println(" Machine will home  and restart");
  // Serial.println();

  delay(700);
  digitalWrite(motorEnable, LOW);
  homeAxis(zStepper, zStop);

  // Serial.print("starting x before homing: ");
  // Serial.println(xStepper.currentPosition());

  homeAxis(xStepper, xStop);
  float xUpdate = 0.0f;
  xUpdate = -(returnStepCount / 10.0);
  EEPROM.put(1, xUpdate);  // save this x-axis adjustment in register 1, dist from first port to home, in the EEPROM memnory.
  Serial.println("EEPROM, PUT 1: " + String(xUpdate));
  delay(200);

  // Serial.println("homing y");
  homeAxis(yStepper, yStop);
  float yUpdate = 0.0f;
  yUpdate = -((returnStepCount + 1025) / 50.0);  // 50 steps per vertical mm? *1025 means account row height in internal logic
  EEPROM.put(22, yUpdate);                       // save y adjustment into register 22
  delay(20);

  Serial.println("restarting...");
  resetMachine();
}

void lase() {  // pulse laser to find fault visually; no probe insertion into the port
  digitalWrite(laserPin, HIGH);
  delay(100);
  digitalWrite(laserPin, LOW);
}

void backZ() {
  ////Serial.println("Z retract...");
  zStepper.move(-12000);
  while (zStepper.distanceToGo() != 0) {
    zStepper.run();
  }
  ////Serial.println("Z homed");
}

void indicate(int blinks) {
  for (int i = blinks; i = 0; i--) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void cueModeToggle(bool state) {  // implements move to next port on light cue directly from the meter
  if (state) {
    pinMode(cuePin, INPUT);
    cueMode = true;
    resetPeriod = 150;
    // Serial.println("cue mode ");//{"commandType":0, "port":2};
  } else {
    ////Serial.println("standard mode");
    cueMode = false;
  }
}

void cueMove() {  // action and sensor read-out for cue mode.
  bool cue = digitalRead(cuePin);

  if (cue) {  // if pin 31 is HIGH and no timer is working yet..
    delay(20);
    if (cue && !cueFlag) {  // we assume cue is present still for at least 50 milliseconds
      cueFlag = true;
      // Serial.println("CUE ");
      // Serial.println();
    }
  } else {  // if pin 31 is low
    delay(10);
    if (!cue && cueFlag) {  // if pin 31 is now low but was high recently
      cueCounter++;
    }
    if (cueCounter > cueWaitCycles) {
      cueCounter = 0;
      cueFlag = false;

      // Serial.println("Moving to next port: ");
      //  positionProbe(currentPort+1);  // move to the next port
      requestedPort = currentPort + 1;
      requestedCommandType = 1;
    }
  }
}

void rapidMove(bool plunge = false) {  // mechancics of rapid connects, determined by portsToRapid
  if (portsToRapid > 0 && currentPort == 0) {
    requestedCommandType = 2;  // dont connect by default
    if (plunge) {
      requestedCommandType = 1;
    }  // if plunge true, connect probe
    requestedPort = previousPort + 1;
    portsToRapid--;
    delay(1000);  // keep some pause, e.g. to shine laser,
  }
}

void releaseProbe() {
  digitalWrite(motorEnable, LOW);
  homeAxis(zStepper, zStop);
  Serial.println("relasing probe now");
  stepTo(zStepper, releaseDepth);
  digitalWrite(motorEnable, HIGH);  // motors off
  flushNotes();
  requestedPort = 0;
  requestedCommandType = 0;
}

void acceptProbe() {
  if (digitalRead(zStop) == HIGH && zClear == true) {  // if probe NOT homed, but was most recently homed (i.e. was release, missing)

    digitalWrite(motorEnable, LOW);
    // Serial.println("accepting probe now");
    stepTo(zStepper, -300);
    homeAxis(zStepper, zStop);
    digitalWrite(motorEnable, HIGH);
    flushNotes();
    requestedPort = 0;
    requestedCommandType = 0;
  } else if (digitalRead(zStop) == HIGH && zClear == false) {  // if probe not homed and hasn't reach home position (i.e. inserted)
    homeAxis(zStepper, zStop);                                 // first home
    acceptProbe();                                             //  then release;
  } else if (digitalRead(zStop) == LOW) {
    // Serial.println("probe inserted and homed already");
  }
}

bool instr2(String command) {  // disp sends AT instructions from within the sketch
  Serial3.println(command);    // send AT instruction
  Serial3.flush();
  String resp;

  for (int i = 0; i < 50; i++) {
    if (Serial3.available() > 0) {

      char c = Serial3.read();
      resp += c;
      if (c == '\n') {
        if (resp.indexOf("OK") != -1) {
          Serial.println("instr " + resp);
          return true;
        } else if (resp.indexOf("FAILED") != -1 || resp.indexOf("ERROR") != -1) {
          Serial.println("failed..");
          return false;
        }
      }
    }
    delay(60);
  }
  Serial.println("unexpected response from modem: " + resp);
  return false;
}

bool instr(String command, int waitPeriod = 18) {
  Serial3.println(command);  // send command to WIFI chip
  Serial3.flush();           // wait until command goes throughf
  String resp;
  int wait = 1;

  while (wait <= waitPeriod) {  // time out implemented through intervalled checks
    resp += Serial3.readString();
    if (resp.indexOf("OK") != -1 || resp.indexOf("WIFI CONNECTED") != -1) {
      Serial.println(command + " > " + resp);
      return true;
    } else if (resp.indexOf("ERROR") != -1) {  // if ERROR found in resp, break cycle, return false
      Serial.print("modem error: ");
      Serial.println(resp);
      return false;
    } else if (resp.indexOf("FAIL") != -1) {  // if FAIL found, return false
      Serial.print("modem fail: ");
      Serial.println(resp);
      return false;
    }

    // if no answer, give more time..
    Serial.print(".");
    delay(50);
    wait++;
  }

  Serial.println("Failed or no response");
  return false;
}

bool conv(String message) {  // convey: send String message through TCP, AT commands
  return true;
  // String messageLen = String(message.length());
  // String instruction = String("AT+CIPSEND=" + messageLen);
  // Serial.println("sending "+String(instruction));
  // instr(instruction);  //  first indicate number of bytes we are sending..
  // Serial.println("sending actual message");
  // if (instr(message)) { // send message, confirm if OK or CONNECTED returned
  //   return true;
  // }
}

bool createHotspot() {
  // broadcast a wifi hotspot, create server
  while (!instr("AT+CWMODE=1")) {
  }
  while (!instr("AT+CIPMUX=1")) {
  }
  delay(700);

  while (!instr("AT+CIPSERVER=1,840")) {
  }  // create server; no spacesA
  // while(!instr("AT+CIPSTO=600")){}       // set the server timeout at 3 min
  Serial.println("server created");
  return true;
  // a computer can now see a ESP- hotspot. Once connected to it
  // read ESP+MEGA output in the Terminal.
  // Terminal > nc 192.168.4.1 333 (default port is 333)
  // ready to send/receive data to the ESP/MEGA
}

bool joinHotSpot() {  /// joins a hotspot. tcp link to the server; or create own server

  String ResponseVariable;
  bool cipserver;
  int connAttempts = 4;
  String preferredWifi = "";

  /// interactive mode, optional
  if (interactive) {
    Serial.println("interactive mode. Type AT commands. Type exit to run main code.");
    Serial.println("1. manually type:");
    Serial.println("AT+CWJAP=\"Saids phone15\",\"dushanbe\"");
    Serial.println("AT+CIPMUX=1");
    Serial.println("AT+CIPSERVER=1");
    Serial.println("AT+CIPSTA?");

    while (1) {
      if (Serial.available()) {
        String interactiveData = Serial.readString();
        // Serial.print(interactiveData);
        if (interactiveData == "exit\r") {
          Serial.println("exiting interactive");
          interactive = false;
          break;
        }

        Serial3.println(interactiveData);
        Serial3.flush();
        delay(50);
      }

      if (Serial3.available()) {
        Serial.println(Serial3.readString());
        Serial3.flush();
      }
      delay(300);
    }
  }

  // if already connected, return true;
  if (Serial3.available() > 0) {  // if there is already data in the Serial, e.g. quick connect
    String autoConnected = Serial3.readString();
    if (autoConnected.indexOf("GOT IP") != -1 || autoConnected.indexOf(" CONNECTED") != -1) {
      Serial.println(autoConnected);
      Serial.println("Auto Joined Known Hotspot");
      isWifi = true;  // flag that wifi hotspot is connected
    }
  }

  // if no known wifi, make a few attempts to connect to a hotspot
  if (!isWifi) {

    preferredWifi = AustinWifi;  // testwifiCredentials; // // try joining tes wifi

    for (int i = 1; i <= connAttempts; i++) {
      Serial.print("joining " + preferredWifi + ", attempt " + String(i));

      if (instr(testwifiCredentials, 10)) {  //// main line attempting hotspot connection. 2.5 sec
        Serial.println("WIFI");
        isWifi = true;
        break;
      } else {
        if (i >= (connAttempts / 2)) {      // if for half allowed attempts did not work, try backup wifi network
          preferredWifi = wifiCredentials;  // try joining home wifi
        }
        delay(1000);
      }
    }

    if (!isWifi) {  // couldn't join wifi
      Serial.println("Failed to join wifi hotspot");
      return false;
    }
  }

  // if wifi, create a server or join a remote server>?
  if (isWifi) {
    Sta = true;
    Serial.println("Sta");

    // create server on esp?
    if (createServer) {
      if (!instr("AT+CIPMUX=1")) {  // make several attempts, defined inside instr function,
        Serial.println("failed at CIPMUX");
        return false;
      } else {
        Serial.println("CIPMUX ");
        delay(500);  // two more seconds then attempt a server creation
        while (!instr("AT+CIPMUX=1")) {
        }
        Serial.println("CIPMUX confirm");

        if (!instr("AT+CIPSERVER=1")) {  /// server port must be a number above 500, for security no comm allowed at lower numbers
          Serial.println("Failed at CIPSERVER");
          return false;
        } else {
          Serial.println("SERVER");
          cipserver = true;
        }
      }
    } else {
      cipserver = true;
    }
  }

  // if esp8622 on arduino has created a server, and previously joined wifi, obtain IP address
  if (cipserver) {
    Serial3.println("AT+CIPSTA?");
    while (Serial3.available() == 0) {
    }

    if (Serial3.available() > 0) {  // if answer for an IP request available after a 1 sec..
      delay(100);

      String confirm = Serial3.readString();
      Serial.println("cipsta busy?: " + confirm);

      if (confirm.indexOf("busy") != -1) {  // if you get busy.. continue
        Serial.println("busy");
      } else if (confirm.indexOf(':') == -1) {  // there is a response, but it is not an ip address..
        Serial.println("no IP address obtained, or can't be identified. No wifi link");
        Serial.println(confirm);
        return false;
      } else {  // there is some IP address returned..
        confirm.remove(0, confirm.indexOf("+CIPSTA:ip:") + 13);
        confirm = confirm.substring(0, 12);
        String test = confirm.substring(0, 3);
        if (test.toInt() != 0) {
          ipAddress = confirm;
          Serial.println("ip address: " + confirm);

          Serial.print("joining cloud server 34.58.60.73, port 4000...");

          if (instr("AT+CIPSTART=\"TCP\",\"34.58.60.73\",4000")) {  // "tcp" connectino, google server iP, port 4000, keep connection alive 2 hours
            Serial.println(" connected");
            delay(100);
            conv("M2");
            delay(100);
            tone(buzzer, 1000, 200);
            delay(100);
            tone(buzzer, 2000, 200);
            prevMillis = millis();  // start watchdog timer

            tcpLink = true;
            Serial.println("tcpLink");
            instr("AT+CIPDINFO=0");
            return true;
          } else {
            Serial.println(" tcp connection failed. Check that server is running");
          }
        } else {
          Serial.print("test: ");
          Serial.println(test);
        }
      }
    } else {
      delay(1000);
      Serial.println(Serial3.readString());
      Serial.println("no answer or confirmation of IP address");
      return false;
    }
  } else {
    Serial.println("failed to create server");
    return false;
  }
}

bool tcpConnectHTTP() {  // notify of tcp connection and parse POST requests
  // this function needs to be updated such that:
  // toggle CWMODE 3, to be access point and station mode
  // multi connections on: CIPMUX = 1, delay()
  // create server = 1, if true. Sound notification and light indicator
  // listen to connections()
  // timeout or ardu should simultaneously attemp connecting to the online server
  String tcpDetect;
  if (!tcpFunc) {  // notify of tcp connection only once - DEBUG ONLY
    Serial.println("attempting a tcp connection");
    delay(100);
    tone(buzzer, 1000, 200);
    delay(100);
    tone(buzzer, 2000, 200);
    tcpFunc = true;
  }

  // DETECT 'CONNECT NOTIFICATION FROM ESP8216.
  if (Serial3.available() > 0) {  // check everytime if TCP connection is made;
    tcpDetect = Serial3.readString();
    Serial.print("tcpDetect: ");
    Serial.println(tcpDetect);

    if (tcpDetect.indexOf(",CONNECT") != -1) {  // detect first connection
      Serial.println("tcpLink");

      tcpLink = true;  // toggle online mode
      return true;
    } else if (tcpDetect.indexOf(",CLOSE") != -1) {
      tcpLink = false;
    }
  }

  // Serial.println("no TCP link");
  // instr("AT+CIPCLOSE=0");
  // Sta = false; // do not auto attempt to connect next cycle; await manual order
  // return false;
}

void watchdog() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= everyMinute && currentPort == 0) {
    prevMillis = currentMillis;  // Reset timer

    Serial3.println("AT+CIPSTATUS");
    while (Serial3.available() == 0) {
    }
    String cipStatus = Serial3.readString();
    int respIndex = cipStatus.indexOf(':');
    String statusCode = cipStatus.substring(respIndex + 1, respIndex + 3);
    Serial.println("statusCode" + statusCode);
    if (statusCode.indexOf("4") != -1) {
      Serial.println("reconnecting to the Server");
      joinHotSpot();
    }

    minCounter++;
  }
}

void interpretCommand(String payload) {
  if (payload.indexOf("mode=4") != -1) {
    cueModeToggle(true);
  }
  if (payload.indexOf("commandType=2") != -1) {
    requestedCommandType = 2;
  }  // lase only
  if (payload.indexOf("port=") != -1) {
    int p = payload.indexOf("port=");
    int ct = payload.indexOf("&commandType=");
    String requestedPortWifi = payload.substring(p + 5, ct);
    Serial.print("REQUESTED PORT: ");
    Serial.println(requestedPortWifi);
    Serial.println();
    /// uncomment this section below to send confirmation of order from arduino back to server and to mobile app.
    /// suspected in causing delays
    // String confirmOrder =
    //   "{\n"
    //   "\"port\":"
    //   + requestedPortWifi + ",\n"
    //                         "\"command\":"
    //   + requestedCommandType + ",\n"
    //                            "\"mode\":"
    //   + "\"probe\"" + "\n}";

    // confirmOrder = confirmHeader + confirmOrder;

    // Serial.print("CONF ORDER: ");
    // Serial.println(confirmOrder);
    // conv(confirmOrder);
    ///
  }
}

void ackTCP() {
  tcpConnection = true;
  conv("connected to Machine2. command example {\"commandType\":2, \"port\":2}\n\r");
}

void lights(int port, int brightness = 100) {
  float i;
  FastLED.setBrightness(brightness);

  int localPort = abs(port);  // abs needed because port can be negative, meaning turn off light
  if (localPort > 12) {
    localPort = localPort % 12;
    if (localPort == 0) {
      localPort = 12;
    }
  }
  // i = (((20.0 / 12.0) * localPort) - 20.0) + 1.0;
  // int activeLed = (int)(i) + ((i > (int)(i)) ? 1 : 0);
  // activeLed = abs(activeLed);
  // int activeLed = abs(port-1);
   // int activeLed = (NUM_LEDS -1 - abs(port));

  int activeLed = NUM_LEDS - abs(localPort);
  Serial.println("activeLED "+String(activeLed));

  if (port < 0) {
    // Serial.println("turning off light: negative port");
    leds[activeLed] = CRGB::Black;
    FastLED.show();
    return;
  }
  if (port == 0) {
    if (leds[activeLed] == CRGB::Black) {
      leds[activeLed] = CRGB::White;
    } else {
      leds[activeLed] = CRGB::Black;
    }
    FastLED.show();
    return;
  }
  else{
    // Serial.println("turning on light: positive port " + String(activeLed));
    leds[activeLed] = CRGB::White;
    FastLED.show();
  }
  
}

void parse(String tcp) {
  // interprets command. same func as interpretCommand() uses json
  if(verbose){Serial.println("parse():  raw tcp: " + String(tcp));}

  // loss of all connections?f
  int closeIndex = tcp.lastIndexOf(",CLOSED");
  int connectIndex = tcp.lastIndexOf(",CONNECT");
  // Serial.println("close index:"+String(closeIndex));
  // Serial.println("connected index:"+String(connectIndex));
  if (closeIndex > connectIndex) {  // detects a reconnect. 0,CLOSED followed by 0,CONNECT
    Serial.println("tcp closed");
    tcpLink = false;
    tcpConnection = false;
  }

  tcp.replace("\r", "");
  tcp.replace("\n", "");

  // separate tcp into header and body sections based on markers +IPD
  // this worked for app //String header = tcp.substring(tcp.indexOf("+IPD"), tcp.indexOf("+IPD", tcp.indexOf("+IPD") + 4));
  // tcp.remove(0, tcp.indexOf(":") + 1);  // removes +IDP:7
  // tcp.remove(0, tcp.indexOf(":") + 1); // removes : and white space that comes after IP address
  String header = tcp;


  if (tcp == "\x1b[C") {
    requestedCommandType = 2;
    requestedPort = previousPort + 1;

  } else if (tcp.startsWith("ci")){
    // calibIllum();
    Serial2.print("ci");

  } else if (tcp == "\x1b[D") {
    requestedCommandType = 2;
    requestedPort = previousPort - 1;
  } else if (tcp == "hello") {
    Serial.println("server says hello");
    conv("machine ready\n\r");
    return;
  } else if (tcp == "move") {
    Serial.println(xPos());

    digitalWrite(motorEnable, LOW);
    delay(100);
    stepTo(xStepper, 100);
    delay(100);
    Serial.println(xPos());
    digitalWrite(motorEnable, HIGH);
    return;
  } else if (tcp == "moveY") {
    Serial.println(xPos());
    digitalWrite(motorEnable, LOW);
    delay(100);
    stepTo(yStepper, 400);
    delay(100);
    Serial.println(xPos());
    digitalWrite(motorEnable, HIGH);
    return;
  }

  else if (tcp == "run8") {
    rapidMode = true;
    portsToRapid = 8;
    Serial.println("got run8");
    // conv("running 8 ports rapidly\n\r");
  } else if (tcp == "lightsOff") {
    // lights(-previousPort);
    Serial.println("li off");
    FastLED.clear();
    FastLED.show();

  } else if (tcp.startsWith("lights")) {
    String param = tcp.substring(6);
    Serial.println(param);
    int br = int((param.toInt() / 10.0) * 125);
    Serial.print("br: " + String(br)+" ");
    Serial.println(int((param.toInt() / 10.0) * 125));
    FastLED.setBrightness(br);
    Serial.println("brightness " + String(FastLED.getBrightness()));
    lights(previousPort, br);
  } 
    else if (tcp == "cap") {
    unavailablePorts[currentPort] = 1;
  } else if (tcp == "home") {
    conv("homing..\n\r");
    digitalWrite(motorEnable, LOW);
    if (homeAxis(zStepper, zStop)) {
      // Serial.println("z homed");
      if (homeAxis(xStepper, xStop)) {
        // Serial.println("x homed");
        if (homeAxis(yStepper, yStop)) {
          // Serial.println("x homed");
          digitalWrite(motorEnable, HIGH);
          // conv("homed\n\r");
        }
        xAngle(xStepper.currentPosition());
        Serial2.print("homed\n");
      }
    }
  } else {
    
    // parse JSON
    DynamicJsonDocument jsonDoc(1024);                              // Adjust the size as needed
    DeserializationError error = deserializeJson(jsonDoc, header);  // was body

    if (error) {
      Serial.print(F("Failed to parse JSON: "));
      Serial.println(error.c_str());
      conv(String(error.c_str()) + "\n\r");
      requestedPort = 0;
      requestedCommandType = 0;
    } else {
      String portVal = jsonDoc["port"];
      String modeVal = jsonDoc["mode"];
      String reqComType = jsonDoc["commandType"];
      String val = jsonDoc["val"];

      requestedPort = portVal.toInt();
      requestedCommandType = reqComType.toInt();
      requestedValue = val.toInt();
      received = true;
    }
  }
}

bool microAdjust(){
  // encoder based verificaion and correction
  int encoderCorrectionRawDeg = xAngle(xStepper.currentPosition());  // measure error. 102 encoder ct deg = 40 steps
  int correctionSteps = int((encoderCorrectionRawDeg) / 2.56);       // 200 *8 seps per rev = 4096 encoder ct
  
  // encoder sanity check
  if (abs(correctionSteps) >= 80) {  // should not make about +/- 2mm correction.
    Serial.println("encoder sanity check triggered " + String(correctionSteps));
    return false;
  }
  
  // re-position, blocking motor moves
  stepTo(xStepper, -correctionSteps);
  delay(20);

  // report 
  if(verbose){Serial.println("microAdjust(): corrected " + String(-correctionSteps) + " steps");}
  return true;
}


bool cameraAdjust() {  // read micro adjustments needed based on camera feedback
/// ideally, cameraAdjust solves the issue of connector not wobble, or variation of connection position to a idealized position.
/// Only small changes should be made by cameraAdjust() as the standard positioning should be relatively precise. Large errors from camera indicates anomaly.
/// codes letters, CHK, COR and CONFRM are requests sent to rpi to take corresponding pictures and evaluate errors.

  // int imageErrorInSteps[2];  // hold erro values, in steps
  
  delay(200);
  // send init check  to rpi, for its camera to measure
  Serial2.println("CHK" + String(currentPort) + "\n");
  Serial2.flush();
  delay(50);
  deltaX = Serial2.readStringUntil('\n').toInt(); // read needed correction in pixels not steps?
  deltaY = Serial2.readStringUntil('\n').toInt();
  Serial.println("init correction needed: " + String(deltaX) + " " + String(deltaY));

  // check for for cap / anamoly
  // cap check
  if (deltaX == 100) {
    Serial.println("cap detected");
    return false;
  }
  // sanity check
  // if (abs(deltaX) > 30) {  // erro greater than 3mm
  //   Serial.println("error greater than 3mm");
  //   return false;
  // }
  
  if (abs(deltaX) >= 4){
    stepTo(xStepper, (deltaX / xPixelsPerMM) * xStepsPerMM);  // adj for error
  }
  if (abs(deltaY) >= 4){
    stepTo(yStepper, (deltaY / yPixelsPerMM) * yStepsPerMM);  // blocking microadjst run, inverse of pixel error
  }
  delay(300);
  
  // //// testing: validation 
  // Serial.println("oo deltaY: " + String(abs(deltaY)));
  //   Serial.println("oo deltaX: " + String(abs(deltaX)));
  // // remmember and update current height for this raw of cassettes
  // if(abs(deltaY) <= 4) {
  //   Serial.println("deltaY in steps: " + String(deltaY));
  //   yStartPosition = yStartPosition - (float(deltaY)/yPixelsPerMM);
  //   Serial.print(" >> updated yStartPosition : ");
  //   Serial.println(yStartPosition,4);   
  // }
  // if(abs(deltaX) <= 4){
  //   xStartPosition = xStartPosition - (float(deltaX)/xPixelsPerMM);
  //   Serial.print(" >> updated xStartPosition : ");
  //   Serial.println(xStartPosition,4);

  // }

  // 2nd snapshot and correction
  Serial2.println("COR" + String(currentPort) + "\n");  // send check request again for camera to measure
  Serial2.flush();
  delay(100);

  deltaX = Serial2.readStringUntil('\n').toInt();
  deltaY = Serial2.readStringUntil('\n').toInt();
  Serial.println("second correction needed in px: " + String(deltaX) + " " + String(deltaY));

  // sanity check
  // if (abs(deltaX) > 30) {  // erro greater than 3mm
  //   Serial.println("error greater than 3mm");
  //   return false;
  // }

  if (abs(deltaX) >= 4){
    stepTo(xStepper, (deltaX / xPixelsPerMM) * xStepsPerMM);  // adj for error
  }
  if(abs(deltaY)>=4){
    stepTo(yStepper, (deltaY / yPixelsPerMM) * yStepsPerMM);  // blocking microadjst run, inverse of pixel error
  
  }
  delay(300);
  
  // third confirmation picture
  // Serial2.println("COR" + String(currentPort) + "\n"); // return to "CONFRM"
  // Serial2.flush();
  // // wait for resp..
  // delay(50);
  // deltaX = Serial2.readStringUntil('\n').toInt();
  // deltaY = Serial2.readStringUntil('\n').toInt();

  // if (abs(deltaX) >= 4){
  //   stepTo(xStepper, (deltaX / xPixelsPerMM) * xStepsPerMM);  // adj for error
  // }
  // if(abs(deltaY)>4){
  //     stepTo(yStepper, (deltaY / yPixelsPerMM) * yStepsPerMM);  // blocking microadjst run, inverse of pixel error
  // }

  Serial2.println("CONFRM" + String(currentPort) + "\n");

  return true;
}

void calibIllum(){
  float gain = 0.0;
  String gainString;
  lights(0);
  //request illumination snapshot from rpi
  Serial2.setTimeout(2500);
  Serial2.println("ci"); Serial2.flush();
  gainString = Serial2.readStringUntil('\n');

  if (gainString == "") {
    return;
  }
  
  gain = gainString.toFloat();
  int currentBrightness = FastLED.getBrightness();
  int newBrightness = currentBrightness * gain;
  FastLED.setBrightness(int(newBrightness));
  FastLED.show();
  delay(20);
  FastLED.clear();

  // report
  if(verbose){
    Serial.println("prev brightness: " + String(currentBrightness) + " adj: " + String(gain));
    Serial.println("brighness: " + String(FastLED.getBrightness()));
  }
}

void setup() {
  // buzzer
  pinMode(buzzer, OUTPUT);
  tone(buzzer, 1000, 50);
  delay(50);
  // establish communication
  Serial.begin(115200);   // status to user through Serial prints
  // Serial1.begin(9600); // disabled, comm with LORA module e220
  Serial2.begin(9600);    // co{"commandType":2, "port":2}mm with raspberry pi zero
  Serial3.begin(9600);  // reserved, comm between teh ESP8266 to send and receive TCP commands
  delay(2000);
  Wire.begin();  // current modem, i2c seemed damaged

  // initiate light controls
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);  // Initialize LEDs
  FastLED.setBrightness(100); //125

  //> enable connect cellular
  // notecard.begin(Serial2, 9600); // notecard.begin(); default comm over i2c. alternatively  Serial2 is pins  17 and 16
  // notecard.setDebugOutputStream(Serial);
  // connect_to_LTE(); //attempt to establish a connection to the cell service
  // delay(4000);
  // establish_data_interrupt();  // establish interrupt service bewteen notecard modem and arduino

  // broadcast wifi
  //  while(!createHotspot()){}  // wait until wifi hotspot server is created

  // comment out to include hotspot attempt
  // joinHotSpot(); // make five attempts to join a phone's hotspot. Attempts to join wifi are made right after res

  // assign desired stepper motor speeds and accelerations
  yStepper.setMaxSpeed(15000);  // max 5000 steps per sec
  yStepper.setSpeed(14000);     // standard speed, 2000 fastest
  yStepper.setAcceleration(12000);

  xStepper.setMaxSpeed(xStepperDefaultMaxSpeed);
  xStepper.setSpeed(xStepperDefaultSpeed);  // 3000 fastest
  xStepper.setAcceleration(12000);

  zStepper.setMaxSpeed(19000);
  zStepper.setSpeed(18000);  // 3000 fastest
  zStepper.setAcceleration(16000);

  // attach interrupt events and pins
  pinMode(dataInterruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dataInterruptPin), attnISR, RISING);  // RISING
  pinMode(setupModePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(setupModePin), setupISR, LOW);
  // attachInterrupt(setupModePin, setupISR, FALLING);

  // pinMode(yStop, INPUT);
  // attachInterrupt(digitalPinToInterrupt(yErrorPin),yErrorISR, FALLING);  // listen to y axis driver error going high

  // pinMode(xErrorPin, INPUT);
  // attachInterrupt(digitalPinToInterrupt(xErrorPin),xErrorISR, FALLING);  // listen to y axis driver error going high

  pinMode(A9, OUTPUT);
  digitalWrite(A9, LOW);  // direction of hall sensor encoder
  pinMode(zStop, INPUT);
  // attachInterrupt(digitalPinToInterrupt(zStop), zStopISR, FALLING);
  pinMode(xStop, INPUT);
  // attachInterrupt(digitalPinToInterrupt(xStop), xStopISR, FALLING);
  pinMode(yStop, INPUT);
  // attachInterrupt(digitalPinToInterrupt(yStop), yStopISR, FALLING);
  pinMode(setupPin, INPUT_PULLUP);

  // arm attn pin
  // attnArm();

  // laser pin
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);

  // enable-disable motors
  pinMode(motorEnable, OUTPUT);
  digitalWrite(motorEnable, LOW);
  pinMode(laserPin, OUTPUT);

  // load data  from EEPROM memory
  // EEPROM.get(22, yStartPosition);  // update y offset after y endstop
  // EEPROM.get(1, xStartPosition);   // update x offset
  Serial.println("EEPROM offset x: " + String(xStartPosition));
  Serial.println("EEPROM offset y: " + String(yStartPosition));
  Serial.println("homing..");

  // ** ENABLE WHEN WORKING VIA CLOUD SERVER ** //
  // conv("EEPROM offset x: " + String(xStartPosition)+"\n\r");
  // conv("EEPROM offset y: " + String(yStartPosition)+"\n\r");
  // conv("homing..\n\r");
  homeAxis(zStepper, zStop);
  homeAxis(xStepper, xStop);
  homeAxis(yStepper, yStop);
  digitalWrite(motorEnable, HIGH);  // disable motors to save energy
  Serial.println("homed");
  // conv("homed\n\r");
  Serial2.println("homed\n");
  Serial.print("light is off");

  lights(6);  // turn on the middle light for illumination. 
  FastLED.clear();
  FastLED.show();
  // confirmByLTE(0,2); //> enable if cellular is used
  // flushNotes();  //> enable for cellular. Flush any old notes in the instructions queue
  // calibIllum();

}

void loop() {
  String tcp;

  // wifi hotspot connection?
  if (Sta && !tcpLink) {
    // tcpConnectHTTP(); // if not tcp, turn on server and listen for connections instead.
    //  joinHotSpot();  // attemp to reconnect again
    // Serial2.println("{\"tcpLink\": \"False\"}");
  } else if (Sta && tcpLink) {  // online mode - if wifi was confirmed and tcp link was established
    // Use Terminal of a comp that is on the same wifi as the machine, nc <ip address of machine>, space, 333. Then use jason to send commands
    // nc 192.168.40.61 333
    // {"commandType":2, "port":2}
    // or to connect to goolge vm server, fronm terminal: nc 34.57.37.242 4000

    // if(!tcpConnection) {ackTCP();} //acknowledge of connection once. tcpConnection flag sets to true.

    // first check if message from wifi/hotspot is available, read from SERIAL3 TCP data
    // this is a direct wifi/tcp connection to esp chip on arduino
    if (Serial3.available()) {
      Serial.println("MSG RCEVED!");
      tcp = Serial3.readString();
      // tcp.remove(0, tcp.indexOf(":") + 1);  // removes +IDP:7
      // tcp.remove(0, tcp.indexOf(":") + 1);
      parse(tcp);  // pass tcp string to interpret the message into actionable commands
    }
  }


//  if (Serial1.available() > 15){
//     String loraMessage = Serial1.readString();
//     parse(loraMessage);
//   }

  // manual input on ardu ide
  if (Serial.available()) {
    Serial.println("msg!");
    String userInput = Serial.readString();
    if(userInput.startsWith("rpi:")){
      Serial2.println(userInput); // send message to the RPI directly
    }
    else {                        // send message to arduino 
      parse(userInput);
    }
  }


  // secondly, check for any serial messsageds from RPi
  // this is a connection to RPI zero, which then connects to the arduino
  if (Serial2.available() > 0) {
    // if RPi message contains "commandType", it is a command message from the user, pass to parse();
    // else use for other RPi messages, e.g. pixel offsets requests from,to arduino
    String rpiMessage = Serial2.readString();
    // Serial.println(rpiMessage);
    parse(rpiMessage);
  }

 

  // at this stage, the relevant global variables like requestedPort, requestedCommandType should be set.

  // rapid mode?
  if (rapidMode) {
    rapidMove();
  }

  // setup mode?
  if (digitalRead(setupPin) == LOW && setupMode == false) {
    unsigned long startTime = millis();
    unsigned long curTime;
    while (digitalRead(setupPin) == LOW) {
      curTime = millis();
      // Serial.println((curTime-startTime));
      if ((curTime - startTime) > 2000) {
        // Serial.println("SETUPMODE");
        setupMode = true;
        setupModeToggle();
        break;
      }
      delay(50);
    }
  }

  // cue mode?
  if (cueMode) {
    cueMove();
  }

  // message from cellular?
  // if (attn) {
  //   Serial.println("attn!");
  //   attn = false;
  //   readModem();  // read instruction from the modem and interpret what to do > updates global 'requestedPort'
  // }

  // which mode?
  switch (requestedCommandType) {
    case 15:              // setup
      setupModeToggle();  // setup mode. manual insert probe into port 1, press adj buton (lower butter left of x motor). machine remembers port 1 pos.
      break;

    case 4:
      cueModeToggle(true);  // machine auto inserts into next port based on photo sensor, connected to the OLTS/meter machines LED
      break;

    case 10:
      resetMachine();  // restart
      break;

    case 11:
      releaseProbe();  // push probe out so you can insert cable
      break;

    case 12:
      acceptProbe();  // pull in probe to standby position
      requestedPort = 0;
      break;

    case 6:
      Serial.print("adj by reqValue ");
      Serial.println(requestedValue);
      digitalWrite(motorEnable, LOW);  // power motors on
      stepTo(xStepper, requestedValue * 10);
      digitalWrite(motorEnable, HIGH);  // power motors off
      requestedPort = 0;
      requestedCommandType = 0;
      break;

    default:
      if (requestedCommandType < 4) {
        cueModeToggle(false);
      }
      break;
  }
  
  // what port?
  if (currentPort == 0 && requestedPort > 0 && requestedPort < maxAllowedPort) {  // if we have a new position to port request and Z is clear.. (below 190 no space for test version)
    positionProbe(requestedPort);   

    // calculate how many steps and dir to the requestedPort
    digitalWrite(motorEnable, LOW);                                               // power motors on

    // make sure probe is retracted before moving
    if (digitalRead(zStop) == HIGH) {
      homeAxis(zStepper, zStop);
    }
  }

  // move steppers
  yStepper.run();
  if (yStepper.distanceToGo() == 0) {
    xStepper.run();
  }

  //   // reached destination?
  if (currentPort != 0 && yStepper.distanceToGo() == 0 && xStepper.distanceToGo() == 0) {  // check that x and y movements are complete, then use the probe
    Serial.println();
    lights(currentPort);
    delay(500);

    // microAdjust();

    // action at the port
    if (requestedCommandType != 2) {  // insert probe or lase
      lights(currentPort);
      if (!cameraAdjust()) {
        requestedCommandType = 2;  // camera only, dont insert
        return;
      }
      zStepper.move(plungeDepth);  // insert probe
      while (zStepper.distanceToGo() != 0) {
        zStepper.run();
      }
      digitalWrite(motorEnable, HIGH);  // motors off
      // lightsOff delay to allow user to observe the result
      delay(1000);  // user observation delay
      lights(-currentPort);
      digitalWrite(motorEnable, HIGH);  // motors off
    }

    else {  // dont connect probe
      st = 0;
      if (!cameraAdjust()) {
        Serial.println("cap or obstruction");
      }
      digitalWrite(motorEnable, HIGH);  // motors off
      delay(1000);
      lights(-currentPort);
    }

    // notify app {"commandType":1,"port":3}of arrival, via wifi
    if (received) {
      if (portsToRapid == 0) {  // send confirmation to the server, unless in rapid mode. In rapid mode, ack after group connect is done
        conv("OK\n\r");
      }
      received = false;
    }

    xStepperPrevPos = xStepper.currentPosition();
    encoderPrevDeg = xPos();
    previousPort = currentPort;
    requestedPort = 0;
    currentPort = 0;
  }

  // watchdog(); // check if TCP connection is alive every min; joinHotSpot() and reconnect if not
}

// NEEDS TO BE DONE:
// 1 MAJOR OBJECTIVE RELIABILITY, PORT AIMING.
//   √ POSITION RELIABILITY ABS ENCODER (1/2 implemented as rotary encoder not true absolute)
//   √ port 12  working
//   √ CAMERA AIMING
//   PREVENT PORT CONFUSION
//    BY CROSS-CHECKING WITH ENCODER: max error set to +/- 2mm of projected position
//      or deviation from the table - same thing
//      if cameraAdjust returns an excessive adjustment, go by encoder positioning
//      if cameraAdjust returns an excessive adj, evaluate image quality. 
//    BY CAMERA ADJUST, INDEPENDENTLY SET TO ADJ AT: max error set to +/- 2mm

//   (!) PREDICTABLE ILLUMINATION or AUTO ADJUSTED illumination
//    x illumination failed

// MAJOR OBJECTIVE RELIABILITY,  CONNECT RELIABILITY.
//  DETECT FAILED CONNECT (motor stall? encoder?)
//  FAILED FULL INSERT (encoder? narrow range)
//  PORT ANAMOLY DETECTION - CAP DETECTION (white color filter test for empty port?)

// MAJOR OBJECTIVE RELIABLE COMMUNICATION - SERVER AND CELLULAR NOT RELIABLE

// unused function

// void tcpConnect() {  // notify of tcp connection
//   if (!tcpFunc) {
//     Serial.println("awaiting a tcp connection");
//     tcpFunc = true;
//   }

//   String tcp;
//   if (Serial3.available() > 0) {
//     tcp = Serial3.readString();
//     Serial.println(tcp);

//     if (tcp.indexOf("0,CONNECT") != -1) {  // detect first connection
//       Serial.println("tcp");
//       tcpLink = true;
//       delay(60);
//       if (tcp.indexOf("GET / HTTP/1.1") != -1 || tcp.indexOf("GET /favicon.ico HTTP/1.1") != -1) {  // first time load, request webpage
//         Serial.println("initial connect ack / sending webpage");
//         Serial.println();

//         // if(conv(webpage)){  // wait until convey func sends page, returns true
//         //   instr("AT+CIPCLOSE=0"); // ! close only the connection that was opened
//         //   // Serial3.flush(); //
//         // }
//       } else if (tcp.indexOf('?') != -1) {  // if request containts a ?, it is likly a GET request
//         Serial.print("connect & req. tcp parcing ");
//         tcpLink = true;
//         // parseWebRequest(tcp);
//         Serial.println(parseWebRequest(tcp));  // clips string from ? to http/1.1
//         // Serial.print(Serial3.available());
//         // // Serial3.flush();
//         // refreshPage();
//       } else if (tcp.indexOf("Server!") != -1) {
//         Serial.println("hello Server rcvd");
//         tcpLink = true;
//         conv("arduino ready");
//       }
//     }
//   }
// }