/*
  This code is intended on the Matt Zwarts 39% Mini Droid with 2 dc motors for the foot drives and servo drive for the dome and front arm(s)
  Or 2 DC motors for the foot drives and a DC motor for the dome
  Within the code the servo position minimum and maximum values will need to be set, the values are near the top of the code
  Servos can be attached to pins, A0, A1, A2, A3 and A4 to achieve required funstions, ensure a seperate power supply of 5Volts is given with a common
  ground to all electronics and then data pins for the servos

  Attach the motor driver to pins noted below
  The servo for the front arm attaches to Ananlog pin notoed in code below and the dome Continuous servo attaches to analog pin noted in code below
  The LEDs for the dome can be attached to Pin 12, the leds will blink when the sounds are playing and continue to blink with auto sounds

  Be sure to add the NeoSWSerial, MX1508 libraries to your arduino libraries folders in Tools/ ManageLibraries

  Run the Serial Monitor to ensure the app is connecting via bluetooth and the values are coming in correct before connecting to the droid

  Happy Building, Matt
  ///////////////////////////////////////////////////////////////////////////////////////
  //Connections
  ///////////////////////////////////////////////////////////////////////////////////////
  HC-05  - Arduino
  VCC   -   5V
  GND   -   GND
  RXD   -   TXD
  TXD   -   RXD
*//////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <Servo.h>              //Servo library
#include <NeoSWSerial.h>        //lookup library and add, search in Tools/Manage Libraries then search for NeoSWSerial
#include <MX1508.h>   // motor driver for cheap Chinese tiny dual motor drivers, use PWM pins on arduino

//NeoSWSerial ss( 8, 7 ); // RX, TX //this can be swapped for below line if the software serial communication is not working
NeoSWSerial ss( 7, 8 ); // RX, TX

const int baudrate = 9600;

#define INPUT_SIZE 30

#define IN1 5 //Left Motor Driver IN1
#define IN2 3 //Left Motor Driver IN2
#define IN3 6 //Right Motor Driver IN1
#define IN4 9 //Right Motor Driver IN2
#define IN5 10 //Dome Motor Driver IN1    //uncomment if using dc motor for dome
#define IN6 11 //Dome Motor Driver IN2    //uncomment if using dc motor for dome

////////////////////Variables that can be changed to cusomise////////////////////////////////////////////////////////////////////////////////////////////

//NOTE: adjust the servos until they just move to correct positions to avoid them overheating by trying to drive to a position they can't achieve

// Variables to adust for front arm open and close limits for servo movement
int frontarm1min = 50;
int frontarm1max = 140;

int frontarm2min = 50;
int frontarm2max = 140;

// Variables to adust for Centre leg lift and Tilt servo mechanism
int tiltmin = 70;
int tiltmax = 120;
int liftup = 70;
int liftdown = 120;

// Variables to adust for periscope lift servo
int periup = 120;
int peridown = 70;

// values to adust for the LED random number blink, min and max delays
int ledonmin = 300; // value in milliseconds
int ledonmax = 500;
int ledoffmin = 30; // value in milliseconds
int ledoffmax = 100;

// Deadzone value to reduce centre position error
const int deadzonexy = 15; //original value is 15, this will give the motors enough power to move, add eliminate centering jitter

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Define the motor driver details on the MX1508
MX1508 motorA(IN1, IN2, FAST_DECAY, 2);
MX1508 motorB(IN3, IN4, FAST_DECAY, 2);
MX1508 motorC(IN5, IN6, FAST_DECAY, 2); //dome motor uncomment if using dc motor for dome

Servo servoArm1;     //define servos for front arm and head rotation
Servo servoArm2;
Servo servoDome;    //uncomment if using a servo motor

Servo servoTilt; //define servos for 2-3-2 centre leg lift and tilt
Servo servoLift;
Servo servoPeriscope;

boolean newData = false;  // check for data received

int x1 = 0;                     // value from joystick
int y1 = 0;                     // value from joystick
int x2 = 0;                     // value from joystick
int y2 = 0;                     // value from joystick
int soundbutton = 0;           //value for the sound player, can be used to trigger lights
int lift = 0;
int tilt = 0;
int periscope = 0;

int ledPin = 12;           // pin for the LED in the dome to be turned on and off
int ledinterval = 200;    // delay to blink the LED lights in the dome
int ledState = LOW;             // ledState used to set the LED

float output1 = 0;              // Output from first channel
float output2 = 0;              // Output from second channel
float output3 = 0;              // Output for Dome motor
float output4 = 0;              // Front arm servo
float diff;                       // value for differential steering

int centrelift;
int legtilt;
int periposition;

float rawLeft;
float rawRight;

float dome_speed = 0.75;                                     // dome rotation speed multiplier, increase or decrease value to speed up dome or slow down

float elapsedTime, time, timePrev;

boolean debug = true;

static unsigned long lastMilli = 0;
unsigned long currentMillis; // time current
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long lastservotime = 0;
unsigned long lastDebounceTime = 0;  // place holder for debounce time
unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers


//LED states
long led1_state = 0;

long loop_timer;

/////////////////////////////////// SETUP ////////////////////////////////////////////////////
void setup() {
  Serial.begin(baudrate);                                                 //Used only for debugging on arduino serial monitor
  Serial.println("Mini Droid!");

  ss.begin(baudrate);
  ss.setTimeout(100);

  time = millis();                                                     //Start counting time in milliseconds

  //output pins for motor drivers
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);                           //uncomment if using dc motor for dome
  pinMode(IN6, OUTPUT);                           //uncomment if using dc motor for dome

  // Write the motor pins low so they dont start on power up
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);                          //uncomment if using dc motor for dome
  digitalWrite(IN6, LOW);                          //uncomment if using dc motor for dome

  pinMode(ledPin, OUTPUT);

  loop_timer = millis();                                               //Reset the loop timer

  output1 = 0;
  output2 = 0;
  output3 = 0;                                      //uncomment if using dc motor for dome
  motorA.motorGo(output1);
  motorB.motorGo(output2);
  motorC.motorGo(output3);                          //uncomment if using dc motor for dome

  ///////////////////////////Change pin numbers below to suit your setup
  // Setup the servos
  servoArm1.attach(A0); // Front Arm1 servo pin
  servoArm2.attach(A1); // Front Arm2 servo pin
  servoLift.attach(A2); // Front Arm servo pin
  servoPeriscope.attach(A3); // Periscope servo pin
  servoDome.attach(A4); // Dome servo pin         //uncomment if using a servo motor

  ////write servos to there start positions, change if you want the droid to start in a different pose when turned on
  servoArm1.write(frontarm1min);
  servoArm2.write(frontarm2min);
  servoLift.write(liftdown);
  servoPeriscope.write(peridown);
  servoDome.write(90);                            //uncomment if using a servo motor

  delay(1000);
}


// start of loop ///////////////////////////////////////////////////////////////////////
void loop() {
  currentMillis = millis();
  timePrev = time;                                                      // the previous time is stored before the actual time read
  time = millis();                                                      // actual time read
  elapsedTime = (time - timePrev);                                      //elapsedTime = (time - timePrev) / 1000;
  lastDebounceTime = millis();

  // called functions////////////////////////////////////////////////////////////////////////
  bluetooth_data();
  text();                                                              // check the motor outputs are correct, connect the usb to the arduino, open the serial monitor in arduino, and connect the phone app
  motordriver();                                                      // motor drivers function
  //domeservo();                                                              // Dome servo motor drive
  dome();                                                              // Dome motor drive
  frontarm();                                                         // front arm motion
  periscopemotion();                                                        //periscope motion
  lights();                                                           //Flicker dome lights when sounds are played

  if (millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentMillis;
    liftmechanism();
    tiltmechanism();
  }

  lastMilli = millis();
}
//end of loop ///////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////
void bluetooth_data() {

  char rawData[100] = "";
  char keyword[] = "<<";


  //if (MySerial.available() > 0) {//new data in
  //size_t byteCount = MySerial.readBytesUntil('\n', rawData, sizeof(rawData) - 1); //read in data to buffer
  if (ss.available () > 0) {//new data in
    size_t byteCount = ss.readBytesUntil('\n', rawData, sizeof(rawData) - 1); //read in data to buffer
    rawData[byteCount] = NULL;//put an end character on the data
    //Serial.print("Raw Data = ");
    //Serial.println(rawData);
    //now find keyword and parse
    char *keywordPointer = strstr(rawData, keyword);
    if (keywordPointer != NULL) {
      int dataPosition = (keywordPointer - rawData) + strlen(keyword); //should be position after rawData=
      const char delimiter[] = ",";
      char parsedStrings[8][30];
      int dataCount = 0;
      char *token =  strtok(&rawData[dataPosition], delimiter);//look for first piece of data after keyword until comma
      if (token != NULL && strlen(token) < sizeof(parsedStrings[0])) {
        strncpy(parsedStrings[0], token, sizeof(parsedStrings[0]));
        dataCount++;
      } else {
        //Serial.println("token too big");
        strcpy(parsedStrings[0], NULL);
      }
      for (int i = 1; i < 8; i++) {
        token =  strtok(NULL, delimiter);
        if (token != NULL && strlen(token) < sizeof(parsedStrings[i])) {
          strncpy(parsedStrings[i], token, sizeof(parsedStrings[i]));
          dataCount++;
        } else {
          //Serial.println("token to big");
          strcpy(parsedStrings[i], NULL);
        }
      }
      //Serial.print("Found ");
      //Serial.print(dataCount);
      //Serial.print(" strings:");
      //for (int i = 0; i < 8; i++){
      //Serial.print(parsedStrings[i]);Serial.print(""),Serial.print(",");
      //}
      //Serial.println("");
      if (dataCount == 8) {
        x1 = atoi(parsedStrings[0]);
        y1 = atoi(parsedStrings[1]);
        x2 = atoi(parsedStrings[2]);
        y2 = atoi(parsedStrings[3]);
        soundbutton = atoi(parsedStrings[4]);
        lift = atoi(parsedStrings[5]);
        tilt = atoi(parsedStrings[6]);
        periscope = atoi(parsedStrings[7]);
      }
      else {
        //Serial.println("data no good");
        //ss.flush();
      }
    }
    else {
      //Serial.println("sorry no keyword");
    }
  }
  else { /////////////////////Write the motors to low if no signal is present
    output1 = 0; output2 = 0;
    //output3 = 90;
    output3 = 0;                            //uncomment if using dc motor for dome
    motorA.motorGo(output1);
    motorB.motorGo(output2);
    motorC.motorGo(output3);                //uncomment if using dc motor for dome
    periscope = 0;
    lift = 0;
    tilt = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////////



void motordriver() {     // for the main drive wheels

  //////////////////////////read in the values from bluetooth///////////////////
  output1 = y1;
  output1 = map(output1, 0, 200, 255, -255);

  output2 = x1;
  output2 = map(output2, 0, 200, 255, -255);

  rawLeft = output2 + output1;
  rawRight = output2 - output1;

  diff = abs(abs(output2) - abs(output1));
  rawLeft = rawLeft < 0 ? rawLeft - diff : rawLeft + diff;
  rawRight = rawRight < 0 ? rawRight - diff : rawRight + diff;

  rawLeft = constrain(rawLeft, -255, 255); // constrain the PWM to max 255
  rawRight = constrain(rawRight, -255, 255);
  motorA.motorGo(rawLeft);
  motorB.motorGo(rawRight);
}


////////////////////////////////////////////////////////////////////////////////////////

//Uncomment if using a servo motor for the dome rotation


void domeservo() {

  output3 = x2;
  output3 = constrain(output3, 0, 200);
  output3 = map(output3, 0, 200, 0, 180);

  ///////////////////// change the deadzone value for greater sensitivity
  if (output3 > 100 || output3 < 80) {
    output3 = constrain(output3, 0, 180);
    servoDome.write(output3);
  }
  else {
    servoDome.write(90);
  }

}


////////////////////////////////////////////////////////////////////////////////////////

//uncomment out if using a dc motor for the dome

void dome() {

  output3 = x2;
  output3 = map(output3, 0, 200, -255, 255);

  //////////////////////////////////Motor driver direction control/////////////////////
  if (output3 > deadzonexy) {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  }
  else if (output3 < -deadzonexy) {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }
  else {
    output3 = 0;
  }
  output3 = constrain(output3, -255, 255); // constrain the PWM to max 255
  motorC.motorGo(output3 * dome_speed);
}


////////////////////////////////////////////////////////////////////////////////////////

void frontarm() {

  output4 = y2;
  output4 = constrain(output4, 100, 200);
  output4 = map(output4, 100, 200, frontarm1min, frontarm1max);

  ///////////////////// change the deadzone value for greater sensitivity
  if (output4 < frontarm1max || output4 > frontarm1min) {
    output4 = constrain(output4, frontarm1min, frontarm1max);
    servoArm1.write(output4);
  }
  else {
    servoArm1.write(frontarm1min);
  }

  if (output4 < frontarm2max || output4 > frontarm2min) {
    output4 = constrain(output4, frontarm2min, frontarm2max);
    servoArm2.write(output4);
  }
  else {
    servoArm2.write(frontarm2min);
  }
}


////////////////////////////////////////////////////////////////////////////////////////

void liftmechanism() {

  centrelift = lift;

  if (centrelift == 0) {
    servoLift.write(liftup);
  }
  else if (centrelift = 1) {
    servoLift.write(liftdown);
  }
}




////////////////////////////////////////////////////////////////////////////////////////

void tiltmechanism() {

  legtilt = tilt;

  if (centrelift = 1) { // check the leg is down before tilting the body
    if (legtilt == 0) {
      servoTilt.write(tiltmin);
    }
    else if (legtilt = 1) {
      servoTilt.write(tiltmax);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void periscopemotion() {

  if (periscope = 0) {
    servoPeriscope.write(peridown);
  }
  else if (periscope = 1) {
    servoPeriscope.write(periup);
  }
}



////////////////////////////////////////////////////////////////////////////////////////

void lights() {         // function to flash the dome lights when a sound is being played

  if (soundbutton != 0) {
    if (currentMillis - previousMillis >= ledinterval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
        ledinterval = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
      } else {
        ledState = LOW;
        ledinterval = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
      }
      digitalWrite(ledPin, ledState);
      //ledinterval = random(ledmin, ledmax);       //set a random time dealy when the sound is being played
    }
  }
  else {
    digitalWrite(ledPin, HIGH); // leave the lights on when no sounds are being played
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
void text() { // uncomment lines for debugging
  //Serial.print("elapsedtime:"); Serial.print(elapsedTime); Serial.print("\t");
  Serial.print("out1:"); Serial.print(rawLeft); Serial.print("\t");
  Serial.print("out2:"); Serial.print(rawRight); Serial.print("\t");
  Serial.print("out3:"); Serial.print(output3); Serial.print("\t");
  Serial.print("out4:"); Serial.print(output4); Serial.print("\t");
  Serial.print("lift:"); Serial.print(lift); Serial.print("\t");
  Serial.print("tilt:"); Serial.print(tilt); Serial.print("\t");
  Serial.print("periscope:"); Serial.print(periscope); Serial.print("\t");
  Serial.print("sound:"); Serial.print(soundbutton); Serial.println("\t");
}
