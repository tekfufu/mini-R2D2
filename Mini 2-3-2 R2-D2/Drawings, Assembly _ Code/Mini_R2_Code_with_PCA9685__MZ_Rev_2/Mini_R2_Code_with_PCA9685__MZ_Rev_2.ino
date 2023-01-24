/*
  This code is intended on the Matt Zwarts 39% Mini Droid with 2 dc motors for the foot drives and servo drive for the dome and front arm(s)
  Or 2 DC motors for the foot drives and a DC motor for the dome
  Within the code the servo position minimum and maximum values will need to be set, the values are near the top of the code

  Attach the motor driver to pins noted below
  The servos are driven by a PCA9685 servo board which connects to the 5V, GND, and A4 and A5

  Be sure to add the NeoSWSerial, MX1508 and Adafruit_PWMServoDriver libraries to your arduino libraries folders in Tools/ ManageLibraries

  Run the Serial Monitor to ensure the app is connecting via bluetooth and the values are coming in correct before connecting to the droid

  Happy Building, Matt Zwarts

*//////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <math.h>
#include <NeoSWSerial.h>        //lookup library and add, search in Tools/Manage Libraries then search for NeoSWSerial
#include <MX1508.h>   // motor driver for cheap Chinese tiny dual motor drivers, use PWM pins on arduino
#include <Adafruit_PWMServoDriver.h>

//Call boards for i2c
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// our servo # counter
uint8_t servonum = 0;

//NeoSWSerial ss( 8, 7 ); // RX, TX //this can be swapped for below line if the software serial communication is not working
NeoSWSerial ss( 7, 8 ); // RX, TX

const int baudrate = 9600;

#define INPUT_SIZE 30

#define IN1 5 //Left Motor Driver IN1 5
#define IN2 3 //Left Motor Driver IN2 3
#define IN3 6 //Right Motor Driver IN1 6
#define IN4 9 //Right Motor Driver IN2 9
#define IN5 10 //Dome Motor Driver IN1    //uncomment if using dc motor for dome
#define IN6 11 //Dome Motor Driver IN2    //uncomment if using dc motor for dome

////////////////////Variables that can be changed to cusomise////////////////////////////////////////////////////////////////////////////////////////////

//NOTE: adjust the servos until they just move to correct positions to avoid them overheating by trying to drive to a position they can't achieve
// Second note is that the values are in the range of 150 - 600 for the PCA9685 to write the correct position, this is the pulse length calculation, I've mapped it in the
//code so no worries and just add the values below for servo ranges in degrees

// Variables to adust for front arm open and close limits for servo movement
int frontarm1min = 30;
int frontarm1max = 170;

int frontarm2min = 30;
int frontarm2max = 170;

// Variables to adust for Centre leg lift and Tilt servo mechanism
int tiltmin = 133;
int tiltmax = 40;

int liftup = 125;
int liftdown = 28;

// Variables to adust for periscope lift servo
int periup = 25;
int peridown = 135;

// Variables for dome servo
int domeservomin = 30;
int domeservomax = 150;
int domeservocentre = 90;

// values to adust for the LED random number blink, min and max delays
int ledonmin = 200; // value in milliseconds
int ledonmax = 500;
int ledoffmin = 50; // value in milliseconds
int ledoffmax = 150;

// Holo position values
int holoCentreVal = 300;      // change this value if the servos for the holo proectors keep rotating

int pos1 = 0;    // variable to store the servo1 position
int pos2 = 0;    // variable to store the servo2 position
int pos3 = 0;    // variable to store the servo3 position

int randompos1 = 0; //random number for servo 1
int randompos2 = 0; //random number for servo 2
int randompos3 = 0; //random number for servo 3

int randomtime1 = 0; // random time delay for movement 1
int randomtime2 = 0; // random time delay for movement 2
int randomtime3 = 0; // random time delay for movement 3
int randomtime4 = 0; // random time delay for movement 4

// Dome lights
int hololedinterval1 = 200;    // delay to blink the LED lights in the dome
int hololedinterval2 = 200;    // delay to blink the LED lights in the dome
int hololedinterval3 = 200;    // delay to blink the LED lights in the dome
int periledinterval = 500;    // delay to blink the LED lights in the Periscope
int hololedState1 = LOW;       // ledState used to set the LED
int hololedState2 = LOW;       // ledState used to set the LED
int hololedState3 = LOW;       // ledState used to set the LED
int periledState = LOW;       // ledState used to set the LED
int hololedonmin = 300;
int hololedonmax = 1000;
int hololedoffmin = 100; // value in milliseconds
int hololedoffmax = 500;
int periledonmin = 300; //periscope LED on off values
int periledonmax = 600;
int periledoffmin = 300; //
int periledoffmax = 600;



// Deadzone value to reduce centre position error
const int deadzonexy = 30; //original value is 30, this will give the motors enough power to move, add eliminate centering jitter
const int domedeadzonexy = 40;  // dome centre original value is 40, this will give the motors enough power to move, add eliminate centering jitter

// Upright 2 legged drive speed multiplier, when in 2 leg mode the drive speed is reduced by multplying it by this factor
float uprightdrivespeed = 0.30;

// Dome speed is slowed down on the dome motor rotation by this multiplying value
float dome_speed = 0.60;                                     // dome rotation speed multiplier, increase or decrease value to speed up dome or slow down

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Define the motor driver details on the MX1508
MX1508 motorA(IN1, IN2, FAST_DECAY, 2);
MX1508 motorB(IN3, IN4, FAST_DECAY, 2);
MX1508 motorC(IN5, IN6, FAST_DECAY, 2); //dome motor uncomment if using dc motor for dome

boolean newData = false;  // check for data received

int x1 = 0;                     // value from joystick
int y1 = 0;                     // value from joystick
int x2 = 0;                     // value from joystick
int y2 = 0;                     // value from joystick
int soundbutton = 0;           //value for the sound player, can be used to trigger lights
int lift = 0;
int tilt = 0;
int periscope = 0;

int ledPin = 13;           // pin for the LED in the dome to be turned on and off
int ledinterval = 200;    // delay to blink the LED lights in the dome
int ledinterval1 = 200;
int ledinterval2 = 200;
int ledinterval3 = 200;
int ledinterval4 = 200;
int ledState = LOW;             // ledState used to set the LED
int ledState1 = LOW;
int ledState2 = LOW;
int ledState3 = LOW;
int ledState4 = LOW;

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

float elapsedTime, time, timePrev;

boolean debug = true;

static unsigned long lastMilli = 0;
unsigned long currentMillis; // time current
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long previousMillis4 = 0;
unsigned long previousMillis5 = 0;
unsigned long previousMillis6 = 0;
unsigned long previousMillis7 = 0;
unsigned long previousMillis8 = 0;
unsigned long previousMillis9 = 0;
unsigned long previousMillis10 = 0;
unsigned long previousMillis11 = 0;
unsigned long previousMillis12 = 0;
unsigned long previousMillis13 = 0; // used for testing only
unsigned long previousMillis14 = 0; // used for testing only
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

  pwm1.begin();
  pwm1.setPWMFreq(50);  // standard for analog servos

  pwm2.begin();
  pwm2.setPWMFreq(50);  // standard for analog servos

  ss.begin(baudrate);
  ss.setTimeout(100);

  time = millis();                                                     //Start counting time in milliseconds

  // Convert values from degrees to Pulse ranges for the servo positions to the PCA9685 board 0-180 to 150-600
  frontarm1min = (((450 / 180) * frontarm1min) + 150);
  frontarm1max = (((450 / 180) * frontarm1max) + 150);
  frontarm2min = (((450 / 180) * frontarm2min) + 150);
  frontarm2max = (((450 / 180) * frontarm2max) + 150);
  tiltmin = (((450 / 180) * tiltmin) + 150);
  tiltmax = (((450 / 180) * tiltmax) + 150);
  liftup = (((450 / 180) * liftup) + 150);
  liftdown = (((450 / 180) * liftdown) + 150);
  periup = (((450 / 180) * periup) + 150);
  peridown = (((450 / 180) * peridown) + 150);
  domeservomin = (((450 / 180) * domeservomin) + 150);
  domeservomax = (((450 / 180) * domeservomax) + 150);
  domeservocentre = (((450 / 180) * domeservocentre) + 150);

  //Set the servos to their start positions
  servoSetup(); // view function and end of code
  servoSetup2(); // view function and end of code

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
  //output3 = 300;                                      //uncomment if using servo for dome
  motorA.motorGo(output1);
  motorB.motorGo(output2);
  motorC.motorGo(output3);                          //uncomment if using dc motor for dome

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
  motordrivers();                                                      // motor drivers function
  //domeservo();                                                       // Dome motor drive
  dome();                                                              // Dome motor drive
  frontarm1();                                                         // front arm 1 motion
  frontarm2();                                                         // front arm 2 motion
  LED1();                                                           //Flicker dome lights when sounds are played
  LED2();
  LED3();
  LED4();
  LED5();
  holoLED1();                                                       // holo lights
  holoLED2();
  holoLED3();
  holoServo1();                                                       // randome holo servo timing
  holoServo2();
  holoServo3();

  if (millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentMillis;
    liftmechanism();
    tiltmechanism();
    periscopemotion();                                                   //periscope motion
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
    //output3 = domeservocentre;
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

void motordrivers() {     // for the main drive wheels

  //////////////////////////read in the values from bluetooth///////////////////
  output1 = y1;
  output1 = map(output1, 0, 200, 255, -255);

  output2 = x1;
  output2 = map(output2, 0, 200, -255, 255); // swap the -255 and 255 if the left and rigth turn the wrong way

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

//uncomment out if using a dc motor for the dome

void dome() {

  output3 = x2;
  output3 = map(output3, 0, 200, 255, -255);  // swap the -255 and 255 if the dome spins the wrong way
  output3 = output3 * dome_speed;
  if (output3 > domedeadzonexy || output3 < -domedeadzonexy) {
    output3 = constrain(output3, -255, 255); // constrain the PWM to max -255 to 255
  }
  else {
    output3 = 0;
  }
  motorC.motorGo(output3);
}

////////////////////////////////////////////////////////////////////////////////////////

//Uncomment if using a servo motor for the dome rotation


void domeservo() {

  output3 = x2;
  output3 = constrain(output3, 0, 200);
  output3 = map(output3, 0, 200, domeservomin, domeservomax);

  if (output3 < domeservomax || output4 > domeservomin) {
    output3 = constrain(output3, domeservomin, domeservomax);
    pwm1.setPWM(15, 0, output3);
  }
  else {
    pwm1.setPWM(15, 0, domeservocentre);
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void frontarm1() {

  output4 = y2;
  output4 = constrain(output4, 110, 200);
  output4 = map(output4, 110, 200, frontarm1min, frontarm1max);

  /////////////////////
  if (output4 < frontarm1max || output4 > frontarm1min) {
    output4 = constrain(output4, frontarm1min, frontarm1max);
    pwm1.setPWM(0, 0, output4);
  }
  else {
    pwm1.setPWM(0, 0, frontarm1min);
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void frontarm2() {

  output4 = y2;
  output4 = constrain(output4, 0, 90);
  output4 = map(output4, 0, 90, frontarm2max, frontarm2min);

  /////////////////////
  if (output4 < frontarm1max || output4 > frontarm1min) {
    output4 = constrain(output4, frontarm2min, frontarm2max);
    pwm1.setPWM(1, 0, output4);
  }
  else {
    pwm1.setPWM(1, 0, frontarm2min);
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void liftmechanism() {

  centrelift = lift;

  if (centrelift == 0) {
    pwm1.setPWM(3, 0, liftup);
  }
  else if (centrelift == 1) {
    pwm1.setPWM(3, 0, liftdown);;
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void tiltmechanism() {

  legtilt = tilt;

  if (centrelift == 1) { // check the leg is down before tilting the body
    if (legtilt == 0) {
      pwm1.setPWM(4, 0, tiltmin);
    }
    else if (legtilt == 1) {
      pwm1.setPWM(4, 0, tiltmax);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void periscopemotion() {

  periposition = periscope;

  if (periposition == 0) {
    pwm2.setPWM(10, 0, peridown); // periscope down
  }
  else if (periposition == 1) {
    pwm2.setPWM(10, 0, periup);
  }

  if (periscope == 1) { // blink the periscope LED
    if (currentMillis - previousMillis12 >= periledinterval) {
      previousMillis12 = currentMillis;
      if (periledState == LOW) {
        periledState = HIGH;
        periledinterval = random(periledonmin, periledonmax);       //set a random time dealy
        pwm2.setPWM(11, 0, 4096);
      }
      else {
        periledState = LOW;
        periledinterval = random(periledoffmin, periledoffmax);       //set a random time dealy
        pwm2.setPWM(11, 4096, 0);
      }
    }
  }
  else if (periscope == 0) {
    pwm2.setPWM(11, 0, 4096); // turn off the led in the periscope
  }

}


////////////////////////////////////////////////////////////////////////////////////////

void LED1() {         // function to flash the dome lights when a sound is being played

  if (soundbutton != 0) {
    if (currentMillis - previousMillis7 >= ledinterval) {
      previousMillis7 = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
        ledinterval = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(0, 4096, 0);
      }
      else {
        ledState = LOW;
        ledinterval = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(0, 0, 4096);
      }

    }
    else {
      pwm2.setPWM(0, 4096, 0);
    }
  }
}

void LED2() {         // function to flash the dome lights when a sound is being played

  if (soundbutton != 0) {
    if (currentMillis - previousMillis8 >= ledinterval1) {
      previousMillis8 = currentMillis;
      if (ledState1 == LOW) {
        ledState1 = HIGH;
        ledinterval1 = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(1, 4096, 0);
      }
      else {
        ledState1 = LOW;
        ledinterval1 = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(1, 0, 4096);
      }

    }
    else {
      pwm2.setPWM(1, 4096, 0);
    }
  }
}

void LED3() {         // function to flash the dome lights when a sound is being played

  if (soundbutton != 0) {
    if (currentMillis - previousMillis9 >= ledinterval2) {
      previousMillis9 = currentMillis;
      if (ledState2 == LOW) {
        ledState2 = HIGH;
        ledinterval2 = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(2, 4096, 0);
      }
      else {
        ledState2 = LOW;
        ledinterval2 = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(2, 0, 4096);
      }

    }
    else {
      pwm2.setPWM(2, 4096, 0);
    }
  }
}

void LED4() {         // function to flash the dome lights when a sound is being played

  if (soundbutton != 0) {
    if (currentMillis - previousMillis10 >= ledinterval3) {
      previousMillis10 = currentMillis;
      if (ledState3 == LOW) {
        ledState3 = HIGH;
        ledinterval3 = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(3, 4096, 0);
      }
      else {
        ledState3 = LOW;
        ledinterval3 = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(3, 0, 4096);
      }

    }
    else {
      pwm2.setPWM(3, 4096, 0);
    }
  }
}

void LED5() {         // function to flash the dome lights when a sound is being played

  if (soundbutton != 0) {
    if (currentMillis - previousMillis11 >= ledinterval4) {
      previousMillis11 = currentMillis;
      if (ledState4 == LOW) {
        ledState4 = HIGH;
        ledinterval4 = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(7, 4096, 0);
      }
      else {
        ledState4 = LOW;
        ledinterval4 = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
        pwm2.setPWM(7, 0, 4096);
      }

    }
    else {
      pwm2.setPWM(7, 4096, 0);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////

void holoServo1() {
  //Holo Servo1/////////////////////////////
  randomtime1 = random(500, 2000); //servo 1 random time position generation in milliseconds
  if (currentMillis - previousMillis1 >= randomtime1) {
    previousMillis1 = currentMillis;
    pos1 = random(150, 600);
    pwm2.setPWM(4, 0, pos1);
  }
  else {
    pwm2.setPWM(4, 0, holoCentreVal);
  }
}

void holoServo2() {
  //Holo Servo2//////////////////////////////
  randomtime2 = random(500, 2000); //servo 2 random time position generation in milliseconds
  if (currentMillis - previousMillis2 >= randomtime2) {
    previousMillis2 = currentMillis;
    pos2 = random(150, 600);
    pwm2.setPWM(5, 0, pos2);
  }
  else {
    pwm2.setPWM(5, 0, holoCentreVal);
  }
}

void holoServo3() {
  //Holo Servo3//////////////////////////////
  randomtime3 = random(500, 2000); //servo 3 random time position generation in milliseconds
  if (currentMillis - previousMillis3 >= randomtime3) {
    previousMillis3 = currentMillis;
    pos3 = random(150, 600);
    pwm2.setPWM(6, 0, pos3);
  }
  else {
    pwm2.setPWM(6, 0, holoCentreVal);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////

void holoLED1() {
  //Holo LED 1
  if (currentMillis - previousMillis4 >= hololedinterval1) {
    previousMillis4 = currentMillis;
    if (hololedState1 == LOW) {
      hololedState1 = HIGH;
      hololedinterval1 = random(hololedonmin, hololedonmax);       //set a random time dealy
      pwm2.setPWM(13, 0, 4096);
    }
    else {
      hololedState1 = LOW;
      hololedinterval1 = random(hololedoffmin, hololedoffmax);       //set a random time dealy
      pwm2.setPWM(13, 4096, 0);
    }
  }
}

void holoLED2() {

  //Holo LED 2
  if (currentMillis - previousMillis5 >= hololedinterval2) {
    previousMillis5 = currentMillis;
    if (hololedState2 == LOW) {
      hololedState2 = HIGH;
      hololedinterval2 = random(hololedonmin, hololedonmax);       //set a random time dealy
      pwm2.setPWM(14, 0, 4096);
    }
    else {
      hololedState2 = LOW;
      hololedinterval2 = random(hololedoffmin, hololedoffmax);       //set a random time dealy
      pwm2.setPWM(14, 4096, 0);
    }
  }
}

void holoLED3() {

  //Holo LED 3
  if (currentMillis - previousMillis6 >= hololedinterval3) {
    previousMillis6 = currentMillis;
    if (hololedState3 == LOW) {
      hololedState3 = HIGH;
      hololedinterval3 = random(hololedonmin, hololedonmax);       //set a random time dealy
      pwm2.setPWM(15, 0, 4096);
    }
    else {
      hololedState3 = LOW;
      hololedinterval3 = random(hololedoffmin, hololedoffmax);       //set a random time dealy
      pwm2.setPWM(15, 4096, 0);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
// (pin, 4096, 0) turns pin fully on, (pin, 0, 4096) turns pin fully off
// Body PCA9685 module, front arms, tilt servo and centre lift servo

void servoSetup() {
  pwm1.setPWM(0, 0, frontarm1min); //front arm 1
  pwm1.setPWM(1, 0, frontarm2min); //front arm 2
  pwm1.setPWM(2, 0, 0); //
  pwm1.setPWM(3, 0, liftup); //centre leg lift
  pwm1.setPWM(4, 0, tiltmin);// Body Tilt
  pwm1.setPWM(5, 0, 0);
  pwm1.setPWM(6, 0, 0); //
  pwm1.setPWM(7, 0, 0); //
  pwm1.setPWM(8, 0, 0); //
  pwm1.setPWM(9, 0, 0); //
  pwm1.setPWM(10, 0, 0); //
  pwm1.setPWM(11, 0, 0); //
  pwm1.setPWM(12, 0, 0); //
  pwm1.setPWM(13, 0, 0); //
  pwm1.setPWM(14, 0, 0); //
  pwm1.setPWM(15, 0, 300);  //dome rotation centred for servo motor
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// Dome PCA9685 module, lights and Holo proector servos, board will require the A0 tab to be bridged with solder
// Pulse ranges for the servo positions to the PCA9685 board 0-180 to 150-600

void servoSetup2() {
  pwm2.setPWM(0, 4096, 0); //Dome lights
  pwm2.setPWM(1, 4096, 0); //Dome lights
  pwm2.setPWM(2, 4096, 0); //Dome lights
  pwm2.setPWM(3, 4096, 0); //Dome lights
  pwm2.setPWM(4, 0, holoCentreVal); // Holo Servo 1
  pwm2.setPWM(5, 0, holoCentreVal); // Holo Servo 2
  pwm2.setPWM(6, 0, holoCentreVal); // Holo Servo 3
  pwm2.setPWM(7, 4096, 0); // Dome lights
  pwm2.setPWM(8, 0, 0); //
  pwm2.setPWM(9, 0, 0); //
  pwm2.setPWM(10, 0, peridown); //Periscope
  pwm2.setPWM(11, 4096, 0); //Periscope LED
  pwm2.setPWM(12, 0, 0); //
  pwm2.setPWM(13, 4096, 0); // holo light 1
  pwm2.setPWM(14, 4096, 0); // holo light 2
  pwm2.setPWM(15, 4096, 0); // holo light 3
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
