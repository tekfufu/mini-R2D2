/*
  This code is intended on the Matt Zwarts 39% Mini Chopper or "Choppy" as my daughter calls him
  ///////////////////////////////////////////////////////////////////////////////////////
  //Connections
  ///////////////////////////////////////////////////////////////////////////////////////
  HC-05  - Arduino
  VCC   -   5V
  GND   -   GND
  RXD   -   TXD
  TXD   -   RXD
*//////////////////////////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>
#include <math.h>
#include <MX1508.h>   // motor driver for cheap Chinese tiny dual motor drivers, use PWM pins on arduino

SoftwareSerial MySerial(7, 8); // RX, TX

#define INPUT_SIZE 30

//motor driver pins
// NOTE: Make sure the pins are PWM pins

#define NUMPWM 2 // this pin isnt used?? not sure on the MX1508 setup diagram

#define IN1 5 //Left Motor Driver IN1
#define IN2 3 //Left Motor Driver IN2
#define IN3 6 //Right Motor Driver IN1
#define IN4 9 //Right Motor Driver IN2
#define IN5 10 //Dome Motor Driver IN1
#define IN6 11 //Dome Motor Driver IN2


//Define the motor driver details on the MX1508
MX1508 motorA(IN1, IN2, FAST_DECAY, NUMPWM);
MX1508 motorB(IN3, IN4, FAST_DECAY, NUMPWM);
MX1508 motorC(IN5, IN6, FAST_DECAY, NUMPWM); //dome motor

static int pwm = 1;

const int deadzonexy = 30;

byte receivedChar;        // serial data received from bluetooth device
boolean newData = false;  // check for data received

char rc;                    // received values

int x1 = 0;                     // value from joystick
int y1 = 0;                     // value from joystick
int x2 = 0;                     // value from joystick
int y2 = 0;                     // value from joystick


int motorxposition;        // value to store received motor x position
int motoryposition;        // value to store received motor y position
int headturnposition;      // value to store received head turn position
int headtiltposition;      // value to store received head tilt position

float output1 = 0;              // Output from first channel
float output2 = 0;              // Output from second channel
float output3 = 0;              // Output for Dome motor
float theta;                // conversion to polar coordinates
float radius;               // conversion to polar coordinates
float rawLeft;
float rawRight;
float RawLeft;
float RawRight;

int LeftMotorOutput = 0;        // final motor output
int RightMotorOutput = 0;       // final motor output

float dome_speed = 0.75;                                     // dome rotation speed multiplier
float turning_speed = 0.5;                                    //Turning speed (50%)
float max_target_speed = 1;                                //Max target speed (100%)

float elapsedTime, time, timePrev;

boolean debug = true;

static unsigned long lastMilli = 0;
unsigned long currentMillis; // time current
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long lastservotime = 0;

//LED states
long led1_state = 0;

long loop_timer;

/////////////////////////////////// SETUP ////////////////////////////////////////////////////
void setup() {
  //Wire.begin();
  Serial.begin(9600);                                                 //Used only for debugging on arduino serial monitor
  Serial.println("Hello, my name is C1-10P!");

  MySerial.begin(9600);
  MySerial.setTimeout(40);

  time = millis();                                                     //Start counting time in milliseconds

  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output

  //output pins for motor drivers
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);


  // Write the motor pins low so they dont start on power up
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

  loop_timer = millis();                                               //Reset the loop timer


  output1 = 0;
  output2 = 0;
  output3 = 0;
  motorA.motorGo(output1);
  motorB.motorGo(output2);
  motorC.motorGo(output3);

  delay(1000);

  digitalWrite(13, LOW);                                               //All done, turn the LED off
}


// start of loop ///////////////////////////////////////////////////////////////////////

void loop() {

  currentMillis = millis();
  timePrev = time;                                                      // the previous time is stored before the actual time read
  time = millis();                                                      // actual time read
  elapsedTime = (time - timePrev);                               //elapsedTime = (time - timePrev) / 1000;


  // called functions////////////////////////////////////////////////////////////////////////

  bluetooth_data();
  text();                                                              // check the motor outputs are correct, connect the usb to the arduino, open the serial monitor in arduino, and connect the phone app
  motordrivers();                                                      // motor drivers function
  dome();                                                              // Dome motor drive

  lastMilli = millis();

}

//end of loop ///////////////////////////////////////////////////////////////////////////////////////////



void bluetooth_data() {

  char rawData[100] = "";
  char keyword[] = "<<";


  if (MySerial.available() > 0) {//new data in
    size_t byteCount = MySerial.readBytesUntil('\n', rawData, sizeof(rawData) - 1); //read in data to buffer
    rawData[byteCount] = NULL;//put an end character on the data
    //Serial.print("Raw Data = ");
    //Serial.println(rawData);

    //now find keyword and parse
    char *keywordPointer = strstr(rawData, keyword);
    if (keywordPointer != NULL) {
      int dataPosition = (keywordPointer - rawData) + strlen(keyword); //should be position after Mydata=

      const char delimiter[] = ",";
      char parsedStrings[5][20];
      int dataCount = 0;

      char *token =  strtok(&rawData[dataPosition], delimiter);//look for first piece of data after keyword until comma
      if (token != NULL && strlen(token) < sizeof(parsedStrings[0])) {
        strncpy(parsedStrings[0], token, sizeof(parsedStrings[0]));
        dataCount++;
      } else {
        //Serial.println("token too big");
        strcpy(parsedStrings[0], NULL);
      }

      for (int i = 1; i < 5; i++) {
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
      //Serial.println(" strings:");
      //for (int i = 0; i < 5; i++)
      //Serial.print(parsedStrings[i]);Serial.println("");

      if (dataCount == 5) {
        x1 = atoi(parsedStrings[0]);
        y1 = atoi(parsedStrings[1]);
        x2 = atoi(parsedStrings[2]);
        y2 = atoi(parsedStrings[3]);
      }
      else {
        //Serial.println("data no good");
      }
    }
    else {
      //Serial.println("sorry no keyword");
    }

  }
  else { /////////////////////Write the motors to low if no signal is present
    output1 = 0;
    output2 = 0;
    output3 = 0;
    motorA.motorGo(output1);
    motorB.motorGo(output2);
    motorC.motorGo(output3);
  }
}





//////////////////////////////////////////////////////////////////////////////////////

void motordrivers() {     // for the main drive wheels


  //////////////////////////read in the values from bluetooth///////////////////
  output1 = x1;
  output1 = map(output1, 0, 200, -255, 255);

  output2 = y1;
  output2 = map(output2, 0, 200, -255, 255);

  // Convert both values for tank steering from cartesian to Polar coordinates
  // hypotenuse
  radius = sqrt(output1 * output1 + output2 * output2);
  // angle in radians
  theta = acos(abs(output2) / radius);
  //cater for NaN values
  if (isnan(theta) == true) {
    theta = 0;
  }

  //angle in degrees
  float angle = theta * 180 / 3.1415;
  float tcoeff = -1 + (angle / 90) * 2;
  float turn = tcoeff * abs(abs(output1) - abs(output2));
  turn = round(turn * 100) / 80;
  // And max of output1 or output2 is the movement
  float mov = max(abs(output1), abs(output2));
  // First and third quadrant
  if ((output2 >= 0 && output1 >= 0) || (output2 < 0 && output1 < 0))
  {
    rawLeft = mov;
    rawRight = turn;
  }
  // Second and fourth quadrant
  else
  {
    rawLeft = turn;
    rawRight = mov;
  }
  // Reverse polarity
  if (output1 < 0) {
    rawLeft = 0 - rawLeft;
    rawRight = 0 - rawRight;
  }
  // Update the values
  output2 = rawLeft;
  output1 = rawRight;

  ////////////////////// dont move if the deadzone is too small, i.e. too close to center on joystick
  ///////////////////// change the deadzone value for greater sensitivity
  if (output1 > -deadzonexy && output1 < deadzonexy) {
    output1 = 0;
  }
  if (output2 > -deadzonexy && output2 < deadzonexy) {
    output2 = 0;
  }

  //////////////////////////////////Motor driver direction control/////////////////////
  if (output1 > deadzonexy) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if (output1 < -deadzonexy) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (output2 > deadzonexy) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  if (output2 < -deadzonexy) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  output1 = constrain(output1, -255, 255); // constrain the PWM to max 255
  output2 = constrain(output2, -255, 255);
  motorA.motorGo(output1);
  motorB.motorGo(output2);

}




////////////////////////////////////////////////////////////////////////////////////////

void dome() {

  output3 = x2;
  output3 = map(output3, 0, 200, -255, 255);

  ///////////////////// change the deadzone value for greater sensitivity
  if (output3 > -deadzonexy && output3 < deadzonexy) {
    output3 = 0;
  }

  //////////////////////////////////Motor driver direction control/////////////////////
  if (output3 > deadzonexy) {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  }
  if (output3 < -deadzonexy) {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }

  output3 = constrain(output3, -255, 255); // constrain the PWM to max 255
  motorC.motorGo(output3 * dome_speed);
}




/////////////////////////////////////////////////////////////////////////////////////////
void text() { // uncomment lines for debugging
  //Serial.print("Received:"); Serial.print(receivedChar); Serial.println("\t");
  //Serial.print("elapsedtime:"); Serial.print(elapsedTime); Serial.print("\t");
  Serial.print("out1:"); Serial.print(output1); Serial.print("\t");
  Serial.print("out2:"); Serial.print(output2); Serial.print("\t");
    Serial.print("out3:"); Serial.print(output3); Serial.println("\t");
}
