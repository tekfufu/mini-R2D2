/*
  This code is intended on the Matt Zwarts 39% Mini Droid with 2 dc motors for the foot drives and servo drive for the dome and front arm(s)
  Or 2 DC motors for the foot drives and a DC motor for the dome, 2 MX1508 motor drivers required
  This is the RC control setup, connect channel 1 and 2 from the RC receiver to the arduino pins A0 and A1, and also channel 3 and pins A2 for the dome rotation
  the remaining servos can be controlled by the tranmitter.
  Attach the motor driver to IN pins noted below
  Run the Serial Monitor to ensure the RC transmitter is connected correctly and the values are coming in correct before connecting to the droid

  Happy Building, Matt Zwarts

*//////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 9600 // Define the port output serial communication speed
#define RC_NUM_CHANNELS  6 // list the number of RC channel inputs

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

// Inputs from the RC receiver
#define RC_CH1_INPUT  A0 // Stick x value (Left/Right)
#define RC_CH2_INPUT  A1 // Stick y value (forward/backward)
#define RC_CH3_INPUT  A2 // Dome rotation left right stick
#define RC_CH4_INPUT  A3 // front arm throttle stick
#define RC_CH5_INPUT  A6 // aux channel
#define RC_CH6_INPUT  A7 // aux channel

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

#define INPUT_SIZE 30

#define IN1 5 //Left Motor Driver IN1 5 NOTE: These are PWM pins on arduino nano
#define IN2 3 //Left Motor Driver IN2 3
#define IN3 6 //Right Motor Driver IN1 6
#define IN4 9 //Right Motor Driver IN2 9
#define IN5 10 //Right Motor Driver IN5 10
#define IN6 11 //Right Motor Driver IN6 11

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Deadzone value to reduce centre position error
const int deadzonexy = 20; //original value is 20, this will give the motors enough power to move, add eliminate centering jitter

boolean newData = false;  // check for data received

int led = 12;           // pin for the LED in the dome to be turned on and off
int ledinterval = 200;    // delay to blink the LED lights in the dome
int ledState = LOW;             // ledState used to set the LED
long interval = random(500, 2000); //random blink time increment

float output1 = 0;              // Output from first channel
float output2 = 0;              // Output from second channel
float output3 = 0;              // Output for Dome motor
float output4 = 0;              // Front arm servo
float diff;                       // value for differential steering

float inch1; // Time period input for channel 1
float inch2; // Time period input for channel 2
float inch3; // Time period input for channel 3
float inch4; // Time period input for channel 4
float inch5; // Time period input for channel 5
float inch6; // Time period input for channel 6

float ch1;
float ch2;
float ch3;
float ch4;
float ch5;
float ch6;

float rawLeft;
float rawRight;

float dome_speed = 0.75;                                     // dome rotation speed multiplier, increase or decrease value to speed up dome or slow down

float elapsedTime, time, timePrev;

boolean debug = true;

static unsigned long lastMilli = 0;
unsigned long currentMillis; // time current
unsigned long previousMillis = 0;        // will store last time LED was updated

//LED states
long led1_state = 0;
long loop_timer;

/////////////////////////////////// SETUP ////////////////////////////////////////////////////
void setup() {
  Serial.begin(SERIAL_PORT_SPEED);                                                 //Used only for debugging on arduino serial monitor
  Serial.println("Mini Droid!");

  time = millis();                                                     //Start counting time in milliseconds

  //RC channel pin inputs 6 channels listed
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);

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

  pinMode(led, OUTPUT);
  digitalWrite(12, LOW);

  loop_timer = millis();                                               //Reset the loop timer

  output1 = 0;
  output2 = 0;
  output3 = 0;

  Serial.print("Droid Activated!");

  delay(1000);
}


// start of loop ///////////////////////////////////////////////////////////////////////
void loop() {
  currentMillis = millis();
  timePrev = time;                                                      // the previous time is stored before the actual time read
  time = millis();                                                      // actual time read
  elapsedTime = (time - timePrev);                                      //elapsedTime = (time - timePrev) / 1000;

  // called functions////////////////////////////////////////////////////////////////////////
  rc_read_values();
  text();                                                              // check the motor outputs are correct, connect the usb to the arduino, open the serial monitor in arduino, and connect the phone app
  motordrive();
  dome();
  ledblink();                                                          // led light blink at random time

  inch1 = rc_values[RC_CH1]; // read and store channel value from receiver
  inch2 = rc_values[RC_CH2];
  inch3 = rc_values[RC_CH3];
  inch4 = rc_values[RC_CH4];
  inch5 = rc_values[RC_CH5];
  inch6 = rc_values[RC_CH6];

  lastMilli = millis();
}
//end of loop ///////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////

// Below functions are to read the analog inputs and store the values from the RC receiver
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}
////////////////////////////////////////////////////////////////////////////////////////

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_INPUT);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_INPUT);
}
void calc_ch3() {
  calc_input(RC_CH3, RC_CH3_INPUT);
}
void calc_ch4() {
  calc_input(RC_CH4, RC_CH4_INPUT);
}
void calc_ch5() {
  calc_input(RC_CH5, RC_CH5_INPUT);
}
void calc_ch6() {
  calc_input(RC_CH6, RC_CH6_INPUT);
}

///////////////////////////////////////////////////////////////////////////////////

void motordrive() {     // for the main drive wheels

  // Read the input values from the receiver
  ch1  = inch1;
  ch2  = inch2;
  // Constrain the values from the receiver so they don't go too high or low, normal RC signal is 1000-2000Hz
  ch1 = constrain(ch1, 1000, 2000);
  ch2 = constrain(ch2, 1000, 2000);

  //////////////////////////read in the values from rc transmitter///////////////////
  if (ch1 < 500 || ch2 < 500) { // Failsafe Mode in case transmitter is turned off or loses signal
    ch1 = 0;
    ch2 = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  else {
    output1 = ch1;
    output1 = map(output1, 1000, 2000, -255, 255);

    output2 = ch2;
    output2 = map(output2, 1000, 2000, 255, -255);

    rawLeft = output2 + output1;
    rawRight = output2 - output1;

    diff = abs(abs(output2) - abs(output1));
    rawLeft = rawLeft < 0 ? rawLeft - diff : rawLeft + diff;
    rawRight = rawRight < 0 ? rawRight - diff : rawRight + diff;

    rawLeft = constrain(rawLeft, -255, 255); // constrain the PWM to max 255
    rawRight = constrain(rawRight, -255, 255);

    //Motor Left
    if (rawLeft < -deadzonexy) {
      rawLeft = abs(rawLeft);
      analogWrite(IN1, rawLeft);
      digitalWrite(IN2, LOW);
    }
    else if (rawLeft > deadzonexy) {
      analogWrite(IN2, rawLeft);
      digitalWrite(IN1, LOW);
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

    //Motor Right
    if (rawRight < -deadzonexy) {
      rawRight = abs(rawRight);
      analogWrite(IN3, rawRight);
      digitalWrite(IN4, LOW);
    }
    else if (rawRight > deadzonexy) {
      analogWrite(IN4, rawRight);
      digitalWrite(IN3, LOW);
    }
    else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dome() {     // dome rotation with DC motor

  // Read the input values from the receiver
  ch3  = inch3;
  // Constrain the values from the receiver so they don't go too high or low, normal RC signal is 1000-2000Hz
  ch3 = constrain(ch3, 1000, 2000);

  //////////////////////////read in the values from rc transmitter///////////////////
  if (ch3 < 500) { // Failsafe Mode in case transmitter is turned off or loses signal
    ch3 = 0;
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
  }

  else {
    output3 = ch3;
    output3 = map(output3, 1000, 2000, -255, 255);
    output3 = output3 * dome_speed;
    output3 = constrain(output3, -255, 255); // constrain the PWM to max -255 to 255

    //Motor Left
    if (output3 < -deadzonexy) {
      output3 = abs(output3);
      analogWrite(IN5, output3);
      digitalWrite(IN6, LOW);
    }
    else if (output3 > deadzonexy) {
      analogWrite(IN6, output3);
      digitalWrite(IN5, LOW);
    }
    else {
      digitalWrite(IN5, LOW);
      digitalWrite(IN6, LOW);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void ledblink() {

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(led, ledState);
  }
  interval = random(500, 3000);
}



/////////////////////////////////////////////////////////////////////////////////////////
void text() { // uncomment lines for debugging
  //Serial.print("elapsedtime:"); Serial.print(elapsedTime); Serial.print("\t");
  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  //Serial.print("CH4:"); Serial.print(rc_values[RC_CH4]); Serial.print("\t");
  //Serial.print("CH5:"); Serial.print(rc_values[RC_CH5]); Serial.print("\t");
  //Serial.print("CH6:"); Serial.print(rc_values[RC_CH6]); Serial.print("\t");
  Serial.print("out1:"); Serial.print(rawLeft); Serial.print("\t");
  Serial.print("out2:"); Serial.print(rawRight); Serial.print("\t");
  Serial.print("out3:"); Serial.print(output3); Serial.println("\t");
}
