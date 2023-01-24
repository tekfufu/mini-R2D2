// NOTE this is a stand alone code to run the mini droid dome to control 3 servos and LED's for the holos and logic lights
// The setup will require an arduino Nano in the dome with a seperate power supply
// The battery supply will need to be 5 Volts, 3 AA batteries would work or a 7.4 volt LiPo battery with a DC voltage regulator to drop it to 5 Volts
// The LED's are simple 5mm LED's that fit into the dome light inserts
// The servos will benefit to be powered directly from the 5 Volt supply and signal from the Nano


#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;

unsigned long currentMillis; // time current
unsigned long previousMillis = 0;
float time, timePrev;

int pos1 = 0;    // variable to store the servo1 position
int pos2 = 0;    // variable to store the servo2 position
int pos3 = 0;    // variable to store the servo3 position

int randompos1 = 0; //random number for servo 1
int randompos2 = 0; //random number for servo 2
int randompos3 = 0; //random number for servo 3

int randomtime1 = 0; // random time delay for movement 1
int randomtime2 = 0; // random time delay for movement 2
int randomtime3 = 0; // random time delay for movement 3

int ledPin = 12;           // pin for the LED in the dome to be turned on and off
int ledinterval = 500;    // delay to blink the LED lights in the dome
int ledState = LOW;             // ledState used to set the LED
int ledonmin = 200;
int ledonmax = 1000;
int ledoffmin = 200; // value in milliseconds
int ledoffmax = 500;

int hololedPin1 = 2;           // pin for the holo 1 LED in the dome to be turned on and off
int hololedinterval1 = 200;    // delay to blink the LED lights in the dome
int hololedState1 = LOW;       // ledState used to set the LED
int hololedonmin1 = 300;
int hololedonmax1 = 1000;
int hololedoffmin1 = 30; // value in milliseconds
int hololedoffmax1 = 300;

int hololedPin2 = 3;           // pin for the holo 2 LED in the dome to be turned on and off
int hololedinterval2 = 200;    // delay to blink the LED lights in the dome
int hololedState2 = LOW;       // ledState used to set the LED
int hololedonmin2 = 300;
int hololedonmax2 = 1000;
int hololedoffmin2 = 30; // value in milliseconds
int hololedoffmax2 = 300;

int hololedPin3 = 4;           // pin for the holo 3 LED in the dome to be turned on and off
int hololedinterval3 = 200;    // delay to blink the LED lights in the dome
int hololedState3 = LOW;       // ledState used to set the LED
int hololedonmin3 = 300;
int hololedonmax3 = 1000;
int hololedoffmin3 = 30; // value in milliseconds
int hololedoffmax3 = 300;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);                                                 //Used only for debugging on arduino serial monitor
  Serial.println("Dome Lights!");

  time = millis();

  myservo1.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);
  myservo3.attach(11);

  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);

  pinMode(ledPin, OUTPUT);
  pinMode(hololedPin1, OUTPUT);
  pinMode(hololedPin2, OUTPUT);
  pinMode(hololedPin3, OUTPUT);

  delay(1000); // give things time to start
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  currentMillis = millis();
  timePrev = time;                                                      // the previous time is stored before the actual time read
  time = millis();                                                      // actual time read

  servo1();
  servo2();
  servo3();
  lights();                                                           //Flicker dome lights
  holo1();                                                            // random holo1 lights
  holo2();                                                            // random holo2 lights
  holo3();                                                            // random holo3 lights

  text();

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void servo1() {
  randomtime1 = random(100, 3000); //servo 1 random time position generation in milliseconds
  if (currentMillis - previousMillis >= randomtime1) {
    previousMillis = currentMillis;
    pos1 = random(20, 160);
    myservo1.write(pos1);
  }
  else {
    myservo1.write(90); //90 is the servo stopped
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void servo2() {
  randomtime2 = random(100, 3000); //servo 2 random time position generation in milliseconds
  if (currentMillis - previousMillis >= randomtime2) {
    previousMillis = currentMillis;
    pos2 = random(20, 160);
    myservo2.write(pos2);
  }
  else {
    myservo2.write(90);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void servo3() {
  randomtime3 = random(100, 3000); //servo 3 random time position generation in milliseconds
  if (currentMillis - previousMillis >= randomtime3) {
    previousMillis = currentMillis;
    pos3 = random(20, 160);
    myservo3.write(pos3);
  }
  else {
    myservo3.write(90);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void lights() {         // function to flash the dome lights when a sound is being played

  if (currentMillis - previousMillis >= ledinterval) {
    previousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
      ledinterval = random(ledonmin, ledonmax);       //set a random time dealy when the sound is being played
    }
    else {
      ledState = LOW;
      ledinterval = random(ledoffmin, ledoffmax);       //set a random time dealy when the sound is being played
    }
    digitalWrite(ledPin, ledState);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////


void holo1() {         // function to turn on LED's  in the holo proectors

  if (currentMillis - previousMillis >= hololedinterval1) {
    previousMillis = currentMillis;
    if (hololedState1 == LOW) {
      hololedState1 = HIGH;
      hololedinterval1 = random(hololedonmin1, hololedonmax1);       //set a random time dealy
    }
    else {
      hololedState1 = LOW;
      hololedinterval1 = random(hololedoffmin1, hololedoffmax1);       //set a random time dealy
    }
    digitalWrite(hololedPin1, hololedState1);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////


void holo2() {         // function to turn on LED's  in the holo proectors

  if (currentMillis - previousMillis >= hololedinterval2) {
    previousMillis = currentMillis;
    if (hololedState2 == LOW) {
      hololedState2 = HIGH;
      hololedinterval2 = random(hololedonmin2, hololedonmax2);       //set a random time dealy
    }
    else {
      hololedState2 = LOW;
      hololedinterval2 = random(hololedoffmin2, hololedoffmax2);       //set a random time dealy
    }
    digitalWrite(hololedPin2, hololedState2);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////


void holo3() {         // function to turn on LED's  in the holo proectors

  if (currentMillis - previousMillis >= hololedinterval3) {
    previousMillis = currentMillis;
    if (hololedState3 == LOW) {
      hololedState3 = HIGH;
      hololedinterval3 = random(hololedonmin3, hololedonmax3);       //set a random time dealy
    }
    else {
      hololedState3 = LOW;
      hololedinterval3 = random(hololedoffmin3, hololedoffmax3);       //set a random time dealy
    }
    digitalWrite(hololedPin3, hololedState3);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
void text() { // uncomment lines for debugging
  //Serial.print("elapsedtime:"); Serial.print(elapsedTime); Serial.print("\t");
  Serial.print("servo1:"); Serial.print(pos1); Serial.print("\t");
  //Serial.print("randomtime1:"); Serial.print(randomtime1); Serial.print("\t");
  Serial.print("servo2:"); Serial.print(pos2); Serial.print("\t");
  Serial.print("servo3:"); Serial.print(pos3); Serial.print("\t");
  Serial.print("ledinterval:"); Serial.print(ledinterval); Serial.print("\t");
  Serial.print("ledstate:"); Serial.print(ledState); Serial.print("\t");
  Serial.print("holo1:"); Serial.print(hololedinterval1); Serial.print("\t");
  Serial.print("holo2:"); Serial.print(hololedinterval2); Serial.print("\t");
  Serial.print("holo3:"); Serial.print(hololedinterval3); Serial.println("\t");
}
