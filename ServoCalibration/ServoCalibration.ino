/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver hip;
Adafruit_PWMServoDriver shoulder(0x41);
Adafruit_PWMServoDriver elbow(0x42);
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  0 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2048 // this is the 'maximum' pulse length count (out of 4096)

#define MIN 95
#define MAX 610

int hipCenters[] = {
   345, 342, 340, 372
};
int shoulderCenters[] = {
   373, 362, 359, 339
};
int elbowCenters[] = {
   365, 377, 352, 353
};

int hipMins[] = {
   175, 173, 173, 207
};
int shoulderMins[] = {
   216, 197, 197, 178
};
int elbowMins[] = {
   203, 211, 191, 191
};

int hipMaxes[] = {
   515, 504, 509, 541
};
int shoulderMaxes[] = {
   530, 519, 518, 498
};
int elbowMaxes[] = {
   526, 538, 506, 515
};

void hipsToHome()
{
  for(int i = 0; i < 4; i++) {
    hip.setPWM(i, 0, hipCenters[i]);
  }
}

void shouldersToHome()
{
  for(int i = 0; i < 4; i++) {
    shoulder.setPWM(i, 0, shoulderCenters[i]);
  }
}

void elbowsToHome()
{
  for(int i = 0; i < 4; i++) {
    elbow.setPWM(i, 0, elbowCenters[i]);
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("16 channel Servo test!");

  hip.begin();
  shoulder.begin();
  elbow.begin();
  
  hip.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  shoulder.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  elbow.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  hipsToHome();
  shouldersToHome();
  elbowsToHome();
}

/*
// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}
*/

void loop() {

  delay(3000);
  /*
  for(float a = 0.0; a < 1.0; a += 0.01) {
    for(int i = 0; i < 4; i++) {
      hip.setPWM(i, 0, int(float(hipMins[i])*a + float(hipCenters[i])*(1.0 - a)));
      shoulder.setPWM(i, 0, int(float(shoulderMins[i])*a + float(shoulderCenters[i])*(1.0 - a)));
      elbow.setPWM(i, 0, int(float(elbowMins[i])*a + float(elbowCenters[i])*(1.0 - a)));
      //delay(1);
    }
  }

  delay(3000);
  
  for(float a = 0.0; a < 1.0; a += 0.01) {
    for(int i = 0; i < 4; i++) {
      hip.setPWM(i, 0, int(float(hipCenters[i])*a + float(hipMins[i])*(1.0 - a)));
      shoulder.setPWM(i, 0, int(float(shoulderCenters[i])*a + float(shoulderMins[i])*(1.0 - a)));
      elbow.setPWM(i, 0, int(float(elbowCenters[i])*a + float(elbowMins[i])*(1.0 - a)));
      //delay(1);
    }
  }

  delay(3000);

  for(float a = 0.0; a < 1.0; a += 0.01) {
    for(int i = 0; i < 4; i++) {
      hip.setPWM(i, 0, int(float(hipMaxes[i])*a + float(hipCenters[i])*(1.0 - a)));
      shoulder.setPWM(i, 0, int(float(shoulderMaxes[i])*a + float(shoulderCenters[i])*(1.0 - a)));
      elbow.setPWM(i, 0, int(float(elbowMaxes[i])*a + float(elbowCenters[i])*(1.0 - a)));
      //delay(1);
    }
  }
  
  delay(3000);
  
  for(float a = 0.0; a < 1.0; a += 0.01) {
    for(int i = 0; i < 4; i++) {
      hip.setPWM(i, 0, int(float(hipCenters[i])*a + float(hipMaxes[i])*(1.0 - a)));
      shoulder.setPWM(i, 0, int(float(shoulderCenters[i])*a + float(shoulderMaxes[i])*(1.0 - a)));
      elbow.setPWM(i, 0, int(float(elbowCenters[i])*a + float(elbowMaxes[i])*(1.0 - a)));
      //delay(1);
    }
  }
  */
  /*
  for(int i = MIN; i < MAX; i++) {
    pwm.setPWM(0, 0, i);
    Serial.println(i);
    delay(5);
  }
  for(int i = (MAX-1); i >= MIN; i--) {
    pwm.setPWM(0, 0, i);
    Serial.println(i);
    delay(5);
  }
  Serial.println("Loop");
  */
  /*
  // Drive each servo one at a time
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  
  delay(500);
  */
  /*
  servonum ++;
  if (servonum > 7) servonum = 0;
  */
}
