#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver dog;
// Adafruit_PWMServoDriver dog(0x41);
// Adafruit_PWMServoDriver dog(0x42);

int dogCenters[] = {
   345, 342, 350, 372,
   383, 382, 389, 339,
   407, 357, 397, 337
};
int dogMins[] = {
   175, 173, 173, 207,
   216, 197, 197, 178,
   203, 211, 191, 191
};
int dogMaxes[] = {
   515, 504, 509, 541,
   530, 519, 518, 498,
   526, 538, 506, 515
};

int mag = 20;
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("dog.begin()");
  dog.begin();
  delay(1000);
  
  Serial.println("dog.setPWMFreq(60)");
  dog.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(1000);
  
  Serial.print("Swing +");
  Serial.println(mag);
}

void loop() {
  for(int i = 0; i < 12; i++) {
    dog.setPWM(i, 0, dogCenters[i]);
    delay(3);
  }
  /*
  for(int s = dogCenters[i]; s < dogCenters[i] + mag; s++) {
    dog.setPWM(i, 0, s);
    delay(33);
  }
  delay(500);
  
  Serial.print("Swing -");
  Serial.println(mag);
  for(int s = dogCenters[i] + mag; s > dogCenters[i] - mag; s--) {
    dog.setPWM(i, 0, s);
    delay(33);
  }
  delay(500);
  
  Serial.print("Swing +");
  Serial.println(mag);
  for(int s = dogCenters[i] - mag; s > dogCenters[i]; s++) {
    dog.setPWM(i, 0, s);
    delay(33);
  }
  */
}
