#include <Arduino.h>
#include <Servo.h>
Servo leftservo;  // create servo object to control a servo
Servo rightservo;  // create servo object to control a servo

int pos =0;
const uint8_t LEFTSERVOPIN=30;
const uint8_t RIGHTSEROVPIN=31;

void setup() {

   Serial.begin(9600);


  leftservo.attach(LEFTSERVOPIN);
  rightservo.attach(RIGHTSEROVPIN);

  Serial.println("setup complete");

}

void loop() {
  leftservo.write(45);              // tell servo to go to position in variable 'pos'
    delay(15);
    Serial.println("servo movement leftservo");

    rightservo.write(135);

    Serial.println("servo movement rightservo");

}
