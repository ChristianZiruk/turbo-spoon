#include <Arduino.h>

void limit_switch_ISR(){
    // tell servoes and ir position motor to stop
digitalWrite(motorenablepin,0);
servo_stop();
digitalWrite(LEFTSERVOPIN,0);
digitalWrite(RIGHTSEROVPIN,0);
    Serial.print("Limit switch hit, halting");

    while(1);
}
