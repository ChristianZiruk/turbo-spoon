#include <Arduino.h>


void flushyaw(){
    while(Serial1.available()>0){
        Serial1.read();
    }
    while (Serial1.read() != '\n'); //continues readings bytes until /n
}

int getyaw (void){
        // read the incoming byte:
        int yaw;
        flushyaw();


        char buffer[4] = {0};
        Serial1.readBytesUntil('\n',buffer,4);// waits until the nano sends over signal
        yaw = atoi(buffer); //
        return yaw;
}
