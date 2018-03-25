#include <Arduino.h>

int getyaw (void){
        // read the incoming byte:
        int yaw;

        char buffer[4] = {0};
        Serial1.readBytesUntil('\n',buffer,4);
        yaw = atoi(buffer); //
        return yaw;
}
