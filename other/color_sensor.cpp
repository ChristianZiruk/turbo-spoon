#include <Arduino.h>
// Color Sensor Pins---------------------------------------------------------
int colorsensorPin=A7; // Color Sensor Anolog 7

int sensorPin = A7;    // select the analog input pin for the photoresistor
int bPin = 34;  // select the digital output pin for blue LED
int gPin = 36; // select the digital output pin for green LED
int rPin = 38; // select the digital output pin for red LED


// COLOR TYPES---------------------------------------------------------------
typedef enum {
        BLACK_PAD, // 0
        RED_PAD, // 1
        WHITE_PAD, // 2
        BLUE_PAD // 3
} COLOR_TYPE;

COLOR_TYPE scancolors (void){

        int sensorValue = 0; // Variable to store the value coming from the photoresistor

//--RAW MAX COLOR CALIBRATION------------------------------------------------RAW MAX COLOR CALIBRATION----------
//                       R   G   B
        int maxblack[]={723,680,711}; // the max reading that will be obtained from a black surface {RED,GREEN,BLUE}
        int minwhite[]={391,343,430}; // the min reading that will be obtained from a white surface {RED,GREEN,BLUE}

        int colorchannel[]={0,0,0}; // array for the readings (red, green, blue)
        int colorpins[3] = {rPin,gPin,bPin};
        for(int i=0; i<3; ++i)
        {
                digitalWrite(colorpins[i], LOW);
        }

        for(int i=0; i<3; ++i)  {
                digitalWrite(colorpins[i],HIGH);
                delay(300); //wait for the photresistor value to settle
                sensorValue = analogRead(sensorPin); // read the photoresistor value
                colorchannel[i]=sensorValue; // record the red reading
                colorchannel[i]=constrain(sensorValue,minwhite[i],maxblack[i]); //constrain the reading such that it is between the white and black values
                colorchannel[i]=map(colorchannel[i],maxblack[i],minwhite[i],0,100); // map the reading between 0 and 100 such that black is 0, 100 is white
                digitalWrite(colorpins[i], LOW);
        }
// PRINTING COLOR VALUES------------------------------------------------------
        Serial.print("RED: ");
        Serial.print(colorchannel[0]);
        Serial.print(" GREEN: ");
        Serial.print(colorchannel[1]);
        Serial.print(" BLUE: ");
        Serial.print(colorchannel[2]);

        Serial.print('\n');

        //CALIBRATION-----------------COLOR PAD--------------------CALIBRATION
        if ((colorchannel[0] <= 10) &&   //RED_LED Black Surface
            (colorchannel[1] <= 10) &&   //GREEN_LED Black Surface
            (colorchannel[2] <= 10))     //BLUE_LED Black Surface
        {
                return BLACK_PAD;
        }
        else if((colorchannel[0] >= 95) && //RED_LED White surface
                (colorchannel[1] >= 95) &&//GREEN_LED black surface
                (colorchannel[2] >= 95)) //BLUE_LED blue surface
        {
                return WHITE_PAD;
        }
        else if ((colorchannel[0] >= 85))// RED_LED Red Surface
        {
                return RED_PAD;
        }
        else if ((colorchannel[2] >= 78))// BlUE_LED Blue Surface
        {
                return BLUE_PAD;
        }
// while(1) //Forever Loop Stops color not recognized
//         {
//                 Serial.println("Recolorbrate");
//  delay(1000);
//         }
//END CALIBRATION----------------COLOR_PADS---------------------END_CALIBRATION
}
