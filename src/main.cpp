#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SharpDistSensor.h>

// Color Sensor Pins

int sensorPin = A7;    // select the analog input pin for the photoresistor
int bPin = 34;  // select the digital output pin for blue LED
int gPin = 36; // select the digital output pin for green LED
int rPin = 38; // select the digital output pin for red LED


// Servo
Servo leftservo;  // create servo object to control a servo
Servo rightservo;  // create servo object to control a servo
int pos =0;
const uint8_t LEFTSERVOPIN=30;
const uint8_t RIGHTSEROVPIN=31;

// IR POSITION
int irpositionPin=A8;
int irSensorPin = A9;
int colorsensorPin = A7;
SharpDistSensor irsensor(irSensorPin, 3);


/* Set the power fit curve coefficients and range
 * C and P: Coefficients in Distance = C*A^P relation
 * where A is the analog value read from the sensor.
 */
const float C = 90373.;
const float P = -1.027;

/*
 * Minimum and maximum analog values for which to return a distance
 * These should represent a range of analog values within which the
 * power fit curve is valid.
 */
const unsigned int minVal = 90; // ~800 mm
const unsigned int maxVal = 875; // ~50mm
// Functions.

// Color Sensing

// RED RED RED RED RED RED
typedef enum {
  BLACK_PAD,
  RED_PAD,
  WHITE_PAD,
  BLUE_PAD
} COLOR_TYPE;



COLOR_TYPE scancolors (void){

  int sensorValue = 0; // variable to store the value coming from the photoresistor

  int maxblack[]={810,764,843}; // the max reading that will be obtained from a black surface {RED,GREEN,BLUE}
  int minwhite[]={407,363,484}; // the min reading that will be obtained from a white surface {RED,GREEN,BLUE}

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
// Printing COLOR VALUES
        Serial.print("RED: ");
        Serial.print(colorchannel[0]);
        Serial.print(" GREEN: ");
        Serial.print(colorchannel[1]);
        Serial.print(" BLUE: ");
        Serial.print(colorchannel[2]);

        Serial.print('\n');
if ((colorchannel[0] <= 10) &&
  (colorchannel[1] <= 10) &&
  (colorchannel[2] <= 20))
  {
    return BLACK_PAD;
  }
else if((colorchannel[0] >= 90) &&
  (colorchannel[1] >= 90) &&
  (colorchannel[2] >= 90))
  {
    return WHITE_PAD;
  }
else if ((colorchannel[0] >= 90))
  {
      return RED_PAD;
    }
else if ((colorchannel[2] >= 90))
      {
        return BLUE_PAD;
      }


//while(1) //Forever Loop Stops color not recognized
{
  Serial.println("Recolorbrate");
//  delay(1000);
}





}
// ROBOT ORIENTATION SENSOR
        int getyaw (void){
                // read the incoming byte:
                int yaw;

                char buffer[4] = {0};
                Serial1.readBytesUntil('\n',buffer,4);
                yaw = atoi(buffer);
                return yaw;
        }

        void setup() {


                Serial.begin(115200);
                Serial1.begin(115200);
// Color SENSOR
// declare the LED pins as an OUTPUT:
                pinMode(rPin, OUTPUT);
                pinMode(gPin, OUTPUT);
                pinMode(bPin, OUTPUT);

// Servo
                leftservo.attach(LEFTSERVOPIN);
                rightservo.attach(RIGHTSEROVPIN);
// IR SENSOR

                irsensor.setPowerFitCoeffs(C,P, minVal, maxVal);

// SET UP COMPLETE
                Serial.println("setup complete");


        }

        void loop() {

          Serial.println(scancolors());

                // leftservo.write(90);              // tell servo to go to position in variable 'pos'
                //   delay(15);
                //   Serial.println("servo movement leftservo");
                //
                //   rightservo.write(90);
                //
                //   Serial.println("servo movement rightservo");


                //get the yaw
                int yaw;

                //yaw = getyaw();

                // INITIALIZE THE ANALOG INPUT PORTS


                // INITIALIZE ANY VARIABLE THAT YOU USE FOR DISTANCE MEASUREMENT AND CALCULATIONS

                // {
                //
                //
                // //READ FROM THE SENSOR
                // float irSensorValue = 0;
                // float irSensorVoltage =0;
                // float distance =0;
                //
                // irSensorValue = analogRead(irSensorPin);
                // irSensorVoltage= (float)irSensorValue/1023*3.3;
                // distance = 7.946*pow(irSensorVoltage,-.695);
                //
                // Serial.print("Sensor1:");
                // Serial.print(distance); //PRINT THE OUTPUT FOR THE FIRST SENSOR IN VOLTAGE
                // Serial.println("");
                //
                // }

                // // Get distance from sensor
                // unsigned int distance = irsensor.getDist();
                // Serial.println(distance);

                // IR Position Sensor

                // int irposition = analogRead(irpositionPin);
                // Serial.println(irposition);

                // Color Sensor



        }
