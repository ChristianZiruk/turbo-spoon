#include <Arduino.h>
#include <SharpDistSensor.h>
#include <SoftPWMServo.h> // "Pulse Width Modulation" Library. Used to drive DC Motor

int kp=5;
int ki=5;
int desiredangle;
int lastpidtime;
int pidintegral;


// IR-Sensor Position Motor---------------------------------------------------
int motorenablepin = 3; // motor enable pin
int motordirectpin= 4; // motor directional pin
// IR POSITION----------------------------------------------------------------
// IR Position Anolog Pin 8
int irpositionPin=A8;
int irSensorPin=A9; // IR Sensor Anolog 9
float getirposition(void)
{
        int rawirposition = analogRead(irpositionPin);

        //Serial.println(irposition);
        float irposition= 0.3515 *rawirposition -139.44;
        return irposition;
}


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

// PID CALL BACK---------------------------------------------------------------


uint32_t PID_CALL_BACK(uint32_t currentTime)
{
        float currentangle= getirposition();

        float error= desiredangle-currentangle;

        float deltatime = currentTime-lastpidtime;

        pidintegral += error/deltatime;





        float control = error*kp + pidintegral * ki;

        if (control > 0)
        {
                digitalWrite (motordirectpin,0);
        }
        else
        {
                digitalWrite (motordirectpin,1);
        }

        // Serial.println(currentangle); //----------------------SERIAL PRINT

        SoftPWMServoPWMWrite(motorenablepin, abs (control));

        return (currentTime + CORE_TICK_RATE*10);
}


void scan(int a[])
{
    attachCoreTimerService(PID_CALL_BACK);
    for(int i = 0; i <= 180; ++i)
    {
        // instruct pid to goto angle
        desiredangle = i - 90;
        //give pid some time to reach angle
        delay(100);
        //read the irsensor and store
        a[i] = irsensor.getDist();
    }
    desiredangle = 0;
    delay(200);
    detachCoreTimerService(PID_CALL_BACK);
    SoftPWMServoPWMWrite(motorenablepin, 0);

    for(int i=0; i <= 180; ++i)
    {
    Serial.print(i-90);
    Serial.print(" ");
    Serial.println(a[i]);
}
}


// int main (void
// {
//
//     int i;        //  0    1   2     3    4    5    6    7    8
//     int array[9] = {800, 800, 750, 700, 650, 700, 750, 800, 800};
//
//     printf("for loop:\n");
//     for (i=0; i < 20; ++i)
//     {
//         //printf("%d\n", array[i]);
//         printf("The %dth value of array is %d\n", i, array[i]);
//         //printf("array[%d]=%d, i=%d\n", i, array[i], i);
//     }
//     printf("finished for loop\n\n");
//
//     return 0;
// }
