
//LIBRARYS-----------------------------------------------------------------
#include <Arduino.h> // Arduino Library
#include <Servo.h> // Servo Library
#include <SharpDistSensor.h> // IR Sensor Library
#include <SoftPWMServo.h> // "Pulse Width Modulation" Library. Used to drive DC Motor

//PID DESIRED ANGLE & Proportional Term-------------------------------------
float kp= 4; //Proportional Gain
float ki=40;// Integral Gain
float kd=0; // Derivitive Gain. This has no code implimented!!!!!

float previous_angle= 0;
int desiredangle = 0; //Target angle

long int lastpidtime=0;
float pidintegral =0;

// Color Sensor Pins---------------------------------------------------------

int sensorPin = A7;    // select the analog input pin for the photoresistor
int bPin = 34;  // select the digital output pin for blue LED
int gPin = 36; // select the digital output pin for green LED
int rPin = 38; // select the digital output pin for red LED

// IR-Sensor Position Motor-------------------------------------------------
int motorenablepin = 3; // motor enable pin
int motordirectpin= 4; // motor directional pin

// Servo Pins----------------------------------------------------------------
Servo leftservo;  // create servo object to control a servo
Servo rightservo;  // create servo object to control a servo
int pos =0;
const uint8_t LEFTSERVOPIN=30; // Left Servo plugged into j6
const uint8_t RIGHTSEROVPIN=31;// Right Servo plugged into j5

// IR POSITION----------------------------------------------------------------
int irpositionPin=A8; // IR Position Anolog Pin 8
int irSensorPin=A9; // IR Sensor Anolog 9
int colorsensorPin=A7; // Color Sensor Anolog 7
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

// IR Position
float getirposition(void)
{
        int rawirposition = analogRead(irpositionPin);

        //Serial.println(irposition);
        float irposition= 0.3515 *rawirposition -139.44;
        return irposition;
}

// COLOR TYPES---------------------------------------------------------------
typedef enum {
        BLACK_PAD, // 0
        RED_PAD, // 1
        WHITE_PAD, // 2
        BLUE_PAD // 3
} COLOR_TYPE;

COLOR_TYPE scancolors (void){

        int sensorValue = 0; // Variable to store the value coming from the photoresistor

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
// PRINTING COLOR VALUES------------------------------------------------------
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
// ROBOT ORIENTATION SENSOR BNO 0555-----------------------------------------

int getyaw (void){
        // read the incoming byte:
        int yaw;

        char buffer[4] = {0};
        Serial1.readBytesUntil('\n',buffer,4);
        yaw = atoi(buffer); //
        return yaw;
}

//PID CALL BACK---------------------------------------------------------------
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
// IR frequency---------------------------------------------------------------
int get_signal =0;
int count =0
int time;

//STATE MACHINE --------------------------------------------------------------

typedef enum {
    STATE_INIT, // initial state
    STATE_GET_COMMAND, // find the command signal, note it's frequency, then rotate 180
    STATE_SCAN, // sweep the IR sensor, note pillar sizes and their angular locations
    STATE_GOTO_PILLAR, // orient to the proper pillar, move forward, note color
    STATE_PARK, // find the command signal, move forward, park on color
    STATE_FINAL // success, we're done!
} STATE_T;

char *state_names[] = {"STATE_INIT",
                       "STATE_TASK_A",
                       "STATE_NEXT",
                       }

STATE_T current_state;
int command_signal;
int num_successful_runs;


//SETUP--------------------------------------------------------------- SETUP
void setup() {
        Serial.begin(115200); //Hardware Serial Initialize
        Serial1.begin(115200);//Software Seerial Initialized Talking to the BNO0055 orientation sensor

// COLOR SENSOR--------------------------------------------------------------

// declare the LED pins as an OUTPUT:
        pinMode(rPin, OUTPUT); //RED PIN
        pinMode(gPin, OUTPUT); //GREEN PIN
        pinMode(bPin, OUTPUT); // BLUE PIN

// SERVO--------------------------------------------------------------------
        leftservo.attach(LEFTSERVOPIN);
        rightservo.attach(RIGHTSEROVPIN);
// IR SENSOR-----------------------------------------------------------------
        irsensor.setPowerFitCoeffs(C,P, minVal, maxVal);
// IR POSITION MOTOR----------------------------------------------------------
        pinMode (motorenablepin,OUTPUT);
        pinMode (motordirectpin, OUTPUT);
        SoftPWMServoPWMWrite(motorenablepin, 0); //Writing to pin three as LOW or desired vlue for movement eg. try 30 where low is placed
        digitalWrite (motordirectpin, 0);
        attachCoreTimerService(PID_CALL_BACK);
// IR FREQUENCY ------------------------------------------------------------
        attatchInterupt(4,Decoder,FALLING);
        attatchCoreTimerService(counter);
//STATE MACHINE ---------------------------------------------------------------

{
    current_state = STATE_INIT;
    num_successful_runs = 0;
    apple = 0;
}

// SET UP COMPLETE
        Serial.println("setup complete");
// DO NOT PRINT OR ADD BELOW IN SETUP. !KEEP CODE! ABOVE "SETUP COMPLETE"
}
//LOOP-------------------------------------------------------------------Loop
void loop() {
//STATE MACHINE------------------------------------------------STATE MACHINE
{
    Serial.print("current state: ");// Debugger
    Serial.println(state_names[current_state]);
    switch (current_state)
    {

        case STATE_INIT:
            current_state = STATE_GET_COMMAND;
            break;
        case STATE_GET_COMMAND:
            current_state = find_command_signal();
            break;
        case STATE_SCAN:
            current_state = scan_for_pillars();
            break;
        case STATE_GOTO_PILLAR:
            current_state = goto_pillar();
            //do stuff
            break;
        case STATE_PARK:
            current_state = park();
            //do stuff
            break;
        case STATE_FINAL:
            //do stuff
            while(1)
            {
                Serial.println("success!");
                delay(1000);
            }
            break;

        default:
            // should never get here unless something went wrong with our state
            // transitions
            while(1)
            {
                Serial.println("transitioned to unrecognized state");
                delay(1000);
            }
    }

    delay(10);
}


int get_signal()
{
    //I'm going to read the sensor and return the freq I found, 0 if I don't
    //find anything
}

int home_angle = 0;

STATE_T find_command_signal()
{
    int signal = get_signal();
    while (signal != 0)
    {
   //tell servos to rotate
        delay(250);
        signal = get_signal();
    }
    // stop servos

    // once here, signal is something other than 0
    // Note the signal, going to use it later
    command_signal = signal;

    // set your home angle
    home_angle = getyaw() - 180;

    goto_angle(home_angle);


    return STATE_SCAN;
}

STATE_T scan_for_pillars()
{
    return STATE_GOTO_PILLAR;
}

STATE_T goto_pillar()
{
    return STATE_PARK;
}

STATE_T park()
{



    ++num_successful_runs;

    if (num_succesful_runs >= 2)
    {
        return STATE_FINAL;
    }
    else
    {
        return STATE_GET_COMMAND;
    }
}


void goto_angle(int desired_angle)
{
    float error = desired_angle - getyaw();
    while (abs(error) > 3)
    {
        //determine the proper location to rotate and command the servos to
        //start making the robot rotate in that direction
        delay(250);
        error = desired_angle - getyaw();
    }
    //stop servos

    return;
}

//END OF STATE MACHINE-------------------------------------END OF STATE MACHINE







//COLOR SENSOR TESTING
        // Serial.println(scancolors());// ---------------------SERIAL PRINT
        //currentirposition();

        // for(int i=0; i<100; ++i)
        // {
        //     desiredangle = map(i, 0, 100, -90, 0);
        //     delay(100);
        //
        // }
        //
        // delay(5000);
        //
        //
        // desiredangle =-90;
        // delay (5000);
        //
        // desiredangle =0;
        // delay (3000);
        //
        // desiredangle =90;
        // delay (3000);
        //
        // desiredangle =0;
        // delay (3000);
        // leftservo.write(90);              // tell servo to go to position in variable 'pos'
        //   delay(15);
        //   Serial.println("servo movement leftservo");
        //
        //   rightservo.write(90);
        //
        //   Serial.println("servo movement rightservo");

        //GET THE YAW POSITION-------------------------------------------------
        int yaw = getyaw();
        Serial.println(yaw);//--------------------------------------SERIAL PRINT


        // Get distance from sensor-------------------------------------------
        // unsigned int distance = irsensor.getDist();
        // Serial.println(distance);






        // Color Sensor



}
