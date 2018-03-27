
//LIBRARYS-----------------------------------------------------------------
#include <Arduino.h> // Arduino Library
#include <Servo.h> // Servo Library
#include <SharpDistSensor.h> // IR Sensor Library GITHUB LIBRARY " Creator:DRG Freeman"
#include <SoftPWMServo.h> // "Pulse Width Modulation" Library. Used to drive DC Motor
#include "../other/color_sensor.cpp"// color sensor function code
#include "../other/bno_orientation.cpp"//BNO 0555 Oreintation "Creator Adafruit"
#include "../other/ir_position.cpp"// IR_Position Sensor AS56000
#include "../other/frequency.cpp"  // IR_ferquency sensor//
#include "../other/movement.cpp" // movement code eg: encoder movement
#include"../other/limit_switch_interupt.cpp"// Limit Switch interup "Emergency Switch"
//STATE MACHINE --------------------------------------------------------------

int magneto=0;
//int home_angle=0;







typedef enum {
        STATE_START,// startign orientation
        STATE_GET_COMMAND,
        STATE_PILLAR_SCAN,
        STATE_GO_COLOR,
        STATE_PARK,
        STATE_DONE,
} STATE_T;

char *state_names[]= {"STATE_START",
                      "STATE_GET_COMMAND",
                      "STATE_PILLAR_SCAN",
                      "STATE_GO_COLOR",
                      "STATE_PARK",
                      "STATE_DONE",};

STATE_T current_state;
int command_signal;
int num_successful_runs;

// //SETUP--------------------------------------------------------------- SETUP
void setup() {
        Serial.begin(115200); //Hardware Serial Initialize
        Serial1.begin(115200);//Software Seerial Initialized Talking to the BNO0055 orientation sensor
//Interupt
        attachInterrupt(1,limit_switch_ISR,FALLING);
attachInterrupt(2,limit_switch_ISR,FALLING);



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

//FREQUENCY Sensor
        setup_frequency();//Setup declared in "frequency folder"
//STATE MACHINE ---------------------------------------------------------------
        current_state = STATE_START;
        num_successful_runs=0;

// // // SET UP COMPLETE
        Serial.println("setup complete");
// // // DO NOT PRINT OR ADD BELOW IN SETUP. !KEEP CODE! ABOVE "SETUP COMPLETE"
}
//END OF SETUP---------------------------------------------------------------

STATE_T find_command_signal()
{
        int current_frequency=frequency;

        while (current_frequency == 0)
        {
                // frequency changes based on the interupt
                rotate_left(2);


                Serial.print ("current frequency");
                Serial.println(current_frequency);

                current_frequency= frequency;
        }

        detachCoreTimerService(Counter);

        Serial.print ("found frequency: ");
        Serial.println (current_frequency);
        if (is_valid_frequency(current_frequency))// make code for valid frquency function
        {
                // stop servos
                servo_stop();
                // delay(200)
                Serial.println("s stoped");

                // Note the signal, Use it for pillar scan
                command_signal = current_frequency;
//Serial.print("BEFOREHOME");
                // rotate_left(5);
                // delay(2000);

                int home_angle=0;
                home_angle = getyaw() - 180;
                Serial.println (home_angle);
                goto_angle(home_angle);// goes to home angle 180 degrees diffrence from the initial BNO input angle


                 return STATE_PILLAR_SCAN;


Serial.print("AfterHOME") ;
        }
        //implicit else

       return STATE_GET_COMMAND;
}

STATE_T find_diameter_size()
{

        struct pillar_all pillar_data;
        int scan_data [181];
        scan(scan_data);
        pillar_data = clasify_pillars(scan_data);


        switch(frequency)
        {
        case 50:     //large pillar/

                goto_angle(pillar_data.large.angle+getyaw());  //goes to the "go_to_angle function and passes the data from the scan function from the struct class"

                break;
        case 100:    // Medium Pillar
                goto_angle(pillar_data.medium.angle+getyaw()); // ^^^^^^^^
                break;
        case 200:     //Small Pillar
                goto_angle(pillar_data.small.angle+getyaw()); //^^^^^^^^^^
                break;
        }

        //move forward

        return STATE_GO_COLOR;
}
STATE_T color_pillar(){
        parkcolor=forward_until_color();
        return STATE_PARK;
}

STATE_T park(){
        go_backward(5);
        delay(2000);
        setup_frequency();
        while (frequency == 0)
        {
                // frequency changes based on the interupt
                rotate_left(3);
        }
        detachCoreTimerService(Counter);

        Serial.print ("found frequency: ");
        forward_until_color();
        COLOR_TYPE current_color= scancolors();
        switch (current_color) {
        case BLACK_PAD:
                rotate_black();//Function if the robot lands on black first
                break;
        case RED_PAD:
                //rotate_red();//Function if the robot lands on red first
                break;
        case BLUE_PAD:
                //rotate_blue();// Function if the robot lands on blue first
                break;
        }

        return STATE_DONE;
}
// // //LOOP-------------------------------------------------------------------LOOP
void loop() {
        // magneto= getirposition();
        // Serial.println(magneto);

            //while(1);
//STATE MACHINE------------------------------------------------STATE MACHINE`

        Serial.print("current state:");
        Serial.println(state_names[current_state]);
        switch (current_state)
        {

        case STATE_START:
                current_state = STATE_GET_COMMAND;
                break;
        case STATE_GET_COMMAND:
                current_state = find_command_signal();
                break;
        case STATE_PILLAR_SCAN:
                current_state = find_diameter_size();
                break;
        case STATE_GO_COLOR:
                current_state = color_pillar();
                break;
        case STATE_PARK:
                current_state = park();
                break;
        case STATE_DONE:
                delay(3000);
                break;

        }

        //Serial.println(scancolors());// ---------------------SERIAL PRINT

        //Get distance from sensor-------------------------------------------
        //unsigned int distance = irsensor.getDist();
        //Serial.println(distance);








//
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





// //
// //         //GET THE YAW POSITION-------------------------------------------------
        //int yaw = getyaw();
        // Serial.println(yaw);//--------------------------------------SERIAL PRINT
// //
// //

// //
//  Serial.println("Detect Frequency:");//-------------------------------SERIAL PRINT
        //Serial.println(frequency);
        //delay(300);






}

//-----------------------------------------------------------------------------------
