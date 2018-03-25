
//LIBRARYS-----------------------------------------------------------------
#include <Arduino.h> // Arduino Library
#include <Servo.h> // Servo Library
#include <SharpDistSensor.h> // IR Sensor Library
#include <SoftPWMServo.h> // "Pulse Width Modulation" Library. Used to drive DC Motor
#include "../other/color_sensor.cpp"// color sensor function code
#include "../other/ir_sensor.cpp" //ir sensor function code
#include "../other/bno_orientation.cpp"//BNO Oreintation
#include "../other/ir_position.cpp"// IR_Position Sensor AS56000
#include "../other/frequency.cpp"  // IR_ferquency sensor
#include "../other/movement.cpp" // movement code eg: encoder movement

//STATE MACHINE --------------------------------------------------------------
int home_angle=0;




typedef enum {
        STATE_START,// startign orientation
        STATE_GET_COMMAND,
        STATE_PILLAR_SCAN,
        STATE_GET_MISSION,
        STATE_DONE,
} STATE_T;

char *state_names[]= {"STATE_START",
                      "STATE_GET_COMMAND",
                      "STATE_PILLAR_SCAN",
                      "STATE_GET_MISSION",
                      "STATE_DONE",};

STATE_T current_state;
int command_signal;
int num_successful_runs;

// //SETUP--------------------------------------------------------------- SETUP
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
        while (current_frequency == 0)         // frequency changes based on the interupt
        {
                //go_forward();
                delay(250);


                // Serial.print ("current frequency");
                //Serial.println(current_frequency);

                current_frequency= frequency;

        }
        detachCoreTimerService(Counter);

        Serial.print ("found frequency: ");
        Serial.println (frequency);
        // stop servos
        //servo_stop;
        Serial.println("servos stoped");
        // once here, signal is something other than 0
        // Note the signal, going to use it later
        command_signal = current_frequency;

        // set your home angle
        home_angle = getyaw() - 180;
        //Serial.println (home_angle);
        // goto_angle(home_angle);


        return STATE_PILLAR_SCAN;
}

STATE_T find_diameter_size()
{

        int scan_data [181];
        scan(scan_data);

        return STATE_DONE;

}







// // //LOOP-------------------------------------------------------------------LOOP
void loop() {




//STATE MACHINE------------------------------------------------STATE MACHINE

        Serial.print("current state: ");
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
        case STATE_DONE:
                delay(3000);
                break;

        }

        // Serial.println(scancolors());// ---------------------SERIAL PRINT

        // Get distance from sensor-------------------------------------------
        // unsigned int distance = irsensor.getDist();
        //Serial.println(distance);









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
// //         //int yaw = getyaw();
// //         //Serial.println(yaw);//--------------------------------------SERIAL PRINT
// //
// //

// //
//  Serial.println("Detect Frequency:");//-------------------------------SERIAL PRINT
        //Serial.println(frequency);
        //delay(300);







}
//-----------------------------------------------------------------------------------
