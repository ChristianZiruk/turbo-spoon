#include <Arduino.h>
#include <Servo.h>
COLOR_TYPE parkcolor; // collor type


Servo rightservo;  // create servo object to control a servo
Servo leftservo;  // create servo object to control a servo


const uint8_t LEFTSERVOPIN=31; // Left Servo plugged into j6
const uint8_t RIGHTSEROVPIN=32;// Right Servo plugged into j5
int pos =0;

void rotate_left(int speed) //Rotates servos in opposite direction to turn left
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90-speed);
    rightservo.write(90-speed);
}
void rotate_right(int speed) //Rotates servos in opposite direction to turn right
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90+speed);
    rightservo.write(90+speed);
}
void go_forward(int speed) // Both servos forward
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90+speed);
    rightservo.write(90-speed);
}

void go_backward(int speed) //Both servos Backwards
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90-speed);
    rightservo.write(90+speed);
}

void servo_stop() // Servos stop
{

    leftservo.write(90);
    rightservo.write(90);
}

void goto_angle (int angle){  // uses the BNO055 ("getyaw") to go to an angle within 20 degrees
    int current_yaw = getyaw();
        while(!(current_yaw > angle-10) && (current_yaw < angle+10)){ // command to go to home angle
    rotate_left(3);



    current_yaw= getyaw();
    }
    Serial.print("Found pillar side");
}

//Find color function used in the COLOR_STATE
COLOR_TYPE forward_until_color(){
        while((scancolors() == WHITE_PAD)){
        go_forward(5);
//The robot goes forward until the the color is no longer white
    }
    servo_stop();
    return scancolors();
}


// ------------------------------SWITCH CASE FOR PARK STATE------------------------------

// When the color is found to be black the robot will rotate to the color that is found from the pillar
void rotate_black(){

if ( parkcolor == BLACK_PAD) { // if park color and color landed on is the same ---SUCCESS!

    servo_stop();
    Serial.print("success Turbo-Spoon Parked for Black");
}
    else if (parkcolor == RED_PAD ){  // If pillar color is red the robot will rotate left and go forward until park color is found
        while((scancolors() == parkcolor)){
        rotate_left(4);
        delay(300);
        go_forward(3);
        Serial.print("success Turbo-Spoon Parked for RED");
        }
    }
    else if (parkcolor == BLUE_PAD ){
        while((scancolors() == parkcolor)){
            rotate_left(4);
            Serial.print("success Turbo-Spoon Parked For BLUE");

        }
    }


    }
//Rotate RED

// When the color is found to be black the robot will rotate to the color that is found from the pillar
void rotate_red(){

if ( parkcolor == RED_PAD) {

    servo_stop();
    Serial.print("success Turbo-Spoon Parked for RED");
}
    else if (parkcolor == BLACK_PAD ){
        while((scancolors() == parkcolor)){
        rotate_right(4);
        delay(300);
        go_forward(4);
        Serial.print("success Turbo-Spoon Parked for Black");
        }
    }
    else if (parkcolor == BLUE_PAD ){
        while((scancolors() == parkcolor)){
            rotate_right(4);
            Serial.print("success Turbo-Spoon Parked For BLUE");

        }
    }


    }

// When the color is found to be black the robot will rotate to the color that is found from the pillar
//Rotate BLUE
void rotate_blue(){

if ( parkcolor == 0) {

    servo_stop();
    Serial.print("success Turbo-Spoon Parked for Blue");
}
    else if (parkcolor == 1){
        while((scancolors() == parkcolor)){
        rotate_left(4);
        Serial.print("success Turbo-Spoon Parked for RED");
        }
    }
    else if (parkcolor == 3 ){
        while((scancolors() == parkcolor)){
            rotate_right(4);
            Serial.print("success Turbo-Spoon Parked For BLACK");

        }
    }


    }
