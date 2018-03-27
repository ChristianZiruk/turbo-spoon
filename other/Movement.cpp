#include <Arduino.h>
#include <Servo.h>
COLOR_TYPE parkcolor; // collor type


Servo rightservo;  // create servo object to control a servo
Servo leftservo;  // create servo object to control a servo


const uint8_t LEFTSERVOPIN=31; // Left Servo plugged into j6
const uint8_t RIGHTSEROVPIN=32;// Right Servo plugged into j5
int pos =0;

void rotate_left(int speed)
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90-speed);
    rightservo.write(90-speed);
}
void rotate_right(int speed)
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90+speed);
    rightservo.write(90+speed);
}
void go_forward(int speed)
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90+speed);
    rightservo.write(90-speed);
}

void go_backward(int speed)
{
    speed = map(speed, 0, 100, 0, 90);
    leftservo.write(90-speed);
    rightservo.write(90+speed);
}

void servo_stop()
{

    leftservo.write(90);
    rightservo.write(90);
}

void goto_angle (int angle){
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

    }
    servo_stop();
    return scancolors();
}

void rotate_black(){

if ( parkcolor == BLACK_PAD) {

    servo_stop();
    Serial.print("success Turbo-Spoon Parked for Black");
}
    else if (parkcolor == RED_PAD ){
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
