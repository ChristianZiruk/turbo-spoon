#include <Arduino.h>
#include <Servo.h>

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
        while(!(current_yaw > angle-3) && (current_yaw < angle+3)){ // command to go to home angle
    rotate_right(10);
    current_yaw= getyaw();
    }
}

//Find color function used in the COLOR_STATE
COLOR_TYPE forward_until_color(){
        while((scancolors() == WHITE_PAD)){
        go_forward(5);

    }
    servo_stop();
    return scancolors();
}
