#include <Arduino.h>
#include <Servo.h>

Servo rightservo;  // create servo object to control a servo
Servo leftservo;  // create servo object to control a servo


const uint8_t LEFTSERVOPIN=30; // Left Servo plugged into j6
const uint8_t RIGHTSEROVPIN=31;// Right Servo plugged into j5
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

void servo_stop(int speed)
{
    speed = map(speed,0,100,0,90);
    leftservo.write(0-speed);
    rightservo.write(0-speed);
}
