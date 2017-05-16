#include <ros.h>
#include <std_msgs/String.h>
#include <sailboat_message/Mach_msg.h>
#include <FlexiTimer2.h>
#include <Servo.h>
#define pi 3.141592654;

//servo configure
Servo motor, rudder, sail;
int motorMicroSec = 1500, motorMicroSecOld = 1500;
int motorDeadZone = 45;

int rudderAng = 140, rudderAngOld = 140;
int sailAng = 140, sailAngOld = 140;
//read data servoData,motorData
int motorData = 0, rudderData = 0, sailData = 0;
int motorDataOld = 100, rudderDataOld = 90, sailDataOld = 90;
//remote control pins
const int elevPin = 3, throPin = 4, ruddPin = 5, gearPin = 6;
//auto control flag
boolean autoFlag = 1;
//loop count
int count = 0;
// remote controller duration
int durGear; // used for control type switch (control type: auto or manual)
// velocity limit
int motorVelLimit = 20;
int rudderVelLimit = 6;
int rudderLimit = 35;
int sailVelLimit = 10;
int sailLimit = 40;

//ROS define 
ros::NodeHandle  nh;
//talker
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
//listener
float motordata= 0;
float rudderdata = 0;
float saildata = 0;
String ss;
char c[10];
void MachmessageCb( const sailboat_message::Mach_msg & msg){
  motordata = msg.motor;
  rudderdata = msg.rudder;
  saildata = msg.sail;
}
ros::Subscriber<sailboat_message::Mach_msg> sub("mach", MachmessageCb);
//endROS

void setup()
{
  //ros node setup
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
  motor.attach(9, 1100, 1900);  // attaches the servo on pin 9 to the servo object
  rudder.attach(10, 1100, 1900);
  sail.attach(11, 1100, 1900);
  pinMode(elevPin, INPUT);
  pinMode(throPin, INPUT);
  pinMode(ruddPin, INPUT);
  pinMode(gearPin, INPUT);
  motor.writeMicroseconds(motorMicroSec);
  rudder.write(rudderAng);
  sail.write(sailAng);
  delay(1000);
  durGear = pulseIn(gearPin, HIGH);
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}


void signalSelection() {
  if (autoFlag == 1) {
    motorMicroSec = map(motorData, 0, 200, 1400, 1600);
    rudderAng = map(rudderData, 50, 130, 100, 180);
    sailAng = map(sailData, 50, 130, 100, 180);
  }
  else {
    motorMicroSec = pulseIn(elevPin, HIGH);
    motorMicroSec = map(motorMicroSec, 1100, 1900, 1300, 1700);
    int durRudd = pulseIn(ruddPin, HIGH);
    rudderAng = map(durRudd, 1100, 1900, 100, 180);
    int durThro = pulseIn(throPin, HIGH);
    sailAng = map(durThro, 1100, 1900, 100, 180);
  }
}

void veloLimit() {
  // motorMicroSec = constrain(motorMicroSec, motorMicroSecOld - motorVelLimit, motorMicroSecOld + motorVelLimit);
  rudderAng = constrain(rudderAng, rudderAngOld - rudderVelLimit, rudderAngOld + rudderVelLimit); //limit up to 50 deg/s
  sailAng = constrain(sailAng, sailAngOld - sailVelLimit, sailAngOld + sailVelLimit);
}


void servoCtrl() {
  motorMicroSecOld = motorMicroSec = constrain(motorMicroSec, 1400, 1600);
  rudderAngOld = rudderAng = constrain(rudderAng, 140 - rudderLimit, 140 + rudderLimit);
  sailAngOld = sailAng = constrain(sailAng, 140 - sailLimit, 140 + sailLimit);
  motor.writeMicroseconds(motorMicroSec);
  int rudderAngReverse = map(rudderAng, 100, 180, 180, 100); //reverse the rudder signal
  rudder.write(rudderAngReverse);
  sail.write(sailAng);
}


void flash() {
  count ++;
  if (count == 10) { //read the gearPin every 10 intervals
    durGear = pulseIn(gearPin, HIGH, 20000);
    count = 0;
  }
  if (durGear < 1950 && durGear > 1050) {
    if (durGear < 1500)
      autoFlag = 0;
    else
      autoFlag = 1;
  }
  //test flag
  //autoFlag = 1;
  signalSelection();
  veloLimit();
  servoCtrl();
}


void loop()
{
  dtostrf(motordata,3,3,c);
  str_msg.data = c;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(50);
}
