#include <Servo.h>
#include <FlexiTimer2.h>
//servo
Servo motor, rudder, sail;
const int datanum = 4;
int motor_speed = 90;
int motor_speed_old = 90;
int rudder_pos = 90;
int rudder_pos_old = 90; //initial motor speed pwm must be 90
int sail_pos = 50;
int sail_pos_old = 50; // valid input of sailservo is from 50 to 130 baseed on test

int REMOTE_MAX = 2490;
int REMOTE_MIN = 1450;

int MIN_MOTOR = 70;
int MAX_MOTOR = 110;

int durElev = 0;
int durRudd = 0;
int durThro = 0;
int durGear = 0; // used for control type switch (control type: auto or manual)

int serial_out_count = 0; //used to count the times that serial communication error
int mark = 0;

int enableCtrl = 1;//todo

//remote control pins
const int elevPin = 3, throPin = 4, ruddPin = 5, gearPin = 6;
//auto control flag
boolean autoFlag = 1;
//loop count
int count = 0;
// remote controller duration

// velocity limit
int motorVelLimit = 20;
int rudderVelLimit = 6;
int rudderLimit = 35;
int sailVelLimit = 10;
int sailLimit = 40;

struct //total 2+10+2=14 bytes
{
  //header (total 2 bytes)
  byte header1;  // 1 bytes
  byte header2;  // 1 bytes

  //control part (total 10 bytes)
  int readMark;   // 2
  int autoFlag;   // 2
  int motorSpeed; // 2 bytes
  int rudderAng;  // 2
  int sailAng;    // 2


  //crc (total 2 bytes)
  unsigned int crcnum;  //2

} 
arduinoData = {
  0};

void structDataSend() {
  int tmp_motor = map(motor_speed,70,110,0,100);
  arduinoData.readMark = mark;
  arduinoData.autoFlag = autoFlag;
  arduinoData.motorSpeed = tmp_motor;
  arduinoData.rudderAng = rudder_pos - 90;
  arduinoData.sailAng = (sail_pos - 50)/8*9;
  //arduinoData.sailAng = enableCtrl;


  byte *tobyte = (byte*)&arduinoData;
  arduinoData.crcnum = CRC16(tobyte + 2, sizeof(arduinoData) - 4); //the valid data part as used to generate crc
  //  Serial.println(sizeof(sensorData));
  Serial.write(tobyte, sizeof(arduinoData));
}

void setup() {
  Serial.begin(115200);
  motor.attach(9);  // attaches the servo on pin 9 to the servo object
  rudder.attach(10);
  sail.attach(11);

  pinMode(elevPin, INPUT);
  pinMode(throPin, INPUT);
  pinMode(ruddPin, INPUT);
  pinMode(gearPin, INPUT);

  arduinoData.header1 = 0x4f;
  arduinoData.header2 = 0x5e;

  while (!Serial);
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}

unsigned int CRC16(const byte *pBuffer, unsigned int bufferSize)
{
  unsigned int poly = 0x8408;
  unsigned int crc = 0;
  byte carry;
  byte i_bits;
  unsigned int j;
  for (j = 0; j < bufferSize; j++)
  {
    crc = crc ^ pBuffer[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc ^ poly;
      }
    }
  }
  return crc;
}


unsigned int calcCRC(int data[8])
{
  unsigned int poly = 0x8408;
  unsigned int crc = 0;
  int carry;
  int i_bits;
  for (int j = 0; j < 8; j++)
  {
    crc = crc ^ data[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc ^ poly;
      }
    }
  }
  return crc;
}

void serial_read_3() {
  int serialdata[10];
  boolean header_find_flag = false;
  int servodata[4] = { 
    -1  };
  while (Serial.available()) {
    if ((int)Serial.read() == 255 && (int)Serial.read() == 1) {
      header_find_flag = true;
      break;
    }
  }
  if (header_find_flag == true && Serial.available() > 9) {
    for (int i = 0; i < 10; i++) {
      serialdata[i] = (int)Serial.read();
    }
    int Data[8];
    for (int i = 0; i < 8; i++) {
      Data[i] = serialdata[i];
    }
    unsigned int crcnum = serialdata[8] * 256 + serialdata[9];
    unsigned int crchecknum = calcCRC(Data);
    //    Serial.print(crcnum);
    //    Serial.print(",");
    //    Serial.print(crchecknum);
    //    Serial.print(",");
    if (crcnum == crchecknum) {
      for (int i = 0; i < 4; i++) {
        servodata[i] = Data[2 * i] * 256 + Data[2 * i + 1];
      }
      mark = 1;
      motor_speed = map(servodata[0], 0, 100, MIN_MOTOR, MAX_MOTOR);
      rudder_pos = servodata[1];
      sail_pos = servodata[2];
      enableCtrl = servodata[3];
      //foresail_pos = servodata[3];
      serial_out_count = 0;
      //      Serial.print(servodata[0]);
      //      Serial.print(",");
      //      Serial.print(servodata[1]);
      //      Serial.print(",");
      //      Serial.print(servodata[2]);
      //      Serial.print(",");
      //      Serial.println(servodata[3]);
    }
  }
  else {
    serial_out_count ++;
    if (serial_out_count > 50) {  //timeout=10*0.1=1s
      motor_speed = 90;
      rudder_pos = 90;
      sail_pos = 50;
      //foresail_pos = 50;
      mark = 0;
    }
    else {
      motor_speed = motor_speed_old;
      rudder_pos = rudder_pos_old;
      sail_pos = sail_pos_old;
      //foresail_pos = foresail_pos_old;
    }
  }
}

void signalSelection() {
  if (autoFlag == 0){

    durElev = pulseIn(elevPin, HIGH);
    motor_speed = map(durElev, REMOTE_MIN, REMOTE_MAX, MIN_MOTOR,MAX_MOTOR);
    durRudd = pulseIn(ruddPin, HIGH);
    rudder_pos = map(durRudd, REMOTE_MIN, REMOTE_MAX, 50, 130);
    durThro = pulseIn(throPin, HIGH);
    sail_pos = map(durThro, REMOTE_MIN, REMOTE_MAX, 50, 130);
  }
}


void veloLimit() {
  // motorMicroSec = constrain(motorMicroSec, motorMicroSecOld - motorVelLimit, motorMicroSecOld + motorVelLimit);
  rudder_pos = constrain(rudder_pos, rudder_pos_old - rudderVelLimit, rudder_pos_old + rudderVelLimit); //limit up to 50 deg/s
  //sail_pos = constrain(sail_pos, sail_pos_old - sailVelLimit, sail_pos_old + sailVelLimit);
}

void WriteData() {
  motor_speed = constrain(motor_speed, 0, 180);
  if (abs(motor_speed - motor_speed_old) > 4) {
    motor.write(motor_speed);
    motor_speed_old = motor_speed;
  }
  if (rudder_pos >= 0 && rudder_pos <= 180 && abs(rudder_pos - rudder_pos_old) > 3) {
    rudder.write(rudder_pos);
    rudder_pos_old = rudder_pos;
  }
  if (sail_pos >= 45 && sail_pos <= 145) {
    sail.write(sail_pos);
    sail_pos_old = sail_pos;
  }
}

void flash() {
  count ++;
  if (count == 10) { //read the gearPin every 10 intervals
    durGear = pulseIn(gearPin, HIGH, 20000);
    count = 0;
  }

  if (mark == 1 && enableCtrl == 1)
  {
    autoFlag =1;
  }
  else
  {
    if (durGear > 1300 && durGear < 1900) {
      autoFlag = 1;
    }
    else
    {
      autoFlag = 0;
    }
  }


  serial_read_3();
  signalSelection();
  veloLimit();
  WriteData();
  structDataSend();
}

void loop() {

}

