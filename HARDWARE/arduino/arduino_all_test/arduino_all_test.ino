#include <Servo.h>
#include <FlexiTimer2.h>
//servo
Servo motor, rudder, mainsail, foresail;
const int datanum = 4;
int motor_speed = 90, rudder_pos = 90;
int motor_speed_old = 90, rudder_pos_old = 90; //initial motor speed pwm must be 90
int mainsail_pos = 45, foresail_pos = 45;
int mainsail_pos_old = 50, foresail_pos_old = 50; // valid input of sailservo is from 45 to 145 baseed on test
int serial_out_count = 0; //used to count the times that serial communication error
int mark = 0;
// encoder
const byte CS = 2;
const byte CL = 3;
const byte DA1 = 4;
const byte DA2 = 5;
float sail = 0; //deg
float dogvane = 0; //deg
// voltage
float voltage = 0;

void setup() {
  Serial.begin(115200);
  motor.attach(9);  // attaches the servo on pin 9 to the servo object
  rudder.attach(10);
  mainsail.attach(11);
  //foresail.attach(12);
  pinMode(CS, OUTPUT);
  pinMode(CL, OUTPUT);
  pinMode(DA1, INPUT);
  pinMode(DA2, INPUT);
  while (!Serial);
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}

unsigned int crc16(int data[8]) {
  unsigned int poly = 0x8408;
  unsigned int crc = 0x0;
  for (int i = 0; i < 8; i++) {
    int bbyte = data[i];
    crc = crc ^ bbyte;
    for (int j = 0; j < 8; j++) {
      unsigned int last = (0xffff & crc) & 1;
      crc = (0xffff & crc) >> 1;
      if (last == 1) {
        crc = crc ^ poly;
      }
    }
  }
  return crc & 0xffff;
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
  int servodata[4] = { -1};
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
      motor_speed = servodata[0];
      rudder_pos = servodata[1];
      mainsail_pos = servodata[2];
      foresail_pos = servodata[3];
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
    if (serial_out_count > 30) {  //timeout=10*0.1=1s
      motor_speed = 90;
      rudder_pos = 90;
      mainsail_pos = 50;
      foresail_pos = 50;
      mark = 0;
    }
    else {
      motor_speed = motor_speed_old;
      rudder_pos = rudder_pos_old;
      mainsail_pos = mainsail_pos_old;
      foresail_pos = foresail_pos_old;
    }
  }
}

void WriteData() {
  if (motor_speed >= 0 && motor_speed <= 180 && abs(motor_speed - motor_speed_old) > 4) {
    motor.write(motor_speed);
    motor_speed_old = motor_speed;
  }
  if (rudder_pos >= 0 && rudder_pos <= 180 && abs(rudder_pos - rudder_pos_old) > 3) {
    rudder.write(rudder_pos);
    rudder_pos_old = rudder_pos;
  }
  if (mainsail_pos >= 45 && mainsail_pos <= 145) {
    mainsail.write(mainsail_pos);
    mainsail_pos_old = mainsail_pos;
  }
  if (foresail_pos >= 45 && foresail_pos <= 145) {
    foresail.write(foresail_pos);
    foresail_pos_old = foresail_pos;
  }
}

void ReadEncoder() {
  long int Angle_1 = 0;
  long int Angle_2 = 0;
  int Angle_1_data = 0;
  int Angle_2_data = 0;
  digitalWrite(CS, LOW);
  for (int i = 0; i < 16; i++) {
    digitalWrite(CL, LOW);
    delay(1);
    digitalWrite(CL, HIGH);
    if (digitalRead(DA1))  Angle_1 ++;
    if (digitalRead(DA2))  Angle_2 ++;
    Angle_1 = Angle_1 << 1;
    Angle_2 = Angle_2 << 1;
  } //end of for loop
  Angle_1 = Angle_1 >> 1;
  Angle_2 = Angle_2 >> 1;
  digitalWrite(CS, HIGH);
  int Angle_1_Status = 0x003F & Angle_1;
  int Angle_2_Status = 0x003F & Angle_2;
  if ((Angle_1_Status >> 1) == 16 | (Angle_1_Status >> 1) == 19) { // 16=10000B,19=10011B
    Angle_1_data = Angle_1 >> 6;
  }
  else  Angle_1_data = 0;
  if ((Angle_2_Status >> 1) == 16 | (Angle_2_Status >> 1) == 19) { // 16=10000B,19=10011B
    Angle_2_data = Angle_2 >> 6;
  }
  else  Angle_2_data = 0;
  sail = Angle_Cal(Angle_1_data, 0);
  dogvane = Angle_Cal(Angle_2_data, 0);
} //end of ReadEncoder

float Angle_Cal(int data, float Offset) {
  float Ang_data = data * 360.0 / 1024.0;

  if (data != 0) {
    if (Ang_data > 180) {
      Ang_data = Ang_data - 360;
    }
    Ang_data = Ang_data - Offset;
  }
  return Ang_data;
}
void ReadVoltage() {
  int raw = analogRead(0); // use A0 to measure voltage
  voltage = raw / 1024.0;
}
void SendData() {
  Serial.print(12);
  Serial.print(",");
  Serial.print(45);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.println(mark);
}

void flash() {
  serial_read_3();
  WriteData();
  SendData();
}

void loop() {

}
