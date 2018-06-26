#include <Servo.h>
#include <FlexiTimer2.h>

//voltage
int voltage1Pin = 0;
int voltage2Pin = 1;
const float voltageRef = 4.85; // Arduino uno voltage reference
float voltage1;
float tmpVoltage1[5] = {0,0,0,0,0};
float voltage2;
float tmpVoltage2[5] = {0,0,0,0,0};

//light
int green_led = 0;
int yellow_led = 0;
int red_led = 1;
const int greenPin = A2, yellowPin = A3, redPin = A4;

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
int mark = 0;//read data from ros

int pcCtrl = 0;//

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

byte header1 = 0xff;
byte header2 = 0x01;

int read_num = 4;
int len_buf_total = 2+2*read_num+2;
int len_buf_now = 0;
//2*len_buf_total-1
byte read_buf[23];

struct //total 2+14+2=18 bytes
{
    //header (total 2 bytes)
    byte header1;  // 1 bytes
    byte header2;  // 1 bytes

    //control part (total 10 bytes)
    int readMark;   // 2
    int autoFlag;   // 2 //rc_ctrl
    int motorSpeed; // 2 bytes  //current motor
    int rudderAng;  // 2        //rc rudder
    int sailAng;    // 2        //rc sail
    int voltage1ten;   // voltage1*10
    int voltage2ten;

    //crc (total 2 bytes)
    unsigned int crcnum;  //2

} arduinoData = {
    0};

void structDataSend() {
    //int tmp_motor = map(motor_speed, 70, 110, 0, 100);
    arduinoData.readMark = mark;
    arduinoData.autoFlag = autoFlag;
    arduinoData.motorSpeed = motor_speed;
    arduinoData.rudderAng = rudder_pos - 54;
    arduinoData.sailAng = (sail_pos - 41) * 4.5;
    arduinoData.voltage1ten = int(voltage1*10);
    arduinoData.voltage2ten = int(voltage2*10);

    byte *tobyte = (byte * ) & arduinoData;
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

    pinMode(greenPin, OUTPUT);
    pinMode(yellowPin, OUTPUT);
    pinMode(redPin, OUTPUT);

    arduinoData.header1 = 0x4f;
    arduinoData.header2 = 0x5e;

    //while (!Serial);
    FlexiTimer2::set(100, flash);
    FlexiTimer2::start();
}

unsigned int CRC16(const byte *pBuffer, unsigned int bufferSize) {
    unsigned int poly = 0x8408;
    unsigned int crc = 0;
    byte carry;
    byte i_bits;
    unsigned int j;
    for (j = 0; j < bufferSize; j++) {
        crc = crc ^ pBuffer[j];
        for (i_bits = 0; i_bits < 8; i_bits++) {
            carry = crc & 1;
            crc = crc / 2;
            if (carry) {
                crc = crc ^ poly;
            }
        }
    }
    return crc;
}

unsigned int calcCRC(int data[8]) {
    unsigned int poly = 0x8408;
    unsigned int crc = 0;
    int carry;
    int i_bits;
    for (int j = 0; j < 8; j++) {
        crc = crc ^ data[j];
        for (i_bits = 0; i_bits < 8; i_bits++) {
            carry = crc & 1;
            crc = crc / 2;
            if (carry) {
                crc = crc ^ poly;
            }
        }
    }
    return crc;
}


int serial_read(){
    int servodata[read_num] = {-1};
    int n = Serial.available();
    while(Serial.available()) {
        Serial.read();
    }
    
    if (n + len_buf_now <= sizeof read_buf){
        for (int i = 0; i<n; i++){
            read_buf[i+len_buf_now] = Serial.read();
        }
        len_buf_now = len_buf_now + n;
    }
    else{
        if (n >= sizeof(read_buf)){
            for (int i= 0; i < n-sizeof(read_buf); i++){
                Serial.read();
            }
            len_buf_now = 0;
            n = sizeof(read_buf);
        }
        else{
            for (int i=0; i < sizeof(read_buf) - n; i++){
                read_buf[i] = read_buf[i + len_buf_now + n - sizeof(read_buf)];
            }
            len_buf_now = sizeof(read_buf) - n;
        }
        for (int i = len_buf_now; i < sizeof(read_buf); i++)
        {
            read_buf[i] = Serial.read();
        }
        len_buf_now = sizeof(read_buf);
    }

    //header is in idx
    int idx = -1;
    for (int i = 0; i < len_buf_now-1; i++){
        if ((int) read_buf[i] == 255 && (int) read_buf[i+1] == 1) {
            idx = i;
            break;
        }
    }

    if (idx < 0){
        len_buf_now = 0;
        return 0;
    }
    if (len_buf_now - idx < len_buf_total){
        for (int i = 0; i < len_buf_now - idx; i++){
            read_buf[i] = read_buf[i+idx];
        }
        len_buf_now = len_buf_now - idx;
        return 0;
    }
    int Data[2*read_num];
    for (int i = 0; i < 2*read_num; i++) {
        Data[i] = (int)read_buf[i+idx];
    }

    unsigned int crcnum = read_buf[idx + 2*read_num] * 256 + read_buf[idx + 2*read_num + 1];
    unsigned int crchecknum = calcCRC(Data);
    
    mark = 2;
    if (crcnum != crchecknum){
        for (int i = 0; i < len_buf_now - idx - 2; i++){
            read_buf[i] = read_buf[i+idx+2];
        }
        len_buf_now = len_buf_now - idx - 2;
        return 0;
    }
    else{
        for (int i = 0; i < read_num; i++) {
            servodata[i] = Data[2 * i] * 256 + Data[2 * i + 1];
        }
        mark = 1;
        motor_speed = map(servodata[0], 0, 100, MIN_MOTOR, MAX_MOTOR);
        green_led = servodata[1];
        yellow_led = servodata[2];
        red_led = servodata[3];

        serial_out_count = 0;
        //      Serial.print(servodata[0]);
        //      Serial.print(",");
        //      Serial.print(servodata[1]);
        //      Serial.print(",");
        //      Serial.print(servodata[2]);
        //      Serial.print(",");
        //      Serial.println(servodata[3]);
        for (int i = 0; i < len_buf_now - idx; i++){
            read_buf[i] = read_buf[i+idx];
        }
        len_buf_now = len_buf_now - idx;
        return 1;
    }
}


void serial_read_3() {
    int num = 4;
    int serialdata[2*num+2];
    boolean header_find_flag = false;
    int servodata[num] = {-1};
    int num_temp = 0;
    while (Serial.available() >= 2) {
        if ((int) Serial.read() == 255 && (int) Serial.read() == 1) {
            header_find_flag = true;
            break;
        }
        num_temp += 1;
        if (num_temp > 2*num+2+4){
            break;
        }
    }
    if (header_find_flag == true && Serial.available() > 2*num+1) {
        for (int i = 0; i < 2*num+2; i++) {
            serialdata[i] = (int) Serial.read();
        }
        int Data[2*num];
        for (int i = 0; i < 2*num; i++) {
            Data[i] = serialdata[i];
        }
        unsigned int crcnum = serialdata[2*num] * 256 + serialdata[2*num+1];
        unsigned int crchecknum = calcCRC(Data);
        //    Serial.print(crcnum);
        //    Serial.print(",");
        //    Serial.print(crchecknum);
        //    Serial.print(",");
        mark = 2;
        if (crcnum == crchecknum) {
            for (int i = 0; i < num; i++) {
                servodata[i] = Data[2 * i] * 256 + Data[2 * i + 1];
            }
            mark = 1;
            motor_speed = map(servodata[0], 0, 100, MIN_MOTOR, MAX_MOTOR);
            // rudder_pos = servodata[1];
            // sail_pos = servodata[2];
            // pcCtrl = servodata[3];
            green_led = servodata[1];
            yellow_led = servodata[2];
            red_led = servodata[3];

            serial_out_count = 0;
            //      Serial.print(servodata[0]);
            //      Serial.print(",");
            //      Serial.print(servodata[1]);
            //      Serial.print(",");
            //      Serial.print(servodata[2]);
            //      Serial.print(",");
            //      Serial.println(servodata[3]);
        }
    } else {
        serial_out_count++;
        if (serial_out_count > 50) {  //timeout=10*0.1=1s
            motor_speed = 90;
            // rudder_pos = 90;
            // sail_pos = 50;
            green_led = 0;
            yellow_led = 0;
            red_led = 2;
            
            //foresail_pos = 50;
            mark = 0;
        } else {
            motor_speed = motor_speed_old;
            //rudder_pos = rudder_pos_old;
            //sail_pos = sail_pos_old;

            // green_led = 0;
            // yellow_led = 0;
            // red_led = 2;
            //foresail_pos = foresail_pos_old;
        }
    }
}

void signalSelection() {
    if (autoFlag == 0) {
        durElev = pulseIn(elevPin, HIGH);
        motor_speed = map(durElev, REMOTE_MIN, REMOTE_MAX, MIN_MOTOR, MAX_MOTOR);
    }
    durRudd = pulseIn(ruddPin, HIGH);
    rudder_pos = map(durRudd, REMOTE_MIN, REMOTE_MAX, 50, 130);
    durThro = pulseIn(throPin, HIGH);
    sail_pos = map(durThro, REMOTE_MIN, REMOTE_MAX, 50, 77);
}


void veloLimit() {
    motor_speed = constrain(motor_speed, 0, 180);
    // if (motor_speed > 80 and motor_speed < 100){
    //     motor_speed = 90;
    // }
    rudder_pos = constrain(rudder_pos, rudder_pos_old - rudderVelLimit,
                           rudder_pos_old + rudderVelLimit); //limit up to 50 deg/s
    //sail_pos = constrain(sail_pos, sail_pos_old - sailVelLimit, sail_pos_old + sailVelLimit);
}

void WriteData() {
    
    if (abs(motor_speed - motor_speed_old) > 4) {
        motor.write(motor_speed);
        motor_speed_old = motor_speed;
    }
    // if (rudder_poses >= 0 && rudder_pos <= 180 && abs(rudder_pos - rudder_pos_old) > 3) {
    //     rudder.write(rudder_pos);
    //     rudder_pos_old = rudder_pos;
    // }
    // if (sail_pos >= 45 && sail_pos <= 145) {
    //     sail.write(sail_pos);
    //     sail_pos_old = sail_pos;
    // }
    if (green_led != 0){
        analogWrite(greenPin, 0);
    }else{
        analogWrite(greenPin, 255);
    }
    if (yellow_led != 0){
        analogWrite(yellowPin, 0);
    }else{
        analogWrite(yellowPin, 255);
    }
    if (red_led != 0){
        analogWrite(redPin, 0);
    }else{
        analogWrite(redPin, 255);
    }
    


}

void voltageCurrentMeter() {
    tmpVoltage1[0] = tmpVoltage1[1];
    tmpVoltage1[1] = tmpVoltage1[2];
    tmpVoltage1[2] = tmpVoltage1[3];
    tmpVoltage1[3] = tmpVoltage1[4];

    tmpVoltage2[0] = tmpVoltage2[1];
    tmpVoltage2[1] = tmpVoltage2[2];
    tmpVoltage2[2] = tmpVoltage2[3];
    tmpVoltage2[3] = tmpVoltage2[4];

    long analogVoltage1 = 0;
    analogVoltage1 = analogRead(voltage1Pin);
    tmpVoltage1[4] = analogVoltage1 / 1024.0 * 5 * voltageRef; //compute real voltage value
    voltage1 = (tmpVoltage1[0]+tmpVoltage1[1]+tmpVoltage1[2]+tmpVoltage1[3]+tmpVoltage1[4])/5;

    long analogVoltage2 = 0;
    analogVoltage2 = analogRead(voltage2Pin);
    tmpVoltage2[4] = analogVoltage2 / 1024.0 * 5 * voltageRef; //compute real voltage value
    voltage2 = (tmpVoltage2[0]+tmpVoltage2[1]+tmpVoltage2[2]+tmpVoltage2[3]+tmpVoltage2[4])/5;
}

void flash() {
    count++;
    if (count == 10) { //read the gearPin every 10 intervals
        durGear = pulseIn(gearPin, HIGH, 20000);
        count = 0;
    }

    if (durGear > 1300 && durGear < 1900) {
        autoFlag = 1;
    } else {
        autoFlag = 0;
    }


    voltageCurrentMeter();
//    int result = serial_read();
//    if (!result){
//        serial_out_count++;
//        if (serial_out_count > 50) {  //timeout=10*0.1=1s
//            motor_speed = 90;
//            // rudder_pos = 90;
//            // sail_pos = 50;
//            green_led = 0;
//            yellow_led = 0;
//            red_led = 2;
//            
//            //foresail_pos = 50;
//            mark = 0;
//        }
//    }
    serial_read_3();
    signalSelection();
    veloLimit();
    WriteData();
    structDataSend();
}


void loop() {
    
}

