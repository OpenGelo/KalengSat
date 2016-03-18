#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <PWMServo.h> 
#include "I2Cdev.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define RXPin 2
#define TXPin 4
#define GPSBaud 9600
#define ConsoleBaud 9600  
double  heading,distanceToDestination,courseToDestination,alt,spd,temperature;
char latt[25];
char longg[25];

 
PWMServo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

//gps
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
#define TITIK_LAT -6.9061874
#define TITIK_LNG 107.5837731
int x4=0;

//end of gps

//sensor
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t HMC5883L = 0x1E; // Address of magnetometer

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

double gyroXrate, gyroYrate, gyroZrate;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

//end of sensor

// motor
int STBY = 10; //standby

//Motor A
int PWMA = 3; //Speed control 
int AIN1 = 6; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction
//end of motor

//PID
float oldangle;
float error;
float prev_error;
float integral;
float derivative;
float angleSetpoint;
float gyro;
float gyro_now;
float prev_gyro;
int i;
float PID;
float actualAngle;
unsigned long now;
unsigned long lastTime;
float timeChange;
int run_speed=0;
int turn_speed=0;
int dir;
int PWM_L, PWM_R;

//End of PID

void setup() {
    Wire.begin();
    Serial.begin(ConsoleBaud);
    ss.begin(GPSBaud);
    timer = micros(); // Initialize the timer
    setupsensor();
    setupmotor();
    myservo.attach(9);
    myservo.write(110);
    prev_error=0;
    integral=0;
    angleSetpoint=0; 
}

void setupmotor(){
   pinMode(STBY, OUTPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
  
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(13, OUTPUT);
}

void setupsensor(){
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  
    while (i2cRead(MPU6050, 0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
      Serial.print(F("Error reading sensor"));
      while (1);
    }
  
    while (i2cWrite(HMC5883L, 0x02, 0x00, true)); // Configure device for continuous mode
    calibrateMag();
  
    delay(100); // Wait for sensors to stabilize
  
    /* Set Kalman and gyro starting angle */
    updateMPU6050();
    updateHMC5883L();
    updatePitchRoll();
    updateYaw();
  
    kalmanX.setAngle(roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;
  
    kalmanY.setAngle(pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;
  
    kalmanZ.setAngle(yaw); // And finally yaw
    gyroZangle = yaw;
    compAngleZ = yaw;
}

bool balancingg = false;
bool homing = false;

void loop() {
    ReceiveData();
    feedgps();
    updateinitial();
    getheading();
    printt();
    if (balancingg){
      updateinitial();
      balancing();
    }
    if (homing){
       ggps(); 
    }
} 

void feedgps()
{
  while (ss.available())
  {
    if (gps.encode(ss.read())){
      if (gps.location.isValid()){
           distanceToDestination = TinyGPSPlus::distanceBetween(
          gps.location.lat(), gps.location.lng(),TITIK_LAT, TITIK_LNG);
           courseToDestination = TinyGPSPlus::courseTo(
          gps.location.lat(), gps.location.lng(), TITIK_LAT, TITIK_LNG);
          alt =gps.altitude.feet();
          spd =gps.speed.mph();
          dtostrf((gps.location.lat()),8,6,latt);
          dtostrf((gps.location.lng()),8,6,longg);
      } else
      {
        Serial.println(F("GPS not LOCKED"));
        digitalWrite(13, HIGH); 
        feedgps();
      }
    }
  }
}

void servokunci(){
 myservo.write(0); 
}

void servobuka(){
 myservo.write(110); 
}

void ReceiveData(){
  
  if (Serial.available()){
    char a = Serial.read();
    switch (a){
      case 'z':
        servokunci();
      break;
      case 'b':
            balancingg=true;
      break;
      case 'g':
            balancingg=false;
            stop();
      break;
      case 'c':
        digitalWrite(13, LOW); 
      break;
      case 'x':
        servobuka();
      break;
      case 'w':
        maju();
      break;
      case 'a':
        kiri();
      break;
      case 'd':
        kanan  ();
      break;
      case 's':
        stop();
      break; 
      case 'e':
        homing=true;
      break; 
      
    } 
    
  }
  
}

void getheading(){
      updateHMC5883L();
    heading = atan2(magX, magY);
    heading = (heading * 180/M_PI)-73;
    heading = map(heading,-253,107,-180,180);
    heading = map(heading,-180,180,0,360);
}


void ggps(){
    if (distanceToDestination <= 5.0)
    {
      stop();
      Serial.println("done");
    }
    else {
      updateinitial();
      getheading();
//      printt();
      x4=courseToDestination-heading;
      hitung();
    }

}


void hitung(){
    if(x4>=-5 && x4<=5){
    forward();
    } else {
          if (x4>=-180 && x4<=0){
              rightturn();
          } 
          else if(x4<-180){
              leftturn();
          } 
          else if(x4>=0 && x4<180){
             leftturn();
          }
          else
          {    
          rightturn();
          }
    }
}
  
void rightturn(){
  updateinitial();
  getheading();
//  printt();
  move(1, 200, 0); //motor 1, full speed, left
  move(0, 0, 0); //motor 1, full speed, left
  delay(60);
  getheading();
  int x6=courseToDestination-heading;  
  if(x6>=-5 && x6<=5){
    ggps();
  }
  rightturn();
}

void leftturn(){
  updateinitial();
  getheading();
//  printt();
  move(0, 200, 0); //motor 1, full speed, left
  move(1, 0, 0); //motor 1, full speed, left
  delay(60);
  getheading();
  int x5=courseToDestination-heading;  
  if(x5>=-5 && x5<=5){
    ggps();
    } 
  leftturn();
}

void forward(){
  updateinitial();
  getheading();
//  printt();
  move(1, 255, 0); //motor 1, full speed, left
  move(2, 255, 0); //motor 2, full speed, left
  delay(60);
  ggps();
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
  return;
}

void updateinitial(){
  /* Update all the IMU values */
  updateMPU6050();
  updateHMC5883L();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();


  /* Roll and pitch estimation */
  updatePitchRoll();
  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


  /* Yaw estimation */
  updateYaw();
  gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    compAngleZ = yaw;
    kalAngleZ = yaw;
    gyroZangle = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


  /* Estimate angles using gyro only */
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
  //gyroYangle += kalmanY.getRate() * dt;
  //gyroZangle += kalmanZ.getRate() * dt;

  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
    
 temperature = (double)tempRaw / 340.0 + 36.53;
}

void updateMPU6050() {
  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = -((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = -(i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = -(i2cData[12] << 8) | i2cData[13];
}

void updateHMC5883L() {
  while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
  magX = ((i2cData[0] << 8) | i2cData[1]);
  magZ = ((i2cData[2] << 8) | i2cData[3]);
  magY = ((i2cData[4] << 8) | i2cData[5]);
}

void updatePitchRoll() {
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;

  magX *= magGain[0];
  magY *= magGain[1];
  magZ *= magGain[2];

  magX -= magOffset[0];
  magY -= magOffset[1];
  magZ -= magOffset[2];

  double rollAngle = kalAngleX * DEG_TO_RAD;
  double pitchAngle = kalAngleY * DEG_TO_RAD;

  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

  yaw *= -1;
}

void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
  i2cWrite(HMC5883L, 0x00, 0x11, true);
  delay(100); // Wait for sensor to get ready
  updateHMC5883L(); // Read positive bias values

  int16_t magPosOff[3] = { magX, magY, magZ };

  i2cWrite(HMC5883L, 0x00, 0x12, true);
  delay(100); // Wait for sensor to get ready
  updateHMC5883L(); // Read negative bias values

  int16_t magNegOff[3] = { magX, magY, magZ };

  i2cWrite(HMC5883L, 0x00, 0x10, true); // Back to normal

  magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
  magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);
  magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);
}

void printt(){
        String data = "";
        data +=kalAngleX;data+=",";
        data +=kalAngleY;data+=",";
        data +=kalAngleZ;data+=",";
        data +=accX / 16384.0;data+=",";
        data +=accY / 16384.0;data+=",";
        data +=accZ / 16384.0;data+=",";
        data +=gyroXrate;data+=",";
        data +=gyroYrate;data+=",";
        data +=gyroZrate;data+=",";
        data +=magX;data+=",";
        data +=magY;data+=",";
        data +=magZ;data+=",";
        data +=temperature;data+=",";
        data +=latt;data+=",";
        data +=longg;data+=",";
        data +=alt;data+=",";
        data +=spd;data+=",";
        data +=distanceToDestination;data+=",";
        data +=courseToDestination;data+=",";
        data +=heading;
        Serial.println(data);    
}

void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void balancingmaju(){
  if (kalAngleX < -140 && kalAngleX > -180) actualAngle=kalAngleX+360;  
  else actualAngle=kalAngleX;
  now=millis();
  timeChange=(now-lastTime)/1000.0; Serial.print("dt:");Serial.print(timeChange); Serial.print("\t");
  error = angleSetpoint-actualAngle;
  integral = constrain(integral+error*timeChange,-100,100);
  derivative = (error - prev_error)/timeChange;
  PID = 4*error + 5*integral + 0.5*derivative;//}
  PWM_L=abs(PID+run_speed+turn_speed);
  PWM_R=abs(PID+run_speed-turn_speed);
  prev_error=error;
  lastTime=now;
  
  if (PWM_L>255) PWM_L=255;
  if (PWM_R>255) PWM_R=255;  
  gyro=gyro_now-prev_gyro;
  
  if (PID>0) dir=0;
  if (PID<0) dir=1;
  
  move(1, PWM_L, dir); //motor 1, full speed, left
  move(2, PWM_R, dir); //motor 2, full speed, left
  prev_gyro=gyroXrate;
}
 
void forwardd()
{
  run_speed=100;
  turn_speed=0;
}

void turnleft()
{
  run_speed=0;
  turn_speed=20;
}

void turnright()
{
  run_speed=0;
  turn_speed=-20;
} 

void balancing(){
  if (kalAngleX < -140 && kalAngleX > -180) actualAngle=kalAngleX+360;  
  else actualAngle=kalAngleX;
  now=millis();
  timeChange=(now-lastTime)/1000.0;
  error = angleSetpoint-actualAngle;
  integral = constrain(integral+error*timeChange,-100,100);
  derivative = (error - prev_error)/timeChange;
  PID = 4*error + 5*integral + 0.5*derivative;//}
  PWM_L=abs(PID);
  PWM_R=abs(PID);
  prev_error=error;
  lastTime=now;
  
  if (PWM_L>255) PWM_L=255;
  if (PWM_R>255) PWM_R=255;  
  gyro=gyro_now-prev_gyro;
  
  if (PID>0) dir=0;
  if (PID<0) dir=1;
  
  move(1, PWM_L, dir); //motor 1, full speed, left
  move(2, PWM_R, dir); //motor 2, full speed, left
  prev_gyro=gyroXrate;
}

void maju(){
  move(1, 255, 0); //motor 1, full speed, left
  move(2, 255, 0); //motor 2, full speed, left 
}

void kiri(){
  move(0, 200, 0); //motor 1, full speed, left
  move(1, 0, 0); //motor 1, full speed, left
}

void kanan(){
  move(1, 200, 0); //motor 1, full speed, left
  move(0, 0, 0); //motor 1, full speed, left
}




