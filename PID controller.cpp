#include <Adafruit_LSM9DS1.h>

// MegaPingTest.ino
// BDK:ESE421:2018C
// Week 2 Lab Sketch -- Test Ping Sensor

#include <SPI.h>
//#include <Adafruit_LSM9DS1.h>
//#include <Adafruit_Sensor.h>
#include <Servo.h>


#include <Adafruit_GPS.h>
//#include <Adafruit_Sensor.h>

#define servoPin 7 // pin for servo signal
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)
#define motorPin 8 
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

//
// global variables
// (yucky but needed to make i2c interrupts work later)
//

unsigned long start_time;
unsigned long end_time;

float gpsLat;
float gpsLon;
float gpsV;
float gpsPsi;
int gpsNSat;

static int trial[2];



Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
Servo steeringServo;

float pingDistanceCM = 0.0;
byte motorPWM=0;

HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

//
// global variables
// (yucky but needed to make i2c interrupts work)
//
byte piCommand;
byte piData[2];
byte d;
byte v_desired;
int  pwm_v_ratio = 5;



void setup() {
    
    Serial.begin(115200);

    //  Activate Interrupt for GPS
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // minimum information only
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    
  if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");
    
    //set ranges for sensor
    
   lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
   lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

    steeringServo.attach(servoPin);
     

//
//  set up the ping sensor pins
//
    pinMode(pingGrndPin,OUTPUT); 
    digitalWrite(pingGrndPin,LOW);
    pinMode(pingTrigPin,OUTPUT);
    pinMode(pingEchoPin,INPUT);
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);

    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveDataI2C);
    Wire.onRequest(sendDataI2C);
}

//  This Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
}

static int camera_angle = 0;

int receiveDataI2C(int nPoints) {
      piCommand = Wire.read();
      //
      // if Pi is sending data, parse it into incoming data array
      //
      
      if (piCommand == 255 && piCommand < 256) {
          d = Wire.read();
          v_desired = Wire.read();
            
      }
      
}

void sendDataI2C(float some, float some2) {

    if (piCommand == 1) {
        float dataBuffer[2];
        dataBuffer[0] = some;
        dataBuffer[1] = some2;
        Wire.write((byte*) &dataBuffer[0], 2*sizeof(float));
        //Serial.println("sending floats");
    }

    else if (piCommand == 2) {
        byte dataBuffer[4];
        dataBuffer[0] = some;
        dataBuffer[1] = some2;
        dataBuffer[2] = some;
        dataBuffer[3] = some2;
        Wire.write(&dataBuffer[0], 4);
        //Serial.println("sending bytes");
    }
}





//////////////////////////////////////////////////////////////////
void loop() {

  //
//  get the ping distance
//
    getPingDistanceCM();
    
    analogWrite(motorPin,motorPWM);
    if (pingDistanceCM < 20.0)
    {
     analogWrite(motorPin,0);
    }
    else
    {
     analogWrite(motorPin,motorPWM);
    }

  //  parse GPS when available (set by interrupt)
//
  
    if (GPS.newNMEAreceived())
    {
       if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
       {
          //Serial.println("GPS Parse Fail");
       }
       else
       {
          gpsLat = (GPS.latitudeDegrees - 40.0);
          gpsLon = (GPS.longitudeDegrees + 75.0);
          gpsV = GPS.speed;
          gpsPsi = GPS.angle;
          gpsNSat = GPS.satellites;
              
       }
    }
  
    

   //IMU stuff

   lsm.read();  /* ask it to read in the data */
   sensors_event_t a, m, g, temp;
   lsm.getEvent(&a, &m, &g, &temp);

    
    static float delta_t = 0.5;
    float prop_gain_heading = 0.5;
    float heading_desired = 90.0;
    static float est_heading = 0.0;
    static float est_imu_heading = 0.0;
    static float est_gps_heading = 0.0;
    static float filtered = 0.0;
    static float tao = 0.318;
    float d_desired = 0;
    float gain_d = 10;
    float filtered_heading;
    float filtered_v;
    float est_v;

    //distance and heading feedback control with complementary filter
    //heading control
    est_imu_heading += g.gyro.z * delta_t;
    filtered_heading += (delta_t / tao) * (gpsPsi - est_imu_heading - filtered_heading);
    est_heading = filtered + est_imu_heading;
    float servo_angle = heading_desired - prop_gain_heading * est_heading;
    steeringServo.write(constrain(servo_angle, -60, 60)); 

    //d control   
    heading_desired = 90 - (d_desired - d) * gain_d; 

    //velocity feedback control
    float tao_v = 300;
    static float est_imu_v = 0;

    est_imu_v += g.gyro.x * delta_t;
    filtered_v += (delta_t / tao_v) * (gpsV - est_imu_v - filtered_v);
    est_v = filtered_v + est_imu_v; 
    motorPWM -=  (v_desired - est_v) * pwm_v_ratio;

    //Serial.println(d);

//  pause 0.05 second
//
    
}

////////////////////////////////////////////////////////////
// Ping Sensor -- update value of pingDistanceCM
////////////////////////////////////////////////////////////
void getPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }

  pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);


}