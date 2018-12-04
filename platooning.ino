#include <Adafruit_LSM9DS1.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GPS.h>


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
#define SLAVE_ADDRESS 0x04

//IMU global variables
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
Servo steeringServo;
float pingDistanceCM = 0.0;

//i2c global variables
byte piCommand;
byte piData[2];
byte d;
byte v_desired;
int  pwm_v_ratio = 5;
byte x;
byte y;
int temp_x;


void setup() {
    Serial.begin(115200);    
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
     
//  set up the ping sensor pins
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

int receiveDataI2C(int nPoints) {
      piCommand = Wire.read();
      //Serial.println("here");
      // if Pi is sending data, parse it into incoming data array
      if (piCommand == 255) {
          x = Wire.read();
          //y = Wire.read(); 
          //sSerial.println(x);
          //Serial.println(y);
          //Serial.println((y *3)); 
                  
      }   
      while (Wire.available()) {Wire.read(); }  
}

void sendDataI2C(float some, float some2) {
    if (piCommand == 1) {
        float dataBuffer[2];
        dataBuffer[0] = some;
        dataBuffer[1] = some2;
        Wire.write((byte*) &dataBuffer[0], 2*sizeof(float));
    }

    else if (piCommand == 2) {
        byte dataBuffer[4];
        dataBuffer[0] = some;
        dataBuffer[1] = some2;
        dataBuffer[2] = some;
        dataBuffer[3] = some2;
        Wire.write(&dataBuffer[0], 4);
    }
}

//////////////////////////////////////////////////////////////////
void loop() {
    static byte motorPWM = 150;
//  get the ping distance
    getPingDistanceCM();    
    analogWrite(motorPin,motorPWM);
    if (pingDistanceCM < 10.0)
    {
     analogWrite(motorPin,0);
    }
    else
    {
     analogWrite(motorPin,motorPWM);
    }

  //  parse GPS when available 
  
   /* ask it to read in the data */
   lsm.read();  
   sensors_event_t a, m, g, temp;
   lsm.getEvent(&a, &m, &g, &temp);

  //Heading and distance to center of the road control variables  
    static float delta_t = 0.5;
    float heading_desired = 90.0;
    static float est_heading = 0.0;
    static float est_imu_heading = 0.0;
    static float tao = 0.318;
    float d_desired = 10;
    ;

    //heading desired
    float turn_gain = 0.3;
    float ping_gain = 0.5;
    float prop_gain_heading = 4;
    heading_desired = (x - 128) / 4 + 90;

    //heading control
    est_imu_heading -= (g.gyro.z * 0.02);
    float servo_angle = heading_desired - prop_gain_heading * est_imu_heading;
    steeringServo.write(constrain(heading_desired, -60, 120));

    //velocity feedback
    float gain_d = 2;

    
    int new_value = (pingDistanceCM * gain_d) + 150;
      //Serial.println(new_value) ; 
    motorPWM = (0.85 * new_value) + (1.0 - 0.15) * motorPWM;
    //Serial.println(nominal);
    //Serial.println(trial + " trial");
    
    Serial.println(motorPWM);
        
}

////////////////////////////////////////////////////////////
// Ping Sensor 
void getPingDistanceCM()
{
  const long timeout_us = 6000;

  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }

  pingDistanceCM = constrain(0.017*echo_time,5.0,100.0);
}
