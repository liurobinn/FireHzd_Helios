#include <Arduino.h>
#include <HeliOS.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

Servo X08_X;
Servo X08_Y;

#define OUTPUT_READABLE_WORLDACCEL
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13

bool blinkState = false;
bool dmpReady = false;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

MPU6050 mpu;
float euler[3];
float ypr[3];
double val=0;
double prev;

double pitch;
double roll;
double yaw;

void IMU_INIT(){
    mpu.initialize();
    mpu.dmpInitialize();

    mpu.setXAccelOffset(1095);
    mpu.setYAccelOffset(-1397);
    mpu.setZAccelOffset(1468);
    mpu.setXGyroOffset(-481);
    mpu.setYGyroOffset(164);
    mpu.setZGyroOffset(-10);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();
}

void imuTask_main(xTask task_, xTaskParm param_){

  if (fifoCount < packetSize){
      fifoCount = mpu.getFIFOCount();
  } 
  else{
    if (fifoCount == 1024) {
    
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        
    }
    else{
    
      if (fifoCount % packetSize != 0) {
        
          mpu.resetFIFO();
            
      }
        else{
    
            while (fifoCount >= packetSize) {
            
                mpu.getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;
                
            }    
        
            mpu.dmpGetQuaternion(&q,fifoBuffer);
            mpu.dmpGetGravity(&gravity,&q);
            mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
            
            Serial.print("ypr\t");
            Serial.print(ypr[0]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[1]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[2]*180/PI);
            Serial.println();
            
      }
    }
  }
}

void servoTaskX_main(xTask task_, xTaskParm parm_) {
  double roll;
  static int t;

  if (1440 == t){
      xTaskSuspend(task_);
  }
      roll=asin(0.05*cos((3.14/180)*t));

      X08_X.write(roll*180+90);
      t++;
      Serial.println(t);
  return;
}

void servoTaskY_main(xTask task_, xTaskParm parm_) {
  double pitch;

  static int t;

  if (1440 == t){
      xTaskSuspend(task_);
  }
      pitch=asin(0.07*sin((3.14/180)*t));

      X08_Y.write(pitch*180+90);
      t++;
      Serial.println(t);
  return;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 24;
  Serial.println("initialized");
  X08_X.attach(3);
  X08_Y.attach(4);

  X08_X.write(90);
  X08_Y.write(90);
  IMU_INIT();

  xSystemInit();

  xTask servoXTest = xTaskCreate("TESTX", servoTaskX_main, NULL);
  xTask servoYTest = xTaskCreate("TESTY", servoTaskY_main, NULL);
  xTask IMU_Update = xTaskCreate("IMU", imuTask_main, NULL);

if (servoXTest && servoYTest && IMU_Update) {

    xTaskWait(servoXTest);
    xTaskWait(servoYTest);
    xTaskWait(IMU_Update);

    xTaskChangePeriod(servoXTest, 5);
    xTaskChangePeriod(servoYTest, 5);
    xTaskChangePeriod(IMU_Update, 1);

    xTaskStartScheduler();

    xTaskDelete(servoXTest);
    xTaskDelete(servoYTest);
    xTaskDelete(IMU_Update);
  }

  xSystemHalt();
}

void loop() {
  /* The loop function is not used and should remain empty. */
}