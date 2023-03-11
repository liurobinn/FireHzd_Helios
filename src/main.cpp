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

float determinant(float matrix[][10], int n) {
  float det = 0;
  if (n == 1) {
    det = matrix[0][0];
  } else if (n == 2) {
    det = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
  } else {
    for (int k = 0; k < n; k++) {
      float submatrix[10][10];
      int i = 0;
      for (int row = 1; row < n; row++) {
        int j = 0;
        for (int col = 0; col < n; col++) {
          if (col != k) {
            submatrix[i][j] = matrix[row][col];
            j++;
          }
        }
        i++;
      }
      det += pow(-1, k)*matrix[0][k]*determinant(submatrix, n-1);
    }
  }
  return det;
}

// float matrix[4][4] = {{1, 2, 3, 4},
//                       {5, 6, 7, 8},
//                       {9, 10, 11, 12},
//                       {13, 14, 15, 16}};
// int n = 4;
// float det = determinant(matrix, n);
// Serial.print("Determinant = ");
// Serial.println(det);

double oppoLengthLawOfCosine (double a, double b, double angleInRadians){
    return sqrt(a*a + b*b - 2*a*b*cos(angleInRadians));
}

double oppoAngleLawOfCosine(double a, double b, double c) {
    // Calculate the cosine of the unknown angle using the law of cosines
    double cos_angle = (a*a + b*b - c*c) / (2*a*b);

    // Convert the cosine to an angle in radians using the inverse cosine function
    double angle = acos(cos_angle);

    return angle;
}

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


void getPitch_main(xTask task_, xTaskParm param_){

        if (fifoCount < packetSize) {
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
                        }
                }
        }
}

void getYaw_main(xTask task_, xTaskParm param_){

        if (fifoCount < packetSize) {
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

                                Serial.print("\t");
                                Serial.print(ypr[1]*180/PI);
                        }
                }
        }
}

void getRoll_main(xTask task_, xTaskParm param_){

        if (fifoCount < packetSize) {
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

        if (1440 == t) {
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

        if (1440 == t) {
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
        xTask xTask_Pitch_Update = xTaskCreate("PITCH", getPitch_main, NULL);
        xTask xTask_Yaw_Update = xTaskCreate("YAW", getYaw_main, NULL);
        xTask xTask_Roll_Update = xTaskCreate("ROLL", getRoll_main, NULL);

        if (servoXTest && servoYTest && xTask_Pitch_Update && xTask_Yaw_Update && xTask_Roll_Update) {

                xTaskWait(servoXTest);
                xTaskWait(servoYTest);
                xTaskWait(xTask_Pitch_Update);
                xTaskWait(xTask_Yaw_Update);
                xTaskWait(xTask_Roll_Update);

                xTaskChangePeriod(servoXTest, 5);
                xTaskChangePeriod(servoYTest, 5);
                xTaskChangePeriod(xTask_Pitch_Update, 1);
                xTaskChangePeriod(xTask_Yaw_Update, 1);
                xTaskChangePeriod(xTask_Roll_Update, 1);

                xTaskStartScheduler();

                xTaskDelete(servoXTest);
                xTaskDelete(servoYTest);
                xTaskDelete(xTask_Pitch_Update);
                xTaskDelete(xTask_Yaw_Update);
                xTaskDelete(xTask_Roll_Update);
        }

        xSystemHalt();
}

void loop() {
        /* The loop function is not used and should remain empty. */
}