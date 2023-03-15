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

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

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


void BMP_INIT(){
        if (!bmp.begin()) {
                        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                 "try a different address!"));
                }

                bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X4, /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X2); /* Filtering. */
}
void IMU_INIT(){
        mpu.initialize();
        mpu.dmpInitialize();
        mpu.setXAccelOffset(-1685);
        mpu.setYAccelOffset(-2219);
        mpu.setZAccelOffset(1259);
        mpu.setXGyroOffset(61);
        mpu.setYGyroOffset(-70);
        mpu.setZGyroOffset(93);
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        fifoCount = mpu.getFIFOCount();
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
}

void getAltitude_main(xTask task_, xTaskParm param_){
        xTaskNotification notif = xTaskNotifyTake(task_);

        if (notif) {

                float altitude = bmp.readAltitude(1018.55); // Change to your local pressure
                Serial.print("\tALT=\t");
                Serial.print(altitude);
                Serial.println("\tm");

                /* Free the heap memory allocated for the direct-to-task
                   notification. */
                xMemFree(notif);
        }

}

void getYPR_main(xTask task_, xTaskParm param_){
        static double lastPitch;
        static double lastYaw;
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
                                double curPitch = ypr[1]*180/PI+90;
                                double curYaw = ypr[2]*180/PI+90;
                                Serial.print("Roll:\t");
                                Serial.print(ypr[0]*180/PI);
                                Serial.print("\tPitch:\t");
                                Serial.print(curPitch);
                                Serial.print("\tYaw:\t");
                                Serial.print(curYaw);
                                Serial.print("\tOMEGA_Pitch:\t");
                                Serial.print((curPitch - lastPitch)/0.005);
                                Serial.print("\tOMEGA_Yaw:\t");
                                Serial.print((curYaw - lastYaw)/0.005);
                                lastPitch = curPitch;
                                lastYaw = curYaw;
                        }
                }
        }

        //call the altitude function
        xTask receiver = xTaskGetHandleByName("ALT");
        if (receiver) {
                xTaskNotifyGive(receiver, 1, "W");
        }
}


// void servoTaskX_main(xTask task_, xTaskParm parm_) {
//         double roll;
//         static int t;

//         if (1440 == t) {
//                 xTaskSuspend(task_);
//         }

//         roll=asin(0.05*cos((3.14/180)*t));
//         X08_X.write(roll*180+90);

//         t++;

//         Serial.println(t);
//         return;
// }

// void servoTaskY_main(xTask task_, xTaskParm parm_) {
//         double pitch;
//         static int t;

//         if (1440 == t) {
//                 xTaskSuspend(task_);
//         }

//         pitch=asin(0.07*sin((3.14/180)*t));
//         X08_Y.write(pitch*180+90);

//         t++;

//         Serial.println(t);
//         return;
// }

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
        BMP_INIT();

        xSystemInit();

        xTask xTask_YPR_Update = xTaskCreate("YPR", getYPR_main, NULL);
        xTask xTask_Altitude_Update = xTaskCreate("ALT", getAltitude_main, NULL);

        if (xTask_YPR_Update && xTask_Altitude_Update) {

                xTaskWait(xTask_YPR_Update);
                xTaskWait(xTask_Altitude_Update);

                xTaskChangePeriod(xTask_YPR_Update, 1);

                xTaskStartScheduler();

                xTaskDelete(xTask_YPR_Update);
                xTaskDelete(xTask_Altitude_Update);
        }

        xSystemHalt();
}

void loop() {
        /* The loop function is not used and should remain empty. */
}