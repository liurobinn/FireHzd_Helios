#include <Arduino.h>
#include <HeliOS.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

/*Define servo names for the TVC*/
Servo X08_X;
Servo X08_Y;


/*Define buzzer with its pin number*/
#define BUZZER_PIN 10

#define F15Impulse 25.26
#define WETMASS 1.2
#define CRITICAL_LAUNCH_ACCELERATION 15.45 //m/s^2 upwards
#define GRAVITATION_ACCELERATION -9.81 //m/s^2

/*Define three LED colors with their pin numbers*/
#define LED_RED 14
#define LED_GREEN 15
#define LED_BLUE 16

/*Define four pyro channels with their pin numbers*/
#define PYRO_UL 5
#define PYRO_UR 6
#define PYRO_LR 7
#define PYRO_LL 8

bool blinkState = false;
bool dmpReady = false;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

/* orientation/motion vars */
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

MPU6050 mpu;
float euler[3];
float ypr[3];
float val=0;
float prev;

float pitch;
float roll;
float yaw;
int16_t ax, ay, az;

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

float determinant(float matrix[][10], int n) {
        float det = 0; /* initialize determinant */
        if (n == 1) { /* base case for 1x1 matrix */
                det = matrix[0][0];
        } else if (n == 2) { /* base case for 2x2 matrix */
                det = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
        } else { /* recursive case */
                for (int k = 0; k < n; k++) { /* iterate over first row to create submatrices */
                        float submatrix[10][10]; /* initialize submatrix */
                        int i = 0;
                        for (int row = 1; row < n; row++) { /* iterate over rows to create submatrix */
                                int j = 0;
                                for (int col = 0; col < n; col++) { /* iterate over columns to create submatrix */
                                        if (col != k) { /* exclude the column being used to create submatrix */
                                                submatrix[i][j] = matrix[row][col];
                                                j++;
                                        }
                                }
                                i++;
                        }
                        det += pow(-1, k)*matrix[0][k]*determinant(submatrix, n-1); /* recursively calculate determinant of submatrix */
                }
        }
        return det; /* return the determinant */
}
// float matrix[4][4] = {{1, 2, 3, 4},
//                       {5, 6, 7, 8},
//                       {9, 10, 11, 12},
//                       {13, 14, 15, 16}};
// int n = 4;
// float det = determinant(matrix, n);
// Serial.print("Determinant = ");
// Serial.println(det);

class PID {
private:
        float lastTime = micros();
        float integral = 0;
        float lastErr = 90      ;

public:
        float p;
        float i;
        float d;

        float UPDATE(float curErr) { //curErr should be in degrees/s
                float curTime = millis()/1000;
                float dt = (curTime - lastTime);
                float dx = curErr - lastErr;
                lastErr=curErr;
                integral += curErr*dt;
                lastTime = millis()/1000;
        return p*curErr + i*integral + d*dx/dt;
}
};

PID xPID;
PID yPID;

bool launch_Detected(){
        if (fifoCount < packetSize) {
                /* If there is not enough data, get the number of bytes available in the FIFO buffer */
                fifoCount = mpu.getFIFOCount();
                Serial.print(millis());
                Serial.println("\tGROUND");
                return false;
        } else {
                /* If there is enough data in the FIFO buffer to process, check for FIFO overflow */
                if (fifoCount == 1024) {
                        /* If there is FIFO overflow, reset the FIFO buffer and print an error message */
                        mpu.resetFIFO();
                        Serial.println(F("FIFO overflow!"));
                } else {
                        /* If there is no FIFO overflow, check if the number of bytes in the FIFO buffer is a multiple of the packet size */
                        if (fifoCount % packetSize != 0) {
                                /* If the number of bytes in the FIFO buffer is not a multiple of the packet size, reset the FIFO buffer */
                                mpu.resetFIFO();
                        } else {
                                /* If there is enough data in the FIFO buffer and no overflow, process the data in the buffer */
                                while (fifoCount >= packetSize) {
                                        /* Get the next packet of data from the FIFO buffer */
                                        mpu.getFIFOBytes(fifoBuffer, packetSize);
                                        /* Subtract the number of bytes in the packet from the number of bytes in the FIFO buffer */
                                        fifoCount -= packetSize;
                                }

                                /* Get the quaternion, gravity, and yaw-pitch-roll values from the MPU sensor */
                                mpu.dmpGetQuaternion(&q, fifoBuffer);
                                mpu.dmpGetGravity(&gravity, &q);
                                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                                mpu.getAcceleration(&ax, &ay, &az);
                                float curZ_Acc = (az/4096.0)*9.81;
                                if(curZ_Acc > CRITICAL_LAUNCH_ACCELERATION-2){
                                        return true;
                                }else{
                                        Serial.print(millis());
                                        Serial.println("\tGROUND");
                                        return false;
                                }
                                
                        }
                }
        }
        
}

void PYRO_INIT(){
        pinMode(PYRO_LL, OUTPUT);   /* Set the lower left pyroelectric sensor pin as an output */
        pinMode(PYRO_LR, OUTPUT);   /* Set the lower right pyroelectric sensor pin as an output */
        pinMode(PYRO_UL, OUTPUT);   /* Set the upper left pyroelectric sensor pin as an output */
        pinMode(PYRO_UR, OUTPUT);   /* Set the upper right pyroelectric sensor pin as an output */
}

/* BMP280 Initiation*/
void BMP_INIT(){
        /*Check if the BMP280 sensor is detected.*/
        if (!bmp.begin()) {
                Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                 "try a different address!"));
        }
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,         /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X1,         /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X2,         /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_OFF,
                        Adafruit_BMP280::STANDBY_MS_1);         /* Filtering. */
}

/* LED Initiation*/
void LED_INIT(){
        pinMode(LED_RED, OUTPUT);   //R
        pinMode(LED_GREEN, OUTPUT); //G
        pinMode(LED_BLUE, OUTPUT);  //B
}

void LED_TEST() {
        /*Flash the LED*/
        digitalWrite(LED_RED, LOW);
        delay(500);
        digitalWrite(LED_RED, HIGH);
        delay(500);

        digitalWrite(LED_BLUE, LOW);
        delay(500);
        digitalWrite(LED_BLUE, HIGH);
        delay(500);

        digitalWrite(LED_GREEN, LOW);
        delay(500);
        digitalWrite(LED_GREEN, HIGH);
        delay(500);
}

/* Buzzer Initiation*/
void BUZZER_INIT(){
        /*Attach Buzzer Pin*/
        pinMode(BUZZER_PIN,OUTPUT);
}

void BUZZER_TEST() {
        // define note frequencies in Hz
  #define NOTE_C5 523
  #define NOTE_D5 587
  #define NOTE_E5 659
  #define NOTE_F5 698
  #define NOTE_G5 784
  #define NOTE_A5 880

        // define melody notes and durations
        int melody[] = { NOTE_C5, NOTE_E5, NOTE_G5, NOTE_A5,
                         NOTE_G5, NOTE_E5, NOTE_C5 };
        int noteDurations[] = { 4, 4, 4, 4,
                                4, 4, 2 };

        // play melody
        for (int i = 0; i < sizeof(melody)/sizeof(melody[0]); i++) {
                int duration = 1000 / noteDurations[i];
                tone(BUZZER_PIN, melody[i], duration);
                delay(duration * 1.30);
                noTone(BUZZER_PIN);
        }
}

/* IMU Initiation */
void IMU_INIT() {
        /* Initialize the MPU6050 */
        mpu.initialize();
        mpu.dmpInitialize();

        /* Set gyroscope/accelerometer offsets. Derived from IMU_Zero */
        mpu.setXAccelOffset(-1685);
        mpu.setYAccelOffset(-2219);
        mpu.setZAccelOffset(1259);
        mpu.setXGyroOffset(61);
        mpu.setYGyroOffset(-70);
        mpu.setZGyroOffset(93);

        /* Enable DMP on the MPU6050 */
        mpu.setDMPEnabled(true);

        /* Get the packet size and FIFO count */
        packetSize = mpu.dmpGetFIFOPacketSize();
        fifoCount = mpu.getFIFOCount();

        /* Set the full-scale gyro range to 2000 degrees per second */
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
}



/* Main task for the BMP280 get Altitude Fuction*/
void getAltitude_main(xTask task_, xTaskParm param_) {

        /* Wait for a notification to be received */
        xTaskNotification notif = xTaskNotifyTake(task_);

        /* If a notification is received, proceed to read the altitude */
        if (notif) {

                /* Read the altitude from the BMP280 sensor using the readAltitude() function */
                float altitude = bmp.readAltitude(1025.06); /* Change to your local pressure in hPa*/

                /* Print the altitude value to the serial monitor */
                Serial.print("\tALT=\t"); /* Print a label for the altitude value */
                Serial.print(altitude); /* Print the altitude value */
                Serial.println("\tm"); /* Print the unit of meters */

                /* Free the heap memory allocated for the direct-to-task notification */
                xMemFree(notif);
        }
}

/* Define a function named getYPR_main that takes a task and a parameter as input */
void getYPR_main(xTask task_, xTaskParm param_) {
        /* Declare two static float variables to store the last pitch and yaw values */
        static float lastPitch;
        static float lastYaw;

        /* Check if there is enough data in the FIFO buffer to process */
        if (fifoCount < packetSize) {
                /* If there is not enough data, get the number of bytes available in the FIFO buffer */
                fifoCount = mpu.getFIFOCount();
        } else {
                /* If there is enough data in the FIFO buffer to process, check for FIFO overflow */
                if (fifoCount == 1024) {
                        /* If there is FIFO overflow, reset the FIFO buffer and print an error message */
                        mpu.resetFIFO();
                        Serial.println(F("FIFO overflow!"));
                } else {
                        /* If there is no FIFO overflow, check if the number of bytes in the FIFO buffer is a multiple of the packet size */
                        if (fifoCount % packetSize != 0) {
                                /* If the number of bytes in the FIFO buffer is not a multiple of the packet size, reset the FIFO buffer */
                                mpu.resetFIFO();
                        } else {
                                /* If there is enough data in the FIFO buffer and no overflow, process the data in the buffer */
                                while (fifoCount >= packetSize) {
                                        /* Get the next packet of data from the FIFO buffer */
                                        mpu.getFIFOBytes(fifoBuffer, packetSize);
                                        /* Subtract the number of bytes in the packet from the number of bytes in the FIFO buffer */
                                        fifoCount -= packetSize;
                                }

                                /* Get the quaternion, gravity, and yaw-pitch-roll values from the MPU sensor */
                                mpu.dmpGetQuaternion(&q, fifoBuffer);
                                mpu.dmpGetGravity(&gravity, &q);
                                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                                mpu.getAcceleration(&ax, &ay, &az);

                                /* Calculate the current pitch and yaw values from the yaw-pitch-roll array */
                                float curPitch = ypr[1] * 180 / PI + 90;
                                float curYaw = ypr[2] * 180 / PI + 90;

                                /* Print the current roll, pitch, and yaw values, as well as the angular rate of change in pitch and yaw over time */
                                Serial.print("Roll:\t");
                                Serial.print(ypr[0] * 180 / PI);
                                Serial.print("\tPitch:\t");
                                Serial.print(curPitch);
                                Serial.print("\tYaw:\t");
                                Serial.print(curYaw);
                                Serial.print("\tAccZ:\t");
                                Serial.print((az/4096.0)*9.81);
                                Serial.print("\tAngularRate_Pitch:\t");
                                Serial.print((curPitch - lastPitch) / 0.005);
                                Serial.print("\tAngularRate_Yaw:\t");
                                Serial.print((curYaw - lastYaw) / 0.005);

                                /* Store the current pitch and yaw values as the last pitch and yaw values for the next iteration */
                                lastPitch = curPitch;
                                lastYaw = curYaw;

                                /* Call the altitude function by notifying the ALT task */
                                xTask receiver = xTaskGetHandleByName("ALT");
                                if (receiver) {
                                        xTaskNotifyGive(receiver, 1, "W");
                                }
                        }
                }
        }
}



// void servoTaskX_main(xTask task_, xTaskParm parm_) {
//         float roll;
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
//         float pitch;
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
        /* Initialize serial communication */
        Serial.begin(9600);

        /* Initialize I2C bus */
        Wire.begin();

        /* Set I2C clock speed to 400kHz */
        TWBR = 24;

        /* Attach servos to pins 3 and 4 */
        X08_X.attach(3);
        X08_Y.attach(4);

        /* Set servos to neutral position */
        X08_X.write(90);
        X08_Y.write(90);
        BUZZER_INIT();
        // BUZZER_TEST();

        LED_INIT();
        LED_TEST();

        PYRO_INIT();
        /* Initialize system tasks */
        xSystemInit();
        /* Initialize IMU */
        IMU_INIT();
        /* Initialize barometer */
        BMP_INIT();
        /* Create YPR and altitude update tasks */
        xTask xTask_YPR_Update = xTaskCreate("YPR", getYPR_main, NULL);
        xTask xTask_Altitude_Update = xTaskCreate("ALT", getAltitude_main, NULL);

        /* Check if tasks were created successfully */
        if (xTask_YPR_Update && xTask_Altitude_Update) {
                /* Wait for tasks to complete initialization */
                xTaskWait(xTask_YPR_Update);
                xTaskWait(xTask_Altitude_Update);

                /* Set YPR update task period to 1ms */
                xTaskChangePeriod(xTask_YPR_Update, 1);
                float startTime = millis();
                
                while(!launch_Detected()){
                        if(millis()-startTime >= 5000){
                                Serial.println("inactive for 5 seconds, system halt");
                                delay(5);
                                xSystemHalt();
                        }
                        delay(.5);
                }

                /* Start the HeliOS scheduler */
                xTaskStartScheduler();

                /* Delete tasks when they complete execution */
                xTaskDelete(xTask_YPR_Update);
                xTaskDelete(xTask_Altitude_Update);
        }

        /* Halt the system */
        xSystemHalt();
}

void loop() {
        /* The loop function is not used and should remain empty. */
}