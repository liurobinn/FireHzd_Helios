#include <Arduino.h>
#include <HeliOS.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>


//X IS THE OUTER RING
//Y IS THE INNER RING

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

/*Define servo names for the TVC*/
Servo X08_X;
Servo X08_Y;

SimpleKalmanFilter altFilter(20,20, 5);

/*Define buzzer with its pin number*/
#define BUZZER_PIN 10

#define OFFSETX 1
#define OFFSETY -3

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


#define OVERTIME_ERROR "Inactive for 5 seconds, system halt......"

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

class PID {
private:
        double lastTime = millis();
        double integral = 0;
        double lastErr = 90;

public:
        double p = 0.2;
        double i = 0;
        double d = 0.02;

        double UPDATE(double curErr, double rate) { //curErr should be in degrees/s
                double curTime = millis()/1000;
                double dx = curErr - lastErr;
                lastErr=curErr;
                integral += curErr*0.005;//Riemann sum

                return p*curErr + i*integral + d*rate;
        }
};

PID xPID;
PID yPID;

class FourBarLinkage{      
public:        
        double AO = 15; //Servo Arm Length CHANGE!!!
        double AB = 30; //Paper Clip length CHANGE!!!
        double BC = 39; //washer to mount length CHANGE!!!

        double HC=BC-AO;
        const double CO=sqrt(AB*AB+HC*HC);
        const double HCO= atan(AB/HC);

        double getAngleServo (double deviationBCB){
                double BCO = (deviationBCB/180)*M_PI+HCO;
                double BO = oppoLengthLawOfCosine(BC, CO, BCO);
                double BOA = oppoAngleLawOfCosine(BO, AO, AB);
                double AOA = atan(AB/AO)-BOA;
                return (AOA/M_PI)*180;
        }
};

FourBarLinkage FourBarTransformX;

FourBarLinkage FourBarTransformY;

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

void LAUNCH_IGNITION(){
        digitalWrite(PYRO_LR, HIGH);
        delay(50);
        digitalWrite(PYRO_LR, LOW);
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
                        Adafruit_BMP280::SAMPLING_X16,         /* Pressure oversampling */
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
                float altitude = bmp.readAltitude(1013.25);
                float KAL_altitude = altFilter.updateEstimate(altitude); /* Change to your local pressure in hPa*/

                /* Print the altitude value to the serial monitor */
                Serial.print("\tKAL_ALT=\t"); /* Print a label for the altitude value */
                Serial.print(KAL_altitude,1); /* Print the altitude value */
                Serial.print("\tm"); /* Print the unit of meters */

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
                                // Serial.print("Roll:\t");
                                // Serial.print(ypr[0] * 180 / PI);

                                // Serial.print("\tPitch:\t");
                                // Serial.print(curPitch);

                                // Serial.print("\tYaw:\t");
                                // Serial.print(curYaw);

                                // Serial.print("\tAccZ:\t");
                                // Serial.print((az/4096.0)*9.81);
                                double AngularRate_Pitch = (curPitch - lastPitch) / 0.005;
                                // Serial.print("\tAngularRate_Pitch:\t");
                                // Serial.print(AngularRate_Pitch);

                                double AngularRate_Yaw = (curYaw - lastYaw) / 0.005;
                                // Serial.print("\tAngularRate_Yaw:\t");
                                // Serial.print(AngularRate_Yaw);

                                double errorX = 90-curPitch;
                                double errorY = 90-curYaw;
                                // Serial.print("\tActX\t");
                                // Serial.print(FourBarTransform.getAngleServo(xPID.UPDATE(errorX,AngularRate_Pitch))+90);
                                double ServoXError = FourBarTransformX.getAngleServo(xPID.UPDATE(errorX,AngularRate_Pitch));
                                double ServoYError = FourBarTransformY.getAngleServo(yPID.UPDATE(errorY,AngularRate_Yaw));

                                if(abs(ServoXError)<=10) X08_X.write(ServoXError+90+OFFSETX);
                                if(abs(ServoYError)<=10) X08_Y.write(-1*ServoYError+90+OFFSETY);
                                // Serial.print("\ttActY\t");
                                // Serial.print(FourBarTransform.getAngleServo(yPID.UPDATE(errorY,AngularRate_Yaw))+90);

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

void TVC_TEST(){
        //rotation test
        for (int t=0; t>= 0 && t <=1440; t++) {
                roll=asin(0.03*cos((3.14/180)*t));
                pitch=asin(0.03*sin((3.14/180)*t));

                X08_X.write(FourBarTransformX.getAngleServo(roll * 180)+90+OFFSETX);
                X08_Y.write(FourBarTransformY.getAngleServo(pitch * 180)+90+OFFSETY);
                delay(5);
        }
}

void COUNTDOWN_ONE_MIN(){
                int i=1;
                int counter;
                for(counter=6; counter >1; counter--) {
                        for (i=1; i<=counter; i++) {
                                digitalWrite(10,HIGH);
                                digitalWrite(15,LOW);
                                delay(200);
                                digitalWrite(10,LOW);
                                digitalWrite(15,HIGH);
                                delay(200);
                        }
                        digitalWrite(10,LOW);
                        digitalWrite(15,HIGH);
                        delay(10000);
                }
                int finalCount;
                for (finalCount=0; finalCount <= 10; finalCount++){
                digitalWrite(10,HIGH);
                digitalWrite(15,LOW);
                delay(500);
                digitalWrite(10,LOW);
                digitalWrite(15,HIGH);
                delay(500);
              }
}

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
        X08_X.write(90+OFFSETX);
        X08_Y.write(90+OFFSETY);

        // delay(1000000000);
        FourBarTransformX.AO = 15;
        FourBarTransformX.AB = 30;
        FourBarTransformX.BC = 39;

        FourBarTransformY.AO = 15;
        FourBarTransformY.AB = 27;
        FourBarTransformY.BC = 33;

        TVC_TEST();
        
        X08_X.write(90+OFFSETX);
        X08_Y.write(90+OFFSETY);

        BUZZER_INIT();
        BUZZER_TEST();

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

                COUNTDOWN_ONE_MIN();
                // LAUNCH_IGNITION(); //Launch Ignition

                /* Set YPR update task period to 1ms */
                xTaskChangePeriod(xTask_YPR_Update, 1);

                float startTime = millis(); /*Set starting time to detect system inactive*/
                
                while(!launch_Detected()){
                        if(millis()-startTime >= 5000){
                                Serial.println(OVERTIME_ERROR);
                                digitalWrite(10,HIGH);
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