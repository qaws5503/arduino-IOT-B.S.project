#include "I2Cdev.h"
#include <SoftwareSerial.h>   // 引用程式庫
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

//#define OUTPUT_READABLE_EULER


#define OUTPUT_READABLE_YAWPITCHROLL
#define EMG





#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int checkEMG();
int current=0;
int count=0;
int s=0;
int muscle_state = 0;
int sensorValue = 0;
int num = 0;
int y = 0;
char cmd[1];
char give_data[10];
char give_state[1];
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(100, true);
    #endif

    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    BT.begin(9600);

    mpu.setRate(6);
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
   // while (!Serial.available());                 // wait for data
   // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}




// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }
    }

    Serial.println(F("input action and number ex:a15"));
    while(!BT.available()){}
    if(BT.available()){
      BT.readBytes(cmd,1);
      char action = cmd[0]; 
      num = BT.parseInt();
      y = BT.parseInt();
      Serial.println(num);
      Serial.println(action);

      mpu.resetFIFO();
      
      while(num > count){
      //Serial.println("section 1");
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      //Serial.println("section 1-1");
      delay(2);
      mpuIntStatus = mpu.getIntStatus();
      //Serial.println("section 1-2");
      // get current FIFO count
      delay(4);
      fifoCount = mpu.getFIFOCount();
      //Serial.println("section 1-3");
      
  
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            fifoCount = mpu.getFIFOCount();
            Serial.println(F("FIFO overflow!"));
    
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // wait for correct available data length, should be a VERY short wait
            //Serial.println("section 2");
            while (fifoCount < packetSize) {
              //Serial.println("section 2-1");
              delay(4);
              fifoCount = mpu.getFIFOCount();
              //Serial.println("section 2-2");
            }
            
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            
            sensorValue = analogRead(A0);
            muscle_state = checkEMG(sensorValue);
            //Serial.println(muscle_state);
            
            if(muscle_state == 0){
              //Serial.println("section 3");
              #ifdef OUTPUT_READABLE_YAWPITCHROLL
              switch(action){
                case 'a':
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                current=int(ypr[1] * 180/M_PI);
                Serial.println(current);
                
                while(current==-150){
                  s=1;
                  break;
                  }
                while(s==1 && current==150){
                  count=count+1;
                  Serial.println(count);
                  s=0;
                  break;
                }
                break;
                
                case 'b':
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                current=int(ypr[1] * 180/M_PI);
                //Serial.println(current);
                while(current==-135){
                  s=1;
                  break;
                  }
                while(s==1 && current==150){
                  count=count+1;
                  Serial.println(count);
                  s=0;
                  break;
                }
                break;
    
                case 'c':
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                current=int(ypr[1] * 180/M_PI);
                while(current==-158){
                  s=1;
                  break;
                  }
                while(s==1 && current==150){
                  count=count+1;
                  s=0;
                  break;
                }
                break;
    
                case 'd':
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                current=int(ypr[1] * 180/M_PI);
                Serial.println(current);
                while(current==-5){
                  s=1;
                  break;
                  }
                while(s==1 && current==-59){
                  count=count+1;
                  s=0;
                  break;
                }
                break;
              }
              #endif
            }
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            
            itoa(count, give_data, 10);
            
            if(muscle_state == 1){
              give_state[0] ={'Y'};
            }
            if(muscle_state == 0){
              give_state[0] ={'N'}; 
            }
            for(int i=0; i<1; i++) {
                BT.write(give_state[i]);
            }
            for(int i=0; i<2; i++) {
                BT.write(give_data[i]);
            }
            delay(2);
        }
      }
      reset();
    }
}

int checkEMG(int sensorValue){
          int state;
          // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
          float voltage = sensorValue * (5.0 / 1023.0);
          // print out the value you read:
          if(voltage > 2.5)
          {
            state = 1;
            char give_state[10] ={"Y"};
          }
          else{
            state = 0;
          }
          return state;
}

void reset(void){
    current=0;
    count=0;
    s=0;
    muscle_state = 0;
    cmd[0] = 0;
    num = 0;
}
