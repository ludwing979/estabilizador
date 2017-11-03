#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13
bool blinkState = true;
Servo servo_x, servo_y;
float angulo_x, angulo_y, angulo_z;
MPU6050 mpu;               

// MPU control/status vars
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
float euler[3];         

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define Y   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define X  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define Z   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  servo_x.attach(10);
  servo_y.attach(9);
  delay(50);
  servo_x.write(0);
  //servo_y.write(0);
  delay(500);
  servo_x.write(180);
  //servo_y.write(180);
  delay(500);
  servo_x.write(90);
  //servo_y.write(90);
  delay(500);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();


    // supply your own gyro offsets here, scaled for min sensitivity 
    mpu.setXGyroOffset(127); //220
    mpu.setYGyroOffset(-49); //76
    mpu.setZGyroOffset(19); //-85
    
    mpu.setXAccelOffset(1183); //1788
    mpu.setYAccelOffset(-950);
    mpu.setZAccelOffset(1361); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)"));
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }
  pinMode(LED_PIN, OUTPUT);

}



void loop(void){
  processAccelGyro();
}


void processAccelGyro()
{
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (mpuIntStatus & 0x02)  // otherwise continue processing
  {
    // check for correct available data length
    if (fifoCount < packetSize)
      return; //  fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    // flush buffer to prevent overflow
    mpu.resetFIFO();

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    
    angulo_x = euler[X] * 180 / M_PI + 90;
    angulo_y = euler[Y] * 180 / M_PI + 90;
    angulo_z = euler[Z] * 180 / M_PI + 90;
    
    Serial.println("");
    //Serial.print("\ty: "); Serial.print(angulo_y);
    //Serial.print("\tz: "); Serial.print(angulo_z);
    mpu.resetFIFO();

    
    if (angulo_x >= 0 && angulo_x <= 190){
      Serial.print("\tx: "); Serial.print(angulo_x);
      servo_x.write(angulo_x);
    }

    if (angulo_y >= 0 && angulo_y <= 190){
      Serial.print("\ty: "); Serial.print(angulo_y);
      servo_y.write(angulo_y);
    }
    
    mpu.resetFIFO();

    
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}


            
            
            




