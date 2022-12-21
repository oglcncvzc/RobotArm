#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define buttonPin 13
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

//*******************************************************************************
//************************ VARIABLE DEFINITIONS **********************************
//*******************************************************************************


//***********  MPU variables *******************************

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t gx, gy, gz; //gyro tanımlama

uint8_t MotionX;
uint8_t MotionY;
uint8_t MotionZ;

uint8_t ButtonStatus;


uint32_t SensorMillis = 20;
uint32_t SensorMillisCounter;


//******************* SENSORS variables ***************************

uint32_t BtMillisCounter;
const int flexPin = A0; //pin A0 to read analog input
uint16_t Flexsensorvalue; //save analog value
uint16_t Flexsensormapped;
int16_t temp;
uint8_t SensorValueAverage;
uint8_t SensorAvgCnt;

//********************* BLUETOOTH variables ****************************

uint8_t SendArray[20];
uint8_t Sendarraylen = 12;



// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//***************************************************************************


void FLEXSENSOR_READ()
{
  Flexsensorvalue = analogRead(flexPin);         //Read and save analog value from potentiometer
  //Serial.print(Flexsensorvalue);      Serial.print("       ");          //Print value
  temp = map(Flexsensorvalue, 800, 950, 0, 180);//Map value 0-1023 to 0-255 (PWM)

  if (temp > 180)
  {
    temp = 180;
  }
  if (temp < 0 )
  {
    temp = 0;
  }

  if (SensorAvgCnt < 5)
  {
    SensorAvgCnt++;
    Flexsensormapped =  Flexsensormapped + temp;
  }
  else
  {
    SensorAvgCnt = 0;
    SensorValueAverage = Flexsensormapped / 5 ;
    Flexsensormapped = 0;
  }
  // Serial.println(SensorValueAverage);
}

uint8_t MpuOptimization(uint16_t Value)
{
  if (Value > 180 )
  {
    Value = 180;
  }
  if (Value < 0 )
  {
    Value = 0;
  }
  return Value;
}

void BTCom(byte len)
{
  SendArray[0] = 'T';
  SendArray[1] = 'E';
  SendArray[2] = 'X';
  SendArray[3] = MotionX;
  SendArray[4] = 'Y';
  SendArray[5] = MotionY;
  SendArray[6] = 'Z';
  SendArray[7] = MotionZ;
  SendArray[8] = 'F';
  SendArray[9] = SensorValueAverage;
  SendArray[10] = ButtonStatus;
  SendArray[11] = 'K';

  for (byte i = 0 ; i < len ; i++)
  {
    Serial.write(SendArray[i]);
  }
}

void buttonRead()
{
  if (digitalRead(buttonPin) == HIGH)
  {
    ButtonStatus = 0x01;
  }
  else
  {
    ButtonStatus = 0;
  }


}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  pinMode(buttonPin, INPUT);
  Serial.begin(115200); // BAUDRATE

  while (!Serial);

  // initialize device
  mpu.initialize();

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(88);
  mpu.setYGyroOffset(23);
  mpu.setZGyroOffset(-5);
  mpu.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
  }

  if ( millis() - SensorMillisCounter >= SensorMillis)
  {
    SensorMillisCounter = millis();
    buttonRead();
    FLEXSENSOR_READ();

    gx = map(ypr[0] * 180 / M_PI, -90, 90, 0, 180); // İvme sensöründen okunan değer 0-180 arasına indirgeniyor
    MotionX = MpuOptimization(gx);

    gy = map(ypr[1] * 180 / M_PI, -90, 90, 0, 180);
    MotionY = MpuOptimization(gy);

    gz = map(ypr[2] * 180 / M_PI, -90, 90, 0, 180);
    MotionZ = MpuOptimization(gz);
  }

  if ( millis() - BtMillisCounter >= 100)
  {
    BtMillisCounter = millis();
    BTCom(Sendarraylen);
  }
}
