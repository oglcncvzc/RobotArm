
#include<Servo.h> // Servo motor kütüphanesi

#define Motor1Pin1  47
#define Motor1Pin2  46
#define Motor2Pin1  44
#define Motor2Pin2  42


Servo motorTip;
Servo motorX;
Servo motorY;
Servo motorZ;



typedef enum {
  COMM_STATE_COMM_STATE_T = 1,
  COMM_STATE_COMM_STATE_E,
  COMM_STATE_COMM_STATE_X,
  COMM_STATE_COMM_STATE_DATAX,
  COMM_STATE_COMM_STATE_Y,
  COMM_STATE_COMM_STATE_DATAY,
  COMM_STATE_COMM_STATE_Z,
  COMM_STATE_COMM_STATE_DATAZ,
  COMM_STATE_COMM_STATE_F,
  COMM_STATE_COMM_STATE_DATAF,
  COMM_STATE_COMM_STATE_BUTTON,
  COMM_STATE_COMM_STATE_K,
} COMM_STATE;

COMM_STATE CommState = COMM_STATE_COMM_STATE_T;

uint8_t DataReady = 0;

uint8_t DCmotorFlag;
uint8_t DCmotorFB;
uint8_t DCmotorRL;

uint32_t motorMillis = 20;
uint32_t motorMillisCounter;

uint8_t XpreValue;
uint8_t XTempValue;

uint8_t YpreValue;
uint8_t YTempValue;

uint8_t ZpreValue;
uint8_t ZTempValue;

uint8_t FpreValue;
uint8_t FTempValue;

uint8_t XValueHIGH;
uint8_t YValueHIGH;
uint8_t ZValueHIGH;
uint8_t FValueHIGH;

uint8_t XmotorValueSet;
uint8_t YmotorValueSet;
uint8_t ZmotorValueSet;
uint8_t FmotorValueSet;
uint32_t BtMillisCounter;

uint8_t MotorValueX;
uint8_t MotorValueY;
uint8_t MotorValueZ;
uint8_t flexSensorValue;

uint8_t RecievedPacketBuffer[12];


uint8_t RecievedArrayToPack(uint8_t NewByte)
{
  if (CommState == COMM_STATE_COMM_STATE_T)
  {
    if ('T' == NewByte)
    {
      RecievedPacketBuffer[0] = NewByte;
      CommState = COMM_STATE_COMM_STATE_E;
    }
  }
  else if (CommState == COMM_STATE_COMM_STATE_E)
  {
    if ('E' == NewByte)
    {
      //Serial.println("E");
      RecievedPacketBuffer[1] = NewByte;
      CommState = COMM_STATE_COMM_STATE_X;
    }
    else
    {
      //Serial.println("e");
      CommState = COMM_STATE_COMM_STATE_T;
    }
  }
  else if (CommState == COMM_STATE_COMM_STATE_X)
  {
    if ('X' == NewByte)
    {
      DataReady = 0;
      //Serial.println("E");
      RecievedPacketBuffer[2] = NewByte;
      CommState = COMM_STATE_COMM_STATE_DATAX;
    }
    else
    {
      //Serial.println("e");
      CommState = COMM_STATE_COMM_STATE_T;
    }
  }
  else if (CommState == COMM_STATE_COMM_STATE_DATAX)
  {
    RecievedPacketBuffer[3] = NewByte;
    CommState = COMM_STATE_COMM_STATE_Y;
  }
  else if (CommState == COMM_STATE_COMM_STATE_Y)
  {
    if ('Y' == NewByte)
    {
      RecievedPacketBuffer[4] = NewByte;
      CommState = COMM_STATE_COMM_STATE_DATAY;
    }
    else
    {
      //Serial.println("e");
      CommState = COMM_STATE_COMM_STATE_T;
    }
  }

  else if (CommState == COMM_STATE_COMM_STATE_DATAY)
  {
    RecievedPacketBuffer[5] = NewByte;
    CommState = COMM_STATE_COMM_STATE_Z;
  }

  else if (CommState == COMM_STATE_COMM_STATE_Z)
  {
    if ('Z' == NewByte)
    {
      RecievedPacketBuffer[6] = NewByte;
      CommState = COMM_STATE_COMM_STATE_DATAZ;
    }
    else
    {
      //Serial.println("e");
      CommState = COMM_STATE_COMM_STATE_T;
    }
  }
  else if (CommState == COMM_STATE_COMM_STATE_DATAZ)
  {
    RecievedPacketBuffer[7] = NewByte;
    CommState = COMM_STATE_COMM_STATE_F;
  }
  else if (CommState == COMM_STATE_COMM_STATE_F)
  {
    if ('F' == NewByte)
    {
      RecievedPacketBuffer[8] = NewByte;
      CommState = COMM_STATE_COMM_STATE_DATAF;
    }
    else
    {
      //Serial.println("e");
      CommState = COMM_STATE_COMM_STATE_T;
    }
  }
  else if (CommState == COMM_STATE_COMM_STATE_DATAF)
  {
    //Serial.println("data");
    RecievedPacketBuffer[9] = NewByte;
    CommState = COMM_STATE_COMM_STATE_BUTTON;
  }
  else if (CommState == COMM_STATE_COMM_STATE_BUTTON)
  {
    //Serial.println("data");
    RecievedPacketBuffer[10] = NewByte;
    CommState = COMM_STATE_COMM_STATE_K;
  }
  else if (CommState == COMM_STATE_COMM_STATE_K)
  {
    //Serial.println("K");
    if ('K' == NewByte)
    {
      RecievedPacketBuffer[11] = NewByte;
      DataReady = 1;
      //Serial.println("K");
      CommState = COMM_STATE_COMM_STATE_T;
    }
    else
    {
      CommState = COMM_STATE_COMM_STATE_T;
    }
  }
}


void MotorXValueSet(uint8_t Value )
{
  if (XmotorValueSet == 1)
  {

    if (Value - XpreValue >= 10 )
    {
      XValueHIGH = 1;
      XmotorValueSet = 0;
      XTempValue = Value;
    }
    if (XpreValue - Value >= 10 )
    {
      XValueHIGH = 0;
      XmotorValueSet = 0;
      XTempValue = Value;
    }
  }
  if (XmotorValueSet == 0)
  {
    if (XValueHIGH == 1)
    {
      motorX.write(XpreValue++);
      if (XTempValue == XpreValue)
      {
        XpreValue = XTempValue;
        XmotorValueSet = 1;
      }
    }
    if (XValueHIGH == 0)
    {
      motorX.write(XpreValue--);
      if (XTempValue == XpreValue)
      {
        XpreValue = XTempValue;
        XmotorValueSet = 1;
      }
      //Serial.print(Value); Serial.print("        ");
    }
  }
}

void MotorYValueSet(uint8_t Value )
{
  if (YmotorValueSet == 1)
  {

    if (Value - YpreValue >= 10 )
    {
      YValueHIGH = 1;
      YmotorValueSet = 0;
      YTempValue = Value;
    }
    if (YpreValue - Value >= 10 )
    {
      YValueHIGH = 0;
      YmotorValueSet = 0;
      YTempValue = Value;
    }
  }
  if (YmotorValueSet == 0)
  {
    if (YValueHIGH == 1)
    {
      motorY.write(YpreValue++);
      if (YTempValue == YpreValue)
      {
        YpreValue = YTempValue;
        YmotorValueSet = 1;
      }
    }
    if (YValueHIGH == 0)
    {
      motorY.write(YpreValue--);
      if (YTempValue == YpreValue)
      {
        YpreValue = YTempValue;
        YmotorValueSet = 1;
      }
      //Serial.print(Value); Serial.print("        ");
    }
  }
}

void MotorZValueSet(uint8_t Value )
{
  if (ZmotorValueSet == 1)
  {

    if (Value - ZpreValue >= 10 )
    {
      ZValueHIGH = 1;
      ZmotorValueSet = 0;
      ZTempValue = Value;
    }
    if (ZpreValue - Value >= 10 )
    {
      ZValueHIGH = 0;
      ZmotorValueSet = 0;
      ZTempValue = Value;
    }
  }
  if (ZmotorValueSet == 0)
  {
    if (ZValueHIGH == 1)
    {
      motorZ.write(ZpreValue++);
      if (ZTempValue == ZpreValue)
      {
        ZpreValue = ZTempValue;
        ZmotorValueSet = 1;
      }
    }
    if (ZValueHIGH == 0)
    {
      motorZ.write(ZpreValue--);
      if (ZTempValue == ZpreValue)
      {
        ZpreValue = ZTempValue;
        ZmotorValueSet = 1;
      }
      //Serial.print(Value); Serial.print("        ");
    }
  }
}
void MotorTipValueSet(uint8_t Value )
{
  if (FmotorValueSet == 1)
  {

    if (Value - FpreValue >= 10 )
    {
      FValueHIGH = 1;
      FmotorValueSet = 0;
      FTempValue = Value;
    }
    if (FpreValue - Value >= 10 )
    {
      FValueHIGH = 0;
      FmotorValueSet = 0;
      FTempValue = Value;
    }
  }
  if (FmotorValueSet == 0)
  {
    if (FValueHIGH == 1)
    {
      motorTip.write(FpreValue++);
      if (FTempValue == FpreValue)
      {
        FpreValue = FTempValue;
        FmotorValueSet = 1;
      }
    }
    if (FValueHIGH == 0)
    {
      motorTip.write(FpreValue--);
      if (FTempValue == FpreValue)
      {
        FpreValue = FTempValue;
        FmotorValueSet = 1;
      }
      //Serial.print(Value); Serial.print("        ");
    }
  }
}

void DCmotor(uint8_t FB , uint8_t RL)
{
  if ( FB >= 130)
  {
   digitalWrite(Motor1Pin1, HIGH);
    digitalWrite(Motor2Pin1, HIGH);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin2, LOW);
  }
  if ( FB <= 30)
  {    
     digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor1Pin2, HIGH);
    digitalWrite(Motor2Pin2, HIGH);
  }
  if( FB <= 130 &&  FB >= 30)
  {
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin2, LOW);
  }
  if ( RL >= 130)
  {
    digitalWrite(Motor1Pin1, HIGH);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin2, HIGH);
  }
  if ( RL <= 30)
  {
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor2Pin1, HIGH);
    digitalWrite(Motor1Pin2, HIGH);
    digitalWrite(Motor2Pin2, LOW);
  }
  if ( FB >= 130 && RL >= 130 || FB >= 130 && RL <= 30 || FB <= 30 && RL >= 130 || FB <= 30 && RL <= 30)
  {
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin2, LOW);
  }
}
void setup() {

  motorTip.attach(30); // servo motorun bağlandığı arduino pini
  motorZ.attach(31);
  motorY.attach(29);
  motorX.attach(27);

  pinMode(47, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(42, OUTPUT);



  Serial1.begin(115200);
  Serial.begin(115200);

  digitalWrite(47, LOW);
  digitalWrite(46, LOW);
  digitalWrite(44, LOW);
  digitalWrite(42, LOW);

  MotorXValueSet(90);
  MotorYValueSet(90);
  MotorZValueSet(90);
  MotorTipValueSet(90);

}

void loop() {
 if (Serial1.available())
  {
    RecievedArrayToPack(Serial1.read());
  }
  if (DataReady == 1)
  {
    DataReady = 0;
    if (RecievedPacketBuffer[10] == 0x01)
    {
      DCmotorFlag = 1;
      DCmotorFB = RecievedPacketBuffer[7];
      DCmotorRL = RecievedPacketBuffer[5];
    }
    else
    {
      DCmotorFlag = 0;
      MotorValueX = RecievedPacketBuffer[3];
      MotorValueY = RecievedPacketBuffer[5];
      MotorValueZ = RecievedPacketBuffer[7];
      flexSensorValue = RecievedPacketBuffer[9];
    }

    Serial.print(MotorValueX); Serial.print("   "); Serial.print(MotorValueY); Serial.print("     ");   Serial.print(MotorValueZ); Serial.print("    ");
    Serial.print(flexSensorValue); Serial.print("    ");   Serial.println(" ");
  }


 // if ( millis() - motorMillisCounter >= 10)
 // {
   // motorMillisCounter = millis();
    if (DCmotorFlag == 0)
    {
      MotorXValueSet(MotorValueX);
      MotorYValueSet(MotorValueY);
      MotorZValueSet(MotorValueZ);
      MotorTipValueSet(flexSensorValue);
    }
    else if ( DCmotorFlag == 1)
    {
      MotorXValueSet(90);
      MotorYValueSet(90);
      MotorZValueSet(90);
      MotorTipValueSet(90);
      
      DCmotor(DCmotorFB , DCmotorRL);

    }


    //                     Serial.print(RecievedPacketBuffer[0]);Serial.print("     "); Serial.print(RecievedPacketBuffer[1]);Serial.print("       ");          Serial.print(RecievedPacketBuffer[2]);Serial.print("     ");
    //                     //MotorValueSet(RecievedPacketBuffer[2]);
    //                     Serial.print(RecievedPacketBuffer[3]);
    //                    Serial.println(" ");
  //}



  //****************************************************

  //  if ( millis() - BtMillisCounter >= 50)
  //  {
  //    BtMillisCounter = millis();
  //byte bufferSize;
  //    bufferSize = Serial1.available();
  //
  //
  //    if (bufferSize > 10)
  //    {
  //      if (bufferSize == 11)
  //      {
  //        for (byte i = 0 ; i < 11; i++)
  //        {
  //          RecievedPacketBuffer[i] = Serial1.read();
  //        }
  //
  //        if (RecievedPacketBuffer[0] == 'T' && RecievedPacketBuffer[1] == 'E' && RecievedPacketBuffer[10] == 'K')
  //        {
  //           MotorValueX =RecievedPacketBuffer[3];
  //           MotorValueY =RecievedPacketBuffer[5];
  //           MotorValueZ =RecievedPacketBuffer[7];
  //           flexSensorValue = RecievedPacketBuffer[9];
  //           Serial.print(MotorValueX);Serial.print("   "); Serial.print(MotorValueY);Serial.print("     ");   Serial.print(MotorValueZ);Serial.print("    ");
  //           Serial.print(flexSensorValue);Serial.print("    ");   Serial.println(" ");
  //        }
  //
  //        Serial1.flush();
  //      }
  //
  //    }
  //
  //  }




}
