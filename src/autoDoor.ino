///pin 2 hall 
///pin 3 remote
///pin 10 PWM
///pin 11 reverse///forward:close ; reverse:open
///pin A0 current sensor
///pin A3 button
//#define DEBUGGING
#include "motor.h"
#include<EEPROM.h>

const int ADDR_ms=30;
const int pinHall=2;
const int pinRemote=3;
const int pinPWM=10;
const int pinReverse=11;
const int pinCurrentSensor=A0;
const int pinButton=9;
const int pinLED=13;

Motor * motor;


void setup() {
  pinMode(pinHall,INPUT);
  pinMode(pinRemote,INPUT);
  pinMode(pinPWM,OUTPUT);
  pinMode(pinReverse,OUTPUT);
  pinMode(pinCurrentSensor,INPUT);
  pinMode(pinButton,INPUT);
  pinMode(pinLED,OUTPUT);

  analogReference(INTERNAL);
  analogRead(pinCurrentSensor);
  #ifdef DEBUGGING
  Serial.begin(9600);
  #endif
  
  MotorSettings ms;
  
  msDataRead(ms,ADDR_ms);
//  settingMode(ms);
  motor= new Motor(pinHall,pinPWM,pinReverse,pinCurrentSensor,ms);

digitalWrite(pinLED,LOW);
  if(digitalRead(pinButton)==false)
  {
    delay(30);
    digitalWrite(pinLED,HIGH);
    while(digitalRead(pinButton)==false);
    
    settingMode(ms);
    motor->updateSettings(ms);
  }
  

  attachInterrupt(digitalPinToInterrupt(pinHall),hallTrigering,RISING);
  attachInterrupt(digitalPinToInterrupt(pinRemote),remoteTrigering,RISING);
  motor->initiation();
  motor->updateTime();
}

void hallTrigering()
{
  
  motor->trigerHall();
  
}

void remoteTrigering()
{
  MotorStatus status=motor->getStatus();
  switch(status)
  {
    case MotorStatus::Closed:
      motor->open();
    break;
    case MotorStatus::Opening:
    case MotorStatus::Closing:
      motor->stop();
    break;
    case MotorStatus::Opened:
      motor->hold();
    break;
    case MotorStatus::Holding:
      motor->close();
    break;
    case MotorStatus::Error:
      motor->initiation();
    break;
  }
  if(status==MotorStatus::Closed)
  {
    motor->open();
  }else if(status==MotorStatus::Opening)
  {
    motor->stop();
  }else if(status==MotorStatus::Opened)
  {
    motor->hold();
  }else if(status==MotorStatus::Closing)
  {
    motor->stop();
  }else if(status==MotorStatus::Error)
  {
    
  }
}

void msDataUpdate(MotorSettings & ms,int ADDR_ms)
{
  const byte* p=(const byte*)(const void*)&ms;
  
  for(int i=0;i<sizeof(MotorSettings);i++)
  {
    EEPROM.update(ADDR_ms,*p);
    ADDR_ms++;
    p++;
  }
}
void msDataRead(MotorSettings & ms,int ADDR_ms)
{
  byte* p=(byte*)(void*)&ms;
  
  for(int i=0;i<sizeof(MotorSettings);i++)
  {
    *p=EEPROM.read(ADDR_ms);
    p++;
    ADDR_ms++;
  }
}



void loop() {
    // put your main code here, to run repeatedly:
  motor->step();

}
/*
int lowSpeedThreshold;      1
int lowSpeedCountThreshold; 2
int slowZone;               3
int varyingZone;            4
int openingSpeed;           5
int closingSpeed;           6
int lowestSpeed;            7
int openHoldTime;           8
float stoppingCurrent;      9
    */

void settingMode(MotorSettings & ms)
{
  Serial.begin(9600);
  
  delay(1000);
  while(true)
  {
    showSettings(ms);
    int No=0;
    int var=0;
    Serial.println("-1 to Exit");
    Serial.println("Enter No.:");
    while(true)
    {
      if(Serial.available()>0)
      {
        No=Serial.parseInt();
        int trash=Serial.parseInt();
       break; 
      }
    }
    Serial.println(No);
    Serial.println("Enter value:");
    while(true)
    {
      if(Serial.available()>0)
      {
        var=Serial.parseInt();
        int trash=Serial.parseInt();
       break; 
      }
    }
    Serial.println(var);
    switch(No)
    {
      case 1:
        ms.lowSpeedThreshold=var;
        break;
      case 2:
        ms.lowSpeedCountThreshold=var;
        break;
      case 3:
        ms.slowZone=var;
        break;
      case 4:
        ms.varyingZone=var;
        break;
      case 5:
        ms.openingSpeed=var;
        break;
      case 6:
        ms.closingSpeed=var;
        break;
      case 7:
        ms.lowestSpeed=var;
        break;
      case 8:
        ms.openHoldTime=var;
        break;
      case 9:
          ms.stoppingCurrent=var/10.0f;
        break;
    }
    if(No==-1)
    {
      Serial.println("Exiting...");
      break;
    }
  }
  
msDataUpdate(ms,ADDR_ms);
//msDataRead(ms,ADDR_ms);
}


void showSettings(MotorSettings & ms)
{
Serial.print("1.lowSpeedThreshold:");
Serial.print(ms.lowSpeedThreshold);
Serial.print("      ");
Serial.print("2.lowSpeedCountThreshold:");
Serial.println(ms.lowSpeedCountThreshold);

Serial.print("3.slowZone:");
Serial.print(ms.slowZone);
Serial.print("      ");
Serial.print("4.varyingZone:");
Serial.println(ms.varyingZone);

Serial.print("5.openingSpeed:");
Serial.print(ms.openingSpeed);
Serial.print("      ");
Serial.print("6.closingSpeed:");
Serial.println(ms.closingSpeed);

Serial.print("7.lowestSpeed:");
Serial.print(ms.lowestSpeed);
Serial.print("      ");
Serial.print("8.openHoldTime:");
Serial.println(ms.openHoldTime);


Serial.print("9.stoppingCurrent:");
Serial.println(ms.stoppingCurrent);
}
