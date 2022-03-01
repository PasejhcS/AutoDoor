
//#define DEBUGGING
#include "motor.h"
#include "Arduino.h"
Motor::Motor(int pHall,int pPWM,int pRev,int pCS,MotorSettings& ms)
{
  pinHall=pHall;
  pinPWM=pPWM;
  pinReverse=pRev;
  pinCurrentSensor=pCS;
  updateSettings(ms);
  
  pinMode(pinHall,INPUT);
  pinMode(pinPWM,OUTPUT);
  pinMode(pinReverse,OUTPUT);
  //pinMode(pinCurrentSensor,INPUT);

  hallCountLimit=2000;
  currentTemp=0;
  currentSampleTime=0;
  steppingTime=100;
  steppingFrame=millis();
  delayTimerFlag=false;
}

void Motor::updateTime()
{
  steppingFrame=millis();
}
void Motor::updateSettings(MotorSettings &ms)
{
  
  lowSpeedThreshold=ms.lowSpeedThreshold;
  lowSpeedCountThreshold=ms.lowSpeedCountThreshold;
  slowZone=ms.slowZone;
  varyingZone=ms.varyingZone;
  openingSpeed=ms.openingSpeed;
  closingSpeed=ms.closingSpeed;
  lowestSpeed=ms.lowestSpeed;
  openHoldTime=ms.openHoldTime;
  stoppingCurrent=ms.stoppingCurrent;
  
}
MotorStatus Motor::getStatus()
{
  return status;
}
Motor::~Motor()
{
}


void Motor::initiation()
{
  analogWrite(pinPWM,0);//stop
  digitalWrite(pinReverse,pinReverseOPEN);//open
  hallCount=0;
  lastHallCount=0;
  lowSpeedCount=0;
  analogWrite(pinPWM,lowestSpeed);//slowly open
  //hallTemp=0;
  status=MotorStatus::Init;
  initStatus=InitStatus::Opening;

}
void Motor::step()
{
  //TODO time step
  
  //currentTemp+=analogRead(pinCurrentSensor);
  //currentSampleTime++;
  if(millis()-steppingFrame>steppingTime)//per step
  {
    steppingFrame+=steppingTime;//=millis();
    speed=hallCount-lastHallCount;//cal speed of door
    lastHallCount=hallCount;//

    //float current=(float)currentTemp/currentSampleTime;
    //currentTemp=0;
    //currentSampleTime=0;
    #ifdef DEBUGGING
      #warning DEBUGGING_MODE
      int int_status=(int)(status);
      Serial.print(current);
      Serial.print("mA;");
      Serial.print(int_status);
      Serial.print(";");
      Serial.print(hallCount);
      Serial.print(",");
      Serial.print(speed);
      Serial.print(",");
      Serial.println(distance);
    #endif
    if(status==MotorStatus::Init)
    {
      switch(initStatus)
      {
        case InitStatus::Opening:
          if(speed<lowSpeedThreshold)//if speed is too low in some time;
            lowSpeedCount++;
          else
            lowSpeedCount=0;//if door could still moving,just let it go.
          if(lowSpeedCount>lowSpeedCountThreshold || hallCount>hallCountLimit)//stopped too long. Fully opened!
          {
            analogWrite(pinPWM,0);//stop
  
            hallCount=0;//reset and swith to Opened
            lastHallCount=0;
            lowSpeedCount=0;
            initStatus=InitStatus::Opened;
          } 
        break;
        case InitStatus::Opened:
          if(!delayTimerFlag)//delay few times;
          {
            delayTimerFlag=true;
            delayTimer=15;  
            
          }
          if(delayTimer>0)
          {delayTimer--;break;}
          delayTimerFlag=false;

          hallCount=0;//reset and swith to Opened
          lastHallCount=0;
          lowSpeedCount=0;
          digitalWrite(pinReverse,pinReverseCLOSE);//set and swith to closing
          analogWrite(pinPWM,lowestSpeed);
          initStatus=InitStatus::Closing;
        break;
        case InitStatus::Closing:
          if(speed<lowSpeedThreshold)//if speed is too low in some time;
            lowSpeedCount++;
          else
            lowSpeedCount=0;//if door could still moving,just let it go.
          if(lowSpeedCount>lowSpeedCountThreshold || hallCount>hallCountLimit)//stopped too long. Fully Opened!
          {
            analogWrite(pinPWM,0);//stop
            distance=hallCount;
            hallCount=0;//reset and swith to opened
            lastHallCount=0;
            lowSpeedCount=0;
            initStatus=InitStatus::Closed;
            status=MotorStatus::Closed;
          } 
          break;
          
      }

    }//end if(status init)
    else if(status==MotorStatus::Opening)
    {
      //          0 fromLeft      0 fromLeft                     <---
      //distance   slowZone       varyingZone                       0
      //.            .                  .                           .
      //.............................................................
      if(distance-hallCount>varyingZone)//fastMoving Region
      {
        if(speed<lowSpeedThreshold)//if speed is too low in some time;
          lowSpeedCount++;
        else
          lowSpeedCount=0;//if door could still moving,just let it go.
        if(lowSpeedCount>lowSpeedCountThreshold)//stopped too long. Error!
        {
          analogWrite(pinPWM,0);//stop
          
          //switch to Error
          status=MotorStatus::Error;
          lowSpeedCount=0;
          hallCount=0;//reset and swith to opened
          lastHallCount=0;
        }
      }else if(distance-hallCount>slowZone)//in speed varying zone
      {
        int motorSpeed=lowestSpeed+
           (openingSpeed-lowestSpeed)*((float)(distance-hallCount-slowZone))/((float)(varyingZone-slowZone));
        analogWrite(pinPWM,motorSpeed);

        if(speed<lowSpeedThreshold)//if speed is too low in some time;
          lowSpeedCount++;
        else
          lowSpeedCount=0;//if door could still moving,just let it go.
        if(lowSpeedCount>lowSpeedCountThreshold)//stopped too long. Error!
        {
          analogWrite(pinPWM,0);//stop
          
          //switch to Error
          status=MotorStatus::Error;
          lowSpeedCount=0;
          hallCount=0;//reset and swith to opened
          lastHallCount=0;
        }

        
      }else 
      {
        analogWrite(pinPWM,lowestSpeed);
        if(speed<lowSpeedThreshold)//if speed is too low in some time;
          lowSpeedCount++;
        else
          lowSpeedCount=0;//if door could still moving,just let it go.
        if(lowSpeedCount>lowSpeedCountThreshold || hallCount>distance*1.1f)//stopped. touch dowm
        {
          analogWrite(pinPWM,0);//stop
          
         hallCount=0;//reset and swith to opened
        lastHallCount=0;
        lowSpeedCount=0;
        status=MotorStatus::Opened;
        }
      }
    }//end if opening
    else if(status==MotorStatus::Opened)
    {
      //countdown timer
      if(!delayTimerFlag)
      {        
        delayTimerFlag=true;
      }else
      {
        delayTimer++;
        if(delayTimer>openHoldTime/steppingTime)
        {
          delayTimerFlag=false;
          delayTimer=0;
          status=MotorStatus::Closing;
          digitalWrite(pinReverse,pinReverseCLOSE);
          analogWrite(pinPWM,closingSpeed);
          hallCount=0;//reset and swith to opened
          lastHallCount=0;
          lowSpeedCount=0;
        }
      }
    }//end if opened
    else if(status==MotorStatus::Closing)
    {
      //          0 fromLeft      0 fromLeft                     <---
      //distance   slowZone       varyingZone                       0
      //.            .                  .                           .
      //.............................................................
      if(distance-hallCount>varyingZone)//fastMoving Region
      {
        if(speed<lowSpeedThreshold)//if speed is too low in some time;
          lowSpeedCount++;
        else
          lowSpeedCount=0;//if door could still moving,just let it go.
        if(lowSpeedCount>lowSpeedCountThreshold)//stopped too long. Error!
        {
          analogWrite(pinPWM,0);//stop
          
          //switch to Error
          status=MotorStatus::Error;
          lowSpeedCount=0;
          hallCount=0;//reset and swith to opened
          lastHallCount=0;
        }
      }else if(distance-hallCount>slowZone)//in speed varying zone
      {
        int motorSpeed=lowestSpeed+
           (closingSpeed-lowestSpeed)*((float)(distance-hallCount-slowZone))/((float)(varyingZone-slowZone));
        analogWrite(pinPWM,motorSpeed);
        
        if(speed<lowSpeedThreshold)//if speed is too low in some time;
          lowSpeedCount++;
        else
          lowSpeedCount=0;//if door could still moving,just let it go.
        if(lowSpeedCount>lowSpeedCountThreshold)//stopped too long. Error!
        {
          analogWrite(pinPWM,0);//stop
          
          //switch to Error
          status=MotorStatus::Error;
          lowSpeedCount=0;
          hallCount=0;//reset and swith to opened
          lastHallCount=0;
        }

        
      }else
      {
        analogWrite(pinPWM,lowestSpeed);
        if(speed<lowSpeedThreshold)//if speed is too low in some time;
          lowSpeedCount++;
        else
          lowSpeedCount=0;//if door could still moving,just let it go.
        if(lowSpeedCount>lowSpeedCountThreshold || hallCount>distance*1.1f)//stopped. touch dowm
        {
          analogWrite(pinPWM,0);//stop
          
          //switch to Error
         hallCount=0;//reset and switch to opened
        lastHallCount=0;
        lowSpeedCount=0;
        status=MotorStatus::Closed;
        }
      }
    }//endif closing
    else if(status==MotorStatus::Error)
    {
      analogWrite(pinPWM,0);
    }else if(status==MotorStatus::Holding)
    {
      
    }
  }
  
}
void Motor::hold()
{
  if(status==MotorStatus::Opened)
  {
    status=MotorStatus::Holding;
    delayTimerFlag=false;
    delayTimer=0;
  }
}
void Motor::stop()
{
  status=MotorStatus::Error;
  analogWrite(pinPWM,0);
  hallCount=0;
  lastHallCount=0;
  lowSpeedCount=0;
}

void Motor::open()
{
  if(status==MotorStatus::Closed)
  {
    status=MotorStatus::Opening;
    digitalWrite(pinReverse,pinReverseOPEN);
    analogWrite(pinPWM,openingSpeed);
    hallCount=0;
    lastHallCount=0;
    lowSpeedCount=0;    
  }
  
}

void Motor::close()
{
  if(status==MotorStatus::Opened || status==MotorStatus::Holding)
  {
          delayTimerFlag=false;
          delayTimer=0;
          status=MotorStatus::Closing;
          digitalWrite(pinReverse,pinReverseCLOSE);
          analogWrite(pinPWM,closingSpeed);
          hallCount=0;//reset and swith to opened
          lastHallCount=0;
          lowSpeedCount=0;
  }
}

void Motor::trigerHall()
{
  hallCount++;
}
  
