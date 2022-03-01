#ifndef _MOTOR_H__
#define _MOTOR_H__
#include "Arduino.h"


enum class MotorStatus
{
  Init=0,
  Opening,
  Closing,
  Stop,
  Opened,
  Holding,
  Closed,
  Error
  
};
enum class InitStatus
{
  Opening=0,
  Opened,
  Closing,
  Closed
};

class MotorSettings
{
  private:

  public:
    int lowSpeedThreshold;
    int lowSpeedCountThreshold;
    int slowZone;
    int varyingZone;
    int openingSpeed;
    int closingSpeed;
    int lowestSpeed;
    int openHoldTime;
    float stoppingCurrent;
};

class Motor
{
  private:
    int pinReverse;
    int pinPWM;
    int pinHall;
    int pinCurrentSensor;
    int lowSpeedThreshold;
    int lowSpeedCountThreshold;
    int slowZone;
    int varyingZone;
    int openingSpeed;
    int closingSpeed;
    int lowestSpeed;
    int openHoldTime;
    float stoppingCurrent;
    volatile  int hallCount;
    volatile  int speed;
    volatile  int lastHallCount;
    const int pinReverseOPEN=LOW;
    const int pinReverseCLOSE=HIGH;
    //int hallTemp;
    
    int lowSpeedCount;
    int distance;
    int hallCountLimit;
    MotorStatus status;
    InitStatus initStatus;

    //int highestSpeed;
  
    unsigned long steppingTime;
    unsigned long steppingFrame;

    unsigned int currentSampleTime;
    unsigned long currentTemp;
    
    int delayTimer;
    bool delayTimerFlag;
  public:
   Motor(int pHall,int pPWM,int pRev,int pCS,MotorSettings& ms);
    ~Motor();
  void updateSettings(MotorSettings& ms);
  void initiation();
  void stop();
  void open();
  void close();
  void hold();
  void setLowestSpeed(int s);
  void setHighestSpeed(int s);
  void setStoppingCurrent(float c);
  void setSteppingTime(unsigned long t);
  void trigerHall();
  void updateTime();
  MotorStatus getStatus();
  void step();
};


#endif
