/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\11611                                            */
/*    Created:      Sat Sep 18 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// frontR               motor         10              
// frontL               motor         1               
// backR                motor         9               
// backL                motor         2               
// gyros                inertial      6               
// lockF                digital_out   A               
// convL                motor         7               
// lockB                digital_out   B               
// liftL                motor         3               
// liftR                motor         4               
// lockC                digital_out   C               
// convR                motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;
competition Competition;

#define HOLD 3
#define BRAKE 2
#define COAST 1

#define ACC_ONLY 1
#define DEC_ONLY 2
#define ACC_NOR_DEC 3
#define ACC_AND_DEC 4

brakeType driverStop = brake;
double ACCPART = 0.1; //distance for which the robot is accelerating
double DECPART = 0.1; //see above
double ACCTURNPART = 0.1; //same thing but for turns
double DECTURNPART = 0.1; 
double GyroTurnStandard = 0.65;
double gyroStart = 0;
double c = 0;

int status = 0;
int status2 = 0;
bool lockStatus1 = false;
bool lockStatus2 = false;

brain timer871; //general timer
brain timeX; //secondary timer
brain cTime; //calibration timer
brain timeSave; //timer to prevent runAutoEncode and other functions from being stuck
brain brakeTime; //brake timer
brain autoTimer;
int selfInspect = 0;

void spin(motor m, int v){
  m.spin(directionType::fwd, v, velocityUnits::pct);
}

void wait(int time){
  vex::task::sleep(time);
}

double sign(double x){
  if(x > 0) return 1;
  if(x < 0) return -1;
  return 0;
}

void run(int spd){
  spin(frontR, spd);
  spin(frontL, spd);
  spin(backL, -spd);
  spin(backR, -spd);
}

void runAuto(int spd, int time){
  run(spd);
  wait(time);
}

void runStop(brakeType bt){
  frontL.stop(bt);
  frontR.stop(bt);
  backL.stop(bt);
  backR.stop(bt);
}

void frontLock(){
  if(Controller1.ButtonX.pressing() && status == 0){
    status = 1;
    if(!lockStatus1){
      lockStatus1 = true;
    }
    else if(lockStatus1){
      lockStatus1 = false;
    }
  }
  else if(!Controller1.ButtonX.pressing() && status == 1){
    status = 0;
  }

  if (lockStatus1){
    lockF.set(true);
  }
  else if(!lockStatus1){
    lockF.set(false);
  }
}

void backLock(){
  if(Controller1.ButtonB.pressing() && status2 == 0){
    status2 = 1;
    if(!lockStatus2){
      lockStatus2 = true;
    }
    else if(lockStatus2){
      lockStatus2 = false;
    }
  }
  else if(!Controller1.ButtonB.pressing() && status2 == 1){
    status2 = 0;
  }

  if (lockStatus2){
    if(!lockB.value()){
    lockB.set(true);
    timer871.resetTimer();
    }
    if(timer871.timer(msec) > 200){
      lockC.set(true);
    }
    
  }
  else if(!lockStatus2){
    if(lockB.value()){
    lockB.set(false);
    timer871.resetTimer();
    }
    if(timer871.timer(msec) > 200){
      lockC.set(false);
    }
  }
}

void liftFront(int power){
  if(power != 0){
    spin(liftR, -power);
    spin(liftL, -power);
  }
  else{
    liftR.stop(hold);
    liftL.stop(hold);
  }
}

void conv(int power){
  if(power != 0){
    spin(convR, -power);
    spin(convL, -power);
  }
  else{
    convR.stop(hold);
    convL.stop(hold);
  }
}

void convAuto(int power, int time){
  conv(power);
  wait(time);
  conv(0);
}

void liftFAuto(int pwr, int time){
  liftFront(pwr);
  wait(time);
  liftFront(0);
}

void runSeparate(double left, double right){
  if(left != 1){
    spin(backL, -left);
    spin(frontL, left);
  }
  else{
    frontL.stop(hold);
    backL.stop(hold);
  }
  if(right != 1){
    spin(backR, -right);
    spin(frontR, right);
  }
  else{
    frontR.stop(hold);
    backR.stop(hold);
  }
}

double gyroValue(){
  return gyros.heading(degrees);
}

void gyroReset(){
  gyroStart = c*360 + gyros.heading(deg);
}

void turn(int spd){
  runSeparate(spd, -spd);
}

void gyroRecalibrate(int time){
  cTime.resetTimer();
  gyros.calibrate();
  while((gyros.isCalibrating()) && (cTime.timer(vex::timeUnits::msec) < time)){  
    runStop(hold);
    gyros.calibrate();
    if(std::abs(Controller1.Axis3.value())>50||std::abs(Controller1.Axis4.value())>50||std::abs(Controller1.Axis1.value())>50
    ||std::abs(Controller1.Axis2.value())>50
    ||Controller1.ButtonL1.pressing()||Controller1.ButtonL2.pressing()||Controller1.ButtonR1.pressing()
    ||Controller1.ButtonR2.pressing()||Controller1.ButtonUp.pressing()||Controller1.ButtonDown.pressing()
    ||Controller1.ButtonLeft.pressing()||Controller1.ButtonRight.pressing()||Controller1.ButtonB.pressing()||Controller1.ButtonX.pressing()){
      selfInspect=2;
      break;
    }
  }

  while(cTime.timer(vex::timeUnits::msec) < time){
    if(std::abs(Controller1.Axis3.value())>50||std::abs(Controller1.Axis4.value())>50||std::abs(Controller1.Axis1.value())>50
    ||std::abs(Controller1.Axis2.value())>50
    ||Controller1.ButtonL1.pressing()||Controller1.ButtonL2.pressing()||Controller1.ButtonR1.pressing()
    ||Controller1.ButtonR2.pressing()||Controller1.ButtonUp.pressing()||Controller1.ButtonDown.pressing()
    ||Controller1.ButtonLeft.pressing()||Controller1.ButtonRight.pressing()||Controller1.ButtonB.pressing()||Controller1.ButtonX.pressing()){
      selfInspect=2;
      break;
    }
    runStop(hold);
  }
       
  if(selfInspect!=2){
    selfInspect=1;
  }
}

void runAutoEncode(double pwr, double dist, int statement, int stopPwr, int stopTime, int stopStatement, int timeSafe){
  timeSave.resetTimer();
  backL.resetRotation();
  double accdist = dist*ACCPART;
  double decdist = dist*DECPART;
  double accslope, accintercept, accrate;
  double decslope, decintercept, decrate;

  if(statement == ACC_ONLY){
    decdist = 0;
  }
  else if(statement == DEC_ONLY){
    accdist = 0;
  }
  else if(statement == ACC_NOR_DEC){
    accdist = 0;
    decdist = 0;
  }
  else{
    statement = ACC_AND_DEC;
  }

  while(backL.rotation(rotationUnits::deg) < accdist && accdist != 0 && timeSave.timer(vex::timeUnits::msec) < timeSafe){
    double d;

    accslope = 0.8/accdist;
    accintercept = 0.05;
    accrate = accslope*backL.rotation(rotationUnits::deg) + accintercept;

    if(std::abs(pwr)*accrate < 10){
      d = 10;
    }
    else{
      d = std::abs(pwr)*accrate;
    }

    run(d*sign(pwr));
  }

  while(std::abs(backL.rotation(rotationUnits::deg)) < dist - decdist && timeSave.timer(timeUnits::msec) < timeSafe){
    run(pwr);
  }

  while(backL.rotation(rotationUnits::deg) < dist && decdist != 0 && timeSave.timer(timeUnits::msec) < timeSafe){
    double v;

    decslope = (0.05 - 1)/decdist;
    decintercept = 0.05 - decslope*dist;
    decrate = decslope*backL.rotation(rotationUnits::deg) + decintercept;

    if(std::abs(pwr)*decrate < 10){
      v = 10;
    }
    else{
      v = std::abs(pwr)*decrate;
    }
    
    run(v*sign(pwr));
  }

  brakeTime.resetTimer();
  while(brakeTime.timer(vex::timeUnits::msec) < stopTime && timeSave.timer(vex::timeUnits::msec) < timeSafe){
    run(stopPwr);
  }

  if(stopStatement == HOLD){
    runStop(hold);
  }

  if(stopStatement == BRAKE){
    runStop(brake);
  }

  if(stopStatement == COAST){
    runStop(coast);
  }
}

void gyroTurn(double degamount, int spd, int stopPwr, int stopTime, int stopStatement, int timeSafe){
  timeSave.resetTimer();
  double deg = degamount*GyroTurnStandard;

  if(spd < 0){
    gyros.setHeading(357, rotationUnits::deg);
    if(deg < 360){
      while(deg > 357 - gyroValue() && timeSave.timer(msec) < timeSafe){
        turn(spd);
      }
    }
  }
  
  else if(spd > 0){
    gyros.setHeading(3, degrees);
    if(deg < 360){
      while(gyros.heading(degrees) < deg + 3 && timeSave.timer(msec) < timeSafe){
        turn(spd);
      }
    }
  }

  turn(stopPwr);
  wait(stopTime);

  if(stopStatement == HOLD){
    runStop(hold);
  }
  if(stopStatement == BRAKE){
    runStop(brake);
  }
  if(stopStatement == COAST){
    runStop(coast);
  }
}

void runLift(double pwr, double fpwr, double bpwr, double dist, int statement, int stopPwr, int stopTime, int stopStatement, int timeSafe){
  timeSave.resetTimer();
  backL.resetRotation();
  double accdist = dist*ACCPART;
  double decdist = dist*DECPART;
  double accslope, accintercept, accrate;
  double decslope, decintercept, decrate;

  if(statement == ACC_ONLY){
    decdist = 0;
  }
  else if(statement == DEC_ONLY){
    accdist = 0;
  }
  else if(statement == ACC_NOR_DEC){
    accdist = 0;
    decdist = 0;
  }
  else{
    statement = ACC_AND_DEC;
  }

  while(backL.rotation(rotationUnits::deg) < accdist && accdist != 0 && timeSave.timer(vex::timeUnits::msec) < timeSafe){
    double d;

    accslope = 0.8/accdist;
    accintercept = 0.05;
    accrate = accslope*backL.rotation(rotationUnits::deg) + accintercept;

    if(std::abs(pwr)*accrate < 10){
      d = 10;
    }
    else{
      d = std::abs(pwr)*accrate;
    }

    run(d*sign(pwr));
  }

  while(std::abs(backL.rotation(rotationUnits::deg)) < dist - decdist && timeSave.timer(timeUnits::msec) < timeSafe){
    run(pwr);
    liftFront(fpwr);
  }

  while(backL.rotation(rotationUnits::deg) < dist && decdist != 0 && timeSave.timer(timeUnits::msec) < timeSafe){
    double v;

    decslope = (0.05 - 1)/decdist;
    decintercept = 0.05 - decslope*dist;
    decrate = decslope*backL.rotation(rotationUnits::deg) + decintercept;

    if(std::abs(pwr)*decrate < 10){
      v = 10;
    }
    else{
      v = std::abs(pwr)*decrate;
    }
    
    run(v*sign(pwr));
  }

  brakeTime.resetTimer();
  while(brakeTime.timer(vex::timeUnits::msec) < stopTime && timeSave.timer(vex::timeUnits::msec) < timeSafe){
    run(stopPwr);
  }

  if(stopStatement == HOLD){
    runStop(hold);
  }

  if(stopStatement == BRAKE){
    runStop(brake);
  }

  if(stopStatement == COAST){
    runStop(coast);
  }
}

void gyroRun(double spd, int dist, int runState, double target, double stopPwr, int stopTime, int stopState, int timeSafe){
  if(runState == 5){
    run(spd);
  }

  int accdist = dist*ACCPART;
  int decdist = dist*DECPART;
  double value;
  double accslope, accintercept, accrate;
  double decslope, decintercept, decrate;
  double endr;

  if(runState == ACC_NOR_DEC){
    decdist = 0;
    accdist = 0;
  }
  else if(runState == ACC_ONLY){
    decdist = 0;
  }
  else if(runState == DEC_ONLY){
    accdist = 0;
  }

  timeSave.resetTimer();
  double rightslope = 0, leftslope = 0;
  double rightfunc, leftfunc;
  backL.resetRotation();
  double gyrost = gyroValue() + target;

  while(backL.rotation(degrees) < dist && spd > 0 && timeSave.timer(msec) < timeSafe && runState != 5){
    if(backL.rotation(degrees) < accdist){
      accslope = 0.8/accdist;
      accintercept = 0.05;
      accrate = accslope*backL.rotation(degrees) + accintercept;
      if(spd*accrate < 10 && spd != 0){
        value = 10/spd;
      }
      else{
        value = accrate;
      }
    }

    if(backL.rotation(degrees) < dist-decdist && backL.rotation(degrees) >= accdist){
      value = 1;
    }

    if(backL.rotation(degrees) < dist && backL.rotation(degrees) >= dist-decdist){
      decslope = -0.95/decdist;
      decintercept = 1-decslope*dist+decslope*decdist;
      decrate = decslope*backL.rotation(degrees) + decintercept;
      value = decrate;
    }

    double fixed = gyroValue() - gyrost;
    if(std::abs(fixed) >= 40){
      endr = 0.1;
    }
    else if(std::abs(fixed) > 20){
      endr = 0.3;
    }
    else{
      endr = 0.9;
    }

    if(fixed < 0){
      rightslope = (1 - endr)/10;
    }
    else{
      rightslope = 1;
    }

    if(fixed > 0){
      leftslope = (endr - 1)/10;
    }
    else{
      leftslope = 1;
    }

    if(rightslope == 1){
      rightfunc = 1;
    }
    else{
      rightfunc = rightslope * fixed + 1;
    }

    if(leftslope == 1){
      leftfunc = 1;
    }
    else{
      leftfunc = leftslope * fixed + 1;
    }

    if(leftfunc < endr){
      leftfunc = endr;
    }
    if(leftfunc > 1){
      leftfunc = 1;
    }
    if(rightfunc > 1){
      rightfunc = 1;
    }
    if(rightfunc < endr){
      rightfunc = endr;
    }

    runSeparate(spd*leftfunc*value, -spd*rightfunc*value);
  }

  while(backL.rotation(degrees) > -dist && spd < 0 && timeSave.timer(msec) < timeSafe && runState != 5){
    if(backL.rotation(degrees) > -accdist && spd < 0){
      accslope = 0.8/accdist;
      accintercept = 0.05;
      accrate = accslope*backL.rotation(degrees) + accintercept;
      if(spd*accrate > -10 && spd != 0){
        value= -10/spd;
      }
      else{
        value = accrate;
      }
    }

    if(backL.rotation(deg) > -dist + decdist && spd < 0 && backL.rotation(deg) <= -accdist){
      value = 1;
    }

    if(backL.rotation(deg)> -dist && backL.rotation(deg)<= -dist + decdist && spd < 0){
      decslope = -0.95/decdist;
      decintercept = 1 + decslope*dist - decslope*decdist;
      decrate = decslope*backL.rotation(deg) + decintercept;
      value = decrate;
    }

    double fixed = gyroValue() - gyrost;

    if (std::abs(fixed) >= 40)
    {
      endr = 0.1;
    }
    else
    {
      if (std::abs(fixed) > 20)
      {
        endr = 0.3;
      }
      else
      {
        endr = 0.9;
      }
    }

    if (fixed < 0)
      leftslope = (1 - endr)/10;
    else 
      leftslope = 1;

    if (fixed > 0)
      rightslope = (-1 + endr)/10;
    else
      rightslope = 1;

    if (rightslope == 1)
      rightfunc = 1;
    else
      rightfunc = rightslope * fixed + 1;
    
    if (leftslope == 1)
      leftfunc = 1;
    else
      leftfunc = leftslope * fixed + 1;

    if (leftfunc < endr)
      leftfunc = endr;
    
    if (leftfunc > 1)
      leftfunc = 1;
    
    if (rightfunc > 1)
      rightfunc = 1;
    
    if (rightfunc < endr)
      rightfunc = endr;

    runSeparate(spd*leftfunc*value, -spd*rightfunc*value);
  }
}

#warning Autonomous

void Autonomous(){
  autoTimer.resetTimer();
  timeX.resetTimer();
  while (timeX.timer(msec) < 200){
    run(-30);
    liftFront(-20);
  }
  //grab neutral goal
  runAutoEncode(95, 2600, ACC_NOR_DEC, 0, 100, COAST, 2000);
  wait(500);
  //lift neutral goal off the ground for mobility
  lockF.set(true);
  liftFAuto(100, 300);
  //back up, put down neutral goal
  runAutoEncode(-30, 200, ACC_NOR_DEC, 10, 100, BRAKE, 2200);
  wait(100);
  gyroTurn(245, -35, 5, 100, BRAKE, 1500);
  wait(100);
  runAutoEncode(30, 2000, ACC_NOR_DEC, -10, 100, BRAKE, 4000);
  wait(100);
  lockF.set(false);
  //go for alliance goal
  runAutoEncode(-30, 400, ACC_NOR_DEC, -10, 100, BRAKE, 2000);
  gyroTurn(105, 35, 5, 100, BRAKE, 1500);
  runAutoEncode(-30, 1400, ACC_NOR_DEC, 0, 100, COAST, 3000);
  wait(300);
  lockB.set(true);
  wait(200);
  lockC.set(true);
  wait(300);
  //score in alliance goal
  convAuto(50, 700);
  runAutoEncode(50, 600, ACC_NOR_DEC, 0, 100, COAST, 2000);
  Controller1.Screen.print(autoTimer.timer(vex::timeUnits::msec)*0.001);

}

#warning driver

void Driver(){
  while(true){
    if(selfInspect == 0){
      gyroRecalibrate(3000);
    }

    if(selfInspect == 1){
      Controller1.rumble(".-.-");
      selfInspect = 2;
    }

    double left = Controller1.Axis3.value() + Controller1.Axis1.value();
    double right = Controller1.Axis3.value() - Controller1.Axis1.value();
    if(std::abs(left)>=5||std::abs(right)>=5)
    {
      runSeparate(left,right);
    }
    else{
      runStop(driverStop);
    }
    

    frontLock();
    backLock();

    if(Controller1.ButtonR1.pressing()){
      liftFront(100);
    }
    else if(Controller1.ButtonR2.pressing()){
      liftFront(-100);
    }
    else{
      liftFront(0);
    }

    if(Controller1.ButtonL1.pressing()){
      conv(50);
    }
    else if(Controller1.ButtonL2.pressing()){
      conv(-60);
    }
    else{
      conv(0);
    }
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Competition.autonomous(Autonomous);
  Competition.drivercontrol( Driver );
  vexcodeInit();
}
