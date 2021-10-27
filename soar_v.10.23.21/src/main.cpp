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
// frontR               motor         1               
// frontL               motor         10              
// backL                motor         9               
// backR                motor         2               
// liftB                motor         5               
// liftF                motor         6               
// gyros                inertial      7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

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

brain timer871; //general timer
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

double abso(double x){
  if(x < 0) return -x;
  return x;
}

void run(int spd){
  spin(frontR, spd);
  spin(frontL, spd);
  spin(backL, spd);
  spin(backR, spd);
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

void runSeparate(double left, double right){
  if(left != 1){
    spin(backL, left);
    spin(frontL, left);
  }
  else{
    frontL.stop(hold);
    backL.stop(hold);
  }
  if(right != 1){
    spin(backR, right);
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
    if(abso(Controller1.Axis3.value())>50||abso(Controller1.Axis4.value())>50||abso(Controller1.Axis1.value())>50
    ||abso(Controller1.Axis2.value())>50
    ||Controller1.ButtonL1.pressing()||Controller1.ButtonL2.pressing()||Controller1.ButtonR1.pressing()
    ||Controller1.ButtonR2.pressing()||Controller1.ButtonUp.pressing()||Controller1.ButtonDown.pressing()
    ||Controller1.ButtonLeft.pressing()||Controller1.ButtonRight.pressing()||Controller1.ButtonB.pressing()||Controller1.ButtonX.pressing()){
      selfInspect=2;
      break;
    }
  }

  while(cTime.timer(vex::timeUnits::msec) < time){
    if(abso(Controller1.Axis3.value())>50||abso(Controller1.Axis4.value())>50||abso(Controller1.Axis1.value())>50
    ||abso(Controller1.Axis2.value())>50
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

    if(abso(pwr)*accrate < 10){
      d = 10;
    }
    else{
      d = abso(pwr)*accrate;
    }

    run(d*sign(pwr));
  }

  while(abso(backL.rotation(rotationUnits::deg)) < dist - decdist && timeSave.timer(timeUnits::msec) < timeSafe){
    run(pwr);
  }

  while(backL.rotation(rotationUnits::deg) < dist && decdist != 0 && timeSave.timer(timeUnits::msec) < timeSafe){
    double v;

    decslope = (0.05 - 1)/decdist;
    decintercept = 0.05 - decslope*dist;
    decrate = decslope*backL.rotation(rotationUnits::deg) + decintercept;

    if(abso(pwr)*decrate < 10){
      v = 10;
    }
    else{
      v = abso(pwr)*decrate;
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
    if(abso(fixed) >= 40){
      endr = 0.1;
    }
    else if(abso(fixed) > 20){
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

    if (abso(fixed) >= 40)
    {
      endr = 0.1;
    }
    else
    {
      if (abso(fixed) > 20)
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
}

#warning driver

void Driver(){
  while(true){
    if(selfInspect == 0){
      gyroRecalibrate(3000);
    }

    if(selfInspect == 1){
      Controller1.rumble(".-.-");
    }
    
    double left = Controller1.Axis3.value() + Controller1.Axis1.value()*0.8;
    double right = Controller1.Axis3.value() - Controller1.Axis1.value()*0.8;
    if(abso(left)>=5||abso(right)>=5)
    {
      runSeparate(left,right);
    }
    else
    {
      runStop(driverStop);
    }
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Competition.autonomous(Autonomous);
  Competition.drivercontrol( Driver );
  vexcodeInit();
  
}
