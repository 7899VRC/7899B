/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      9/29/2024, 2:37:00 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>
#include <algorithm>


using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;
timer gyroTimmer;

//Declare motors
motor FL = motor(PORT6, ratio6_1, false);
motor ML = motor(PORT10, ratio6_1, true);
motor BL = motor(PORT4, ratio6_1, true);
motor FR = motor(PORT3, ratio6_1, true);
motor MR = motor(PORT2, ratio6_1, false);
motor BR = motor(PORT1, ratio6_1, false);
motor roller = motor (PORT19, ratio6_1, true);
motor Ladybrown = motor(PORT8, ratio36_1, false);
digital_out Pneu1 = digital_out(Brain.ThreeWirePort.A);
inertial  Gyro=inertial(PORT21);
digital_out corner = digital_out (Brain.ThreeWirePort.B);
digital_out rollerposition = digital_out(Brain.ThreeWirePort.C);
optical colorsensor = optical(PORT5);

void drive(int lspeed, int rspeed, int wt){
  lspeed *= 0.12;
  rspeed *= 0.12;


  FL.spin(forward, lspeed, volt);
  ML.spin(forward, lspeed, volt);
  BL.spin(forward, lspeed, volt);
  FR.spin(forward, rspeed, volt);
  MR.spin(forward, rspeed, volt);
  BR.spin(forward, rspeed, volt);
  wait(wt, msec);
}

void driveBrake(){
  FR.stop(brake);
  MR.stop(brake);
  BR.stop(brake);
  BL.stop(brake);
  ML.stop(brake);
  FL.stop(brake);
}


//global variables 
float pi = 3.14; 
float dia = 2.75;
float gearRatio = 0.75; 




bool isRollerSpinningForward = false;
bool isRollerSpinningBackward = false;
int colortosort = 4;
bool stop = false;


float ladybrownposition = 0;
float autoposition = 0;


void ladybrownAuto(){
  autoposition = 0;
  Ladybrown.spinTo(290,deg);
  Ladybrown.spinTo(0,deg);
  Ladybrown.spin(reverse,100,pct);
  wait(200,msec);
  Ladybrown.stop(brake);
  Ladybrown.resetPosition();
  while(true){
  if (autoposition == 1){
  Ladybrown.spinTo(40.5,deg,true);
  }
  else if (autoposition == 2){
    Ladybrown.spinTo(240,deg,true);
  }
  else Ladybrown.spinTo(0,deg,true);
  }
}

void loweststake(){

}

void ladybrownmacro(){
  Ladybrown.setVelocity(100,pct);


  if(ladybrownposition == 0){
    Ladybrown.spinTo(30.5, degrees, true);
    ladybrownposition = 1;
  }
  else{
    Ladybrown.spinTo(240, degrees, true);
    ladybrownposition = 2;
  }
}
int stuckcount = 0;
bool prev_spinning = 0;
int justStartingCnt = 0;

void intakecontrol(){
  while(true){
    wait(10, msec);
  int colorTolerance = 223;
  if (abs(colorsensor.value() - colortosort) > colorTolerance and stop == false) {
    roller.stop(brake);
    stop = true;
  }
  else if (isRollerSpinningForward){
    if(!prev_spinning) justStartingCnt = 5;
    prev_spinning=1;
    roller.spin(reverse,100,pct);
  }
  else if (isRollerSpinningBackward){
    roller.spin(fwd,100,pct);
  }
  else{
    prev_spinning=false;
    roller.stop(brake);
  }
  
  if(justStartingCnt==0 && isRollerSpinningForward && fabs(roller.velocity(pct)) <= 1){
    stuckcount+=1;
    if (ladybrownposition == 1 || autoposition == 1){
      if (stuckcount >=1){
      roller.stop(brake);
      isRollerSpinningForward = false;

      }
    }
  }else{
    stuckcount=0;
  }

  if(justStartingCnt>0){
    justStartingCnt--;
  }
  }
}

void ladybrownrest(){
  Ladybrown.spinTo(0, degrees, true);
  ladybrownposition = 0;
}

  void spinFunction(){
    if(isRollerSpinningForward == true){
      // roller.stop(brake);
      isRollerSpinningForward = false;
    }
    else{
      // roller.spin(reverse,100, pct);
      isRollerSpinningForward = true;
      isRollerSpinningBackward = false;
    }
  }

  void CornerClear(){
    corner.set(!corner.value());
  }

  void rollerrise(){
    rollerposition.set(!rollerposition.value());
  }

  void reverseSpinFunction(){
    if(isRollerSpinningBackward == true){
      // roller.stop(brake);
      isRollerSpinningBackward = false;
    }
    else{
      // roller.spin(forward,100, pct);
      isRollerSpinningForward = false;
      isRollerSpinningBackward = true;
    }  
  }

void pneuclamp(){
  Pneu1.set(!Pneu1.value());
}

float target2 = 0;

void gyroTurn(float target, float b = 2.4){
		float heading=0.0; //initialize a variable for heading
		double accuracy=0.3; //how accurate to make the turn in degrees
		double error=target-heading;
    double ki = 0.001;
    double intergal = 0;

		double kp=6;//7.85;//was 6
    double speed = 0;
    double kd = 0.3;//0.65;//was 0.3
    double last_error = 0;
    double dt = 0.01; //reset Gyro to zero degrees
		int count = 0;
    vex::timer timer;  // Create a timer object

    timer.clear();  // Clear any previous timer value
    int timeLimit = 1400;
		while(fabs(error)>=accuracy or count<=3 ){
      heading=Gyro.rotation();  //measure the heading of the robot
      std::cout<<heading<<"/n";
			error=target-heading;
      if (error > 180){
        error = error - 360;
      }
      else if (error < -180){
        error= error + 360;
      }
      intergal +=error;
      if (intergal >=40){
        intergal = 40;
      }
      else if (intergal <=-40){
        intergal = -40;
      }
      
      
        //calculate error      
			speed = kp * error + kd * (error - last_error) / dt + b * error / fabs(error);


      drive(speed, -speed, dt*1000); //turn right at speed
      last_error = error;

      if(fabs(error)<=accuracy+0.1){
        count++;
      }
      else count = 0;

      if (timer.time(vex::timeUnits::msec) >= timeLimit ){
        break;
      }


		}
      
			driveBrake();  //stope the drive
      std::cout<<"new turn"<<"/n";
}


void inchDrive(float target, float timeLimit, int b =1.5, int c = 0){
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 1;
  float turn_speed = 0;
  float angleD = 0;

  ML.setPosition(0,rev);
  float x=0;
  float error=target;
  float kp=6;//was6;
  float speed =kp*error;
  float accuracy=0.05;//was 0.05
  float kd = 0.3;//was 0.3
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  vex::timer timer;  // Create a timer object
  timer.clear(); 
  target2 = Gyro.rotation();

  while(fabs(error)>=accuracy){
    heading = Gyro.rotation();
    if (target2 > 180){
      target - 360;
    }
    else if (target2 < -180){
      target + 360;
    }
    x=ML.position(rev)*pi*dia*gearRatio;
    error=target-x;
    angle_error = target2-heading;
    speed=kp*error +kd * (error - last_error) / dt + b*error/fabs(error);
    turn_speed = angleP * angle_error+ angleD*(angle_error - angle_last_error)/dt;
    if (speed >= 100){
      speed = 100;
    }
    else if (speed <=-100){
      speed = -100;
    }

    drive(speed+ turn_speed ,speed -turn_speed,10);
    last_error = error;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      break;
    } 
  }
  driveBrake();

}
void inchDriveC(float target, float timeLimit, int b =1.5, int c = 0){
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 1;
  float turn_speed = 0;
  float angleD = 0;

  ML.setPosition(0,rev);
  float x=0;
  float error2 = target;
  float error=target;
  float kp=6;
  float speed =kp*error;
  float accuracy=2;//was 0.05
  float kd = 0.65;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  vex::timer timer;  // Create a timer object
  timer.clear(); 
  target2 = Gyro.rotation();
  while(fabs(error)>=accuracy){
    heading = Gyro.rotation();
    if (target2 > 180){
      target - 360;
    }
    else if (target2 < -180){
      target + 360;
    }
    x=ML.position(rev)*pi*dia*gearRatio;
    error=target-x;
    angle_error = target2-heading;
    speed=kp*error +kd * (error - last_error) / dt + b*error/fabs(error);
    turn_speed = angleP * angle_error+ angleD*(angle_error - angle_last_error)/dt;
    if (speed >= 100){
      speed = 100;
    }
    else if (speed <=-100){
      speed = -100;
    }
    if(fabs(speed) < 40 && error > 3){
      if(speed>0){
        speed = 40;
      }
      else{
        speed = -40;
      }
    }

    drive(speed+ turn_speed ,speed -turn_speed,10);
    last_error = error;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      break;
    } 
  }

}
void arcturn(float target, float arcdegree, float timeLimit, int b =1.5, int c = 0){
  float heading = 0;
  float angle_error = 0;
  float angle_last_error = 0;
  float angleP = 1.2;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  ML.setPosition(0,rev);
  float x=0;
  float error=target;
  float kp=6;
  float speed =kp*error;
  float accuracy=0.1;//was 0.05
  float kd = 0.35;
  double last_error = 0;
  double dt = 0.01;
  vex::timer timer;  // Create a timer object
  timer.clear(); 
  while(fabs(error)>accuracy or fabs(angle_error) > angle_accuracy){
    heading = Gyro.rotation();
    x=ML.position(rev)*pi*dia*gearRatio;
      if (angle_error > 180){
        error = error - 360;
      }
      else if (angle_error < -180){
        error= error + 360;
      }
    error=target-x;
    angle_error = arcdegree-heading;
    speed=kp*error +kd * (error - last_error) / dt + b*error/fabs(error);
    turn_speed = angleP * angle_error + c*angle_error/fabs(angle_error);
    if(speed >= 100){
        speed = 100;
    }
    else if (speed <= -100)
    {
        speed = -100;
    }
    

    drive(speed + turn_speed,speed- turn_speed,10);
    last_error = error;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      break;
    } 
  }
  driveBrake();

}
void gyroTurnF(float target, float b = 2.4){
		float heading=0.0; //initialize a variable for heading
		double accuracy=0.2; //how accurate to make the turn in degrees
		double error=target-heading;
    double ki = 0.28;
    double intergal = 0;

		double kp=6;//7.85;//was 6
    double speed = 0;
    double kd = 0.43;//0.65;//was 0.3
    double last_error = 0;
    double dt = 0.01; //reset Gyro to zero degrees
		int count = 0;
    vex::timer timer;  // Create a timer object

    timer.clear();  // Clear any previous timer value
    int timeLimit = 1400;
		while(fabs(error)>=accuracy or count<=7 ){
      heading=Gyro.rotation();  //measure the heading of the robot
      std::cout<<heading<<"\n";
			error=target-heading;
      if (error > 180){
        error = error - 360;
      }
      else if (error < -180){
        error= error + 360;
      }

      if (fabs(error)<10 && fabs(error)>1){
      intergal +=error;
      } 
      else{
        intergal = 0;
      }


      if (intergal >=50){
        intergal = 50;
      }
      else if (intergal <=-50){
        intergal = -50;
      }



      
      
        //calculate error      
			speed = kp * error + kd * (error - last_error) / dt + ki*intergal;


      drive(speed, -speed, dt*1000); //turn right at speed
      last_error = error;
      if(fabs(error)<=accuracy+0.3){
      count++;
      }
      else count = 0;

      if (timer.time(vex::timeUnits::msec) >= timeLimit ){
        break;
      }


		}
      
			driveBrake();
      std::cout<<"newline"<<"\n";  //stope the drive
}
void arcturnL(int r, float arcdeg, int timeLimit,float max_drift = 0.15){
  float heading = Gyro.rotation();


  float angle_last_error = 0;
  float angleP = 6.5;
  float angleD = 0.1;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  float robotwidth = 15; 
  ML.setPosition(0,rev);
  MR.setPosition(0,rev);
  float Lx=0;
  float Rx = 0;
  float settletime = 150;
  double Ltarget =2*pi*(r)*(arcdeg/360.0);
  double Rtarget = 2*pi*(r-robotwidth)*(arcdeg/360.0);
  float Lerror= Ltarget;
  float Rerror = Rtarget;
  float Lkp=6.5;
  float count =0;
  float Rkp=6.5;
  float accuracy=0.1;//was 0.05
  float Lkd = 0.35; 
  float Rkd = 0.35;
  double Llast_error = 0;
  double Rlast_error = 0;
  double dt = 0.01;
  double Lki = 0.1;
  double Lintergal = 0;
  double Rki = 0.1;
  double Rintergal = 0;
  float angle_target = (Rx * 360)/(2*pi*r);
  float Lspeed =Lkp*Lerror +Lkd * (Lerror - Llast_error) / dt+Lintergal*Lki;
  float Rspeed =Rkp*Rerror +Rkd * (Rerror - Rlast_error) / dt+Rintergal*Rki;
  float angle_error = angle_target;
  vex::timer timer;
  vex::timer errortimer;  // Create a timer object
  timer.clear(); 
  while(fabs(Lerror)>= accuracy || fabs(Rerror) >= accuracy||count>=7){
  
    heading = Gyro.rotation();
    Lx=ML.position(rev)*pi*dia*gearRatio;
    Rx=MR.position(rev)*pi*dia*gearRatio;
    angle_target = (Lx * 360)/(2*pi*r);

    Lerror=Ltarget - Lx;
    Rerror=Rtarget - Rx;
    angle_error = angle_target-heading;

    if (angle_error > 180){
      angle_error = angle_error - 360;
    }
    else if (angle_error < -180){
      angle_error= angle_error + 360;
    }
     if (fabs(Lerror)<10 && fabs(Lerror)>1){
      Lintergal +=Lerror;
      } 
      else{
        Lintergal = 0;
      }
      if (Rintergal >=40){
        Rintergal = 40;
      }
      else if (Rintergal <=-40){
        Rintergal = -40;
      }
      if (fabs(Rerror)<10 && fabs(Rerror)>1){
      Rintergal +=Rerror;
      } 
      else{
        Rintergal = 0;
      }
      if (Rintergal >=40){
        Rintergal = 40;
      }

      else if (Rintergal <=-40){
        Rintergal = -40;
      }
  Lspeed =Lkp*Lerror +Lkd * (Lerror - Llast_error) / dt+Lintergal*Lki;
  Rspeed =Rkp*Rerror +Rkd * (Rerror - Rlast_error) / dt+Rintergal*Rki;
  Brain.Screen.printAt(1,20,"Lspeed = %.2f     Rspeed = %.2f  ",Lspeed,Rspeed );
  turn_speed = angleP * angle_error + angleD * (angle_error,angle_last_error)/dt;
    turn_speed =  std::min(std::fabs(turn_speed), std::fabs(Lspeed) * max_drift);
    
    if(Lspeed >= 100){
        Lspeed = 100;
    }
    else if (Lspeed <= -100)
    {
        Lspeed = -100;
    }
    if(Rspeed >= 100){
        Rspeed = 100;
    }
    else if (Rspeed <= -100)
    {
        Rspeed = -100;
    }

    if(fabs(angle_error) > angle_accuracy){
      errortimer.clear();
    }
      if(fabs(Lerror)<=accuracy+0.3||fabs(Rerror)<=accuracy+0.3){
      count++;
      }
      else count = 0;

    drive(Lspeed-turn_speed,Rspeed+turn_speed,10);
    Llast_error = Lerror;
    Rlast_error = Rerror;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      break;
    } 
  }
  driveBrake();

}
void arcturnR(float r, float arcdeg, int timeLimit,float max_drift = 0.1){
  float heading = Gyro.rotation();


  float angle_last_error = 0;
  float angleP = 6.5;
  float angleD = 0.1;
  float turn_speed = 0;
  float angle_accuracy = 0.3;
  float robotwidth = 15; 
  ML.setPosition(0,rev);
  MR.setPosition(0,rev);
  float Lx=0;
  float Rx = 0;
  float settletime = 150;
  double Ltarget =2*pi*(r-robotwidth)*(arcdeg/360.0);
  double Rtarget = 2*pi*(r)*(arcdeg/360.0);
  float Lerror= Ltarget;
  float Rerror = Rtarget;
  float Lkp=6.5;
  float count =0;
  float Rkp=6.5;
  float accuracy=0.1;//was 0.05
  float Lkd = 0.35; 
  float Rkd = 0.35;
  double Llast_error = 0;
  double Rlast_error = 0;
  double dt = 0.01;
  double Lki = 0.1;
  double Lintergal = 0;
  double Rki = 0.1;
  double Rintergal = 0;
  float angle_target = (Rx * 360)/(2*pi*r);
  float Lspeed =Lkp*Lerror +Lkd * (Lerror - Llast_error) / dt+Lintergal*Lki;
  float Rspeed =Rkp*Rerror +Rkd * (Rerror - Rlast_error) / dt+Rintergal*Rki;
  float angle_error = angle_target;
  vex::timer timer;
  vex::timer errortimer;  // Create a timer object
  timer.clear(); 
  while(fabs(Lerror)>= accuracy || fabs(Rerror) >= accuracy||count<=5){
  
    heading = Gyro.rotation();
    Lx=ML.position(rev)*pi*dia*gearRatio;
    Rx=MR.position(rev)*pi*dia*gearRatio;
    angle_target = (Lx * 360)/(2*pi*r);

    Lerror=Ltarget - Lx;
    Rerror=Rtarget - Rx;
    angle_error = angle_target-heading;

    if (angle_error > 180){
      angle_error = angle_error - 360;
    }
    else if (angle_error < -180){
      angle_error= angle_error + 360;
    }
     if (fabs(Lerror)<10 && fabs(Lerror)>1){
      Lintergal +=Lerror;
      } 
      else{
        Lintergal = 0;
      }
      if (Rintergal >=40){
        Rintergal = 40;
      }
      else if (Rintergal <=-40){
        Rintergal = -40;
      }
      if (fabs(Rerror)<10 && fabs(Rerror)>1){
      Rintergal +=Rerror;
      } 
      else{
        Rintergal = 0;
      }
      if (Rintergal >=40){
        Rintergal = 40;
      }

      else if (Rintergal <=-40){
        Rintergal = -40;
      }
  Lspeed =Lkp*Lerror +Lkd * (Lerror - Llast_error) / dt+Lintergal*Lki;
  Rspeed =Rkp*Rerror +Rkd * (Rerror - Rlast_error) / dt+Rintergal*Rki;
  Brain.Screen.printAt(1,20,"Lspeed = %.2f     Rspeed = %.2f  ",Lspeed,Rspeed );
  turn_speed = angleP * angle_error + angleD * (angle_error,angle_last_error)/dt;
    turn_speed =  std::min(std::fabs(turn_speed), std::fabs(Lspeed) * max_drift);
    
    if(Lspeed >= 100){
        Lspeed = 100;
    }
    else if (Lspeed <= -100)
    {
        Lspeed = -100;
    }
    if(Rspeed >= 100){
        Rspeed = 100;
    }
    else if (Rspeed <= -100)
    {
        Rspeed = -100;
    }

    if(fabs(angle_error) > angle_accuracy){
      errortimer.clear();
    }
      if(fabs(Lerror)<=accuracy+0.3||fabs(Rerror)<=accuracy+0.3){
      count++;
      }
      else count = 0;

    drive(Lspeed-turn_speed,Rspeed+turn_speed,10);
    Llast_error = Lerror;
    Rlast_error = Rerror;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      break;
    } 
  }
  driveBrake();

}
float lastpos = 0;

void hang(){
  while(ML.position(rev) == lastpos ){//use old motor position and new motor position to find if movement works.
  drive(50,50,5);
  lastpos = ML.position(rev);
  }
}

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous Jonathan   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific is gay here.     */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
    Gyro.setRotation(0,deg);
    autoposition = 0;
    Ladybrown.setVelocity(100,pct);
    Ladybrown.resetPosition();
    vex::thread intakeControlThread(intakecontrol);
    vex::thread ladybrownauto(ladybrownAuto);
    isRollerSpinningForward = false;
    wait(450,msec);
    inchDrive(-7,700);
    gyroTurnF(-90);
    inchDrive(-18,1000);
    inchDrive(-9,800);
    pneuclamp();
    wait(300,msec);
    gyroTurnF(180);
    isRollerSpinningForward = true;
    inchDrive(29,1000);
    gyroTurnF(120);
    autoposition = 1;
    inchDrive(42, 1600);
    gyroTurnF(90);
    wait(100,msec);
    autoposition = 2;

    inchDrive(18,1000);
    inchDrive(-20,1200);
    autoposition = 0;
    isRollerSpinningForward = true;
    gyroTurnF(0);
    inchDrive(48,1500);
    inchDriveC(9,800);
    arcturnL(13,180,1500);
    inchDriveC(17,400);
    gyroTurnF(180);
    pneuclamp();
    inchDrive(-24,1000);
    isRollerSpinningForward = false;
    inchDrive(8,900);
    gyroTurnF(90);
    inchDrive(24,1200);
    // Gyro.setRotation(90,deg);
    inchDrive(-80,2500);
    inchDrive(-14,900);
    pneuclamp();
    wait(100,msec);
    gyroTurnF(180);
    isRollerSpinningForward = true;
    inchDrive(29,1000);
    gyroTurnF(-120);
    wait(100,msec);
    autoposition = 1;
    inchDrive(40,1600);
    gyroTurnF(-90);
    wait(100,msec);
    autoposition = 2;
    inchDrive(18,1000);
    inchDrive(-22,1200);
    autoposition = 0;
    isRollerSpinningForward = true;
    gyroTurnF(0);
    inchDrive(50,1200);
    inchDrive(18,900);
    gyroTurnF(-135);
    inchDrive(22,1000);
    gyroTurnF(180);
    pneuclamp();
    inchDrive(-24,1000);


    inchDrive(50,1000);
    gyroTurnF(135);
    inchDrive(95,1100);
    isRollerSpinningForward = false;
    gyroTurnF(45);
    inchDrive(-70,1400);
    gyroTurnF(45);
    inchDrive(20,1200);
    gyroTurnF(-90);
    inchDrive(-50,1500);
    inchDrive(-10,900);
    pneuclamp();
    wait(100,msec);
    gyroTurnF(90);
    isRollerSpinningForward = true;
    inchDrive(30,1000);
    
    pneuclamp();
    inchDrive(-30,1000);
    // arcturn(30,-180,900);
    // arcturn(75,120,1300);
    // isRollerSpinningForward = false;
    // arcturn(55,45,1100);
    // inchDrive(-70,1800);
    // arcturn(20,-90,900);
    // arcturn(150,5,2000);

    










    // inchDrive(26,2600);
    

   

  
  //score ring onto mogo next

}



void usercontrol(void) {
  // User control code here, inside the loop
  colorsensor.brightness(20);

    vex::thread intakeControlThread(intakecontrol);
    Controller1.ButtonL1.pressed(ladybrownmacro);
    Controller1.ButtonL2.pressed(ladybrownrest);


    Controller1.ButtonR1.pressed(spinFunction);
    Controller1.ButtonR2.pressed(reverseSpinFunction);


    //Controller1.ButtonRight.pressed(autonomous);
    roller.setMaxTorque(90,pct);
    Ladybrown.resetPosition();
  while (true) {

    float lstick = Controller1.Axis3.position();
	  float rstick = Controller1.Axis1.position();

    //Spins conveyor belt forward if R1 is pressed, reverse if R2 is pressed

    // if (Controller1.ButtonR1.pressing()){
    //   isRollerSpinningForward = true;
    //   isRollerSpinningBackward = false;
    //   roller.spin(reverse, 12000, voltageUnits::volt);
    // }
    // else if (Controller1.ButtonR2.pressing()) {
    //   isRollerSpinningBackward = true;
    //   isRollerSpinningForward = false;
    //   roller.spin(fwd, 12000, voltageUnits::volt);
    // }
    // else {
    //   isRollerSpinningForward = false;
    //   isRollerSpinningBackward = false;
    //   roller.stop();
    // }


    if (Controller1.ButtonA.pressing()){
      pneuclamp();
      wait(150,msec);
    }
    if (Controller1.ButtonB.pressing()){
      CornerClear();
      wait(150,msec);
    }
    if (Controller1.ButtonX.pressing()){
      rollerrise();
      wait(150,msec);
    }
if (Controller1.ButtonLeft.pressing()){
  if(!(ladybrownposition == 3)){
    Ladybrown.spinTo(60,deg,true);
    ladybrownposition == 3;
  }
}
if (Controller1.ButtonDown.pressing()){
  if (!(ladybrownposition == 4)){
  Ladybrown.spinTo(250,deg);
  ladybrownposition = 4;
  }
  else if(!(ladybrownposition == 5)){
  Ladybrown.spinTo(290,deg);
  ladybrownposition = 5;
  }   
  wait(150,msec);
}
if (Controller1.ButtonX.pressing()){
  inchDrive(4,300);
}
    if (Controller1.ButtonUp.pressing()){
      Ladybrown.spin(reverse,100,pct);
      wait(200,msec);
      Ladybrown.stop(brake);
      Ladybrown.resetPosition();
    }
    


    drive(lstick + rstick , lstick - rstick, 20);
    wait(20, msec);
  }
}



// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Ladybrown.resetPosition();
  //rollerrise();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}