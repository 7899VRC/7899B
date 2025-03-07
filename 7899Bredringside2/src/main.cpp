/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      9/29/2024, 2:37:00 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include<iostream>
#include<algorithm>

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
float team = 2;

bool stop = false;
bool isRollerSpinningForward = false;
bool isRollerSpinningBackward = false;
float ladybrownposition = 0;
int stuckcount = 0;
bool prev_spinning = 0;
int justStartingCnt = 0;

int getcolor(){
  colorsensor.setLightPower(25,pct);
  wait(5,msec);
  int c = colorsensor.hue();
  return c;
}
void intakecontrol(){

  Brain.Screen.printAt(1,20,"hue =");

  while(true){
    int c = getcolor();
    Brain.Screen.printAt(1,20,"hue = %d  ",c);
    if (team == 1){
      int colortosort = 4;

      int colorTolerance = 25;
      if ((abs(c) - colortosort) < colorTolerance and stop == false){
        wait(220,msec);
        roller.stop(brake);
        wait(200,msec);
        stop = true;
      }
      else if (isRollerSpinningForward){
        roller.spin(reverse,100,pct);
        stop = false;
      }
      else if (isRollerSpinningBackward){ 
        roller.spin(fwd,100,pct);
        stop = false;
      }
      else roller.stop(brake);
    }

  
  else {

  int colortosort = 200;

  int colorTolerance = 25;
  Brain.Screen.printAt(1,20,"hue = %d  ",c);
  if (abs(c - colortosort) < colorTolerance){
    wait(220,msec);
    roller.stop(brake);
  }
  else if (isRollerSpinningForward){
    roller.spin(reverse,100,pct);
  }
  else if (isRollerSpinningBackward){
    roller.spin(fwd,100,pct);
  }
  else roller.stop(brake);
  if(justStartingCnt==0 && isRollerSpinningForward && fabs(roller.velocity(pct)) <= 1){
    stuckcount+=1;
    if (ladybrownposition == 1 ){
      if (stuckcount >=3){
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
  wait(10,msec);
}
}


void ladybrownAuto(){
  wait(200,msec);
  Ladybrown.spinTo(270,deg);
  wait(400,msec);
  Ladybrown.spinTo(0,deg);
  Ladybrown.spin(reverse,100,pct);
  wait(100,msec);
  Ladybrown.stop(brake);
  Ladybrown.resetPosition();

}
void ladybrownmacro(){
  Ladybrown.setVelocity(100,pct);


  if(ladybrownposition == 0){
    Ladybrown.spinTo(40.5, degrees, true);
    ladybrownposition = 1;
  }
  else{
    Ladybrown.spinTo(240, degrees, true);
    ladybrownposition = 2;
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
  float kp=6;
  float speed =kp*error;
  float accuracy=0.05;//was 0.05
  float kd = 0.35;
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
		double accuracy=0.3; //how accurate to make the turn in degrees
		double error=target-heading;
    double ki = 0.1;
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
		while(fabs(error)>=accuracy or count<=7 ){
      heading=Gyro.rotation();  //measure the heading of the robot
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


      if (intergal >=40){
        intergal = 40;
      }
      else if (intergal <=-40){
        intergal = -40;
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
		driveBrake();  //stope the drive
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
  float Lkp=6.05;
  float count =0;
  float Rkp=6.05;
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
  float Lkp=6.05;
  float count =0;
  float Rkp=6.05;
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
void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          xZERsd5e    Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
    roller.setMaxTorque(100,pct);
    Ladybrown.setVelocity(100,pct); 
    vex::thread intakeControlThread(intakecontrol);
    vex::thread ladybrownauto(ladybrownAuto);
    gyroTurnF(-45);
    wait(200,msec);
    inchDrive(-10,900);
    gyroTurnF(-10);
    inchDrive(-26,1000);
    inchDrive(-11,400);
    pneuclamp();
    wait(200,msec);
    gyroTurnF(135);
    isRollerSpinningForward = true;
    arcturnR(41,42,800);
    gyroTurnF(90);
    wait(100,msec);
    inchDrive(30,600);
    wait(400,msec);
    inchDrive(10,400);
    arcturnR(35,-90,1000);
    gyroTurnF(90);
    inchDrive(27,1000);
    gyroTurnF(-90);
    inchDrive(60,1000);
    // gyroTurn(-110);
    // inchDrive(24,1000);
    // gyroTurn(-80);
    // inchDrive(14,1000);
    // inchDrive(-10,900);
    // gyroTurn(30);
    // inchDrive(12,900);
    // inchDrive(-30,1000);
    // gyroTurn(-90);
    // inchDrive(20,1800);





    // inchDrive(26,2600);
    

  

  
  //score ring onto mogo next

}




void usercontrol(void) {
  // User control code here, inside the loop


    vex::thread intakeControlThread(intakecontrol);
    Controller1.ButtonL1.pressed(ladybrownmacro);
    Controller1.ButtonL2.pressed(ladybrownrest);
    Controller1.ButtonR1.pressed(spinFunction);
    Controller1.ButtonR2.pressed(reverseSpinFunction);
    roller.setMaxTorque(90,pct);

  while (true) {
    float lstick = Controller1.Axis3.position();
	  float rstick = Controller1.Axis1.position();

    //Spins conveyor belt forward if R1 is pressed, reverse if R2 is pressed

   

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
    if (Controller1.ButtonLeft.pressing()){
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
  colorsensor.setLightPower(25,pct);
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
