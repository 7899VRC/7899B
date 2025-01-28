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
motor roller = motor (PORT19, ratio6_1, false);
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
  Ladybrown.spinTo(540,deg);
  Ladybrown.spinTo(0,deg);
  Ladybrown.spin(reverse,100,pct);
  wait(200,msec);
  Ladybrown.stop(brake);
  Ladybrown.resetPosition();
  while(true){
  if (autoposition == 1){
  Ladybrown.spinTo(70.5,deg,true);
  }
  else if (autoposition == 2){
    Ladybrown.spinTo(360,deg,true);
    wait(100,msec);
    Ladybrown.spinTo(460,deg,true);
  }
  else Ladybrown.spinTo(0,deg,true);
  }
}
void loweststake(){

}
void ladybrownmacro(){
  Ladybrown.setVelocity(100,pct);


  if(ladybrownposition == 0){
    Ladybrown.spinTo(70.5, degrees, true);
    ladybrownposition = 1;
  }
  else if (ladybrownposition == 1){
    Ladybrown.spinTo(360, degrees, true);
    ladybrownposition = 2;
  }
  else{
     Ladybrown.spinTo(450, degrees, true);                                                                                                                                                                                                                                                                                                                                                                                             
    ladybrownposition = 3;
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

		double kp=7.85;//was 6
    double speed = 0;
    double kd = 0.65;//was 0.3
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
void inchDrive2(float target, float timeLimit, int b =1.5, int c = 0){
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

    drive(speed+ turn_speed ,speed -turn_speed,10);
    last_error = error;
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
    wait(750,msec);
    inchDrive(-7,400);
    gyroTurn(-90);
    inchDrive(-27,1100);
    pneuclamp();
    wait(200,msec);
    gyroTurn(180);
    isRollerSpinningForward = true;
    inchDrive(29,1000);
    wait(100,msec);
    gyroTurn(120);
    wait(200,msec);
    autoposition = 1;
    inchDrive(42, 1600);
    gyroTurn(90);
    wait(500,msec);
    autoposition = 2;

    inchDrive(18,1000);
    wait(100,msec);
    inchDrive(-24,1200);
    autoposition = 0;
    isRollerSpinningForward = true;
    gyroTurn(0);
    inchDrive(48,1500);
    wait(200,msec);
    inchDrive(17,900);
    gyroTurn(135);
    inchDrive(18,1000);
    wait(200,msec);
    gyroTurn(-135);
    pneuclamp();
    inchDrive(-24,1000);
    wait(600,msec);
    isRollerSpinningForward = false;
    inchDrive(12,900);
    gyroTurn(180);
    inchDrive(-24,1200);
    inchDrive(18,1200);
    gyroTurn(90);
    inchDrive(28,1200);
    Gyro.setRotation(90,deg);
    inchDrive2(-80,2500);
    inchDrive(-14,900);
    pneuclamp();
    wait(200,msec);
    isRollerSpinningForward = true;
    gyroTurn(180);
    inchDrive(29,1000);
    gyroTurn(-120);
    wait(300,msec);
    autoposition = 1;
    inchDrive(40,1600);
    gyroTurn(-90);
    wait(300,msec);
    autoposition = 2;
    inchDrive(18,1000);
    wait(100,msec);
    inchDrive(-30,1200);
    autoposition = 0;
    isRollerSpinningForward = true;
    gyroTurn(0);
    inchDrive(50,1500);
    wait(100,msec);
    inchDrive(18,900);
    gyroTurn(-135);
    inchDrive(18,1000);
    gyroTurn(135);
    pneuclamp();
    inchDrive(-24,1000);


    inchDrive(25,1000);
    gyroTurn(180);
    inchDrive(75,1200);
    gyroTurn(120);
    isRollerSpinningForward = false;
    inchDrive(50,1000);
    gyroTurn(45);
    inchDrive(-70,2000);
    inchDrive(35,1200);
    gyroTurn(-90);
    inchDrive(-80,1300);
    pneuclamp();
    isRollerSpinningForward = true;
    gyroTurn(-45);
    inchDrive(-80, 1500);
    pneuclamp();
    inchDrive(15,800);

    










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

if (Controller1.ButtonDown.pressing()){
  if (!(ladybrownposition == 4)){
  Ladybrown.spinTo(500,deg);
  ladybrownposition = 4;
  }
  else if(!(ladybrownposition == 5)){
  Ladybrown.spinTo(560,deg);
  ladybrownposition = 5;
  }   
  wait(150,msec);
}
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