/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      9/29/2024, 2:37:00 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"


using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;
timer gyroTimmer;

//Declare motors
motor FL = motor(PORT18, ratio6_1, true);
motor ML = motor(PORT19, ratio6_1, true);
motor BL = motor(PORT10, ratio6_1, true);
motor FR = motor(PORT7, ratio6_1, false);
motor MR = motor(PORT11, ratio6_1, false);
motor BR = motor(PORT17, ratio6_1, false);
motor roller = motor (PORT19, ratio6_1, false);
motor Ladybrown = motor(PORT9, ratio36_1, false);
digital_out Pneu1 = digital_out(Brain.ThreeWirePort.B);
inertial  Gyro=inertial(PORT12);
digital_out corner = digital_out (Brain.ThreeWirePort.A);

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

float ladybrownposition = 0;
void ladybrownmacro(){
  Ladybrown.setVelocity(100,pct);
  if(ladybrownposition == 0){
    Ladybrown.spinTo(73.5, degrees, true);
    ladybrownposition = 1;
  }
  else if (ladybrownposition == 1){
    Ladybrown.spinTo(360, degrees, true);
    ladybrownposition = 2;
  }
  else{
    Ladybrown.spinTo(480, degrees, true);
    ladybrownposition = 3;
  }
}
void ladybrownrest(){
  Ladybrown.spinTo(0, degrees, true);
  ladybrownposition = 0;
}

  void spinFunction(){
    if(isRollerSpinningForward == true){
      roller.stop(brake);
      isRollerSpinningForward = false;
    }
    else{
      roller.spin(reverse,100, pct);
      isRollerSpinningForward = true;
      isRollerSpinningBackward = false;
    }
  }
  void CornerClear(){
    corner.set(!corner.value());
  }
  void reverseSpinFunction(){
    if(isRollerSpinningBackward == true){
      roller.stop(brake);
      isRollerSpinningBackward = false;
    }
    else{
      roller.spin(forward,100, pct);
      isRollerSpinningForward = false;
      isRollerSpinningBackward = true;
    }  
  }

void pneuclamp(){
  Pneu1.set(!Pneu1.value());
}
void gyroTurn(float target, float b = 16.8){
		float heading=0.0; //initialize a variable for heading
		double accuracy=0.3; //how accurate to make the turn in degrees
		double error=target-heading;
		double kp=1.75;
    double speed = 0;
    double kd = 0.2;
    double last_error = 0;
    double dt = 0.01;
		Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
		int count = 0;
    vex::timer timer;  // Create a timer object

    timer.clear();  // Clear any previous timer value
    int timeLimit = 2500;
		while(fabs(error)>=accuracy or count<=7 ){
      heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;  //calculate error
      
			speed = kp * error + kd * (error - last_error) / dt + b * error / fabs(error);


      drive(speed, -speed, 10); //turn right at speed
      last_error = error;
      if(fabs(error)<=accuracy+0.1){
      count++;
      }
      else count = 0;

      if (timer.time(vex::timeUnits::msec) >= timeLimit ){
        return;
      }
		}
      
			driveBrake();  //stope the drive
}


void inchDrive(float target, float timeLimit, int b =20, int c = 1){
  Gyro.setRotation(0.0, degrees);
  float heading = 0;
  float angle_error = 0;
  float angleP = 1.5;
  float turn_speed = 0;

  float x=0;
  float error=target;
  float kp=4.6;
  float speed =kp*error ;
  float accuracy=0.05;
  float kd = 0.5;
  double last_error = 0;
  double dt = 0.01;
  FL.setPosition(0.0, rev);
    vex::timer timer;  // Create a timer object

    timer.clear(); 
  while(fabs(error)>accuracy){

    heading = Gyro.rotation();
    angle_error = 0-heading;
    x=FL.position(rev)*pi*dia*gearRatio;
    error=target-x;
    speed=kp*error +kd * (error - last_error) / dt + b*error/fabs(error);
    turn_speed = angleP * angle_error + c*angle_error/fabs(angle_error);

    drive(speed+turn_speed,speed-turn_speed,10);
    last_error = error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      return;
    }
  }
  driveBrake();
  Brain.Screen.printAt(10,20,"accuracy %.2f",error);

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
    inchDrive(-21, 2500);
    pneuclamp();
    roller.spin(reverse, 100, pct);
    gyroTurn(-90);
    inchDrive(26,2600);
    

  

  
  //score ring onto mogo next

}



void usercontrol(void) {
  // User control code here, inside the loop
   

  while (true) {
    float lstick = Controller1.Axis3.position();
	  float rstick = Controller1.Axis1.position();

    Brain.Screen.print("Roller efficiency: ");
    Brain.Screen.print(roller.efficiency());
    Brain.Screen.newLine();
    //Spins conveyor belt forward if R1 is pressed, reverse if R2 is pressed

   
    if (Controller1.ButtonA.pressing()){
      pneuclamp();
      wait(150,msec);
    }
    if (Controller1.ButtonB.pressing()){
      CornerClear();
      wait(150,msec);
    }
    Controller1.ButtonL1.pressed(ladybrownmacro);
    Controller1.ButtonL2.pressed(ladybrownrest);
    Controller1.ButtonR1.pressed(spinFunction);
    Controller1.ButtonR2.pressed(reverseSpinFunction);
    //drive(lstick + rstick , lstick + rstick, 20);
    wait(20, msec);
  }
}



// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Ladybrown.resetPosition();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
