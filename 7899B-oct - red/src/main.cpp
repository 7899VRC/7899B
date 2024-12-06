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

//Declare motors
motor FL = motor(PORT11, ratio6_1, true);
motor ML = motor(PORT1, ratio6_1, true);
motor BL = motor(PORT12, ratio6_1, true);
motor FR = motor(PORT7, ratio6_1, false);
motor MR = motor(PORT9, ratio6_1, false);
motor BR = motor(PORT20, ratio6_1, false);
motor roller = motor (PORT21, ratio6_1, false);
digital_out Pneu1 = digital_out(Brain.ThreeWirePort.E);
inertial  Gyro=inertial(PORT2);


void drive (int lspeed ,int rspeed,int wt){
  FL.spin(forward, lspeed, pct);
  ML.spin(forward, lspeed, pct);
  BL.spin(forward, lspeed, pct);
  FR.spin(forward, rspeed, pct);
  MR.spin(forward, rspeed, pct);
  BR.spin(forward, rspeed,pct);
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

void gyroTurn(float target){
  
		float heading=0.0; //initialize a variable for heading
		float accuracy=0.5; //how accurate to make the turn in degrees
		float error=target-heading;
		float kp=0.7;
		float speed=kp*error;
    int  cnt= 0;
		Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
		
		while(fabs(error)>=accuracy){
			speed=kp*error;
			drive(speed, -speed, 10); //turn right at speed
			heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;  //calculate error
   
      if(fabs(error)<accuracy){
         cnt++;
      }
      else {
        cnt=0;
      }
		}

		driveBrake();  //stope the drive
}


bool isRollerSpinningForward = false;
bool isRollerSpinningBackward = false;

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

void inchDrive(float target){ 

  float x = 0; 
  FL.setPosition(0, rev); 
  x = FL.position(rev)*dia*pi*gearRatio; 

  if (target >= 0 ){ //if your target is greater than 0 we will drive forward
  while (x <= target ) { 
    drive(60, 60, 10); 
    x = FL.position(rev)*dia*pi*gearRatio; 
    Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  }
  }
  else if (target <0){ 
    while (x <=fabs(target)){ //target less than 0 the robot will drive backward
      drive(-60, -60, 10); 
      x = -FL.position(rev)*dia*pi*gearRatio;
      Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 

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
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  inchDrive(-37);
  wait(100,msec);
  gyroTurn(40);
  inchDrive(-5);
  pneuclamp();
  inchDrive(10);
  gyroTurn(-90);
  roller.spin(reverse,100,pct);
  wait(300,msec);
  inchDrive(-2);
  pneuclamp();
  gyroTurn(-20); 
  inchDrive(8);
  
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
    Controller1.ButtonR1.pressed(spinFunction);
    Controller1.ButtonR2.pressed(reverseSpinFunction);

    if (Controller1.ButtonA.pressing()){
      pneuclamp();
      wait(1000,msec);
    }
    

    drive(lstick + rstick , lstick + (rstick*-1), 20);
    wait(20, msec);

  }
}



// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
 