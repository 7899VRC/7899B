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
motor FL = motor(PORT18, ratio6_1, true);
motor ML = motor(PORT19, ratio6_1, true);
motor BL = motor(PORT10, ratio6_1, true);
motor FR = motor(PORT7, ratio6_1, false);
motor MR = motor(PORT11, ratio6_1, false);
motor BR = motor(PORT17, ratio6_1, false);
motor roller2 = motor(PORT2, ratio6_1,false);
motor roller = motor (PORT1, ratio6_1, false);
digital_out Pneu1 = digital_out(Brain.ThreeWirePort.C);
inertial  Gyro=inertial(PORT12);
digital_out corner = digital_out (Brain.ThreeWirePort.D);


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

void tempDisplay (int offset){
  int temp = MR.temperature();
  if (temp>=55) Brain.Screen.setFillColor(red);
  else if (temp>=45) Brain.Screen.setFillColor(yellow);
  else Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(1,offset - 20,25,25);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(30,offset," %d degrees C", temp);
  
}

//global variables 
float pi = 3.14; 
float dia = 2.75;
float gearRatio = 0.75; 

void gyroTurn(float target){
  
		float heading=0.0; //initialize a variable for heading
		float accuracy=0.7; //how accurate to make the turn in degrees
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
      roller2.stop(brake);
      isRollerSpinningForward = false;
    }
    else{
      roller.spin(reverse,100, pct);
      roller2.spin(reverse,100,pct);
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
      roller2.stop(brake);
      isRollerSpinningBackward = false;
    }
    else{
      roller.spin(forward,100, pct);
      roller2.spin(forward,100,pct);
      isRollerSpinningForward = false;
      isRollerSpinningBackward = true;
    }  
  }

void pneuclamp(){
  Pneu1.set(!Pneu1.value());
}

void inchDrive(float target, int speed){ 
  Gyro.setRotation(0.0, degrees);
  float heading = 0;
  float angle_error = 0;
  float angleP = 0.25;
  float turn_speed = 0;
  int c = 1;

  float x = 0; 
  FL.setPosition(0, rev); 
  x = FL.position(rev)*dia*pi*gearRatio; 

  if (target >= 0 ){ //if your target is greater than 0 we will drive forward
  while (x <= target ) { 
    x = FL.position(rev)*dia*pi*gearRatio; 
    Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
    heading = Gyro.rotation();
    angle_error = 0-heading;
    turn_speed = angleP * angle_error + c*angle_error/fabs(angle_error);
    drive(speed + turn_speed, speed - turn_speed, 10); 

  }
  }
  else if (target <0){ 
    while (x <=fabs(target)){ //target less than 0 the robot will drive backward

      x = -FL.position(rev)*dia*pi*gearRatio;
      Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
      heading = Gyro.rotation();
      angle_error = 0-heading;
      turn_speed = angleP * angle_error + c*angle_error/fabs(angle_error);
      drive(-speed + turn_speed, -speed - turn_speed, 10); 


    }
  }

  driveBrake();

}
void motorDisplay(int offset){
  float vM=0.0;
  float speed=0.0;
  float current=0.0;
  vM=MR.voltage(voltageUnits::volt);
  speed=MR.velocity(rpm);
  current=MR.current(amp);
  Brain.Screen.printAt(1,offset," %0.1f V, %0.1f rpm,  %0.2f  Amps", vM, speed, current);
}
void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
void stallMotorTest(){
  int offset=20;
  float speed=20;
  Brain.Screen.clearScreen();
  // User control code here, inside the loop
  while (speed<100) {
    speed=speed+20;
    offset=offset+20;
    drive(speed, speed, 0);
    wait(500,msec);
    motorDisplay(offset);
  }
  driveBrake();
  tempDisplay(offset+25);
  Brain.Screen.printAt(1,180,"Done  push X on the remote to run Test");
  Brain.Screen.printAt(1,220,"Done  push Y on the remote to run stall Test");
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
  inchDrive(-8, 40);
  gyroTurn(40);
  inchDrive(-15, 40);
  wait(100,msec);
  pneuclamp();
  roller2.spin(reverse,100,pct);
  roller.spin(reverse,100,pct);
  gyroTurn(50);
  inchDrive(12, 40);
  gyroTurn(90);
  inchDrive(14, 40);
  wait(300,msec);
  inchDrive(-10, 40);
  gyroTurn(-20);
  inchDrive(14.5, 40);
  wait(300,msec);
  inchDrive(-35, 50);
  gyroTurn(100);
  roller2.spin(fwd,100,pct);
  roller.spin(fwd,100,pct);
  inchDrive(60, 60);     
  // inchDrive(4, 40);
  // gyroTurn(-100);
  // inchDrive(-25, 40);
  // pneuclamp();
  // gyroTurn(-90);
  // inchDrive(12, 50);
  // gyroTurn(180);
  // inchDrive(20, 50);
    
  
 




}



void usercontrol(void) {
  // User control code here, inside the loop
  Controller1.ButtonY.pressed(stallMotorTest);
   

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
      wait(150,msec);
    }
    if (Controller1.ButtonB.pressing()){
      CornerClear();
      wait(150,msec);
    }
    Controller1.ButtonY.pressed(stallMotorTest);

    drive(lstick + (rstick*0.7) , lstick + (rstick*-0.7), 20);
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
