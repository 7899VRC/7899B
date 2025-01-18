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
motor FL = motor(PORT9, ratio6_1, false);
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

void intakecontrol(){
  while(true){
  int colorTolerance = 223;
  if (abs(colorsensor.value() - colortosort) > colorTolerance and stop == false) {
    roller.stop(brake);
    stop = true;
  }
  else if (isRollerSpinningForward){
    roller.spin(reverse,90,pct);
  }
  else if (isRollerSpinningBackward){
    roller.spin(fwd,90,pct);
  }
  else roller.stop(brake);
  
  if(stop){
    wait(100,msec);
    stop = false;

  }
  }
}
float ladybrownposition = 0;
float autoposition = 0;
void ladybrownAuto(){
  autoposition = 0;
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
void gyroTurn(float target, float b = 2.4){
		float heading=0.0; //initialize a variable for heading
		double accuracy=0.3; //how accurate to make the turn in degrees
		double error=target-heading;
		double kp=6;
    double speed = 0;
    double kd = 0.3;
    double last_error = 0;
    double dt = 0.01; //reset Gyro to zero degrees
		int count = 0;
    vex::timer timer;  // Create a timer object

    timer.clear();  // Clear any previous timer value
    int timeLimit = 1600;
		while(fabs(error)>=accuracy or count<=7 ){
      heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;
      if (error > 180){
        error = error - 360;
      }
      else if (error < -180){
        error= error + 360;
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
  float angleP = 0.5;
  float turn_speed = 0;
  ML.setPosition(0,rev);
  float x=0;
  float error=target;
  float kp=6;
  float speed =kp*error;
  float accuracy=0.1;//was 0.05
  float kd = 0.35;
  double last_error = 0;
  double dt = 0.01;
  double last_speed = 0;
  vex::timer timer;  // Create a timer object
  timer.clear(); 
  while(fabs(error)>=accuracy){
    heading = Gyro.rotation();
    if (target > 180){
      target - 360;
    }
    else if (target < -180){
      target + 360;
    }
    x=ML.position(rev)*pi*dia*gearRatio;
    error=target-x;
    angle_error = 0-heading;
    speed=kp*error +kd * (error - last_error) / dt + b*error/fabs(error);
    turn_speed = angleP * angle_error + c*angle_error/fabs(angle_error);
    if (speed >= 100){
      speed = 100;
    }
    else if (speed <=-100){
      speed = -100;
    }

    drive(speed ,speed,10);
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
    roller.spin(reverse,100,pct);
    isRollerSpinningForward = true;
    wait(500,msec);
    inchDrive(13,900);
    gyroTurn(90);
    inchDrive(-29,1200);
    pneuclamp();
    wait(200,msec);
    gyroTurn(0);
    inchDrive(29,1000);
    wait(100,msec);
    gyroTurn(-60);
    autoposition = 1;
    inchDrive(45, 1600);
    gyroTurn(-90);
    autoposition = 2;
    inchDrive(14,1000);
    wait(100,msec);
    inchDrive(-28,1200);
    autoposition = 0;
    gyroTurn(180);
    inchDrive(50,1500);
    wait(100,msec);
    inchDrive(13,900);
    gyroTurn(-45);
    inchDrive(18,1000);
    gyroTurn(45);
    pneuclamp();
    inchDrive(-24,1000);
    wait(600,msec);
    inchDrive(17,900);
    gyroTurn(-90);
    // inchDrive(-100,3000);
    // pneuclamp();
    // gyroTurn(0);
    // inchDrive(29,1000);






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
