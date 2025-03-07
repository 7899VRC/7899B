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
float team = 1;




bool isRollerSpinningForward = false;
bool isRollerSpinningBackward = false;
int colortosort = 130;
bool stop = false;
int stuckcount = 0;
bool prev_spinning = 0;
int justStartingCnt = 0;
float ladybrownposition = 0;
bool on = true;
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
  if ((abs(c) - colortosort) < colorTolerance&& on == true){
    wait(180,msec);
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

  
  else {

  int colortosort = 200;

  int colorTolerance = 25;
  Brain.Screen.printAt(1,20,"hue = %d  ",c);
  if (abs(c - colortosort) < colorTolerance&& on == true){

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
  Ladybrown.spinTo(570,deg);
  wait(400,msec);
  Ladybrown.spinTo(0,deg);
  Ladybrown.spin(reverse,100,pct);
  wait(200,msec);
  Ladybrown.stop(brake);
  Ladybrown.resetPosition();

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
  void doinkerauto(){
    while (true){
        wait(780,msec);
        CornerClear();
        wait(1000,msec);
        CornerClear();
        wait(10000000,sec);
    }
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
    double dt = 0.01;
		Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
		int count = 0;
    vex::timer timer;  // Create a timer object

    timer.clear();  // Clear any previous timer value
    int timeLimit = 1200;
		while(fabs(error)>=accuracy or count<=7 ){
      heading=Gyro.rotation();  //measure the heading of the robot
			error=target-heading;  //calculate error
      
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
  Gyro.setRotation(0.0, degrees);
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
  vex::timer timer;  // Create a timer object
  timer.clear(); 
  while(fabs(error)>accuracy){
    heading = Gyro.rotation();
    x=ML.position(rev)*pi*dia*gearRatio;
    error=target-x;
    angle_error = 0-heading;
    speed=kp*error +kd * (error - last_error) / dt + b*error/fabs(error);
    turn_speed = angleP * angle_error + c*angle_error/fabs(angle_error);

    drive(speed + turn_speed,speed- turn_speed,10);
    last_error = error;
    angle_last_error = angle_error;
    if (timer.time(vex::timeUnits::msec) >= timeLimit ){
      break;
    } 
  }
  driveBrake();

}
void arcturn(float target, float arcdegree, float timeLimit, int b =1.5, int c = 0){
  Gyro.setRotation(0.0, degrees);
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
  vex::timer timer;  // Create a timer object
  timer.clear(); 
  while(fabs(error)>accuracy){
    heading = Gyro.rotation();
    x=ML.position(rev)*pi*dia*gearRatio;

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
    on = true;
  colorsensor.brightness(255);
    roller.setMaxTorque(90,pct);
    Ladybrown.setVelocity(100,pct);
    vex::thread intakeControlThread(intakecontrol);
    vex::thread doinker(doinkerauto);
    arcturn(44.5,-13,1200);
    arcturn(-44.5,13,1200);
    gyroTurn(165);
    inchDrive(-25,900);
    inchDrive(-8,500);
    pneuclamp();
    wait(200,msec);
    isRollerSpinningForward = true;
    inchDrive(15,500);
    gyroTurn(180);
    pneuclamp();
    gyroTurn(-25);
    inchDrive(35,900);
    isRollerSpinningForward = false;
    gyroTurn(-25);
    inchDrive(-34,900);
    inchDrive(-13,500);
    pneuclamp();
    isRollerSpinningForward = true;
    wait(200,msec);
    arcturn(12,-90,1000);
    inchDrive(9,600);
    CornerClear();
    arcturn(45,90,1000);
    gyroTurn(135);
    inchDrive(30,1000);

    
    






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
    if (Controller1.ButtonY.pressing()){
      on = !on;
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
  // Set up callbacks for autonomous and driver control periods.
  Ladybrown.resetPosition();
  colorsensor.brightness(255);
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
