/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       georgekirkman                                             */
/*    Created:      9/19/2024, 6:25:11 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftFront = motor(PORT16, ratio18_1, true);
motor LeftBack = motor(PORT12, ratio18_1, false);
motor RightFront = motor(PORT3, ratio18_1, false);
motor RightBack = motor(PORT9, ratio18_1, true);
motor LeftMid = motor(PORT7, ratio18_1, false);
motor RightMid = motor(PORT10, ratio18_1, true);

//----------Custom Functions----------
int YOFFSET = 20;
void MotorDisplay(double y, double curr, double temp) {
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
  if (curr < 1)
    Brain.Screen.setFillColor(green);
  else if (curr >= 1 && curr <= 2.5)
    Brain.Screen.setFillColor(yellow);
  else
    Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);

  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
  if (temp < 45)
    Brain.Screen.setFillColor(green);
  else if (temp <= 50 && temp >= 45)
    // TRUE and TRUE --> True
    // TRUE and FALSE --> False
    // FALSE and FALSE --> False
    Brain.Screen.setFillColor(yellow);
  else
    Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
  Brain.Screen.setFillColor(transparent);
}

// Displays information on the brain
void Display() {
  double leftFrontCurr = LeftFront.current(amp);
  double leftFrontTemp = LeftFront.temperature(celsius);
  double leftBackCurr = LeftBack.current(amp);
  double leftBackTemp = LeftBack.temperature(celsius);
  double leftMidCurr = LeftMid.current(amp);
  double leftMidTemp = LeftMid.temperature(celsius);
  double rightFrontCurr = RightFront.current(amp);
  double rightFrontTemp = RightFront.temperature(celsius);
  double rightBackCurr = RightBack.current(amp);
  double rightBackTemp = RightBack.temperature(celsius);
  double rightMidCurr = RightMid.current(amp);
  double rightMidTemp = RightMid.temperature(celsius);

  if (LeftFront.installed()) {
    MotorDisplay(1, leftFrontCurr, leftFrontTemp);
    Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
  } else
    Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront Problem");

  if (LeftBack.installed()) {
    MotorDisplay(31, leftBackCurr, leftBackTemp);
    Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
  } else
    Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack Problem");

  if (RightFront.installed()) {
    MotorDisplay(61, rightFrontCurr, rightFrontTemp);
    Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
  } else
    Brain.Screen.printAt(5, YOFFSET + 61, "RightFront Problem");

  if (RightBack.installed()) {
    MotorDisplay(91, rightBackCurr, rightBackTemp);
    Brain.Screen.printAt(300, YOFFSET + 91, "RightBack");
  } else
    Brain.Screen.printAt(5, YOFFSET + 91, "RightBack Problem");

  if (LeftMid.installed()) {
    MotorDisplay(31, leftMidCurr, leftMidTemp);
    Brain.Screen.printAt(300, YOFFSET + 121, "LeftMid");
  } else
    Brain.Screen.printAt(5, YOFFSET + 121, "LeftMid Problem");
  if (RightMid.installed()) {
    MotorDisplay(31, rightMidCurr, rightMidTemp);
    Brain.Screen.printAt(300, YOFFSET + 151, "RightMid");
  } else
    Brain.Screen.printAt(5, YOFFSET + 151, "RightMid Problem");
}
void DriveVolts(double lspeed, double rspeed, int wt) {
  lspeed = lspeed * 120;
  rspeed = rspeed * 120;
  LeftFront.spin(forward, lspeed, voltageUnits::mV);
  LeftMid.spin(forward, lspeed, voltageUnits::mV);
  LeftBack.spin(forward, lspeed, voltageUnits::mV);
  RightFront.spin(forward, rspeed, voltageUnits::mV);
  RightMid.spin(forward, rspeed, voltageUnits::mV);
  RightBack.spin(forward, rspeed, voltageUnits::mV);
  task::sleep(wt);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    while (true) {
    DriveVolts(Controller1.Axis3.position(), Controller1.Axis2.position(), 10);
    Display();
    wait(20, msec);
  }
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
