#include "algorithm"
#include "math.h"
#include "robot-config.h"

using namespace vex;
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                    */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// Creates a competition object that allows access to Competition methods.
vex::competition Competition;

// A minimum velocity is necessary. This could also be done per command instead
// of globally.
double minimum_velocity = 40.0;

// This function provides an increasing speed as the robot moves away from start
double increasing_speed(double starting_point, double current_position) {
  static const double acceleration_constant = 30.0;
  return acceleration_constant * std::abs(current_position - starting_point) +
         minimum_velocity;
}

// This function provides a decreasing speed as the robot approaches the end
double decreasing_speed(double ending_point, double current_position) {
  static const double deceleration_constant = 30.0;
  return deceleration_constant * std::abs(ending_point - current_position) +
         minimum_velocity;
}

// This function takes a distance, a maximum velocity, and tries to send the
// robot in a straight line for that distance using a trapezoidal motion profile
// controlled by increasing_speed, decreasing_speed, and maxVelocity
void forward(double distanceIn, double maxVelocity) {
  // record nominal wheel circumference
  static const double circumference = 3.14159 * 4;

  // if we've got a joker on our hands, punch out
  if (distanceIn == 0)
    return;

  // figure out which direction we're supposed to be going
  double direction = distanceIn > 0 ? 1.0 : -1.0;

  // using circumference and commanded inches, convert to revolutions
  double wheelRevs = distanceIn / circumference;

  // set the motors to do a position move with 0 velocity
  //(this is just a cheatyface way to make them stop when arriving at target)
  rf.spin(directionType::fwd, direction * minimum_velocity, velocityUnits::pct);
  lf.spin(directionType::fwd, direction * minimum_velocity, velocityUnits::pct);
  rb.spin(directionType::fwd, direction * minimum_velocity, velocityUnits::pct);
  lb.spin(directionType::fwd, direction * minimum_velocity, velocityUnits::pct);

  // record starting positions and ending positions
  double leftStartPoint = lf.rotation(rotationUnits::rev);
  double leftEndPoint = leftStartPoint + wheelRevs;
  double rightStartPoint = rf.rotation(rotationUnits::rev);
  double rightEndPoint = rightStartPoint + wheelRevs;

  // Back Motors
  double leftBStartPoint = lb.rotation(rotationUnits::rev);
  double leftBEndPoint = leftBStartPoint + wheelRevs;
  double rightBStartPoint = rb.rotation(rotationUnits::rev);
  double rightBEndPoint = rightBStartPoint + wheelRevs;

  // execute motion profile
  while ((direction * (rf.rotation(rotationUnits::rev) - rightStartPoint) <
          direction * wheelRevs) ||
         (direction * (lf.rotation(rotationUnits::rev) - leftStartPoint) <
          direction * wheelRevs) ||
         (direction * (lb.rotation(rotationUnits::rev) - leftBStartPoint) <
          direction * wheelRevs) ||
         (direction * (rb.rotation(rotationUnits::rev) - rightBStartPoint) <
          direction * wheelRevs)) {

    // set right motor speed to minimum of increasing function, decreasing
    // function, and max velocity, based on current position
    if (direction * (rf.rotation(rotationUnits::rev) - rightStartPoint) <
        direction * wheelRevs) {
      rf.setVelocity(
          direction *
              std::min(
                  maxVelocity,
                  std::min(increasing_speed(rightStartPoint,
                                            rf.rotation(rotationUnits::rev)),
                           decreasing_speed(rightEndPoint,
                                            rf.rotation(rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      rf.stop(brakeType::brake);
    }

    // do the same for the left motor
    if (direction * (lf.rotation(rotationUnits::rev) - leftStartPoint) <
        direction * wheelRevs) {
      lf.setVelocity(
          direction *
              std::min(
                  maxVelocity,
                  std::min(increasing_speed(leftStartPoint,
                                            lf.rotation(rotationUnits::rev)),
                           decreasing_speed(leftEndPoint,
                                            lf.rotation(rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      lf.stop(brakeType::brake);
    }

    if (direction * (lb.rotation(rotationUnits::rev) - leftBStartPoint) <
        direction * wheelRevs) {
      lb.setVelocity(
          direction *
              std::min(
                  maxVelocity,
                  std::min(increasing_speed(leftBStartPoint,
                                            lb.rotation(rotationUnits::rev)),
                           decreasing_speed(leftBEndPoint,
                                            lb.rotation(rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      lb.stop(brakeType::brake);
    }

    if (direction * (rb.rotation(rotationUnits::rev) - rightBStartPoint) <
        direction * wheelRevs) {
      rb.setVelocity(
          direction *
              std::min(
                  maxVelocity,
                  std::min(increasing_speed(rightBStartPoint,
                                            rb.rotation(rotationUnits::rev)),
                           decreasing_speed(rightBEndPoint,
                                            rb.rotation(rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      rb.stop(brakeType::brake);
    }
  }
}

void forward(double distanceIn) {
  // no max velocity specified, call the version that uses it with max velocity
  // of 100%
  forward(distanceIn, 100.0);
}

void stopH() {
  lf.stop(brakeType::hold);
  lb.stop(brakeType::hold);
  rf.stop(brakeType::hold);
  rb.stop(brakeType::hold);
}

void stopB() {
  lf.stop(brakeType::brake);
  lb.stop(brakeType::brake);
  rf.stop(brakeType::brake);
  rb.stop(brakeType::brake);
}

void stopC() {
  lf.stop(brakeType::coast);
  lb.stop(brakeType::coast);
  rf.stop(brakeType::coast);
  rb.stop(brakeType::coast);
}

void turnR(int tdistance, int tspeed, int twait) {
  vex::task::sleep(twait);
  lb.resetRotation();
  vex::task::sleep(150);

  while (abs(lb.rotation(vex::rotationUnits::deg)) < tdistance) {
    lf.spin(vex::directionType::fwd, tspeed, vex::velocityUnits::rpm);
    lb.spin(vex::directionType::fwd, tspeed, vex::velocityUnits::rpm);
    rf.spin(vex::directionType::fwd, -tspeed, vex::velocityUnits::rpm);
    rb.spin(vex::directionType::fwd, -tspeed, vex::velocityUnits::rpm);
  }
  lf.stop(vex::brakeType::coast);
  lb.stop(vex::brakeType::coast);
  rf.stop(vex::brakeType::coast);
  rb.stop(vex::brakeType::coast);
}

void turnL(int tdistance, int tspeed, int twait) {
  vex::task::sleep(twait);
  lb.resetRotation();
  vex::task::sleep(150);

  while (abs(lb.rotation(vex::rotationUnits::deg)) < tdistance) {
    lf.spin(vex::directionType::fwd, -tspeed, vex::velocityUnits::rpm);
    lb.spin(vex::directionType::fwd, -tspeed, vex::velocityUnits::rpm);
    rf.spin(vex::directionType::fwd, tspeed, vex::velocityUnits::rpm);
    rb.spin(vex::directionType::fwd, tspeed, vex::velocityUnits::rpm);
  }
  lf.stop(vex::brakeType::coast);
  lb.stop(vex::brakeType::coast);
  rf.stop(vex::brakeType::coast);
  rb.stop(vex::brakeType::coast);
}

// void gyroL(int tdistance, int tspeed, int twait) {
//     vex::task::sleep(twait);
//     Gyro.startCalibration();
//     vex::task::sleep(1300);

//     while (Gyro.value(rotationUnits::deg) > tdistance) {
//         lf.spin(vex::directionType::fwd, -tspeed,vex::velocityUnits::rpm);
//         lb.spin(vex::directionType::fwd,-tspeed,vex::velocityUnits::rpm);
//         rf.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
//         rb.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
//     }

//     lf.stop(vex::brakeType::coast);
//     lb.stop(vex::brakeType::coast);
//     rf.stop(vex::brakeType::coast);
//     rb.stop(vex::brakeType::coast);
// }

// void gyroR(int tdistance, int tspeed, int twait) {
//     vex::task::sleep(twait);
//     Gyro.startCalibration();
//     vex::task::sleep(1300);

//     while (Gyro.value(rotationUnits::deg) < tdistance) {
//         lf.spin(vex::directionType::fwd, tspeed,vex::velocityUnits::rpm);
//         lb.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
//         rf.spin(vex::directionType::fwd,- tspeed,vex::velocityUnits::rpm);
//         rb.spin(vex::directionType::fwd,- tspeed,vex::velocityUnits::rpm);
//     }

//     lf.stop(vex::brakeType::coast);
//     lb.stop(vex::brakeType::coast);
//     rf.stop(vex::brakeType::coast);
//     rb.stop(vex::brakeType::coast);
// }

void fwStart(int speed) {
  fw.spin(directionType::fwd, speed, velocityUnits::rpm);
  fw2.spin(directionType::fwd, speed, velocityUnits::rpm);
}

void fwChange(int speed) {
  fw.setVelocity(speed, velocityUnits::rpm);
  fw2.setVelocity(speed, velocityUnits::rpm);
}

void fwStop() { fw.stop(); }

void armUp() {
  arm.rotateFor(0.23, rotationUnits::rev, 200, velocityUnits::rpm);
  arm.stop(brakeType::hold);
}

void armDown() {
  arm.rotateFor(-0.23, rotationUnits::rev, 200, velocityUnits::rpm);
  arm.stop(brakeType::hold);
}

void intake(int time, int speed) {
  in.spin(directionType::rev, -speed, velocityUnits::rpm);
  task::sleep(time);
}

void outake(int time, int speed) {
  in.spin(directionType::rev, speed, velocityUnits::rpm);
  task::sleep(time);
}

void intakeStop() { in.stop(); }

void intakeForever() { in.spin(directionType::rev, -200, velocityUnits::rpm); }

void reset(int totalTime, int speed) {
  int seventyPercent = (totalTime / 10) * 7;
  int thirtyPercent = (totalTime / 10) * 7;

  lf.spin(directionType::fwd, -speed, velocityUnits::rpm);
  rf.spin(directionType::fwd, -speed, velocityUnits::rpm);
  lb.spin(directionType::fwd, -speed, velocityUnits::rpm);
  rb.spin(directionType::fwd, -speed, velocityUnits::rpm);

  task::sleep(seventyPercent);

  /*
  stopC();

  task::sleep(thirtyPercent);
  */
}

int correct() {
  if (lf.rotation(vex::rotationUnits::deg) -
          rf.rotation(vex::rotationUnits::deg) >
      0) {
    while (lf.rotation(vex::rotationUnits::deg) -
               rf.rotation(vex::rotationUnits::deg) >
           0) {
      lf.spin(vex::directionType::fwd, -15, vex::velocityUnits::rpm);
      lb.spin(vex::directionType::fwd, -15, vex::velocityUnits::rpm);
      rf.spin(vex::directionType::fwd, 15, vex::velocityUnits::rpm);
      rb.spin(vex::directionType::fwd, 15, vex::velocityUnits::rpm);
    }
    lf.stop(vex::brakeType::coast);
    lb.stop(vex::brakeType::coast);
    rf.stop(vex::brakeType::coast);
    rb.stop(vex::brakeType::coast);
  }

  if (lf.rotation(vex::rotationUnits::deg) -
          rf.rotation(vex::rotationUnits::deg) <
      0) {
    while (lf.rotation(vex::rotationUnits::deg) -
               rf.rotation(vex::rotationUnits::deg) <
           0) {
      lf.spin(vex::directionType::fwd, 15, vex::velocityUnits::rpm);
      lb.spin(vex::directionType::fwd, 15, vex::velocityUnits::rpm);
      rf.spin(vex::directionType::fwd, -15, vex::velocityUnits::rpm);
      rb.spin(vex::directionType::fwd, -15, vex::velocityUnits::rpm);
    }
    lf.stop(vex::brakeType::coast);
    lb.stop(vex::brakeType::coast);
    rf.stop(vex::brakeType::coast);
    rb.stop(vex::brakeType::coast);
  }
  return (0);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
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
  fwStart(200);
  armUp();

  task::sleep(10);
  forward(38.0, 80.0);
  stopH();

  in.spin(directionType::fwd, -100, velocityUnits::rpm);

  task::sleep(300);

  in.stop();

  forward(-32.1, 80.0);
  stopH();

  turnL(250, 70, 25);

  in.spin(directionType::fwd, 200, velocityUnits::rpm);

  task::sleep(100);

  in.spin(directionType::fwd, -200, velocityUnits::rpm);

  task::sleep(400);

  in.stop();

  forward(24.5, 80.0);
  stopH();

  in.spin(directionType::fwd, -200, velocityUnits::rpm);

  task::sleep(300);

  forward(4.5, 50);
  stopH();

  forward(-50, 80);
  stopH();

  turnR(260, 70, 25);

  forward(-3.5, 80);
  stopH();

  forward(52.5, 80);
  stopH();
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

void pivotLeft(int tamount) {
  lf.rotateFor(directionType::rev, tamount, rotationUnits::deg, false);
  lb.rotateFor(directionType::rev, tamount, rotationUnits::deg);
}

void pivotRight(int tamount) {
  rf.rotateFor(directionType::rev, tamount, rotationUnits::deg, false);
  rb.rotateFor(directionType::rev, tamount, rotationUnits::deg);
}

void alignRedP1() {
  Brain.Screen.print("test");
  fwChange(450);
  pivotLeft(210);
}

void alignRedP2() {
  fwChange(370);
  pivotLeft(210);
}

void alignRedP3() {
  fwChange(450);
  pivotLeft(110);
}

void alignRedP4() {
  fwChange(360);
  pivotLeft(110);
}

void alignBlueP1() {
  fwChange(450);
  pivotRight(210);
}

void alignBlueP2() {
  fwChange(370);
  pivotRight(210);
}

void alignBlueP3() {
  fwChange(450);
  pivotRight(110);
}

void alignBlueP4() {
  fwChange(360);
  pivotRight(110);
}

void usercontrol(void) {
  while (1) {
    int fwSpeed = 500;

    while (true) {
      // Second Controller

      // Flywheel Speed Settings
      if (Controller2.ButtonX.pressing()) {
        fwSpeed = 57;
      } else if (Controller2.ButtonA.pressing()) {
        fwSpeed = 100;
      } else {
        fwSpeed = 500;
      }

      Controller2.ButtonL1.pressed(alignRedP3);
      Controller2.ButtonL2.pressed(alignRedP4);

      Controller2.ButtonR1.pressed(alignBlueP1);
      Controller2.ButtonR2.pressed(alignBlueP2);

      // First Controller

      // Regular Driver Control
      lf.spin(vex::directionType::fwd,
              (Controller1.Axis3.value() + Controller1.Axis1.value()),
              vex::velocityUnits::pct);
      rf.spin(vex::directionType::fwd,
              (Controller1.Axis3.value() - Controller1.Axis1.value()),
              vex::velocityUnits::pct);
      lb.spin(vex::directionType::fwd,
              (Controller1.Axis3.value() + Controller1.Axis1.value()),
              vex::velocityUnits::pct);
      rb.spin(vex::directionType::fwd,
              (Controller1.Axis3.value() - Controller1.Axis1.value()),
              vex::velocityUnits::pct);

      if (Controller1.ButtonR1.pressing()) {
        in.spin(directionType::rev, 600, velocityUnits::rpm);
      } else if (Controller1.ButtonR2.pressing()) {
        in.spin(directionType::fwd, 600, velocityUnits::rpm);
      } else {
        in.stop();
      }

      if (Controller1.ButtonL1.pressing()) {
        arm.spin(directionType::fwd, 100, velocityUnits::rpm);
      } else if (Controller1.ButtonL2.pressing()) {
        arm.spin(directionType::rev, 100, velocityUnits::rpm);
      } else {
        arm.stop();
      }

      Controller1.ButtonUp.released(alignRedP1);
      Controller1.ButtonDown.pressed(alignRedP2);

      Controller1.ButtonX.pressed(alignBlueP3);
      Controller1.ButtonB.pressed(alignBlueP4);

      fw.spin(directionType::fwd, fwSpeed, velocityUnits::rpm);
      fw2.spin(directionType::fwd, fwSpeed, velocityUnits::rpm);
    }

    vex::task::sleep(20); // Sleep the task for a short amount of time to
                          // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Run the pre-autonomous function.
  pre_auton();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (1) {
    vex::task::sleep(100); // Sleep the task for a short amount of time to
                           // prevent wasted resources.
  }
}