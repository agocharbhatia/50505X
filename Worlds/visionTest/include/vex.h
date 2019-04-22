/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v5.h"
#include "v5_vcs.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

vex::brain Brain;

vex::competition Competition;

//Drive

//LeftFront
vex::motor lf = vex::motor(vex::PORT3, false);
//LeftBack1
vex::motor lb = vex::motor(vex::PORT11, false);

//RightFront
vex::motor rf = vex::motor(vex::PORT9, true);
//RightBack
vex::motor rb = vex::motor(vex::PORT20, true);

//Flywheel
vex::motor fw = vex::motor(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor fw2 = vex::motor(vex::PORT2, vex::gearSetting::ratio6_1, true);

//Intake
vex::motor in = vex::motor(vex::PORT10, vex::gearSetting::ratio6_1, false);

//Descorer/Flipper
vex::motor arm = vex::motor(vex::PORT19, vex::gearSetting::ratio36_1, false);

//Gyro
vex::gyro Gyro = vex::gyro(Brain.ThreeWirePort.G);

//Sonar/Distance
vex::sonar Sonar = vex::sonar(Brain.ThreeWirePort.A);

//Line Sensor
vex::line Line = vex::line(Brain.ThreeWirePort.C);

//Vision
vex::vision Vision (vex::PORT3, 50);

vex::controller Controller1 = vex::controller(vex::controllerType::primary);
vex::controller Controller2 = vex::controller(vex::controllerType::partner);
