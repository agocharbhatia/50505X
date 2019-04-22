#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
using namespace vex;
vex::brain Brain;
vex::motor Leftfront (vex::PORT1, vex::gearSetting::ratio18_1,false);
vex::motor Rightfront (vex::PORT2, vex::gearSetting::ratio18_1,true);
vex::motor Rightpuncher (vex::PORT3, vex::gearSetting::ratio36_1,false);
vex::motor Flipper (vex::PORT4, vex::gearSetting::ratio18_1,false);
vex::motor Leftpuncher (vex::PORT8, vex::gearSetting::ratio36_1,false);
vex::motor Leftback (vex::PORT9, vex::gearSetting::ratio18_1,false);
vex::motor Rightback (vex::PORT10, vex::gearSetting::ratio18_1,true);
vex::gyro Gyro1( Brain.ThreeWirePort.C);
vex::controller Controller1;