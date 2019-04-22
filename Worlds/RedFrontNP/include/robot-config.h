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
//vex::gyro gyro = vex::gyro(vex::triport::);

//Vision
vex::vision::signature SIG_1 (1, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_2 (2, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision (vex::PORT3, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

vex::controller Controller1 = vex::controller(vex::controllerType::primary);
vex::controller Controller2 = vex::controller(vex::controllerType::partner);