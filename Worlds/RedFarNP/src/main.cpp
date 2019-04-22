#include "robot-config.h"
#include "math.h"
#include "algorithm"
#include "string"

using namespace vex;


/*
#################################################################################################
########################################ACCELERATION#####################################################
##########################################################################################################
*/

//A minimum velocity is necessary. This could also be done per command instead
//of globally.
 double minimum_velocity = 50.0;

//This function provides an increasing speed as the robot moves away from start
double increasing_speed (double starting_point, double current_position) {
    static const double acceleration_constant = 30.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

//This function provides a decreasing speed as the robot approaches the end
double decreasing_speed (double ending_point, double current_position) {
    static const double deceleration_constant = 30.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

//This function takes a distance, a maximum velocity, and tries to send the
//robot in a straight line for that distance using a trapezoidal motion profile
//controlled by increasing_speed, decreasing_speed, and maxVelocity
void forward (double distanceIn, double maxVelocity) {
    //record nominal wheel circumference
    static const double circumference = 3.14159 * 4;
    
    //if we've got a joker on our hands, punch out
    if (distanceIn == 0) return;
    
    //figure out which direction we're supposed to be going
    double direction = distanceIn > 0 ? 1.0 : -1.0;
    
    //using circumference and commanded inches, convert to revolutions
    double wheelRevs = distanceIn / circumference;
    
    //set the motors to do a position move with 0 velocity
    //(this is just a cheatyface way to make them stop when arriving at target)
     rf.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     lf.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     rb.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     lb.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    
    //record starting positions and ending positions
    double leftStartPoint = lf.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs;
    double rightStartPoint = rf.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs;
    
    //Back Motors
    double leftBStartPoint = lb.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs;
    double rightBStartPoint = rb.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs;
    
    //execute motion profile
    while (
        (direction * (rf.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) ||
        (direction * (lf.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs)  ||
        (direction * (lb.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs)  ||
        (direction * (rb.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs)  
    ) {
        
        //set right motor speed to minimum of increasing function, decreasing
        //function, and max velocity, based on current position
        if (direction * (rf.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) {
            rf.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        increasing_speed(rightStartPoint,rf.rotation(rotationUnits::rev)),
                        decreasing_speed(rightEndPoint,rf.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            rf.stop(brakeType::brake);
        }
        
        //do the same for the left motor
        if (direction * (lf.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs) {
            lf.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        increasing_speed(leftStartPoint,lf.rotation(rotationUnits::rev)),
                        decreasing_speed(leftEndPoint,lf.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            lf.stop(brakeType::brake);
        }
        
        if (direction * (lb.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs) {
            lb.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        increasing_speed(leftBStartPoint,lb.rotation(rotationUnits::rev)),
                        decreasing_speed(leftBEndPoint,lb.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            lb.stop(brakeType::brake);
        }
        
        if (direction * (rb.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs) {
            rb.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        increasing_speed(rightBStartPoint,rb.rotation(rotationUnits::rev)),
                        decreasing_speed(rightBEndPoint,rb.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            rb.stop(brakeType::brake);
        }
    }
}

void forward (double distanceIn) {
    //no max velocity specified, call the version that uses it with max velocity
    //of 100%
    forward(distanceIn, 100.0);
}

const double tminimum_velocity = 20.0;

double tincreasing_speed (double starting_point, double current_position) 
{
    static const double tacceleration_constant = 100.0;
    return tacceleration_constant * std::abs(current_position - starting_point) + tminimum_velocity;
}

double tdecreasing_speed (double ending_point, double current_position) 
{
    static const double tdeceleration_constant = 50.0;
    return tdeceleration_constant * std::abs(ending_point - current_position) + tminimum_velocity;
}

void turn (double distanceIn, double maxVelocity) 
{
    static const double circumference = 360;
    double direction2 = distanceIn > 0 ? -1 : 1;
    double direction3 = distanceIn > 0 ? 1 : -1;
    double wheelRevs2 = (direction2*std::abs(distanceIn)) / circumference;
    double wheelRevs3 = (direction3*std::abs(distanceIn)) / circumference;
    
    rf.spin(directionType::fwd,direction2 * tminimum_velocity,velocityUnits::pct);
    lf.spin(directionType::fwd,direction3 * tminimum_velocity,velocityUnits::pct);
    rb.spin(directionType::fwd,direction2* tminimum_velocity,velocityUnits::pct);
    lb.spin(directionType::fwd,direction3* tminimum_velocity,velocityUnits::pct);
    
    double leftStartPoint = lf.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs3;
    double rightStartPoint = rf.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs2;
    
    double leftBStartPoint = lb.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs3;
    double rightBStartPoint = rb.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs2;
    
    while (
            (direction2 * (rf.rotation(rotationUnits::rev) - rightStartPoint) < direction2* wheelRevs2) ||
            (direction3 * (lf.rotation(rotationUnits::rev) - leftStartPoint) < direction3 * wheelRevs3)  ||
            (direction3 * (lb.rotation(rotationUnits::rev) - leftBStartPoint) < direction3 * wheelRevs3)  ||
            (direction2 * (rb.rotation(rotationUnits::rev) - rightBStartPoint) < direction2 * wheelRevs2)  
          ) 
    {
        if (direction2 * (rf.rotation(rotationUnits::rev) - rightStartPoint) < direction2 * wheelRevs2) 
        {
            rf.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightStartPoint,rf.rotation(rotationUnits::rev)),
                        tdecreasing_speed(rightEndPoint,rf.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            rf.stop(brakeType::brake);
        }
        
        if (direction3 * (lf.rotation(rotationUnits::rev) - leftStartPoint) < direction3 * wheelRevs3) 
        {
            lf.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftStartPoint,lf.rotation(rotationUnits::rev)),
                        tdecreasing_speed(leftEndPoint,lf.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            lf.stop(brakeType::brake);
        }
        
        if (direction3 * (lb.rotation(rotationUnits::rev) - leftBStartPoint) < direction3 * wheelRevs3) {
            lb.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftBStartPoint,lb.rotation(rotationUnits::rev)),
                        tdecreasing_speed(leftBEndPoint,lb.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            lb.stop(brakeType::brake);
        }
        
        if (direction2 * (rb.rotation(rotationUnits::rev) - rightBStartPoint) < direction2 * wheelRevs2) {
            rb.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightBStartPoint,rb.rotation(rotationUnits::rev)),
                        tdecreasing_speed(rightBEndPoint ,rb.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            rb.stop(brakeType::brake);
        }
    }
    lf.stop(brakeType::brake);
    lb.stop(brakeType::brake);
    rf.stop(brakeType::brake);
    rb.stop(brakeType::brake);
}

/*
#################################################################################################
########################################Functions#####################################################
##########################################################################################################
*/

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

    while (std::abs(lb.rotation(vex::rotationUnits::deg)) < tdistance)
    {
        lf.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
        lb.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
        rf.spin(vex::directionType::fwd,-tspeed,vex::velocityUnits::rpm);
        rb.spin(vex::directionType::fwd,-tspeed,vex::velocityUnits::rpm);
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

    while (std::abs(lb.rotation(vex::rotationUnits::deg)) < tdistance)
    {
        lf.spin(vex::directionType::fwd,-tspeed,vex::velocityUnits::rpm);
        lb.spin(vex::directionType::fwd,-tspeed,vex::velocityUnits::rpm);
        rf.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
        rb.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
    }
    lf.stop(vex::brakeType::coast);
    lb.stop(vex::brakeType::coast);
    rf.stop(vex::brakeType::coast);
    rb.stop(vex::brakeType::coast);
}

void gyroL(int tdistance, int tspeed, int twait) {
    vex::task::sleep(twait); 
    Gyro.startCalibration();
    vex::task::sleep(1300); 

    while (Gyro.value(rotationUnits::deg) > tdistance) {
        lf.spin(vex::directionType::fwd, -tspeed,vex::velocityUnits::rpm);
        lb.spin(vex::directionType::fwd,-tspeed,vex::velocityUnits::rpm);
        rf.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
        rb.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
    }

    lf.stop(vex::brakeType::coast);
    lb.stop(vex::brakeType::coast);
    rf.stop(vex::brakeType::coast);
    rb.stop(vex::brakeType::coast);
}

void gyroR(int tdistance, int tspeed, int twait) {
    vex::task::sleep(twait); 
    Gyro.startCalibration();
    vex::task::sleep(1300); 

    while (Gyro.value(rotationUnits::deg) < tdistance) {
        lf.spin(vex::directionType::fwd, tspeed,vex::velocityUnits::rpm);
        lb.spin(vex::directionType::fwd,tspeed,vex::velocityUnits::rpm);
        rf.spin(vex::directionType::fwd,- tspeed,vex::velocityUnits::rpm);
        rb.spin(vex::directionType::fwd,- tspeed,vex::velocityUnits::rpm);
    }

    lf.stop(vex::brakeType::coast);
    lb.stop(vex::brakeType::coast);
    rf.stop(vex::brakeType::coast);
    rb.stop(vex::brakeType::coast);
}


void fwStart(int speed) {
    fw.spin(directionType::fwd, speed, velocityUnits::rpm);
    fw2.spin(directionType::fwd, speed, velocityUnits::rpm);
}

void fwChange(int speed) {
    fw.setVelocity(speed, velocityUnits::rpm);
    fw2.setVelocity(speed, velocityUnits::rpm);
}

void armUp() {
        arm.rotateFor(0.22, rotationUnits::rev, 25, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void armDown() {
        arm.rotateFor(-0.22, rotationUnits::rev, 25, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void armUpPlat() {
        arm.rotateFor(0.14, rotationUnits::rev, 25, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void armDownPlat() {
        arm.rotateFor(-0.14, rotationUnits::rev, 25, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void armC(double amount, int speed) {
        arm.rotateFor(amount, rotationUnits::rev, speed, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void reset(int totalTime, int speed) {
        double sixtyFivePercent = (totalTime / 10) * 6.5;
        double thirtyFivePercent = (totalTime / 10) * 3;

        lf.spin(directionType::fwd, -speed, velocityUnits::rpm);
        lb.spin(directionType::fwd, -speed, velocityUnits::rpm);
        rf.spin(directionType::fwd, -speed, velocityUnits::rpm);
        rb.spin(directionType::fwd, -speed, velocityUnits::rpm);

         task::sleep(sixtyFivePercent);
         
         stopC();

         task::sleep(thirtyFivePercent);
}

int main() {
    fwStart(440);
    armUp();
    
    task::sleep(5);
    
    forward(23.0, 95.0); 
    stopH();
    
    turnL(173, 70, 20);
    
    forward(4, 80.0); 
    stopH(); 
    
    arm.rotateFor(-0.2, rotationUnits::rev, 37, velocityUnits::rpm);
    arm.stop(brakeType::coast);
    
    task::sleep(150);
    
    in.spin(directionType::fwd, 600, velocityUnits::rpm);
    
    forward(-1.5, 80.0); 
    stopH();
    
    in.stop();
    
    task::sleep(200);
    turn(-33, 70);
    task::sleep(200);
    
    in.spin(directionType::fwd, 600, velocityUnits::rpm);
    
    task::sleep(350);
    
    in.stop();
    
    fw.stop(brakeType::coast);
    fwChange(390);
    
    task::sleep(2500);
    
    in.spin(directionType::fwd, 600, velocityUnits::rpm);
    
    task::sleep(425);
    
    forward(-3, 70);
    stopH();
    
    task::sleep(200);
    turn(228, 70);
    task::sleep(200);
    
    armUp();
    
    fwChange(570);
    
    forward(19, 70);
    stopH();
    
    task::sleep(325);
    
    in.stop();
    
    forward(-6, 70);
    stopH();
    
    task::sleep(200);
    turn(-173, 70);
    task::sleep(200);
    
    in.spin(directionType::fwd, 600, velocityUnits::rpm);
    
    task::sleep(700);
    turn(340, 70);
    task::sleep(200);
    
    armDown();
    
    forward(8.8, 70);
    stopH();
    
    armUp();
    
    /*
    stopB();
    
    rf.spin(vex::directionType::fwd, 100,vex::velocityUnits::rpm);
    rb.spin(vex::directionType::fwd, 100,vex::velocityUnits::rpm);
    
    task::sleep(700);       
    
    stopB();*/
    
    /*
    forward(-2, 80.0); 
    stopH();
    
    arm.rotateFor(0.18, rotationUnits::rev, 80, velocityUnits::rpm, true);
    arm.stop(brakeType::hold);
    
    turnR(167, 70, 20);
    */
}   