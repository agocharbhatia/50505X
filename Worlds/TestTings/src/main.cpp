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
double minimum_velocity = 30.0;

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
    lf.resetRotation();
    rb.resetRotation();
    rf.resetRotation();
    vex::task::sleep(150); 
    
    //int tAverage = ( lf.rotation( rotationUnits::deg ) + lb.rotation( rotationUnits::deg ) 
      //             + rf.rotation( rotationUnits::deg ) + rb.rotation( rotationUnits::deg ) ) / 4;
    
    while (std::abs(lf.rotation(rotationUnits::deg)) < tdistance)
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

void turnL(double tdistance, int tspeed, int twait) {
    vex::task::sleep(twait); 
    lb.resetRotation();
    lf.resetRotation();
    rb.resetRotation();
    rf.resetRotation();
    vex::task::sleep(150); 
    
    //int tAverage = ( lf.rotation( rotationUnits::deg ) + lb.rotation( rotationUnits::deg ) 
                  // + rf.rotation( rotationUnits::deg ) + rb.rotation( rotationUnits::deg ) ) / 4;
    
    while (std::abs(lb.rotation(rotationUnits::deg)) < tdistance)
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

void fwStart(int speed) {
    fw.spin(directionType::fwd, speed, velocityUnits::rpm);
}

void fwStop() {
    fw.stop();
}

void fwChange(int newSpeed) {
    fw.setVelocity(newSpeed, velocityUnits::rpm);
}

void armUp() {
        arm.rotateFor(0.23, rotationUnits::rev, 200, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void armDown() {
        arm.rotateFor(-0.23, rotationUnits::rev, 200, velocityUnits::rpm);
        arm.stop(brakeType::hold);
}

void intake(int time, int speed) {
    in.spin(directionType::rev, speed, velocityUnits::rpm);
    task::sleep(time);
}

void intakeStop() {
    in.stop();
}

void intakeForever() {
    in.spin(directionType::rev, 200, velocityUnits::rpm);
}

void sleep(int time) {
    task::sleep(time);
}

int correct()
{
    if (lf.rotation(vex::rotationUnits::deg) - rf.rotation(vex::rotationUnits::deg) > 0)
    {
        while (lf.rotation(vex::rotationUnits::deg) - rf.rotation(vex::rotationUnits::deg) > 0)
        {
            lf.spin(vex::directionType::fwd,-15,vex::velocityUnits::rpm);
            lb.spin(vex::directionType::fwd,-15,vex::velocityUnits::rpm);
            rf.spin(vex::directionType::fwd,15,vex::velocityUnits::rpm);
            rb.spin(vex::directionType::fwd,15,vex::velocityUnits::rpm);
        }
        lf.stop(vex::brakeType::coast);
        lb.stop(vex::brakeType::coast);
        rf.stop(vex::brakeType::coast);
        rb.stop(vex::brakeType::coast);
    }
    
    if (lf.rotation(vex::rotationUnits::deg) - rf.rotation(vex::rotationUnits::deg) < 0)
    {
        while (lf.rotation(vex::rotationUnits::deg) - rf.rotation(vex::rotationUnits::deg) < 0)
        {
            lf.spin(vex::directionType::fwd,15,vex::velocityUnits::rpm);
            lb.spin(vex::directionType::fwd,15,vex::velocityUnits::rpm);
            rf.spin(vex::directionType::fwd,-15,vex::velocityUnits::rpm);
            rb.spin(vex::directionType::fwd,-15,vex::velocityUnits::rpm);
        }
        lf.stop(vex::brakeType::coast);
        lb.stop(vex::brakeType::coast);
        rf.stop(vex::brakeType::coast);
        rb.stop(vex::brakeType::coast);
    }
    return(0);
}

/* 
#################################################################################################
########################################Auton#####################################################
##########################################################################################################
*/

int main(void) {
    fw.spin(directionType::fwd, 300, velocityUnits::rpm);
    in.spin(directionType::fwd, 600, velocityUnits::rpm);
    fw2.spin(directionType::fwd, 300, velocityUnits::rpm);
}   