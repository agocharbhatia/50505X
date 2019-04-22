#include "robot-config.h"
using namespace vex;

int sFind() {
    while(1) {
        Brain.Screen.printAt( 10, 80, "Line Value: %d", Line.value(vex::analogUnits::range8bit));
    }
}

int main() {
    vex::task find(sFind);
}

