#include "driver.h"
#include "robot.h"
#include "vex.h"
vex::competition Competition;

int main()
{
    Competition.autonomous([]() {});
    Competition.drivercontrol(Driver::startDriver);
    Robot::inertial.calibrate();
    while (true)
    {
        vex::wait(100, vex::msec);
    }
}
