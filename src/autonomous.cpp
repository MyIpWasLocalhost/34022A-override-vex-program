#include "autonomous.h"
#include "motion.h"
#include "robot.h"
#include "vex.h"

namespace Autonomous
{
void autonomous()
{
    while (Robot::inertial.isCalibrating())
        vex::wait(100, vex::msec);
    Robot::liftqueue.push([]() { Robot::Lift.raisefor(10800, 500); });
    Motion::linearDrive(270);
    Motion::turnDrive(-54);
    Motion::linearDrive(530);
    Robot::Lift.dropfor(10800, 140);
    Robot::push();
    vex::wait(600, vex::msec);
    Motion::linearDrive(-400);
    Robot::liftqueue.push([]() { Robot::Lift.dropfor(10800, 600); });
    Motion::turnDrive(51);
    Motion::linearPID.kP = 10;
    Motion::linearDrive(580);
    Robot::back();
    vex::wait(300, vex::msec);
    Robot::Lift.raisefor(10800, 200);
    Motion::linearPID.kP = 20;
    Robot::liftqueue.push([]() { Robot::Lift.raisefor(12000, 400); });
    Motion::linearDrive(-400);
    Motion::turnDrive(-55);
    Robot::DT.reset();
    Robot::liftqueue.push([]() { Robot::Lift.raisefor(12000, 400); });
    Robot::liftqueue.wait();
    Motion::linearPID.kP = 10;
    Motion::linearDrive(630);
    Robot::push();
    Robot::Lift.dropfor(12000, 300);
    Motion::linearDrive(-600);
    Motion::turnDrive(-90);
    Robot::liftqueue.push([]() { Robot::Lift.dropfor(12000, 400); });
    Robot::DT.movefor(8000, 700);
    Motion::turnDrive(0);
    for (int i = 0; i < 2; i++)
    {
        Robot::DT.movefor(-5400, 350);

        Robot::DT.movefor(5400, 150);

        vex::wait(150, vex::msec);
    }
}
} // namespace Autonomous