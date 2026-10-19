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
    Robot::liftqueue.push([]() {
        Robot::Lift.raisefor(10800, 300);
        Robot::Lift.stop(vex::hold);
    });
    Motion::linearDrive(270);
    Motion::turnDrive(-54);
    Motion::linearDrive(530);
    Robot::liftqueue.push([]() {
        Robot::Lift.dropfor(10800, 80);
        Robot::Lift.stop(vex::coast);
    });
    Robot::liftqueue.wait();
    Robot::push();
    vex::wait(600, vex::msec);
    Motion::linearDrive(-400);
    // Robot::liftqueue.push([]() {
    //     Robot::Lift.raisefor(10800, 300);
    //     Robot::Lift.stop(vex::hold);
    // });
    Motion::turnDrive(51);
    Motion::linearPID.kP = 10;
    Motion::linearDrive(570);
    Robot::back();
    vex::wait(300, vex::msec);

    Robot::liftqueue.push([]() {
        Robot::Lift.raisefor(10800, 100);
        Robot::Lift.stop(vex::hold);
    });
    Robot::liftqueue.wait();
    Motion::linearPID.kP = 20;
    Robot::liftqueue.push([]() {
        Robot::Lift.raisefor(12000, 430);
        Robot::Lift.stop(vex::hold);
    });
    Motion::linearDrive(-400);
    Motion::turnDrive(-55);

    Motion::linearPID.kP = 10;
    Motion::linearDrive(630);
    Robot::DT.movefor(3600, 150);
    Robot::push();
    Robot::liftqueue.push([]() {
        Robot::Lift.dropfor(4000, 900);
        Robot::Lift.stop(vex::coast);
    });
    Robot::liftqueue.wait();
    Motion::linearDrive(-600);
    Motion::turnDrive(-90);
    Robot::DT.movefor(8000, 700);
    Motion::turnDrive(0, 3);
    for (int i = 0; i < 2; i++)
    {
        Robot::DT.movefor(-5400, 350);

        Robot::DT.movefor(5400, 150);

        vex::wait(150, vex::msec);
    }

    Motion::turnDrive(33.5);
    Robot::DT.reset();
    Motion::hybridLinearDrive(2820, 6000, 0.6);
    Motion::linearDrive(280);
    vex::wait(100, vex::msec);
    Robot::back();
}
} // namespace Autonomous