#include "driver.h"
#include "motion.h"
#include "robot.h"
namespace Driver
{

void checker()
{
    // Motor Tuning Content
    Robot::Brain.Screen.clearScreen();
    Robot::Brain.Screen.setCursor(1, 1);
    Robot::Brain.Screen.print("l1: %.2f, l2: %.2f, l3: %.2f", Robot::lDT.read(1),
                              Robot::lDT.read(2), Robot::lDT.read(3));
    Robot::Brain.Screen.setCursor(2, 1);
    Robot::Brain.Screen.print("r1: %.2f, r2: %.2f, r3: %.2f", Robot::rDT.read(1),
                              Robot::rDT.read(2), Robot::rDT.read(3));
    // PID Tuning Content from here on
    Robot::Brain.Screen.setCursor(3, 1);
    Robot::Brain.Screen.print("on: %s ", Motion::nameof(Motion::current_typePID));
    Robot::Brain.Screen.print("coefficient: %s", Motion::nameof(Motion::current_coefficient));

    Robot::Brain.Screen.setCursor(5, 1);
    Robot::Brain.Screen.print("step size Q: %d", Motion::quantity);

    Robot::Brain.Screen.setCursor(6, 1);
    Motion::pid *current = Motion::getCurrentPID();
    Robot::Brain.Screen.print("p:%.2f, i:%.2f, d:%.2f", current->kP, current->kI, current->kD);

    Robot::Brain.Screen.setCursor(7, 1);
    Robot::Brain.Screen.print("ll:%.2f, rl:%.2f", Robot::Lift.read(1), Robot::Lift.read(2));

    Robot::Brain.Screen.setCursor(8, 1);
    Robot::Brain.Screen.print("heading:%.2f", Robot::inertial.heading());
}

void control()
{
    double front = Robot::Controller.Axis3.position() * 120;
    double side = Robot::Controller.Axis1.position() * 80;

    double left_out = front + side;
    double right_out = front - side;

    Robot::lDT.spin(left_out);
    Robot::rDT.spin(right_out);
    checker();
}

void raise()
{
    Robot::Lift.raise(9600);
    checker();
}

void drop()
{
    Robot::Lift.drop(9600);
    checker();
}

void stop()
{
    Robot::Lift.stop();
}

void startDriver()
{
    Robot::Controller.ButtonLeft.pressed(Robot::push);
    Robot::Controller.ButtonRight.pressed(Robot::back);

    Robot::Controller.Axis3.changed(control);
    Robot::Controller.Axis1.changed(control);

    Robot::Controller.ButtonL1.pressed(raise);
    Robot::Controller.ButtonR1.pressed(drop);
    Robot::Controller.ButtonL1.released(stop);
    Robot::Controller.ButtonR1.released(stop);

    Robot::Controller.ButtonL2.pressed(Robot::push);
    Robot::Controller.ButtonR2.pressed(Robot::back);
    // PID Tuning Content
    Robot::Controller.ButtonUp.pressed([]() { Motion::getCurrentPID()->increment(); });
    Robot::Controller.ButtonDown.pressed([]() { Motion::getCurrentPID()->decrement(); });

    Robot::Controller.ButtonLeft.pressed(Motion::change_typePID);
    Robot::Controller.ButtonRight.pressed(Motion::change_coefficient);
    Robot::Controller.ButtonA.pressed(Motion::change_quantity);
    Robot::Controller.ButtonB.pressed(Motion::runPID);

    Robot::Controller.ButtonLeft.released(checker);
    Robot::Controller.ButtonRight.released(checker);
    Robot::Controller.ButtonA.released(checker);
    Robot::Controller.ButtonUp.released(checker);
    Robot::Controller.ButtonDown.released(checker);
}
} // namespace Driver