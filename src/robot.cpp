#include "robot.h"
#include "motion.h"
#include "vex.h"

namespace Robot
{
vex::brain Brain = ::vex::brain();
vex::controller Controller = ::vex::controller();
vex::digital_out pneumatics = ::vex::digital_out(Brain.ThreeWirePort.F);
vex::inertial inertial = vex::inertial(vex::PORT6);

void drivetrainside::spin(double voltage)
{
    front.spin(vex::forward, voltage, vex::voltageUnits::mV);
    middle.spin(vex::forward, voltage, vex::voltageUnits::mV);
    back.spin(vex::forward, voltage, vex::voltageUnits::mV);
}

void drivetrainside::stop()
{
    front.stop();
    middle.stop();
    back.stop();
}

double drivetrainside::read(int32_t motor)
{
    switch (motor)
    {
    case 1:
        return front.position(vex::deg);
        break;
    case 2:
        return middle.position(vex::deg);
    case 3:
        return back.position(vex::deg);
    default:
        return 0.0;
        break;
    };
}

double drivetrainside::read()
{
    double sum = read(1) + read(2) + read(3) * 3.0;
    return sum / 3.0;
}

void drivetrainside::reset()
{
    front.resetPosition();
    middle.resetPosition();
    back.resetPosition();
}

drivetrainside lDT = drivetrainside(vex::PORT10, vex::PORT12, vex::PORT13, true);
drivetrainside rDT = drivetrainside(vex::PORT1, vex::PORT2, vex::PORT3, false);

void lift::raise(double voltage)
{
    left.spin(vex::forward, voltage, vex::voltageUnits::mV);
    right.spin(vex::forward, voltage, vex::voltageUnits::mV);
}

void lift::drop(double voltage)
{
    left.spin(vex::reverse, voltage, vex::voltageUnits::mV);
    right.spin(vex::reverse, voltage, vex::voltageUnits::mV);
}

void lift::stop()
{
    left.stop();
    right.stop();
}

double lift::read(int32_t motor)
{
    switch (motor)
    {
    case 1:
        return left.position(vex::deg);
    case 2:
        return right.position(vex::deg);
    }
    return 0.0;
}

double lift::read()
{
    return (read(1) + read(2)) / 2.0;
}

lift Lift = lift(vex::PORT8, vex::PORT9);

void push()
{
    pneumatics.set(true);
}

void back()
{
    pneumatics.set(false);
}

} // namespace Robot