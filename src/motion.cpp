#include "motion.h"
#include "algorithm"
#include "robot.h"
#include "vex.h"
namespace Motion
{

namespace utils
{
double motor_to_distance(double degree)
{
    constexpr double wheel_circumference = 0.22; // unit: m
    constexpr double motor_to_wheel = 0.75;      // due to gear ratio
    constexpr double circular_degree = 360.0;

    constexpr double distance_per_wheel_degree = wheel_circumference / circular_degree;

    return degree * motor_to_wheel * distance_per_wheel_degree;
}
} // namespace utils

double pid::update(double error)
{
    double difference = error - last_error;
    if (fabs(error) < integral_limit)
        integral = integral + error;
    else
        integral = 0;
    last_error = error;
    return kP * error + kI * integral + kD * difference;
}

void pid::clear()
{
    last_error = 0.0;
    integral = 0.0;
}

pid linearPID = pid(7000, 0, 0, 0.2);
pid turnPID = pid(50, 0, 0, 0);

coefficient current_coefficient = p;
sizes quantity = l;
typePID current_typePID = linear;

void change_quantity()
{
    switch (quantity)
    {
    case l:
        quantity = s;
        break;
    case m:
        quantity = l;
        break;
    case s:
        quantity = m;
        break;
    }
}

void change_coefficient()
{
    switch (current_coefficient)
    {
    case p:
        current_coefficient = i;
        break;
    case i:
        current_coefficient = d;
        break;
    case d:
        current_coefficient = p;
        break;
    }
}

void change_typePID()
{
    switch (current_typePID)
    {
    case linear:
        current_typePID = turn;
        break;

    case turn:
        current_typePID = linear;
        break;
    }
}

double getError(double goal)
{
    switch (current_typePID)
    {
    case linear: {
        double average_degree_left = Robot::lDT.read();
        double average_degree_right = Robot::rDT.read();
        double total_average = (average_degree_left + average_degree_right) / 2.0;

        return goal - utils::motor_to_distance(total_average);
    }
    case turn: {
        double error = goal - Robot::inertial.heading();

        // modulus down to nearest plus/minus 180 degree.
        if (error > 180.0)
            error = error - 360;
        else if (error < -180.0)
            error = 360 + error;

        return error;
    }
    }
}

pid *getCurrentPID()
{
    switch (current_typePID)
    {
    case linear:
        return &linearPID;

    case turn:
        return &turnPID;
    }
}

double *pid::getCurrentCoefficient()
{
    switch (current_coefficient)
    {
    case p:
        return &kP;
    case i:
        return &kI;
    case d:
        return &kD;
    }
}

void pid::increment()
{
    *getCurrentCoefficient() += quantity;
}

void pid::decrement()
{
    *getCurrentCoefficient() -= quantity;
}

double step_goal()
{
    return (current_typePID == linear) ? 1.5 : 90;
}

void output(double value)
{
    switch (current_typePID)
    {
    case linear:
        Robot::lDT.spin(value);
        Robot::rDT.spin(value);
        break;
    case turn:
        Robot::lDT.spin(value);
        Robot::rDT.spin(-value);
        break;
    }
}

double acceptance()
{
    switch (current_typePID)
    {
    case linear:
        return 0.005;

    case turn:
        return 1.0;
    }
}

double clamp_volt(double voltage)
{
    return std::max(-12000.0, std::min(12000.0, voltage));
}

void runPID()
{
    Robot::Brain.resetTimer();
    Robot::lDT.reset();
    Robot::rDT.reset();
    getCurrentPID()->clear();
    int settle_counter = 0;
    constexpr int required_count = 5;
    while (Robot::Brain.Timer.time(vex::msec) < 5000)
    {
        double error = getError(step_goal());
        if (fabs(error) < acceptance())
            settle_counter++;
        else
            settle_counter = 0;

        if (settle_counter > required_count) break;

        double result = getCurrentPID()->update(error);
        output(clamp_volt(result));
        vex::wait(10, vex::msec);
    }
    Robot::lDT.stop();
    Robot::rDT.stop();
}

void linearDrive(double degree)
{
    Robot::Brain.resetTimer();
    Robot::lDT.reset();
    Robot::rDT.reset();
    linearPID.clear();
    int settle_counter = 0;
    constexpr int required_count = 5;
    while (Robot::Brain.Timer.time(vex::msec) < 5000)
    {
        double error = degree - (Robot::lDT.read() + Robot::rDT.read()) / 2.0;
        if (fabs(error) < acceptance())
            settle_counter++;
        else
            settle_counter = 0;
        if (settle_counter > required_count) break;

        double result = clamp_volt(linearPID.update(error));
        Robot::lDT.spin(result);
        Robot::rDT.spin(result);
        vex::wait(10, vex::msec);
    }

    Robot::lDT.stop();
    Robot::rDT.stop();
}

void turnDrive(double degree)
{
    Robot::Brain.resetTimer();
    int settle_counter = 0;
    constexpr int required_count = 5;
    while (Robot::Brain.Timer.time(vex::msec) < 5000)
    {
        double error = degree - (Robot::lDT.read() + Robot::rDT.read()) / 2.0;
        if (fabs(error) < acceptance())
            settle_counter++;
        else
            settle_counter = 0;
        if (settle_counter > required_count) break;

        double result = clamp_volt(turnPID.update(error));
        Robot::lDT.spin(-result);
        Robot::rDT.spin(result);
        vex::wait(10, vex::msec);
    }

    Robot::lDT.stop();
    Robot::rDT.stop();
}

} // namespace Motion