#include "vex.h"
namespace Robot
{
struct drivetrainside
{
  private:
    vex::motor front;
    vex::motor middle;
    vex::motor back;

  public:
    drivetrainside(int32_t port_front, int32_t port_middle, int32_t port_back, bool reverse)
        : front(port_front, vex::gearSetting::ratio6_1, reverse),
          middle(port_middle, vex::gearSetting::ratio6_1, reverse), back(port_back, reverse)
    {
    }

    void spin(double voltage);
    void stop();
    double read(int32_t motor);
    double read();
    void reset();
};

struct drivetrain
{
  private:
    drivetrainside left;
    drivetrainside right;

  public:
    drivetrain(drivetrainside leftside, drivetrainside rightside) : left(leftside), right(rightside)
    {
    }

    void move(double voltage);
    void turn(double voltage);
    void stop();
    void reset();
};

struct lift
{
  private:
    vex::motor left;
    vex::motor right;

  public:
    lift(int32_t l, int32_t r) : left(l, false), right(r, true)
    {
        left.setStopping(vex::brakeType::coast);
        right.setStopping(vex::brakeType::coast);
    }

    void raise(double voltage);
    void drop(double voltage);
    double read();
    double read(int32_t motor);
    void stop();
};

extern vex::brain Brain;
extern vex::controller Controller;
extern vex::digital_out pneumatics;
extern vex::inertial inertial;
extern drivetrainside lDT;
extern drivetrainside rDT;
extern drivetrain DT;
extern lift Lift;

void push();
void back();
} // namespace Robot