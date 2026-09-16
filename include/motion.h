#include <algorithm>
namespace Motion
{
constexpr double hard_maximum_velocity = 1.644;
struct tracker
{
  private:
    double velocity_max;
    double acceleration_max;
    double jerk_max;

    double velocity_current;
    double acceleration_current;

  public:
    tracker(double v, double a, double j) : velocity_max(v), acceleration_max(a), jerk_max(j)
    {
        velocity_current = 0.0;
        acceleration_current = 0.0;
    }

    double update(double time, bool acc)
    {
        double k = acc ? 1 : -1;
        acceleration_current += jerk_max * time * k;
        velocity_current += 0.5 * jerk_max * time * time * k;
        clamp();
        return velocity_current;
    }

    double updateAcceleration(double time, bool acc)
    {
        double k = acc ? 1 : -1;
        velocity_current += acceleration_max * time * k;
        return velocity_current;
    }

    void clamp()
    {
        velocity_current = std::min(velocity_max, std::max(-velocity_max, velocity_current));
        acceleration_current =
            std::min(acceleration_max, std::max(-acceleration_max, acceleration_current));
    }

    double velocity_to_voltage(double velocity)
    {
        return velocity / hard_maximum_velocity * 12000;
    }

    bool at_max_velocity()
    {
        return velocity_current == velocity_max || velocity_current == -velocity_max;
    }

    bool at_max_acceleration()
    {
        return acceleration_current == acceleration_max ||
               acceleration_current == -acceleration_max;
    }
};

struct pid
{
    double kP;
    double kI;
    double kD;

    double last_error;
    double integral_limit;
    double integral;

    double update(double error);
    void clear();

    pid(double p, double i, double d, double i_l) : kP(p), kI(i), kD(d), integral_limit(i_l)
    {
        last_error = 0.0;
        integral = 0.0;
    }

    void increment();
    void decrement();
    double *getCurrentCoefficient();
};

enum coefficient
{
    p,
    i,
    d
};

inline const char *nameof(coefficient value)
{
    switch (value)
    {
    case p:
        return "p";
    case i:
        return "i";
    case d:
        return "d";
    }
}

enum sizes
{
    l = 1000,
    m = 100,
    s = 10
};

enum typePID
{
    linear,
    turn
};

inline const char *nameof(typePID value)
{
    switch (value)
    {
    case linear:
        return "linear";
    case turn:
        return "turn";
    }
}

extern pid linearPID;
extern pid turnPID;

extern coefficient current_coefficient;
extern sizes quantity;
extern typePID current_typePID;

void change_quantity();
void change_coefficient();
void change_typePID();
void runPID();
pid *getCurrentPID();
} // namespace Motion