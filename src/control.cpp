#include <array>
#include <cstddef>
#include <stdint.h>
namespace control
{
template <size_t siz> struct filter
{
    std::array<float, siz> arr;
    float sum;
    size_t tail;

    filter()
    {
        arr = {};
        sum = 0.0f;
        tail = siz - 1;
    }

    void push(float f)
    {
        sum = sum - arr[tail] + f;
        arr[tail] = f;
        tail++;
        if (tail == siz) tail = 0;
    }

    float getResult() const
    {
        return sum / siz;
    }
};

struct trapezoid
{
    float distance;
    float maxVelocity;
    float acceleration;
    float currentVelocity;
    float dt;

    trapezoid(float s, float v, float a, float sample)
    {
        distance = s;
        maxVelocity = v;
        acceleration = a;
        dt = sample;
    }

    float ref(float f)
    {
        float target = distance - f;
        float brake = currentVelocity * currentVelocity * 0.5 / acceleration;

        if (target > brake && !reachmax())
        {
            inc();
        }
        if (target <= brake)
        {
            dec();
        }
        clampV();

        if (target < 10.0f)
        {
            zero();
        }

        return currentVelocity;
    }

    bool reachmax()
    {
        return currentVelocity >= maxVelocity;
    }

    void clampV()
    {
        currentVelocity = std::max(0.0f, currentVelocity);
        currentVelocity = std::min(maxVelocity, currentVelocity);
    }

    void zero()
    {
        currentVelocity = 0.0f;
    }

    void inc()
    {
        currentVelocity += acceleration * dt;
    }

    void dec()
    {
        currentVelocity -= acceleration * dt;
    }
};

struct DynamicData
{
    float Error;
    float LastError;
    float Integral;
};

struct ControlParams
{
    float KP;
    float KI;
    float KD;
    float EnableIntegral;
    DynamicData *var;

    ControlParams(float p, float i, float d, DynamicData *v) : KP(p), KI(i), KD(d), var(v)
    {
    }
};
} // namespace control