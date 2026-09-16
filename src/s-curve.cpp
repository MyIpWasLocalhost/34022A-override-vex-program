#include <array>
#include <cmath>
#include <cstddef>
#include <stdint.h>

namespace control
{
struct scurve
{
    float displacement;
    float maxVelocity;
    float maxAcceleration;
    float maxJerk;
};

struct dynamicScurve
{
    struct param
    {
        float time;
        float distance;
    };

    struct point
    {
        float time;
        float displacement;
        float velocity;
        float acceleration;
        float jerk;

        point(const point &ref, float t, float j)
        {
            float dt = t - ref.time;
            time = t;
            jerk = j;
            acceleration = ref.acceleration + dt * j;
            velocity = ref.velocity + dt * ref.acceleration + (1.0 / 2.0) * dt * dt * j;
            displacement = ref.displacement + dt * ref.velocity +
                           (1.0 / 2.0) * dt * dt * ref.acceleration +
                           (1.0 / 6.0) * dt * dt * dt * j;
        }

        point(float t, float j, float a, float v, float d)
        {
            time = t;
            jerk = j;
            acceleration = a;
            velocity = v;
            displacement = d;
        }
    };
    const std::array<const point, 8> collection;

    point sample(float t) const
    {
        if (t <= 0) return collection[0];
        if (t >= collection[7].time) return collection[7];

        int i = 1;
        while (i < 7)
        {
            if (t > collection[i].time)
            {
                i++;
                continue;
            }
            break;
        };

        const point &condition = collection[i - 1];
        float j = collection[i].jerk;
        point res = point(condition, t, j);
        return res;
    }

  private:
    std::array<point, 8> longPath(scurve s, scurve data, param parameter)
    {
        const float timeJerk = data.maxAcceleration / data.maxJerk;
        const float dvJerk = (float)((1.0f / 2.0f) * data.maxJerk * std::pow(timeJerk, 2));
        const float timeAccelerate = (data.maxVelocity - 2 * dvJerk) / data.maxAcceleration;

        point p1 = point(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        point p2 = point(p1, timeJerk, data.maxJerk);
        point p3 = point(p2, timeJerk + timeAccelerate, 0.0f);
        point p4 = point(p3, 2 * timeJerk + timeAccelerate, -data.maxJerk);

        float tconst = parameter.time - 2 * p4.time;
        point p5 = point(p4, p4.time + tconst, 0.0f);
        point p6 = point(p5, p5.time + timeJerk, -data.maxJerk);
        point p7 = point(p6, p6.time + timeAccelerate, 0.0f);
        point p8 = point(p7, p7.time + timeJerk, data.maxJerk);

        return std::array<point, 8>{p1, p2, p3, p4, p5, p6, p7, p8};
    }
};
} // namespace control