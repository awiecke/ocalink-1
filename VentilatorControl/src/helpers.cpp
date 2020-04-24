#include "includes.h"

// Floating point implementation of Math.map()
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Floating point implementation of std::clamp()
double clampf(double x, double out_min, double out_max)
{
    return (x<out_min)?out_min:((x>out_max)?out_max:x);
}
