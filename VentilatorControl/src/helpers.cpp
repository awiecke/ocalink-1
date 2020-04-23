#include "helpers.h"

// Floating point implementation of Math.map()
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double clampf(double v, double lo, double hi)
{
    return (v < lo) ? lo : (hi < v) ? hi : v;
}