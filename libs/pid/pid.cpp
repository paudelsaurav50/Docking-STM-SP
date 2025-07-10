#include "pid.h"

pid::pid()
{
  kp = ki = kd = 0.0;
  ez = iz = dz = 0.0;
  u_max = i_max = INFINITY;
  u_min = i_min = -INFINITY;
  is_p = is_i = is_d = false;
}

pid::~pid()
{
}

void pid::reset_memory(void)
{
  ez = 0.0;
  iz = 0.0;
  dz = 0.0;
}

float pid::clamp_output(float in) const
{
  if (in < u_min) return u_min;
  if (in > u_max) return u_max;
  return in;
}

float pid::clamp_integrator(float in) const
{
  if (in < i_min) return i_min;
  if (in > i_max) return i_max;
  return in;
}

/*           ______________________
            |                      |
  E(s) ---->| kp + ki / s + kd * s |----> U(s)
            |______________________|
*/
float pid::update(float e, float dt)
{
  float p = 0.0, i = 0.0, d = 0.0;

  if (is_p)
  {
    p = kp * e;
  }

  if (is_i)
  {
    i = iz + 0.5f * ki * dt * (e + ez);
    i = clamp_integrator(i);
    iz = i;
  }

  if (is_d)
  {
    d = -dz + 2.0f * kd * (e - ez) / dt;
    dz = d;
  }

  ez = e;
  return clamp_output(p + i + d);
}

void pid::set_kp(float p)
{
  kp = p;
  is_p = true;
}

void pid::set_ki(float i)
{
  ki = i;
  is_i = true;
}

void pid::set_kd(float d)
{
  kd = d;
  is_d = true;
}

void pid::set_integrator_limits(float min, float max)
{
  if (min < max)
  {
    i_min = min;
    i_max = max;
  }
}

void pid::set_output_limits(float min, float max)
{
  if (min < max)
  {
    u_min = min;
    u_max = max;
  }
}
