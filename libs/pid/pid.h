// Implementation PID controller with Tustin's approximation
// Todo add mutex protection for thread safety.
// Add option on how to clamp for dead-zone.
// rms (2023-12-13)

#ifndef _PID_H_
#define _PID_H_

#include <math.h>

class pid
{
private:
  bool is_p, is_i, is_d; // Enable flags
  float ez, iz, dz;      // Past variables
  float u_min, u_max;    // Output limits
  float i_min, i_max;    // Integrator limits

  float clamp_output(float in) const;
  float clamp_integrator(float in) const;

public:
  pid();
  ~pid();

  float kp, ki, kd; // PID gains

  void set_kp(float p);
  void set_ki(float i);
  void set_kd(float d);

  void set_output_limits(float min, float max);
  void set_integrator_limits(float min, float max);

  void reset_memory(void);
  float update(float e, float dt);
};

#endif // pid.h
