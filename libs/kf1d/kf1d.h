// Position filter and velocity estimator for a single TOF sensor.

#ifndef _KF1D_H_
#define _KF1D_H_

#define KF1D_EPSILON    1e-6f
#define KF1D_BLOW_CLAMP 1e6f

#include <inttypes.h>

class kf1d
{
private:
  // Noise covariances
  float Q[2][2];  // Process noise covariance matrix
  float R;        // Measurement noise variance

  // State mean and covariance
  float x[2];    // State (position, velocity) estimates
  float P[2][2]; // State covariance matrix

  void matmult(const float a[2][2], const float b[2][2], float out[2][2]) const;
  void clamp_covariance();

public:
  // Constructor with initialization
  kf1d(float process_var_pos, float process_var_vel, float measure_var,
       float initial_pos = 0.0f, float initial_vel = 0.0f);

  void predict(float dt);
  void update(float measurement);

  float get_position() const { return   x[0]; }
  float get_velocity() const { return   x[1]; }

  void set_position(float pos) {   x[0] = pos; }
  void set_velocity(float vel) {   x[1] = vel; }

  // Reset filter
  void reset(float pos = 0.0f, float vel = 0.0f,
             float pos_uncertainty = 100.0f, float vel_uncertainty = 100.0f);
};

#endif // _KF1D_H_
