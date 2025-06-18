#include "kf1d.h"

// Matrix multiplication
void kf1d::matmult(const float a[2][2], const float b[2][2], float out[2][2]) const
{
  out[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0];
  out[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1];
  out[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0];
  out[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1];
}

// Ensure positive definiteness and symmetry of covariance matrix
void kf1d::clamp_covariance()
{
  P[0][0] = (P[0][0] < KF1D_EPSILON) ? KF1D_EPSILON : (P[0][0] > KF1D_BLOW_CLAMP) ? KF1D_BLOW_CLAMP : P[0][0];
  P[1][1] = (P[1][1] < KF1D_EPSILON) ? KF1D_EPSILON : (P[1][1] > KF1D_BLOW_CLAMP) ? KF1D_BLOW_CLAMP : P[1][1];
  P[0][1] = 0.5f * (P[0][1] + P[1][0]);
  P[1][0] = P[0][1];
}

// Constructor
kf1d::kf1d(float process_var_pos, float process_var_vel,
           float measure_var, float x0, float v0) :
  Q{{process_var_pos, 0.0f}, {0.0f, process_var_vel}},
  R(measure_var)
{
  // Initialize state
  x[0] = x0;
  x[1] = v0;

  // Initialize covariance with large uncertainty
  P[0][0] = 100.0f; P[0][1] = 0.0f;
  P[1][0] = 0.0f;   P[1][1] = 100.0f;
}

// Prediction step
void kf1d::predict(float dt)
{
  // State transition matrix
  const float F[2][2] = {{1.0f, dt}, {0.0f, 1.0f}};

  // Predict state: x = F * x
  const float x0 = x[0], x1 = x[1];
  x[0] = x0 + dt * x1;
  x[1] = x1; // Constant velocity model

  // Predict covariance: P = F * P * F' + Q
  float FP[2][2], FPFt[2][2];
  matmult(F, P, FP);

  // F transposed
  const float Ft[2][2] = {{F[0][0], F[1][0]}, {F[0][1], F[1][1]}};
  matmult(FP, Ft, FPFt);

  // Add process noise
  P[0][0] = FPFt[0][0] + Q[0][0];
  P[0][1] = FPFt[0][1] + Q[0][1];
  P[1][0] = FPFt[1][0] + Q[1][0];
  P[1][1] = FPFt[1][1] + Q[1][1];

  clamp_covariance();
}

// Update step
void kf1d::update(float measurement)
{
  // Innovation (measurement residual)
  const float y = measurement - x[0];

  // Innovation covariance
  const float S = P[0][0] + R;

  // Numerical instability of S leads to crazy Kalman gain
  if (S < KF1D_EPSILON)
  {
    return;
  }

  // Kalman gain
  const float K[2] = {P[0][0] / S, P[1][0] / S};

  // State update
  x[0] += K[0] * y;
  x[1] += K[1] * y;

  // Covariance update: P = (I - K * H) * P
  const float I_minus_KH[2][2] ={{1.0f - K[0], 0.0f}, {-K[1], 1.0f}};

  float new_P[2][2];
  matmult(I_minus_KH, P, new_P);

  // Update covariance with stability checks
  P[0][0] = new_P[0][0];
  P[0][1] = new_P[0][1];
  P[1][0] = new_P[1][0];
  P[1][1] = new_P[1][1];

  clamp_covariance();
}

// Reset filter
void kf1d::reset(float pos, float vel, float pos_uncertainty, float vel_uncertainty)
{
  x[0] = pos;
  x[1] = vel;

  P[0][0] = pos_uncertainty;
  P[0][1] = 0.0f; P[1][0] = 0.0f;
  P[1][1] = vel_uncertainty;
}
