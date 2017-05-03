#include "PID.h"

PID::PID(double Kp_in, double Ki_in, double Kd_in) 
{
  // Initial coefficients
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in; 

  // PID Error terms
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // Last sample error and total error
  last_error = 0.0;
  total_error = 0.0;
}

PID::~PID() {}

double PID::Update(double error) 
{
  // Update the error term
  p_error = error;
  i_error += error;
  d_error = error - last_error;

  // Update last error 
  last_error = error;

  // update total error
  total_error += error * error;

  // Return the evaluated filter
  return -1.0 * (Kp * p_error + Ki * i_error + Kd * d_error);
}
