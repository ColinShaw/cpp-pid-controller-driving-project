#include <iostream>
#include <vector>
#include "Tune.h"

#define STATE_A 0
#define STATE_B 1
#define STATE_C 2

Tune::Tune(double Tp, double Ti, double Td)
{
  // Deltas, best error, state, index
  dp = {Tp, Ti, Td};
  best_error = 1.0e99;
  next_state = STATE_A;
  coefficient_index = 0;
}

Tune::~Tune() {}

void Tune::Update(PID& pid)
{
  // Convert to vector for simplicity with the twiddle factors
  std::vector<double> p = {pid.Kp, pid.Ki, pid.Kd};

  if (next_state == STATE_A)
  {
    p[coefficient_index] += dp[coefficient_index];
    next_state = STATE_B;
  }
  else if (next_state == STATE_B)
  {
    if (pid.total_error < best_error)
    {
      best_error = pid.total_error;
      dp[coefficient_index] *= 1.25;
      coefficient_index = (coefficient_index + 1) % 3;
      next_state = STATE_A;
    } 
    else
    {
      p[coefficient_index] -= 2.0 * dp[coefficient_index];
      next_state = STATE_C;
    }
  }
  else if (next_state == STATE_C)
  {
    if (pid.total_error < best_error)
    {
      best_error = pid.total_error;
      dp[coefficient_index] *= 1.25;
      coefficient_index = (coefficient_index + 1) % 3;
      next_state = STATE_A;
    }
    else 
    {
      p[coefficient_index] += dp[coefficient_index];
      dp[coefficient_index] *= 0.8;
      coefficient_index = (coefficient_index + 1) % 3;
      next_state = STATE_A;
    }
  }

  // Make sure values are non-negative
  for (int i=0; i<3; i++)
  {
    if (p[i] < 0.0)
    {
      p[i] = 0.0;
    }
  }

  // Reset PID controller parameters for next run
  pid.Kp = p[0];
  pid.Ki = p[1];
  pid.Kd = p[2];
  pid.p_error = 0.0;
  pid.i_error = 0.0;
  pid.d_error = 0.0;
  pid.last_error = 0.0;
  pid.total_error = 0.0; 
}

