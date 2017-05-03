#ifndef TUNE_H
#define TUNE_H

#include "PID.h"

class Tune {
public:
  std::vector<double> dp;
  double best_error;
  int next_state;
  int coefficient_index;

  Tune(double Tp, double Ti, double Td);
  ~Tune();
  void Update(PID& pid);
};

#endif /* TUNE_H */
