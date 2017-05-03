#ifndef PID_H
#define PID_H

class PID {
public:
  double Kp;
  double Ki;
  double Kd;
  
  double p_error;
  double i_error;
  double d_error;

  double last_error;
  double total_error;

  PID(double Kp_in, double Ki_in, double Kd_in);
  virtual ~PID();
  double Update(double error);
};

#endif /* PID_H */
