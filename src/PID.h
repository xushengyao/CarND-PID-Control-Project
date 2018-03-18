#ifndef PID_H
#define PID_H

#include <cmath>
#include <vector>

class PID {

private:
  int step;
  std::vector<double> dp;
  double best_err;
  double err;
  int idx;
  int val_step;
  int test_step;
  int fail_counter;

  void MoveIndex();

  bool is_twiddle;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  // For twiddle
  std::vector<double> parameter;

  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
