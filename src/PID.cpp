#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  //twiddle flag
  is_twiddle = true;

  parameter.push_back(Kp);
  parameter.push_back(Ki);
  parameter.push_back(Kd);

  step = 1;
  val_step = 100;
  test_step = 2000;

  for (int i = 0; i < 3; ++i) {
     dp.push_back(0.1 * parameter[i]);
  }
  idx = 0;

  better_err = std::numeric_limits<double>::max();
  err = 0;
  fail_counter = 0;

}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if(is_twiddle){
        if(step % (val_step + test_step) > val_step){
            err += (cte * cte);
        }

        if(step % (val_step + test_step) == 0){
            std::cout<<"==============  step "<<step<<" =============="<<std::endl;
            std::cout << "P: "<< parameter[0]<<" I: "<<parameter[1]<<" D: "<<parameter[2]<<std::endl;
            if (step == (val_step + test_step)){
                if(err < better_err){
                    better_err = err;
                }
                parameter[idx] += dp[idx];
            } else{
                if(err < better_err){
                    better_err = err;
                    dp[idx] *= 1.1;
                    MoveIndex();
                    parameter[idx] += dp[idx];
                    fail_counter = 0;
                } else if(fail_counter == 0){
                    parameter[idx] -= (2*dp[idx]);
                    fail_counter++;
                } else{
                    parameter[idx] += dp[idx];
                    dp[idx] *= 0.9;
                    MoveIndex();
                    parameter[idx] += dp[idx];
                    fail_counter = 0;
                }
            }
            std::cout << "better_err: "<< better_err<<" err: "<<err<<std::endl;
            std::cout << "variable_index: "<<idx<<" new_value: "<<parameter[idx]<<std::endl;
            std::cout <<  std::endl;
            err = 0;
        }
    }
    step++;
}

double PID::TotalError() {
  if (is_twiddle){
    return -parameter[0] * p_error - parameter[1] * i_error - parameter[2] * d_error;
  }
  else{
    return -Kp * p_error - Kd * d_error - Ki * i_error;
  }
}

void PID::MoveIndex() {
    idx++;
    if(idx >=3){
        idx = 0;
    }
}
