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

  //twiddle parameters
  need_twiddle = false;

  step = 1;
   // let the car run at first 100 steps, then in the next 3000 steps add the cte^2 to the total_error
  val_step = 100;
  test_step = 2000;

  for (int i = 0; i < 3; ++i) {
     // init the change rate with the value of 0.1*parameter
     changes.push_back(0.1 * parameter[i]);
  }
  index_param = 0;

  best_error = std::numeric_limits<double>::max();
  total_error = 0;
   // fail to make the total_error better times
  fail_counter = 0;

}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if(need_twiddle){
        if(step % (val_step + test_step) > val_step){
            total_error += (cte * cte);
        }

        if(step % (val_step + test_step) == 0){
            std::cout<<"==============  step "<<step<<" =============="<<std::endl;
            std::cout << "P: "<< parameter[0]<<" I: "<<parameter[1]<<" D: "<<parameter[2]<<std::endl;
            if (step == (val_step + test_step)){
                if(total_error < best_error){
                    best_error = total_error;

                }
                parameter[index_param] += changes[index_param];
            } else{
                if(total_error < best_error){
                    best_error = total_error;
                    changes[index_param] *= 1.1;
                    IndexMove();
                    parameter[index_param] += changes[index_param];
                    fail_counter = 0;
                } else if(fail_counter == 0){
                    parameter[index_param] -= (2*changes[index_param]);
                    fail_counter++;
                } else{
                    parameter[index_param] += changes[index_param];
                    changes[index_param] *= 0.9;
                    IndexMove();
                    parameter[index_param] += changes[index_param];
                    fail_counter = 0;
                }
            }

            std::cout << "best_error: "<< best_error<<" total_error: "<<total_error<<std::endl;
            std::cout << "change_index: "<<index_param<<" new_parameter: "<<parameter[index_param]<<std::endl;
            std::cout <<  std::endl;
            total_error = 0;
        }
    }
    step++;
}

double PID::TotalError() {
  if (need_twiddle){
    return -parameter[0] * p_error - parameter[1] * i_error - parameter[2] * d_error;
  }
  else{
    return -Kp * p_error - Kd * d_error - Ki * i_error;
  }
}

void PID::IndexMove() {
    index_param++;
    if(index_param >=3){
        index_param = 0;
    }
}
