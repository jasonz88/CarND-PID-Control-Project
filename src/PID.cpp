#include "PID.h"
#include <math.h>

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

	p_error = 0;
	i_error = 0;
	d_error = 0;
}

void PID::Update(double KpCoeff, double KiCoeff, double KdCoeff, double v) {
	Kp += KpCoeff * v;
	Ki += KiCoeff * v;
	Kd += KdCoeff * v;
}

double PID::UpdateError(double cte) {
	double pre_cte = p_error;

	p_error  = cte;
	i_error += cte;
	d_error  = cte - pre_cte;
	return fabs(cte-pre_cte);
}

double PID::TotalError() {
	return -Kp*p_error - Ki*i_error - Kd*d_error;
}

double PID::OutputThrottle(double max_thro){
  return max_thro - Kp*p_error - Ki*i_error - Kd*d_error;
}
