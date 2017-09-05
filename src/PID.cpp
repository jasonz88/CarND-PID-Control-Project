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
	Kp += 1e-5 * Kp * KpCoeff * v;
	Ki += 1e-4 * Ki * KiCoeff * v;
	Kd += 1e-4 * Kd * KdCoeff * v;
	if (0.15 < Kp) {
		Kp = 0.15;
	}
	if (0.002 < Ki) {
		Ki = 0.002;
	}
	if (4.0 < Kd) {
		Kd = 4.0;
	}
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

double PID::OutputThrottle(double max_thro) {
	return max_thro - Kp*p_error - Ki*i_error - Kd*d_error;
}
