#ifndef PID_H
#define PID_H

class PID {
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

  // Steering PID control: update parameters when in high speed
  void Update(double KpCoeff, double KiCoeff, double KdCoeff, double v);

  /*
  * Update the PID error variables given cross track error.
  */
  double UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  // output for throttle control
  double OutputThrottle(double max_thro);



};

#endif /* PID_H */
