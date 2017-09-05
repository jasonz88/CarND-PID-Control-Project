# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
[image1]: ./PIDdemo70mph.gif

![alt text][image1]

## Goal
This project aims to control the car's steering angle and the throttle with PID controller.
The experiment runs in the CarND term 2 simulator. While lapping around the track, the simulator sends the car's current cross-track error (CTE) via websocket to the PID controller; PID controller does calculation with current CTE and update steering angle and throttle value. The goal is to drive the car smoothly in the road with constant throttle. A strech goal is to finish a lap as fast as possible.

## What is PID controller?
PID controller works reactively. The input is an error signal. In most case, it can be the difference between the measured and the desired value of a process variable of a system. The output is a set of of control signal, consisting of one or more variables. The PID controller uses its parameters to determine the control signal so that the control signal will lead the system to a smaller error. i.e: a process variable value closer to the desired one.

In our case, the error signal is what the simulator produces as the CTE (cross-track error); which is the distance between a reference trajectory and the actual car position. the PID controller trys to minimize this error by changing the steering angle and possibly the throttle. i.e: the car's direction and speed.

### P (Proportional)
The P, proportional term, accounts for present values of the error. It computes an output proportional to the CTE. For example, if the error is large and positive, the control output will also be large and positive. The gain is given by `-Kp cte`.

A pure P-controller is unstable. The output will always compensate on the error at later movement so that the car oscillates about the reference trajectory.

### I (Integral)
The I, integral term, accounts for all past values of the error. It sums up the cte over time. For example, if the current output is not sufficiently strong, the integral of the error will accumulate over time, and the controller will respond by applying a stronger action. The gain is given by `-Ki sum(cte)`.

The I term can be used to accumate a larger error signal quickly when the car is driving at higher speed. It also helps reduce proportional gain so that the oscillation is mitigated.

### D (Differential)
The D, differential term, accounts for possible future trends of the error, based on its current rate of change. It computes the derivative of the cte. The gain is given by `-Kd d(cte)/dt`.

The D term can help reduce oscillation by taking the trend of error into account. i.e: it avoids overshooting when the rate of change becomes small.

## Tuning the PID parameters
### Constant throttle
In this case, only one PID controller is used to control the steering angle from CTE input. I tuned the PID parameters manually. I didn't choose Twiddle algorithm because it requires a manual process to restart the simulator. Also because the time it takes to finish a lap is not trivial. I tuned the parameters in order of P, D and I.

* start with P = 0.2, with I and D = 0, adjust P until it can make turns
* increase D until oscillation is mitigated
* in case of failure due to slow reaction: increase I or P
* in case of failure due to oscillation: reduce I or P

### Need for speed
Another PID controller is used to control the throttle based on absolute value of steering angle. The output is `max_throttle-steering_gain`; so the it will go full speed when going straight while reduce the speed in case of large absolute steering value. The process to tweak the parameters is the same as above. Ideally we should see speed change only when necessary.

Also, brake is introduced in case of huge CTE difference or CTE value to prevent crash at high speed. The parameters of the steering angle PID controller is also increased slightly based on the current speed and CTE value and its derivative. The goal is to increase both P and I so that it is able to cope with sharp turns at high speed. D is also increased proportionally to mitigate oscillation. There is a upper limit of these parameters in case it goes out of control.

### Paramters
* Constant throttle
 * P, I, D = 0.1, 0.001, 3.5
* Speed drive
 * PID for steering: 0.1, 0.001, 3.5;
    update when `0.12 < cteDiff || 1.5 < fabs(cte)`:
    `
    
    KpCoeff = KiCoeff = KdCoeff = cteDiff * fabs(cte)
    Kp += 1e-5 * Kp * KpCoeff * v;
    Ki += 1e-4 * Ki * KiCoeff * v;
    Kd += 1e-4 * Kd * KdCoeff * v;
    `
    Upper bound of the PID: 0.15, 0.002, 4.0
 * PID for throttle: 0.1, 0, 3.5

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

