# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

This project was created in fulfillment of Udacity's CarND MPC Project. The template 
project was forked from [CarND-MPC-Project](https://github.com/udacity/CarND-MPC-Project "CardND MPC on github")
and improved to meet the specification set forth for the project.

In addition to the project template, code was utilized from the corresponding ```Vehicle Models```
and ```Model Predictive Control``` lessons.

## Implementation

* Student describes their model in detail. This includes the state, actuators 
and update equations.
    
The model employed is a global kinematic model. A more advanced dynamic model could be used,
but that would require more information about a specific vehicle (tire properties, mass, dimensions) and
would be more resource intensive. Since we're using a vehicle/track simulator, the kinematic simplicfication works
quite well.
    
The large-scale nonlinear optimization model used accepts a state for the vehicle, as input. This input includes the position
(x,y), orientation angle (radians), speed (units / second), throttle setting and steering angle (radians)
of the vehicle.  These values are provided by the simulator in map coordinates and 
transformed to the car's coordinate system. Cross track error (CTE) and heading error are calculated
from these values. The model optimizes actuator settings (throttle and steering angle) based 
on the inputs listed above and reference objectives for CTE, heading error and desired speed. 
Reference objectives were set to 0, 0, and 50, respectively.
    
The equations used by the solver, to relate predicted values to prior values at the previous
time step, are:
    
[image2]: ./images/equations.png "Equations"
![equations][image2]

*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed 
duration between timesteps) values. Additionally the student details the previous values
tried.*
    
Two values used by the solver are N and dt. Initially, values of N=10 and 
dt=0.1 were selected, as suggested in the ```Self Driving Car Q&A``` youtube video. Once a stable model was established, other combinations of
values were experimented with: N=10, N=15 and dt=0.10, d=0.05. There wasn't a
lot of difference with the combinations I tried, I eventually settled on N=10 and
dt-0.05 sice that provided very reasonable results at a speed of 50.

*A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to 
the MPC procedure it is described.*
    
Several manipulations on the simulator data are performed prior to involing the
solver:

** the waypoint data is transformed from map coordinates to the car 
coordinate system, where the x-axis runs parallel to the length of the car
and the y-axis is perpendicular to that. The origin is at the car's geometric
center. 
** waypoint data is further transformed from vector<double> to 
Eigen::VectorXd data types.
** next, a polynomial line is fitted to the six waypoints provided by the 
simulator. These points contain a first point near the car and five more future 
waypoints. 
** finally, a low pass filter is applied to the steering angle and throttle
to dampen the effect of recent, more transient, adjustments.   

*The student implements Model Predictive Control that handles a 100 millisecond
latency. Student provides details on how they deal with latency.*

In order to simulate conditions on an actual vehicle, the MPC has a built in 100ms
latency as specified in the project's source code:

```
// Latency
// The purpose is to mimic real driving conditions where
// the car does actuate the commands instantly.
//
// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
// SUBMITTING.
this_thread::sleep_for(chrono::milliseconds(100));
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```

In order to accommodate for this time displacement, the input data to 
the solver
is adjusted. The adjustment is made by predicting, at prevailing speed 
and steering angle, where the vehicle would be after the 100ms delay interval.
This new predicted value is passed to the solver as input.


*costs - trial and error*


## Simulation 

During development, many cars met a fate like this:

[image1]: ./images/CarNDinWater.png "Early Attempts"

![a image of the CarND stunt car in the water][image1]

Fortunately, after multiple iterations, the vehicle successfully drives 
laps around the track at 50, without leaving the road 
surface. The following video illustrates an initial lap showing the final MPC:

[Demo Video](https://www.dropbox.com/s/8cmfhoj80cgyw44/MPCDemo.mp4?dl=0 "Demonstration Video")


## Challenges and a Related Concern

The installation of the Ipopt optimization software was quite problematic. Initially,
the CarND instructions did not work on my Mac system. Eventually, after many attempts, a slack 
posting by Michael Virgo gave me the recipe i needed.

The Ipopt experience raises a concern for me. I sense that the software used here is very established 
and mature, but the unwieldiness and fragilityof an install  is concerning. The thought of putting this solver
into a production driving environment is scary. What happens if a developer doing maintenance 
neglects to enter the correct git hash for one of the components? I suspect this could be
handled through disciplined configuration management and rigorous testing, but the complexity 
and brittleness is unsettling.


## References

[Self-Driving Car Project Q&A | MPC Controller](https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be "Q&A Video")

[Getting Started with Ipopt in 90 Minutes](https://projects.coin-or.org/CoinBinary/export/837/CoinAll/trunk/Installer/files/doc/Short%20tutorial%20Ipopt.pdf "Getting Started with Ipopt")








