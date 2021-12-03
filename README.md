# ROBOT_MOBILE_MANIPULATION
Mobile manipulation of the 'youBot' using MATLAB and Coppelia Sim. 

'Code':

- Runscript.m: initialize all variables, create a robot object from the Robot class,
and displays the steps on the MATLAB console.
- Robot.m: class for the robot, has all the properties and methods used in the
implementation.
- trajDuration.m: function that calculates the duration of each trajectory follow
by the robot.
- Rotz.m: taken from the Spatial Math Toolbox for MATLAB (SMTB). Rotates angle around
the z axis. Used for convenience.

I set a simple trayectory rather than a more complex one. The
robot tends to give equal priority between the wheel movement and the arm joints 
movement. 

Also, when kp = 0 and ki = x > 0, the system is unstable and therefore, the robot 
do not accomplish the desired end configuration.

'mr': 

Modern Robotics library for MATLAB. Available in http://hades.mech.northwestern.edu/index.php/Modern_Robotics#Online_Courses

'results':

- Sim: final movement of the robot in Coppelia Sim.
- ErrorPlot: plot of the 6 errors for angular and linear velocities vs time.
- Log: console output in MATLAB.
- Trayectory: csv file with the trayectory generated for the robot.
- Xerr: MATLAB file with the error data.
- Animation: input file for the Coppelia Sim.
- Parameters: according to the movement in the video.
