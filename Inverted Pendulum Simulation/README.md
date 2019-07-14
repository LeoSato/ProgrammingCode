# Inverted Pendulum Control Simulation

All of this code has been written from scratch as a personal project to get a better understanding of robotics concepts. I have used lecture notes and videos from MIT's Underactuated Robotics course as a reference to write part of this code. Link to the course webpage: http://underactuated.csail.mit.edu/Spring2019/index.html

The single pendulum system used is a simple pendulum with damping. The vertical axis going down from the origin represents theta=0 with counter-clockwise direction being positive.

The double pendulum system used is a combination of two simple pendulum with no damping. The vertical axis going down from the origin represents theta=0 for both thetas (unlike the model shown in the course shown above) with counter-clockwise direction being positive. The double pendulum is an Acrobot, which means only the elbow joint is actuated.

To be able to use the LQR function, it is necessary to install the control package as well as the slycot module. 

The goal of this code is to implement the inverted pendulum. This code will utilize the dynamics of the pendulum and invert the pendulum using a control law derived from a Lyapunov function, which changes the pendulum's energy to match the energy of the desired angles/angular velocities, and a control law derived from a LQR to balance the pendulum at the top. Also, a Kalman Filter option is added for the single pendulum where there is measurement noise on both the angle and angular velocity and an Extended Kalman Filter is used to estimate the actual states.

I have used the section 5.1.1 Discrete-time measurements from the Extended Kalman Filter wiki to help implement in the code. 
Link: https://en.wikipedia.org/wiki/Extended_Kalman_filter
I have also used the Runge-Kutta method wiki to help implement RK4 in the code. 
Link: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

If the .ipynb files won't load on Github, go to this link and paste the URL for the Github page: https://nbviewer.jupyter.org/