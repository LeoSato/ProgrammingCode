# Inverted Pendulum Control Simulation

All of this code has been written from scratch as a personal project to get a better understanding of robotics concepts. I have used lecture notes and videos from MIT's Underactuated Robotics course as a reference to write this code. Link to the course webpage: http://underactuated.csail.mit.edu/Spring2019/index.html

To be able to use the LQR function, it is necessary to install the control package as well as the slycot module. 

The goal of this code is to implement the inverted pendulum. This code will utilize the dynamics of the pendulum and invert the pendulum using a control law derived from a Lyapunov function, which changes the pendulum's energy to match the energy of the desired angles/angular velocities, and a control law derived from a LQR to balance the pendulum at the top.

The single pendulum system used is a simple pendulum with damping. The vertical axis going down from the origin represents theta=0 with counter-clockwise direction being positive.

The double pendulum system used is a combination of two simple pendulum with no damping. The vertical axis going down from the origin represents theta=0 for both thetas (unlike the model shown in the course shown above) with counter-clockwise direction being positive. The double pendulum is an Acrobot, which means only the elbow joint is actuated.

If the .ipynb files won't load on Github, go to this link and paste the URL for the Github page: https://nbviewer.jupyter.org/