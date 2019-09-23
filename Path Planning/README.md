# Path Planning

This folder contains my personal programming code for implementing path planning algorithms to get a better understanding.

The src folder contains python source code for the Jupyter Notebook files.

## A*
I have implemented the A* grid search to find a path from a start node to goal node in an obstacle with obstacles.

## PRM
The code is still a work in progress. I have implemented the roadmap, the final path that connects the nodes from the start to the goal for the PRM, collision detection in 2D, and path smoothing using a combination of shortcutting and Quadratic B-Spline curve (I haven't implemented the collision detection for the smooth curve yet). I still need to implement the PRM in higher dimensions and implement other PRM algorithms. I also might implement constraints on the path in the future.

## RRT
The code is still a work in progress. Currently, I implemented the plot of the RRT, the final path that connects the nodes from the start to the goal for the RRT, collision detection in 2D, and path smoothing using a combination of shortcutting and Quadratic B-Spline curve (I haven't implemented the collision detection for the smooth curve yet). I still need to implement the RRT in higher dimensions and implement other RRT algorithms. I also might implement constraints on the path in the future.