# SLAM-and-probabilistic-path-planning

<p align="center">
  <img src="gif/SLAM_map4.gif">
</p>

The code has been developed using MATLAB’s Robotics System Toolbox™ and Navigation Toolbox™.

## Navigation and mapping algorithm

First, a map in .csv format is loaded and converted into a binary occupancy map using the binaryOccupancyMap function. Simulated data from a 2D laser sensor (rangeSensor) is incorporated into this map using the insertRay function, with data grouped using lidarScan objects. Two additional maps are also created: an empty one to update with sensor data and another to store information about the scanned cells.

The robot model used is a differential drive robot (differentialDriveKinematics) with three degrees of freedom (X, Y position and orientation θ), and its state evolution is computed using the derivative function. For navigation, the Pure Pursuit controller is employed. It follows a path defined by waypoints by continuously adjusting linear and angular velocities. The accuracy of the path following depends on the LookAheadDistance variable.

Finally, an autonomous exploration algorithm is implemented, which moves the robot in the direction of the greatest measured distance reported by the sensor. This direction is added as a new waypoint for the controller. If multiple directions report the same maximum range, the average direction among them is selected. This process will be repeated until either a certain percentage of exploration is reached, or the robot reaches the last generated waypoint (no further waypoints are created because there are no unexplored cells or the robot has become lost), marking the end of the SLAM-based exploration and mapping algorithm.

## Path planning algorithm

The algorithm requires as input a uniformly distributed random sampling set over the map space, which will be used to construct the connectivity graph using the L2 metric, as well as the start and goal points, and the binary map that represents both free space (indicated by the number 0) and the set of unavailable configurations (indicated by the number 1).

As can be observed, the algorithm begins by adding the start point as a vertex in the graph, then proceeds to explore the random sampling set, which is treated as a stack. The last point p from the sampling space is extracted, and the nearest connected vertex n in the graph to p is found, provided a collision-free path can be formed. If no obstacle-free path exists between n and p, the point p is re-added to the end of the sampling set, and a new iteration is forced. Then, it checks whether the goal point can be connected to the existing graph; if so, the loop execution is interrupted, and the goal point is added to the connected graph.

Finally, the shortest path between the start and goal points in the connected graph is computed using the Breadth-First Search graph traversal algorithm. Depending on the distribution of the random sampling points, this technique is significantly faster than the standard RRT algorithm, since it does not require building the graph using all the points from the sampling space.

## Authors

* **Luis Menéndez García**
* **Ruben Sanchez** - [rubinsan](https://github.com/rubinsan)

## License

This project is licensed under the [MIT License](LICENSE).
