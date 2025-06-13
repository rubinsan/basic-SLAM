# SLAM-and-probabilistic-path-planning

The code has been developed using MATLAB’s Robotics System Toolbox™ and Navigation Toolbox™.

The program starts by opening a .csv file and converting it into an occupancy grid map. Using a kinematic model of a differential drive robot, it explores and maps the environment with a 2D LiDAR sensor. When the algorithm determines that the mapping process is complete, the robot stops, and a probabilistic path planning algorithm based on Rapidly Exploring Random Trees is executed. With the generated path, the robot begins its return to the starting point.

<p align="center">
  <img src="gif/SLAM_map4.gif">
</p>



## Author

* **Ruben Sanchez** - [rubinsan](https://github.com/rubinsan)

## License

This project is licensed under the [MIT License](LICENSE).
