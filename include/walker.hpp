/**
 *  MIT License
 *
 *  Copyright (c) 2017 Karan Vivek Bhargava
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    walker.hpp
 *  @author  Karan Vivek Bhargava
 *  @copyright MIT License
 *
 *  @brief Assignment to implement walker behavior for turtlebot
 *
 *  @section DESCRIPTION
 *
 *  This program will run the walker node for the turtlebot
 *
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_
// #include <sstream>
// #include <string>
#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Class for Walker.
 */
class Walker {
 private:
  // Declare a varible to detect possible collisions
  bool collision;
  // Declare a variable for the velocities
  geometry_msgs::Twist msg;
  // Create a node handle
  ros::NodeHandle n;
  // Publish the "velocity" topic to the turtlebot
  ros::Publisher velocityPub;
  // Subscribe to the laserscan topic to get obstacles
  ros::Subscriber sub;
  // Define the linear and turn speeds
  float linSpeed;
  float turnSpeed;

 public:
  /**
   * @brief      Constructor for Walker
   */
  Walker();
  /**
   * @brief      Destroys the object.
   */
  ~Walker();
  /**
   * @brief      Callback function for Walker
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief      Checks for obstacles nearby
   *
   * @return     Boolean value 1 if any obstacles are nearby,
   *             0 otherwise
   */
  bool checkObstacle();

  /**
   * @brief      Run the robot
   */
  void runRobot();
};

#endif  // INCLUDE_WALKER_HPP_
