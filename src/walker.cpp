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
 *  @file    walker.cpp
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
#include <iostream>
#include "walker.hpp"

/**
 * @brief      Constructs the object.
 */
Walker::Walker() {
  ROS_INFO("Creating the walker behaviour...");
  // Set some parameters
  linSpeed = 0.1;
  turnSpeed = 1.0;
  // initialise the collision flag to be false
  collision = false;
  // Publish the velocity to cmd_vel_mux/input/navi
  velocityPub = n.advertise <geometry_msgs::Twist> ("/cmd_vel_mux/input/navi", 1000);
  // Subcribe to the /scan topic and use the laserCallback method
  sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 500, &Walker::laserCallback, this);
  // Define the initial velocity message
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // stop the turtlebot
  velocityPub.publish(msg);
}

/**
 * @brief      Destroys the object.
 */
Walker::~Walker() {
  // Stop the turtlebot before exiting
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // stop the turtlebot
  velocityPub.publish(msg);
}

/**
 * @brief      Callback for the laser scan data
 *
 * @param[in]  msg   The message
 */
void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 0.75) {
      collision = true;
      return;
    }
  }
  collision = false;
}

/**
 * @brief      Returns the collision flag
 *
 * @return     boolean value for the collision flag
 */
bool Walker::checkObstacle() {
  return collision;
}

/**
 * @brief      Runs the robot
 */
void Walker::runRobot() {
  // Set up the publisher rate to 10 Hz
  ros::Rate loop_rate(10);
  // Keep running till ros is running fine
  while (ros::ok()) {
    // Check for obstacle
    if (checkObstacle()) {
      // Obstacle encountered
      ROS_INFO("Obstacle present in path. Turning...");
      // Stop the robot
      msg.linear.x = 0.0;
      // Turn the robot
      msg.angular.z = turnSpeed;
    } else {
      ROS_INFO("Moving Forward...");
      // Stop turning
      msg.angular.z = 0.0;
      // Set forward speed of the robot
      msg.linear.x = linSpeed;
    }

    // Publish the twist message to anyone listening
    velocityPub.publish(msg);

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();

    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();
  }
}
