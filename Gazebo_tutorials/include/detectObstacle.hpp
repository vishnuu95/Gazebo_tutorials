/** @file detecetObstacle.cpp
* @brief Obstacle detector class definition
* @Copyright MIT License 2020 Vishnuu
*/
#ifndef GAZEBO_TUTORIALS_INCLUDE_DETECTOBSTACLE_HPP_
#define GAZEBO_TUTORIALS_INCLUDE_DETECTOBSTACLE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

/**
* @brief detectObstacle Class for obstacle detection, 
* command publish, scan laser topic
*/
class detectObstacle {
 public:
    float omega, vel;
    float threshold;
    bool isNearObs;
    geometry_msgs::Twist msg;
    ros::Publisher myPublisher;
    ros::Subscriber mySubscriber;
    ros::NodeHandle n;
    int rightFov;
    int leftFov;
    detectObstacle(float, float, float);
    ~detectObstacle();
    void scanSpace(const sensor_msgs::LaserScan::ConstPtr& messg);
    void moveForward();
    bool isWithinObsRange();
};

#endif  // GAZEBO_TUTORIALS_INCLUDE_DETECTOBSTACLE_HPP_
