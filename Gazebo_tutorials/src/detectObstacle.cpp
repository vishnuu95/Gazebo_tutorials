/** @file detecetObstacle.cpp
* @brief Obstacle detector class implementation
* @Copyright MIT License 2020 Vishnuu
*/
#include <../include/detectObstacle.hpp>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

/** 
* @brief Constructor for detectObstacle class 
* @param threshold - distance threshold below which obstacle called as detected
* @param vel - forward linear velocity 
* @param omega - angular velocity for turtlebot
* @return None
*/
detectObstacle::detectObstacle(float threshold, float vel, float omega) {
    this->threshold = threshold;
    this->vel = vel;
    this->omega = omega;
    // this->isWithinObsRange = false;
    this->msg;
    this->myPublisher = this->n.advertise<geometry_msgs::Twist>
    ("/cmd_vel", 1000);
    this->mySubscriber = this->n.subscribe<sensor_msgs::LaserScan>
    ("/scan", 1000, &detectObstacle::scanSpace, this);
    this->rightFov = 65;
    this->leftFov = 65;
}

/**
* @brief Destructor
* @param - None
* @return - None
*/
detectObstacle::~detectObstacle() {
    this->msg.linear.x = 0;
    this->msg.linear.y = 0;
    this->msg.linear.z = 0;
    this->msg.angular.x = 0;
    this->msg.angular.y = 0;
    this->msg.angular.z = 0;
    this->myPublisher.publish(this->msg);
}

/**
* @brief runs throught entire lidar scan to look for obstacles
* @param Sensor message - Lidar scan
* @return None
*/
void detectObstacle::scanSpace(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Check if obstacle is present in front right side
    for (auto i = 0; i  < this->rightFov; i ++) {
        if (msg->ranges[i]  < this->threshold) {
            this->isNearObs = true;
            ROS_WARN_STREAM("Obstacle is ahead");
            return;
        }
    }
    // Check if obstacle is present in front left side
    for (auto i = this->leftFov; i < 360; i++) {
        if (msg->ranges[i] < this->threshold) {
            this->isNearObs = true;
            ROS_WARN_STREAM("Obstacle is ahead");
            return;
        }
    }
    this->isNearObs = false;
    return;
}


/**
* @brief Move forward function that moves the robot ahead
* @param None
* @return None
*/
void detectObstacle::moveForward() {
    while (1) {
        if (ros::ok()) {
            if (this->isWithinObsRange()) {
             msg.linear.x = 0;
             this->msg.angular.z = this->omega;
            } else {
             this->msg.linear.x = this->vel;
             this->msg.linear.z = 0;
            }
            // this->msg.linear.x = this->vel;
            this->myPublisher.publish(this->msg);
            ros::spinOnce();
        }
    }
}

/**
* @brief isWithinObsRange function returns the status of obstacle detection
* @param None
* @return bool value
*/
bool detectObstacle::isWithinObsRange() {
    return this->isNearObs;
}

/**
* @brief contains main function implementation
* @param argc, argv
* return None
*/
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "obsdetector");
    detectObstacle myRobot(0.5, 1, 0.2);
    myRobot.moveForward();
}
