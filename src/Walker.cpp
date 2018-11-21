/*
 * @file Walker.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Akshay Rajaraman
 * @author Akshay Rajaraman
 * @brief class that describes the function of the walker node
 *
 */



/*
 * MIT License
 *
 * Copyright (c) 2018 Akshay Rajaraman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "../include/gazebot_walker/Walker.h"
#include <cmath>
#include <limits>
#include <vector>
#include <utility>
/**
 * @brief constructor for initializing Walker object
 * @param NodeHandle n
 */

Walker::Walker(ros::NodeHandle &n) : n(n) {
  publisher =
      n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  subscriber = n.subscribe("scan", 50, &Walker::getLaserData, this);
}

Walker::~Walker() {}
/**
 * @brief callback function that runs when data is being subscribed from the robot
 */
void Walker::getLaserData(const sensor_msgs::LaserScan::ConstPtr &scanData) {
  std::vector<float> laserData = scanData->ranges;
  auto angleRes = scanData->angle_increment;
  auto rangeMin = scanData->range_min;
  auto rangeMax = scanData->range_max;
  size_t length = laserData.size();
  auto mid = length / 2;
  float getRange = 10 * M_PI / 180;
  size_t rangeI = getRange / angleRes;
  float distMin = std::numeric_limits<float>::max();
  for (size_t iterator = mid - rangeI; iterator < mid + rangeI;
       iterator++) {
    if (laserData[iterator] < distMin) {
      distMin = laserData[iterator];
    }
  }
  if (std::isnan(distMin) || distMin > rangeMax || distMin < rangeMin) {
    isObstacle = false;
    move(isObstacle);
    return;
  }
  if (distMin < 1.5) {
    isObstacle = true;
  } else {
    isObstacle = false;
  }
  move(isObstacle);
}
/**
 * @brief function that moves the robot either linearly or angularly
 */
void Walker::move(bool isObstacle) {
  if (isObstacle) {
    msg.linear.x = 0.0;
    msg.angular.z = -0.3;
  } else {
    msg.linear.x = 0.75;
    msg.angular.z = 0.0;
  }
  publisher.publish(msg);
}
