/*
 * @file Walker.h
 * @Copyright MIT license
 * Copyright (c) 2018 Akshay Rajaraman
 * @author Akshay Rajaraman
 * @brief class desciption for the Walker implementation
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


#ifndef _INCLUDE_GAZEBOT_WALKER_WALKER_H_
#define _INCLUDE_GAZEBOT_WALKER_WALKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
/**
 * @brief class description for Walker
 */
class Walker {
 private:
  ros::NodeHandle n;          
  ros::Subscriber subscriber;   
  ros::Publisher publisher; 
  bool isObstacle;  
  geometry_msgs::Twist msg;
  void getLaserData(const sensor_msgs::LaserScan::ConstPtr &scan);
  void move(bool isObstacle);

 public:
  explicit Walker(ros::NodeHandle &n);
  ~Walker();
};

#endif //   _INCLUDE_GAZEBOT_WALKER_WALKER_H_
