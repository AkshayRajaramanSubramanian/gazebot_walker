#ifndef _INCLUDE_GAZEBOT_WALKER_WALKER_H_
#define _INCLUDE_GAZEBOT_WALKER_WALKER_H_
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
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
