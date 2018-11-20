#include "../include/gazebot_walker/Walker.h"

#include <cmath>
#include <limits>
#include <vector>

Walker::Walker(ros::NodeHandle &n) : n(n) {
  publisher =
      n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  subscriber = n.subscribe("scan", 50, &Walker::getLaserData, this);
}

Walker::~Walker() {}

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
