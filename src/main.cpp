#include <ros/console.h>
#include "../include/gazebot_walker/Walker.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle nh;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  Walker walker(nh);
  ros::spin();
  return 0;
}
