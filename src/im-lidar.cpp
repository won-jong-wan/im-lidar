#include <ros/ros.h>
#include <OpenXLSX.hpp>
#include <vector>

#include "geometry_msgs/PoseWithCovarianceStamped.h" //for /amcl_pose
#include "sensor_msgs/Joy.h" //for /bluetooth_teleop/joy
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"

class Im_lidar: ros::NodeHandle
{
public:
  Im_lidar()
  {

  }
private:

};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "im-lidar");
  auto im_lidar = Im_lidar();
  ros::spin();
  ros::shutdown();
  return 0;
}
