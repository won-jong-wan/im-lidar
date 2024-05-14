#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
private:
ros::Nod
public:
  ImageConverter(ros::NodeHandle &nh)
  {

  }
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "/* node_name */");
  auto /* node_name */ = /* namespace_name::ClassName */();
  ros::spin();
  ros::shutdown();
  return 0;
}

