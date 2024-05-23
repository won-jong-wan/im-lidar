#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <fstream>
#include <boost/filesystem.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class Im_lidar
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber odomSub;
  ros::Timer paramT;

  int seq;
  int odomFlag = 0;
  nav_msgs::Odometry odom;
  std::string name;

  std::string path = "./"+name+"/";
  std::string imgPath = "/image/";
  std::string odomPath = "/odom/";

public:
  Im_lidar() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/color/image_rect_color", 1, &Im_lidar::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    odomSub = nh_.subscribe("/odom", 10, &Im_lidar::odomCb, this);
    paramT = nh_.createTimer(ros::Duration(1.0), &Im_lidar::paramCb, this);

    nh_.getParam("name", name);
    path = "./"+name+"/";

    seq = 0;

    boost::filesystem::create_directories(path + imgPath);
    boost::filesystem::create_directories(path + odomPath);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~Im_lidar()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (odomFlag != 0)
    {
      seq++;
    }

    if (seq % 3 == 1)
    {
      cv::imwrite(path + imgPath + "pic" + std::to_string(seq / 3 + 1) + ".png", cv_ptr->image);
      std::fstream odomtxt;

      std::string posStr = "pose( x=" + std::to_string(odom.pose.pose.position.x)
      + " y=" + std::to_string(odom.pose.pose.position.y)
      + " z=" + std::to_string(odom.pose.pose.position.z) + " )";

      std::string oriStr = "orientation( x=" + std::to_string(odom.pose.pose.orientation.x)
      + " y=" + std::to_string(odom.pose.pose.orientation.y)
      + " z=" + std::to_string(odom.pose.pose.orientation.z)
      + " w=" + std::to_string(odom.pose.pose.orientation.w) + " )";

      odomtxt.open(path + odomPath + "odom" + std::to_string(seq / 3 + 1) + ".txt", std::ios::out);
      odomtxt << posStr << std::endl;
      odomtxt << oriStr << std::endl;
      odomtxt.close();

      cv::putText(cv_ptr->image, posStr, cv::Point(0, 40),  3, 1, cv::Scalar(255, 255, 255), 2, 8);
      cv::putText(cv_ptr->image, oriStr, cv::Point(0, 80),  3, 1, cv::Scalar(255, 255, 255), 2, 8);

      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      ROS_INFO("%spic%d.png\n%s\n%s", (path+imgPath).c_str(),seq / 3 + 1,posStr.c_str(), oriStr.c_str());
    }

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr msg)
  {
    odomFlag = 1;
    odom = *msg;
  }

  void paramCb(const ros::TimerEvent & event)
  {
    nh_.getParam("name", name);
    path = "./"+name+"/";
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "im_lidar");
  Im_lidar ic;
  ros::spin();
  return 0;
}
