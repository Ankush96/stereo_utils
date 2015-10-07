#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <stereo_msgs/DisparityImage.h>

class getDepth
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_l;
  ros::Subscriber image_sub_d;
  cv::Mat left_img;
  cv::Mat_<uint8_t> disp;

public:
  getDepth()
    : it_(nh_)
  {
    image_sub_l = it_.subscribe("/stereo/left/image_rect_color", 1, &getDepth::left_callback, this);
    image_sub_d = nh_.subscribe("/stereo/disparity", 1 ,&getDepth::disparity_callback, this);

    cv::namedWindow("left");
    cv::namedWindow("disparity");
    //cv::namedWindow("mask");
    //cv::namedWindow("masked_disparity")
  }

  ~getDepth()
  {
    cv::destroyWindow("left");
    cv::destroyWindow("disparity");
    //cv::destroyWindow("mask");
    //cv::destroyWindow("masked_disparity")
  }

  void left_callback(const sensor_msgs::ImageConstPtr&);
  void disparity_callback(const stereo_msgs::DisparityImagePtr&);
};


void getDepth::left_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    this->left_img = cv_ptr->image;
    cv::imshow("left",cv_ptr->image);
    cv::waitKey(30);
  }
void getDepth::disparity_callback(const stereo_msgs::DisparityImagePtr& msg)
  {
    cv::Mat_<float> dMat(msg->image.height, msg->image.width, reinterpret_cast<float*>(&(msg->image.data[0])));
    this->disp = (dMat) * (255/msg->max_disparity);

    cv::imshow("disparity",disp);
    cv::waitKey(30);
  }  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  getDepth obj;
  ros::spin();
  return 0;
}  