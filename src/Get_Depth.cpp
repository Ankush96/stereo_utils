#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <stereo_msgs/DisparityImage.h>
#include "image_geometry/stereo_camera_model.h"
#include "image_geometry/pinhole_camera_model.h"


using namespace image_geometry;

int l_cam_info_taken = 0;
int r_cam_info_taken = 0;

class getDepth
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_l;
  ros::Subscriber image_sub_d, l_info, r_info;
  cv::Mat left_img;
  cv::Mat_<uint8_t> disp;

  sensor_msgs::CameraInfo left, right;
  StereoCameraModel stereo_model;

  int LowerH;
  int LowerS;
  int LowerV;
  int UpperH;
  int UpperS;
  int UpperV;
  int erosion_kernel;

public:
  getDepth()
    : it_(nh_)
  {

    l_info = nh_.subscribe("/stereo/left/camera_info", 1 ,&getDepth::update_l_info, this);
    r_info = nh_.subscribe("/stereo/right/camera_info", 1 ,&getDepth::update_r_info, this);
    image_sub_l = it_.subscribe("/stereo/left/image_rect_color", 1, &getDepth::left_callback, this);
    image_sub_d = nh_.subscribe("/stereo/disparity", 1 ,&getDepth::disparity_callback, this);

    LowerH = 0;
    LowerS = 0;
    LowerV = 0;
    UpperH = 180;
    UpperS = 196;
    UpperV = 170;
    erosion_kernel = 3;
    cv::namedWindow("left");
    cv::namedWindow("disparity");
    cv::namedWindow("Projected Z");
    cv::namedWindow("mask");
    cv::createTrackbar("LowerH","left",&LowerH,180,NULL);
    cv::createTrackbar("UpperH","left",&UpperH,180,NULL);
    cv::createTrackbar("LowerS","left",&LowerS,256,NULL);
    cv::createTrackbar("UpperS","left",&UpperS,256,NULL);
    cv::createTrackbar("LowerV","left",&LowerV,256,NULL);
    cv::createTrackbar("UpperV","left",&UpperV,256,NULL);
    cv::createTrackbar("ErosionKernel 2n+3","left",&erosion_kernel,6,NULL);

    //cv::namedWindow("masked_disparity")
  }

  ~getDepth()
  {
    cv::destroyWindow("left");
    cv::destroyWindow("disparity");
    cv::destroyWindow("mask");
    cv::destroyWindow("Projected Z");
    //cv::destroyWindow("masked_disparity")
  }

  void left_callback(const sensor_msgs::ImageConstPtr&);
  void disparity_callback(const stereo_msgs::DisparityImagePtr&);
  cv::Mat IsolateColor(const cv::Mat& src);
  cv::Mat reduceNoise(const cv::Mat& src);
  void update_l_info(const sensor_msgs::CameraInfo &msg);
  void update_r_info(const sensor_msgs::CameraInfo &msg);
};

void getDepth::update_l_info(const sensor_msgs::CameraInfo &msg)
{
  if(l_cam_info_taken) return;
  this->left = msg;
  l_cam_info_taken = 1;
}

void getDepth::update_r_info(const sensor_msgs::CameraInfo &msg)
{
  if(r_cam_info_taken) return;
  this->right = msg;
  r_cam_info_taken =1;
}

cv::Mat getDepth::IsolateColor(const cv::Mat& src)
{
    cv::Mat out,img_hsv; 
    cv::cvtColor(src,img_hsv,CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),out);
    return out; 
}

cv::Mat getDepth::reduceNoise(const cv::Mat& src)
{
    cv::Mat dst = src.clone();
    int i, j, k, l, count;

    //-------- Remove Borders ----------//
    for(i=0;i<dst.rows;i++)
    {
        dst.at<uchar>(i,0)=0;
        dst.at<uchar>(i,dst.cols-1)=0;
    }
    for(i=0;i<dst.cols;i++)
    {
        dst.at<uchar>(0,i)=0;
        dst.at<uchar>(dst.rows-1,i)=0;
    }
    //----------------------------------//

    //-------- Perform a variation of Erosion followed by Dilation -----------//
    int kern = 2*erosion_kernel + 3;
    for(i=1;i<src.rows-1;i++)
    {
        for(j=1;j<src.cols-1;j++)
        {
            if(src.at<uchar>(i,j)==255)
            {
                count=0;
                 for(l=i-kern/2;l<=i+kern/2;l++)
                 {
                    for(k=j-kern/2;k<=j+kern/2;k++)
                        {
                            if((!(i==l&&j==k)) && src.at<uchar>(l,k)==255)
                            {
                                count ++;
                            }
                        }
                 }
                 if(count<=(kern*kern)/2) dst.at<uchar>(i,j)=0;
            }

        }
    }
     for(i=1;i<src.rows-1;i++)
    {
        for(j=1;j<src.cols-1;j++)
        {
            if(src.at<uchar>(i,j)==0)
            {
                count=0;
                 for(l=i-kern/2;l<=i+kern/2;l++)
                 {
                    for(k=j-kern/2;k<=j+kern/2;k++)
                        {
                            if((!(i==l&&j==k)) && src.at<uchar>(l,k)==0)
                            {
                                count ++;
                            }
                        }
                 }
                 if(count<=(kern*kern)/2) dst.at<uchar>(i,j)=255;
            }

        }
    }
    return dst;
}

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
    cv::Mat_<uint8_t> reprojected_z;
    float f_max = 10000.0;
    cv::Mat Z(dMat.rows, dMat.cols, CV_32FC1, cv::Scalar(f_max));

    cv::Mat img_mask1 = IsolateColor(this->left_img);
    cv::Mat img_mask2 = reduceNoise(img_mask1);
    cv::Matx44d _q = stereo_model.reprojectionMatrix();
    
    //std::cout<<" left "<<this->left<<std::endl;
    //std::cout<<" right "<<this->right<<std::endl;
    stereo_model.fromCameraInfo(this->left, this->right);
    //std::cout<<" Fy :"<<stereo_model.left_.fy()<<std::endl;
    //PinholeCameraModel r = stereo_model.right();
    // std::cout<<" right.tx : "<<r.Tx()<<" right.fx : "<<r.fx()<<std::endl;
    // std::cout<<"q: "<<_q<<std::endl;

    // std::cout<<" baseline :"<<stereo_model.baseline()<<std::endl;

    // Mask the isolated object on the disparity map and reproject respective points
    for(int i = 0; i < img_mask2.rows; i++)
    {
      for(int j = 0; j < img_mask2.cols; j++)
      {
        if(img_mask2.at<uchar>(i,j) == 255) // We are interested in disparity values of only object pixels
        {
          cv::Point3d xyz;
          stereo_model.projectDisparityTo3d(cv::Point2d(i,j), dMat.at<float>(i,j), xyz);
          std::cout<<xyz.x<<":"<<xyz.y<<":"<<xyz.z<<std::endl;
          Z.at<float>(i,j) = xyz.z;
        }
      }
    }
    reprojected_z = (255) * (1-(Z/f_max)); // Will be complement of mask


    cv::imshow("disparity",disp);
    cv::imshow("Projected Z", reprojected_z);
    cv::imshow("mask",img_mask2);
    cv::waitKey(30);
  }  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  getDepth obj;
  ros::spin();
  return 0;
}  