#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

int LowerH = 0;
int LowerS = 0;
int LowerV = 0;
int UpperH = 180;
int UpperS = 196;
int UpperV = 170;

cv::Mat IsolateColor(const cv::Mat& src)
{
    cv::Mat out,img_hsv; 
    cv::cvtColor(src,img_hsv,CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),out);
    return out; 
}

void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
        cv::imshow("Input", cv_ptr->image);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img_mask1 = IsolateColor(cv_ptr->image);
    cv::imshow(WINDOW, img_mask1);
    cv::waitKey(30);
    
    pub.publish(cv_ptr->toImageMsg());
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    cv::namedWindow("Input");

    cv::createTrackbar("LowerH","Input",&LowerH,180,NULL);
    cv::createTrackbar("UpperH","Input",&UpperH,180,NULL);
    cv::createTrackbar("LowerS","Input",&LowerS,256,NULL);
    cv::createTrackbar("UpperS","Input",&UpperS,256,NULL);
    cv::createTrackbar("LowerV","Input",&LowerV,256,NULL);
    cv::createTrackbar("UpperV","Input",&UpperV,256,NULL);

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, colorDetectionCallback);
    cv::destroyWindow(WINDOW);

    pub = it.advertise("/image_processed", 1);
    ros::spin(); 
}