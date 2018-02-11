/// Author: Savio Pereira

/// Date Created: 20th Jan 2018

/// Last Modified: 10th Feb 2018

/// Description: The Basic objective of this code is to get the Images from the ROS topic , convert them to the OpenCV format and then use the OpenCV
//               functionalities to perform image processing. The base strucure for this code has been taken from the cv_bridge tutorials page:
//               http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
//               The speicifc processing to be performed in this case is the Hough Transform which will be used to detect Line Segments in the Images which will be streamed by the UAV on a ROS topic


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// Class to handle to Processing of the image from the ROS topic:
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to the ROS topic on which the images are being published:
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);

    // Setup the publisher and the topic on which it is going to send the data:
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

  // Call back function for the subscriber where all the image conversion and processing happens:
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
