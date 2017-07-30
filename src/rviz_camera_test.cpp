#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
  ros::init(argc, argv, "image_publish");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("sensor_msgs/Image", 1);

  VideoCapture cap;
  cap.open(0); // +1 for webcam, 0 for default
  if(!cap.isOpened()){
    cout<<"ERROR: camera feed failed to open"<<endl;
    return 1;
  }
  cv::Mat cameraFeed;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);

  //cap.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

  while(nh.ok()){
    cap>>cameraFeed;
    if(!cameraFeed.empty()){
      //cv::imshow("Video", cameraFeed);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cameraFeed).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
