//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <sensor_msgs/image_encodings.h>

//C++ stuff
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <math.h>

//OpenCV stuff
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;

const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

bool caliMode; //used for showing debugging windows, trackbars, etc.
bool mouseDragging;
bool mouseMove;
bool rectangleSelected;
cv::Point initialClickPt, currentMousePt;
cv::Rect rectangleROI;
vector<int> H_ROI, S_ROI, V_ROI;

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//Global Variables for the color detection initiation defaults
int LowH = 40;
int HighH = 80;

int LowS = 0;
int HighS = 90;

int LowV = 100;
int HighV = 255;

cv::Mat raspi_camera;


void clickDragRect(int event, int x, int y, int flags, void* param){
	//only if calibration mode is true will we use the mouse to change HSV values
	if (caliMode == true){
		//get handle to video feed passed in as "param" and cast as Mat pointer
		Mat* videoFeed = (Mat*)param;

		if (event == CV_EVENT_LBUTTONDOWN && mouseDragging == false){
			//keep track of initial point clicked
			initialClickPt = cv::Point(x, y);
			//user has begun dragging the mouse
			mouseDragging = true;
		}
		/* user is dragging the mouse */
		if (event == CV_EVENT_MOUSEMOVE && mouseDragging == true){
			//keep track of current mouse point
			currentMousePt = cv::Point(x, y);
			//user has moved the mouse while clicking and dragging
			mouseMove = true;
		}
		/* user has released left button */
		if (event == CV_EVENT_LBUTTONUP && mouseDragging == true){
			//set rectangle ROI to the rectangle that the user has selected
			rectangleROI = Rect(initialClickPt, currentMousePt);

			//reset boolean variables
			mouseDragging = false;
			mouseMove = false;
			rectangleSelected = true;
		}

		if (event == CV_EVENT_RBUTTONDOWN){
			//user has clicked right mouse button
			//Reset HSV Values
			H_MIN = 0;
			S_MIN = 0;
			V_MIN = 0;
			H_MAX = 255;
			S_MAX = 255;
			V_MAX = 255;

		}
	}
}

void recordHSV_Values(cv::Mat frame, cv::Mat HSV_Frame){

	//save HSV values for ROI that user selected to a vector
	if (mouseMove == false && rectangleSelected == true){
		//clear previous vector values
		if (H_ROI.size() > 0) H_ROI.clear();
		if (S_ROI.size() > 0) S_ROI.clear();
		if (V_ROI.size() > 0)V_ROI.clear();
		//if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
		if (rectangleROI.width < 1 || rectangleROI.height < 1) cout <<"Error: no rectangle created" << endl;
		else{
			for (int i = rectangleROI.x; i < rectangleROI.x + rectangleROI.width; i++){
				//iterate through both x and y direction and save HSV values at each and every point
				for (int j = rectangleROI.y; j < rectangleROI.y + rectangleROI.height; j++){
					//save HSV value at this point
					H_ROI.push_back((int)HSV_Frame.at<cv::Vec3b>(j, i)[0]);
					S_ROI.push_back((int)HSV_Frame.at<cv::Vec3b>(j, i)[1]);
					V_ROI.push_back((int)HSV_Frame.at<cv::Vec3b>(j, i)[2]);
				}
			}
		}
		//reset rectangleSelected so user can select another region if necessary
		rectangleSelected = false;
		//set min and max HSV values from min and max elements of each array
		if (H_ROI.size() > 0){
			//NOTE: min_element and max_element return iterators so we must dereference them with "*"
			H_MIN = *std::min_element(H_ROI.begin(), H_ROI.end());
			H_MAX = *std::max_element(H_ROI.begin(), H_ROI.end());
			cout << "MIN 'H' VALUE: " << H_MIN << endl;
			cout << "MAX 'H' VALUE: " << H_MAX << endl;
		}
		if (S_ROI.size() > 0){
			S_MIN = *std::min_element(S_ROI.begin(), S_ROI.end());
			S_MAX = *std::max_element(S_ROI.begin(), S_ROI.end());
			cout << "MIN 'S' VALUE: " << S_MIN << endl;
			cout << "MAX 'S' VALUE: " << S_MAX << endl;
		}
		if (V_ROI.size() > 0){
			V_MIN = *std::min_element(V_ROI.begin(), V_ROI.end());
			V_MAX = *std::max_element(V_ROI.begin(), V_ROI.end());
			cout << "MIN 'V' VALUE: " << V_MIN << endl;
			cout << "MAX 'V' VALUE: " << V_MAX << endl;
		}

	}

	if (mouseMove == true){
		//if the mouse is held down, we will draw the click and dragged rectangle to the screen
		rectangle(frame, initialClickPt, cv::Point(currentMousePt.x, currentMousePt.y), cv::Scalar(0, 255, 0), 1, 8, 0);
	}

}

string intToString(int num){
  std::stringstream ss;
  ss << num;
  return ss.str();
}

void drawCrosshairsRed(int x, int y, Mat &frame){

	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)

	circle(frame, Point(x, y), 20, Scalar(0, 0, 255), 2);
	if (y - 25 > 0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 0, 255), 2);
	if (y + 25 < FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 0, 255), 2);
	if (x - 25 > 0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 0, 255), 2);
	if (x + 25 < FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 0, 255), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 0, 255), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 0, 255), 2);
}


void drawCrosshairs(int x, int y, Mat &frame){

	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25 > 0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25 < FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25 > 0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25 < FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}

void morphOps(Mat &thresh){
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

void create_Trackbars(){
  //create window
  namedWindow("Control Colors", CV_WINDOW_AUTOSIZE);
  //Create the trackbars
  cvCreateTrackbar("LowH", "Control Colors", &LowH, 179);
  cvCreateTrackbar("HighH", "Control Colors", &HighH, 179);

  cvCreateTrackbar("LowS", "Control Colors", &LowS, 255);
  cvCreateTrackbar("HighS", "Control Colors", &HighS, 255);

  cvCreateTrackbar("LowV", "Control Colors", &LowV, 255);
  cvCreateTrackbar("HighV", "Control Colors", &HighV, 255);
}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	int largestIndex = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0){
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects < MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]){
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					//save index of largest contour to use with drawContours
					largestIndex = index;
				}
				else objectFound = false;
			}
			//let user know you found an object
			if (objectFound == true){
				putText(cameraFeed, "Target Object Found", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				drawCrosshairs(x, y, cameraFeed);
				//draw largest contour
				drawContours(cameraFeed, contours, largestIndex, Scalar(0, 0, 255), 2);
        if(y > 270){
          //This is the horizon line
          line(cameraFeed, Point(0, 270), Point(640, 270), Scalar(0, 255, 0), 2);
        }
        if(y > 270 && (x > 160 && x < 480)){
          //This is the left center line
          line(cameraFeed, Point(160, 0), Point(160, 480), Scalar(0, 255, 0), 2);
          //This is the right center line
          line(cameraFeed, Point(480, 0), Point(480, 480), Scalar(0, 255, 0), 2);
          putText(cameraFeed, "Target Object Locked", Point(0, 100), 2, 1, Scalar(0, 0, 255), 2);
          drawCrosshairsRed(x, y, cameraFeed);
        }
			}
		}
		else putText(cameraFeed, "Error: too much noise, change filter", Point(0, 50), 1, 2, Scalar(0, 255, 0), 2);
	}
}

cv::Mat imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    //cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    //cv_ptr->raspi_camera;
    //cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    //raspi_camera = temp;
    //waitKey(30);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("Could not convert image from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "image_subscriber_color_tracking");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("sensor_msgs/Image", 1);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::Rate loop_rate(5);

  imshow("test", raspi_camera);

  ros::spin();
  loop_rate.sleep();

  return 0;
}
