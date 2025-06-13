/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#define MINDIS 250/12 
#define RAD2DEG(x) ((x)*180./M_PI)
using namespace cv;
#define RPM 75

void setFrame(Mat& frame){
  cvtColor(frame,frame,COLOR_RGB2GRAY);

  threshold(frame, frame, 0, 255, THRESH_BINARY | THRESH_OTSU);
  frame = ~frame;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3)); 
  dilate(frame, frame, kernel, Point(-1,-1), 1); 

}


Point findMinx(Mat frame,Point& por, Point& pol){
  Mat labels, stats, centroids;
  int lable =  connectedComponentsWithStats(frame, labels, stats, centroids);
   double mindistancel = 250/5.0;
  double mindistancer = 250/5.0;
  Point po(250,250);
  pol= Point(250 - mindistancel,250);
  por= Point(250 + mindistancer,250);
  for (int i = 1; i < lable; i++) {
    int x = stats.at<int>(i, CC_STAT_LEFT);
    int y = stats.at<int>(i, CC_STAT_TOP);
    int w = stats.at<int>(i, CC_STAT_WIDTH);
    int h = stats.at<int>(i, CC_STAT_HEIGHT); 
    if(centroids.at<double>(i, 0)<250&&mindistancel>(sqrt(pow((po.x-x-w),2)+pow((po.y -(y+h)),2)))){
      pol = Point(x+w,y+h);
      mindistancel = (sqrt(pow((po.x-x-w),2)+pow((po.y- y-h),2)));
    }
    else if(centroids.at<double>(i, 0)>=250&&mindistancer>(sqrt(pow((po.x-x),2)+pow((po.y -(y+h)),2)))){
      por = Point(x,y+h);
      mindistancer = (sqrt(pow((po.x-x),2)+pow((po.y- y-h),2)));
    }

  }
  //  std::cout<<"각:"<<pol<<por<<po<<std::endl;
   return pol + por - Point(250,250)- Point(0,50);
}
static void scanCb(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub) {
  cv::Mat lidar(500,500,CV_8UC3,cv::Scalar(255,255,255));
  int count = scan->scan_time / scan->time_increment;
  //printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  //printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
  //       RAD2DEG(scan->angle_max));
  //printf("[SLLIDAR INFO]: i : %f\n",RAD2DEG(scan->angle_increment));
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    double deg_ =(degree)*M_PI/180 ;
    double dis_ =scan->ranges[i];
    double aax = 250+(sin(deg_)*dis_*250/5);
    double aay = 250+(cos(deg_)*dis_*250/5);
    cv::rectangle(lidar,cv::Rect(aax,aay,3,3),cv::Scalar(0,0,255),-1);
  }
  //Mat lidar = cv::imdecode(cv::Mat(scan->data),  cv::IMREAD_COLOR);
  Point por,pol,po;
  resize(lidar, lidar, Size(500, 500));
  cv::Mat li;
  lidar(cv::Rect(0,0,lidar.cols,lidar.rows/2)).copyTo(li);
  setFrame(li);
  po = findMinx(li,por,pol);
    double err = po.x-250;
  std::cout<<"각:"<<pol<<por<<po<<"err"<<err<<std::endl;
  line(lidar, Point(250,250), por, Scalar(255,0,0), 2);
  line(lidar, Point(250,250), pol, Scalar(0,255,0), 2);
  line(lidar, Point(250,250), po, Scalar(0,255,255), 2);
  cv::imshow("lidar2",li);
  cv::rectangle(lidar,cv::Rect(lidar.cols/2-1,lidar.rows/2-1,3,3),cv::Scalar(0,0,0),-1);
  cv::imshow("lidar",lidar);  

  cv::waitKey(1);
  geometry_msgs::msg::Vector3 msg;

  msg.x = RPM + (err*0.75);
  msg.y = -1 * RPM +(err*0.75);
  msg.z = 0.0;

  pub->publish(msg);  // 퍼블리시
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_pub = node->create_publisher<geometry_msgs::msg::Vector3>(
      "topic_dxlpub", rclcpp::QoS(rclcpp::KeepLast(10)));

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [lidar_info_pub](sensor_msgs::msg::LaserScan::SharedPtr scan) {
          scanCb(scan, lidar_info_pub);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}