#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <string>
#include <sstream>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <bits/stdc++.h>
using namespace std;


void vecprint(std::vector<double> const &input){
  cout << "[";
  for (int i = 0; i < input.size(); i++) {
    cout << input.at(i) << ',';}
  cout << "]" << endl;
}


void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //const sensor_msgs::LaserScan::ConstPtr testmsg;
  //testmsg = msg;
  vector<double> test2;
  vector<double> testmsg;
  for (int i = 1; i <= msg->intensities.size(); i++){
    testmsg.push_back(msg->intensities[i]);
    if (msg->intensities[i]<1){
      test2.push_back(0);}
    else if (msg->intensities[i]>0.1){
      test2.push_back(msg->intensities[i]);}
  }
  cout<<"["<<msg->intensities.size()<<", "<<msg->range_min<<", "<<msg->range_max<<", "<<msg->intensities[1]<<", "<< msg->angle_min<<", "<<msg->angle_max<<"]"<<endl;
  
  vecprint(test2);
  double max = *max_element(testmsg.begin(), testmsg.end());
  double min = *min_element(testmsg.begin(), testmsg.end());
  vector<double>::iterator it = find(testmsg.begin(),testmsg.end(),max);
  int index = distance(testmsg.begin(), it);
  //int testi = testmsg.begin();
  cout << "maxindex: "<<index;
  cout << " MAX:"<<max<< "front: "<< msg->intensities[333] <<endl;
  
  cout << "this is new" << endl;
  ROS_INFO("end");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan_raw", 1000, chatterCallback);

  ros::spin();

  return 0;
}
