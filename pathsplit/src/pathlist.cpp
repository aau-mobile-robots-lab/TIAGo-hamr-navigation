#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <cmath>
using namespace std;

geometry_msgs::Twist testmsg;
double testit = 1;

void planCB(const nav_msgs::Path::ConstPtr& msg)
{
  double px;
  double py;
  
  int msgsz = msg->poses.size();
  //ROS_INFO("size:[%i]", msgsz);
  
  double sumdist;	//sum distance to goal point
  vector<double> pathdist; pathdist.clear();	//vector of distances between points
  vector<double> subpathd; subpathd.clear();	//vector of distances in <2 path
  int index;
  
  for (int i = 1; i <msg->poses.size();i++){
    sumdist += sqrt(pow((msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x),2)+pow((msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y),2));
    pathdist.push_back(sumdist);
    
  } //cout << sumdist << endl; //total distance of path

  if (sumdist >= 2){
    vector<double>::iterator lb = lower_bound(pathdist.begin(),pathdist.end(),2);  //lowb>upb?
    vector<double>::iterator it = find(pathdist.begin(),pathdist.end(),lb[0]);
    index = distance(pathdist.begin(), it);
    //cout<<"["<<index<<", "<<pathdist[index]<<"]"<<endl;
  }

  else if (sumdist < 2){
    //get remaining path
    //cout <<"short" << endl;

    index = distance(pathdist.begin(), pathdist.end()+1);
    //cout<<"["<<index<<","<<pathdist[index]<<"]"<<endl;
    //cout << "pathdist size " << sizeof(pathdist) << endl;
  }

  //build-up published goal point:
  //yaw Quaternion -> Euler
  double siny_cosp = 2 * (msg->poses[index].pose.orientation.w * msg->poses[index].pose.orientation.z + msg->poses[index].pose.orientation.x * msg->poses[index].pose.orientation.y );
  double cosy_cosp = 1 - 2 * (msg->poses[index].pose.orientation.y * msg->poses[index].pose.orientation.y + msg->poses[index].pose.orientation.z * msg->poses[index].pose.orientation.z );
  
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  testmsg.linear.x = msg->poses[index].pose.position.x;
  testmsg.linear.y = msg->poses[index].pose.position.y;
  testmsg.angular.z = yaw;
  testmsg.angular.x = testit;
  ++testit;
  //setup publishing of message
  //ros::NodeHandle n;
  //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("goal_pub", 1000);
  //pub.publish(testmsg);
  //ros::spinOnce()
}



int main(int argc, char **argv)
{
  cout << "test1" << endl;
  ros::init(argc, argv, "pathlist");
  ros::NodeHandle n;
  ros::Rate lrate(100);
  ros::Rate lrate2(10);
  while (ros::ok()){
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("goal_pub", 1000);
    ros::Subscriber sub = n.subscribe("move_base/TebLocalPlannerROS/global_plan", 1000, planCB);
    cout << "test2" << endl;
    for (int i = 0; i < 10 ;i++){
      pub.publish(testmsg);
      
      lrate.sleep();
    }
    cout << "test3" << endl;
    
    lrate2.sleep();
    ros::spinOnce();
  }
  //cout << "its" << endl;
  //ros::spin();
  
  return 0;
}
