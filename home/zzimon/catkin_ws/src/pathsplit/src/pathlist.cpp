#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <nav_msgs/Path.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <algorithm>
using namespace std;

//nav_msgs::Path gPlan;
void vecprint(std::vector<double> const &input){
  cout << "[";
  for (int i = 0; i < input.size(); i++) {
    cout << input.at(i-1) << ',';
  }cout << "]" << endl;
}


class SubPub
{
public:
  SubPub()
  {
    ros::Publisher pub_ = n_.advertise<geometry_msgs::Twist>("zz", 1000);
    
    ros::Subscriber sub_ = n_.subscribe("move_base/TebLocalPlannerROS/global_plan", 1000, &SubPub::planCB, this);
  }


  void planCB(const nav_msgs::Path::ConstPtr& msg)
  {
    double pathx;// = {};	// extended initializer list(?)
    double pathy;// = {};
    pathx = msg->poses[0].pose.position.x;
    pathy = msg->poses[0].pose.position.y;
    //pathx.push_back(msg->poses[1].pose.position.x);
    int msgsz = msg->poses.size();
    ROS_INFO("size:[%i]", msgsz);
    
    double sumdist;
    vector<double> pathdist; pathdist.clear();
    vector<double> subpathd; subpathd.clear();
    nav_msgs::Path delpath;
    int index;
    for (int i = 1; i <msg->poses.size();i++){
      sumdist += sqrt(pow((msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x),2)+pow((msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y),2));
      pathdist.push_back(sumdist);
      }
    
    cout << sumdist << endl; //total distance of path
    if (sumdist >= 2){
      vector<double>::iterator lb = lower_bound(pathdist.begin(),pathdist.end(),2);  //lb>up?
      vector<double>::iterator it = find(pathdist.begin(),pathdist.end(),lb[0]);
      index = distance(pathdist.begin(), it);
      cout<<"["<<index<<", "<<pathdist[index]<<"]"<<endl;

      for (int i = 0; i <index;i++){
        subpathd.push_back(pathdist[i]);
        }
      /*for (int i = 0; i <index;i++){ //distance to all points in subpath
        //cout << subpathd[i] << ", ";} cout << "]" << endl;*/
      }
    else {
      //get remaining path
      cout <<"short" << endl;
      for (int i = 0; i <sizeof(pathdist); i++){
        subpathd.push_back(pathdist[i]);
        }
      /*for (int i = 0; i <sizeof(pathdist); i++){
        cout << subpathd[i] << ", ";*/ //prints path
      //}// cout << "]" << endl;
      index = distance(pathdist.begin(), pathdist.end());
      cout << "path length:" << index << endl;
      }
    geometry_msgs::Twist testmsg;
    testmsg.linear.x = msg->poses[index].pose.position.x;
  
    ros::NodeHandle n;
    pub_.publish(testmsg);

    ROS_INFO("x:[%f], y:[%f]", pathx,pathy);
  }
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathlist");
  
  SubPub SAPObject;
  
  ros::spin();
  
  return 0;
}
