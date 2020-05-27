#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <cmath>
using namespace std;

class GetLocalGoal
{
public:
    GetLocalGoal()
    {
        //Publish local goal
        pub_ = n_.advertise<geometry_msgs::PoseStamped>("/goal_pub", 10);

        //Subscribe to global plan
        sub_ = n_.subscribe("move_base/GlobalPlanner/plan", 10, &GetLocalGoal::callback, this);
    }

    void callback(const nav_msgs::Path::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped testmsg;
        double goaldist = 2;
        double sumdist;	//sum distance to goal point
        vector<double> pathdist; pathdist.clear();	//vector of distances between points
        vector<double> subpathd; subpathd.clear();	//vector of distances in <2 path
        int index;

        for (int i = 1; i <msg->poses.size();i++){
            sumdist += sqrt(pow((msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x),2)+pow((msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y),2));
            pathdist.push_back(sumdist);
        }

        if (sumdist >= 2){
            vector<double>::iterator lb = lower_bound(pathdist.begin(),pathdist.end(),2);  //lowb>upb?
            vector<double>::iterator it = find(pathdist.begin(),pathdist.end(),lb[0]);
            index = distance(pathdist.begin(), it);
        }
        else{
            index = distance(pathdist.begin(), pathdist.end());
        }
        ROS_INFO("new goal x = [%f] ", msg->poses[index].pose.position.x);
        ROS_INFO("new goal y = [%f]", msg->poses[index].pose.position.y);

        testmsg.pose.position.x = msg->poses[index].pose.position.x;
        testmsg.pose.position.y = msg->poses[index].pose.position.y;
        testmsg.pose.orientation.x = msg->poses[index].pose.orientation.x;
        testmsg.pose.orientation.y = msg->poses[index].pose.orientation.y;
        testmsg.pose.orientation.z = msg->poses[index].pose.orientation.z;
        testmsg.pose.orientation.w = msg->poses[index].pose.orientation.w;
        pub_.publish(testmsg);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};//End of class GetLocalGoal

int main(int argc, char **argv)
{
    //Initiate ROS
    cout << "pathsplit started" << endl;
    ros::init(argc, argv, "pathlist");

    GetLocalGoal getlocalgoal_object;

    ros::spin();

    return 0;
}