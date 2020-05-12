// TODO: Do we want License here?

#ifndef SRC_HAMR_MPC_ROS_H
#define SRC_HAMR_MPC_ROS_H

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/ObstacleMsg.h>

// transforms
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>

// boost classes  TODO: I just copied it here in order if we need
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace hamr_mpc_local_planner{
    //Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
    class HamrMPCROS : public nav_core::BaseLocalPlanner {

    public:

        HamrMPCROS();        //Default constructor for the ros wrapper
        ~HamrMPCROS();       //Default destructor for the ros wrapper

        /** Override BaseLocalPlanner **/
        void initalize(std::string name, tf::TransformListener* tf,
                         costmap_2d::Costmap2DROS* costmap_ros);
        bool isGoalReached();
        bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

    private:

        //Pointer to external objects (do NOT delete object)
        costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
        tf::TransformListener* tf_; //!< pointer to Transform Listener

        // Topics & Services
        ros::Subscriber amcl_sub; ///<@brief subscribes to the amcl topic
        ros::Publisher path_pub; ///<@brief publishes to the bubble shape to visualize on rviz


        // Data
        pos now; // present frame
        pos next; // next frame
        pos nError; // error between present and next frames
        double distance;
        int length; // number of frames in the global plan
        int count; // keeps track of the number for the next frame in the global plan
        std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
        geometry_msgs::Twist cmd; // contains the velocity
        visualization_msgs::Marker points;
        double average;
        int num;
        double average2, average3;
        int num2, howmanytimes;
        int p, hmt;
        double minus;
        double haha, haha2;
        double beforee;
        ofstream file;

        //measuring
        double stopTime, startTime;
        double beginning2, ending2;
        bool firstTime, hasStarted, number1;
        double pathLength;
        double four;

        // Flags
        bool goal_reached_;
        bool initialized_;

        // Velocity methods
        /**
        * @brief Set Vel: function that sets linear speed
        */
        void setVel();

        /**
        * @brief Set Rot: function that sets angular speed
        */
        void setRot();

        /**
        * @brief Set Vel Z: function that sets linear and angular speed to ZERO
        */
        void setVelZ();


        // Methods
        /**
         * @brief Amcl Callback: function is called whenever a new amcl msg is published on that topic
         * @param Pointer to the received message
         */
        void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

        /**
        * @brief getYaw: function calculates the Yaw angle from the position message that amcl sent
        * @param msg: passes the amcl position info to the function
        */
        double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

        /**
        * @brief setNowError: calculates the error between the next and present frame
        */
        void setNowError();

        /**
        * @brief getNext: uses count to set the next goal frame
        */
        void setNext();

        void pathVisualization();

    };
};
class hamr_mpc_ros {

};


#endif //SRC_HAMR_MPC_ROS_H
