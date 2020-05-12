//
// Created by reiserbalazs on 5/10/20.
//

#include <hamr_mpc_local_planner/hamr_mpc_ros.h>
#include "../include/hamr_mpc_local_planner/hamr_mpc_ros.h"

namespace hamr_mpc_local_planner {

    HamrMPCROS::HamrMPCROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                               costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                               goal_reached_(false), no_infeasible_plans_(0), initialized_(false)
    {
    }

    HamrMPCROS::~HamrMPCROS() {
    }

    void hamr_mpc_local_planner::HamrMPCROS::initalize(std::string name, tf::TransformListener *tf,
                                                       costmap_2d::Costmap2DROS *costmap_ros) {
        if (!initialized_) {
            // create Node Handle with name of plugin (as used in move_base for loading)
            ros::NodeHandle nh("~/" + name);

            // reserve some memory for obstacles
            obstacles_.reserve(500);

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

            //Initialize a costmap to polygon converter
            if (!cfg_.obstacles.costmap_converter_plugin.empty())
            {
                try
                {
                    costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
                    std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
                    // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
                    boost::replace_all(converter_name, "::", "/");
                    costmap_converter_->setOdomTopic(cfg_.odom_topic);
                    costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
                    costmap_converter_->setCostmap2D(costmap_);

                    costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
                    ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
                }
                catch(pluginlib::PluginlibException& ex)
                {
                    ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
                    costmap_converter_.reset();
                }
            }
            else
                ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
        }
    }

    bool hamr_mpc_local_planner::HamrMPCROS::isGoalReached() {
        return false;
    }

    bool hamr_mpc_local_planner::HamrMPCROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
        return false;
    }

    bool hamr_mpc_local_planner::HamrMPCROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        return false;
    }

}
