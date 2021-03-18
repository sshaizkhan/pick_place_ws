//
// Created by shahwaz on 2/19/21.
//

#ifndef UR5_ROBOT_MOVE_GROUP_H
#define UR5_ROBOT_MOVE_GROUP_H

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface//planning_scene_interface.h"

#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "control_msgs/GripperCommandActionGoal.h"

#include "moveit_visual_tools/moveit_visual_tools.h"



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Move_Group{
private:
    /* data */
public:

//    Initialize ROS parameters
    ros::NodeHandle nh;

//  ROS_Msgs

    geometry_msgs::Pose target_pose;


//    Initialize Robot planning group
    std::string PLANNING_GROUP_1 = "arm";
    std::string PLANNING_GROUP_2 = "hand";


    std::string arm_initial_position_ = "start";
    std::string hand_initial_position_ = "open";
    std::string hand_final_position_ = "close";


//    Move Group Declaration
    moveit::planning_interface::MoveGroupInterface move_group_arm{PLANNING_GROUP_1};
    moveit::planning_interface::MoveGroupInterface move_group_hand{PLANNING_GROUP_2};


//    Initialize MoveGroup Parameters
    moveit::planning_interface::MoveGroupInterfacePtr ur5_group_ptr_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    const robot_state::JointModelGroup* joint_model_group_{};
    moveit::planning_interface::MoveGroupInterface::Plan ur5_plan;

    moveit_msgs::OrientationConstraint goal_pose_constraint;



//    Publishers and Subscribers


//    Operating Functions declaration
    Move_Group();

    static void plan_execution(moveit::planning_interface::MoveGroupInterface& move_group ,moveit::planning_interface::MoveGroupInterface::Plan& robot_plan);

//    void move_to_pose(geometry_msgs::Pose );

    void first_grasping();

    void second_grasping();

    void hand_close();

    void third_grasping();





};




#endif //UR5_PLANNING_UR5_ROBOT_MOVE_GROUP_H
