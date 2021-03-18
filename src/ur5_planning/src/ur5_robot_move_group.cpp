//
// Created by shahwaz on 2/19/21.
//

#include "ur5_planning/ur5_robot_move_group.h"

Move_Group::Move_Group()
{
    std::cout << "Loading Move Group for UR5" << std::endl;
    std::cout << "Robot Grasping plan initialized....." << std::endl;

//    joint_model_group_ = ur5_group_ptr_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_1);
    move_group_arm.setNamedTarget(arm_initial_position_);
    move_group_hand.setNamedTarget(hand_initial_position_);

//    Put the arm in start position
    plan_execution(move_group_arm, ur5_plan);

//    Put the gripper in open position
    plan_execution(move_group_hand, ur5_plan);

    first_grasping();
    second_grasping();
    hand_close();
    third_grasping();

    move_group_hand.setNamedTarget(hand_final_position_);
    plan_execution(move_group_hand, ur5_plan);

}

//void Move_Group::move_to_pose(geometry_msgs::Pose target_pose_)
//{
//    moveit_msgs::PlanningScene planning_scene;
//    ur5_group_ptr_->setStartStateToCurrentState();
//    geometry_msgs::Pose start_pose;
//    start_pose = ur5_group_ptr_->getCurrentPose().pose;
//    std::cout << "Start pose: " << start_pose << std::endl;
//
//    ur5_group_ptr_->setPoseTarget(target_pose_);
//    std::cout << "Target Pose: " << target_pose_ << std::endl;
//
//    bool success = (ur5_group_ptr_->plan(ur5_plan) == moveit::planning_interface::MoveItErrorCode:: SUCCESS);
//
//    ur5_group_ptr_->move();
//    planning_scene.is_diff = true;
//
//}

void Move_Group::plan_execution(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::MoveGroupInterface::Plan& robot_plan)
{
    bool success = (move_group.plan(robot_plan) == moveit::planning_interface::MoveItErrorCode:: SUCCESS);

    ROS_INFO_NAMED("Visualizing plan (pose goal) %s ", success ? "" : "FAILED");
    if (success)
        move_group.execute(robot_plan);

}

void Move_Group::first_grasping()
{
    target_pose.orientation.w = 0.5;
    target_pose.orientation.x = -0.5;
    target_pose.orientation.y = 0.5;
    target_pose.orientation.z = -0.5;

    target_pose.position.x = 0.15;
    target_pose.position.y = 0.0;
    target_pose.position.z = 1.25;

    move_group_arm.setPoseTarget(target_pose);
    plan_execution(move_group_arm, ur5_plan);

}

void Move_Group::second_grasping()
{
    target_pose.position.z = 1.13;
    move_group_arm.setPoseTarget(target_pose);
    plan_execution(move_group_arm, ur5_plan);
}

void Move_Group::hand_close()
{
    move_group_hand.setNamedTarget(hand_final_position_);
    plan_execution(move_group_hand, ur5_plan);

}

void Move_Group::third_grasping()
{
    target_pose.position.z = 1.5;
    move_group_arm.setPoseTarget(target_pose);
    plan_execution(move_group_arm, ur5_plan);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_robot_planning");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    Move_Group ur5_robot;
    std::cout << "Robot Grasping plan ended..." << std::endl;

    ros::waitForShutdown();

    return 0;

}