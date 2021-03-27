#include <ros/ros.h> 
#include <smartarm_msgs/Position.h>
#include <sensor_msgs/JointState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <kdl/jntarray.hpp>
#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <time.h>

class SmartarmQuickMove
{
    public:
        SmartarmQuickMove()
        {
            timeout_ = 0.1;
            end_effector_position_ << 0, 0, 0;
            end_effector_rotation_ << 1,0,0,0,1,0,0,0,-1;

            //ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

            robot_model_loader::RobotModelLoader rmloader("robot_description");
            robot_model_loader_ = rmloader;
            moveit::core::RobotStatePtr kstate(new moveit::core::RobotState(kinematic_model_));
            kinematic_state_ = kstate;
            kinematic_state_->setToDefaultValues();

            // Using the :moveit_core:`RobotModel`, we can construct a
            // :moveit_core:`RobotState` that maintains the configuration
            // of the robot. We will set all joints in the state to their
            // default values. We can then get a
            // :moveit_core:`JointModelGroup`, which represents the robot
            // model for a particular group, e.g. the "panda_arm" of the Panda
            // robot.

            sub_ = nh_.subscribe("/fake_position1", 1, &SmartarmQuickMove::ikCallback, this);
            //pub_ = nh_.advertise<sensor_msgs::JointState>("/move_group_fake_controller_joint_states", 1);
            pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

        }

        //由rostopic pub确定是回调函数有问题！导致进程退出！process has died [pid 55186, exit code -6, 
        //cmd /home/zuxinrui/ros_ws/devel/lib/smartarm_host/smartarm_quick_move_node __name:=smartarm_quick_move_node
        void ikCallback(const smartarm_msgs::Position& input)
        {
            ROS_INFO("callback called");
            if (input.num == 1.0)
            {
                //Eigen::Isometry3d end_effector_state_ = Eigen::Isometry3d::Identity();
                end_effector_position_ << input.x, input.y, input.z;
                end_effector_state_.rotate(end_effector_rotation_);
                end_effector_state_.pretranslate(end_effector_position_);
                // 由ROS_INFO确定是这一步的问题！
                // 不是private引起的
                ROS_INFO_STREAM("Translation: \n" << end_effector_state_.translation() << "\n");
                ROS_INFO_STREAM("Rotation: \n" << end_effector_state_.rotation() << "\n");
                ikSolve();
            }
        }

        void ikSolve()
        {
            //kinematic_state_->setToRandomPositions(joint_model_group_);
            bool found_ik = kinematic_state_->setFromIK(joint_model_group_, end_effector_state_, timeout_);
            ROS_INFO("callback ik found");
            sensor_msgs::JointState js;

            if (1)
            {
                kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values_);
                ROS_INFO("callback ik joint values got");
                for (std::size_t i = 0; i < joint_names_.size(); ++i)
                {
                ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_values_[i]);
                }
                // TODO: sent the joint values to joint state publisher
                js.header.stamp = ros::Time::now();
                js.name = joint_names_;
                js.position = joint_values_;
                //js.velocity = (double)[0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                //js.effort = (double)[0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                pub_.publish(js);
            }
        }
    private:
        Eigen::Vector3d end_effector_position_;
        Eigen::Matrix3d end_effector_rotation_;
        Eigen::Isometry3d end_effector_state_ = Eigen::Isometry3d::Identity();

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        double timeout_;
        std::vector<double> joint_values_;

    public:
        robot_model_loader::RobotModelLoader robot_model_loader_;
        const moveit::core::RobotModelPtr& kinematic_model_ = robot_model_loader_.getModel();
        moveit::core::RobotStatePtr kinematic_state_;
        const moveit::core::JointModelGroup* joint_model_group_ = kinematic_model_->getJointModelGroup("smartarm");
        const std::vector<std::string>& joint_names_ = joint_model_group_->getVariableNames();

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "quick_move");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    SmartarmQuickMove smartarm_quick_move;

    //ros::spin();

    return 0;
}



