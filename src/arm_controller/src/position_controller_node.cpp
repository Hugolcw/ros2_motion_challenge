#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "std_srvs/srv/trigger.hpp"
#include <memory>
#include <string>
#include <map>
#include <vector>

class PositionControllerNode : public rclcpp::Node
{
public:
    PositionControllerNode();
    void setup(); 

private:
    void move_to_joint_angles(const std::vector<double>& joint_angles, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response);

    void moveToHomeCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void moveToPose1Callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void moveToPose2Callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pose1_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pose2_service_;
};

PositionControllerNode::PositionControllerNode() : Node("position_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "节点构造函数执行完毕。");    
}

void PositionControllerNode::setup()
{
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

    home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/move_to_home",
        std::bind(&PositionControllerNode::moveToHomeCallback, this, std::placeholders::_1, std::placeholders::_2));

    pose1_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/move_to_pose1",
        std::bind(&PositionControllerNode::moveToPose1Callback, this, std::placeholders::_1, std::placeholders::_2));
    
    pose2_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/move_to_pose2",
        std::bind(&PositionControllerNode::moveToPose2Callback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "位置控制节点已启动，提供 /move_to_home, /move_to_pose1, /move_to_pose2 服务。");
}

void PositionControllerNode::move_to_joint_angles(const std::vector<double>& joint_angles, const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
{
    move_group_interface_->setJointValueTarget(joint_angles);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "规划成功，开始执行。");
        move_group_interface_->execute(my_plan);
        response->success = true;
        response->message = "运动执行成功。";
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "运动规划失败。");
        response->success = false;
        response->message = "运动规划失败。";
    }
}

void PositionControllerNode::moveToHomeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到 /move_to_home 指令。");
    move_to_joint_angles({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, response);
}

void PositionControllerNode::moveToPose1Callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到 /move_to_pose1 指令。");
    move_to_joint_angles({0.5, -0.5, 1.0, 0.0, 1.5, 0.0}, response);
}

void PositionControllerNode::moveToPose2Callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到 /move_to_pose2 指令。");
    move_to_joint_angles({-0.5, 0.5, -1.0, 0.0, -1.0, 0.5}, response);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionControllerNode>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}