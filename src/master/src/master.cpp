#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_interface/msg/robot.hpp"
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

// ALIAS
using json = nlohmann::json;

using namespace std::chrono_literals;

#define GOTO 0
#define VEL 1

class MasterNode : public rclcpp::Node
{
public:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_cmd_nav_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_cmd_cancel_nav_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_robot_pose_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_mode_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_battery_;
    rclcpp::Subscription<ros2_interface::msg::Robot>::SharedPtr sub_robot_info;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_status;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Internal state tracking
    geometry_msgs::msg::Pose2D last_pose_;
    int robot_mode_ = -1;
    int battery_ = 100;

    MachineState fsm_robot;

    ros2_interface::msg::Robot Robot_Info;

    //Temp Val glob
    int count_step = 0;
    json json_msg;
    bool sudahterkitim = 0;


    // VAL NAV STATUS
    int res;
    

    MasterNode() : Node("master")
    {
        RCLCPP_INFO(this->get_logger(), "MasterNode initialized.");

        // -----------------------------
        // Publishers
        // -----------------------------
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/vel", 1);
        pub_cmd_nav_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/cmd/nav", 1);
        pub_cmd_cancel_nav_ = this->create_publisher<std_msgs::msg::Int8>("/cmd/cancel_nav", 1);

        // -----------------------------
        // Subscribers
        // -----------------------------
        sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "reeman/pose", 1, std::bind(&MasterNode::callbackRobotPose, this, std::placeholders::_1));

        sub_robot_mode_ = this->create_subscription<std_msgs::msg::Int8>(
            "reeman/mode", 1, std::bind(&MasterNode::callbackRobotMode, this, std::placeholders::_1));

        sub_robot_battery_ = this->create_subscription<std_msgs::msg::Int8>(
            "reeman/battery", 1, std::bind(&MasterNode::callbackBattery, this, std::placeholders::_1));

        sub_robot_info = this->create_subscription<ros2_interface::msg::Robot>(
            "reeman/robot_info", 1, std::bind(&MasterNode::callbackRobotInfo, this, std::placeholders::_1));

        sub_nav_status = this->create_subscription<std_msgs::msg::String>(
            "reeman/nav_status", 1, std::bind(&MasterNode::statusCallback, this, std::placeholders::_1));

        // -----------------------------
        // Optional periodic behavior
        // -----------------------------
        timer_ = this->create_wall_timer(1000ms, std::bind(&MasterNode::timerRoutine, this));
    }

    // ============================================================
    // CALLBACKS
    // ============================================================
    void statusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        json_msg = json::parse(msg->data);

        res = json_msg["res"];
    }

    void callbackRobotInfo(const ros2_interface::msg::Robot::SharedPtr msg)
    {
        Robot_Info = *msg;
    }

    void callbackRobotPose(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        last_pose_ = *msg;
    }

    void callbackRobotMode(const std_msgs::msg::Int8::SharedPtr msg)
    {
        robot_mode_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Mode: %d", robot_mode_);
    }

    void callbackBattery(const std_msgs::msg::Int8::SharedPtr msg)
    {
        battery_ = msg->data;
        if (battery_ <= 20)
        {
            RCLCPP_WARN(this->get_logger(), "Battery low (%d%%)", battery_);
        }
    }

    // ============================================================
    // TIMER ROUTINE
    // ============================================================
    void timerRoutine()
    {
        // Example decision-making logic
        if (battery_ > 0 && battery_ < 20)
        {
            RCLCPP_WARN(this->get_logger(), "Battery critical → cancel navigation.");
            std_msgs::msg::Int8 cancel_msg;
            cancel_msg.data = 1;
            pub_cmd_cancel_nav_->publish(cancel_msg);
            return;
        }

        // Example periodic heartbeat or idle behavior
        // RCLCPP_INFO(this->get_logger(),
        //             "Heartbeat | Mode=%d | Battery=%d | Pose=(%.2f, %.2f)",
        //             robot_mode_, battery_,
        //             last_pose_.x, last_pose_.y);
        //temp val =============
        //======================
        switch (fsm_robot.value)
        {
        case GOTO:
            if(sudahterkitim == 0){
                goTo(0.0,0.0,1.0);
                sudahterkitim = 1;

                RCLCPP_INFO(this->get_logger(),"CEK");
            }
            if(res == 3){
                fsm_robot.value = VEL;
                sudahterkitim = 0;
            }
            break;
        
        case VEL:
            if(count_step < 3){
                sendVelocity(0.0,1.0);
                // sendVelocity(0.5,0.0);
                count_step++;
            }else if(count_step >= 4 && count_step < 6){
                sendVelocity(0.5,0.0);
                // sendVelocity(0.0,1.0);
                count_step++;
            }else if(count_step >= 6) {
                fsm_robot.value = GOTO;
            }
            break;
        
        default:
            break;
        }
    }

    // ============================================================
    // MANUAL COMMAND METHODS (optional)
    // ============================================================

    void sendVelocity(double vx, double wz) // m/s, rad/s
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.angular.z = wz;
        pub_cmd_vel_->publish(msg);
    }

    void goTo(double x, double y, double th)
    {
        geometry_msgs::msg::Pose2D msg;
        msg.x = x;
        msg.y = y;
        msg.theta = th;
        pub_cmd_nav_->publish(msg);
    }

    void cancelNav()
    {
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        pub_cmd_cancel_nav_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MasterNode>();

    RCLCPP_INFO(node->get_logger(), "Master node spinning...");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
