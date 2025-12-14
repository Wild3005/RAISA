#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_interface/msg/robot.hpp"
#include "ros2_interface/msg/personpos.hpp"
#include <nlohmann/json.hpp>
#include <cmath>

using json = nlohmann::json;
using namespace std::chrono_literals;

// FSM States
#define CASE_SittingApproach 0
#define CASE_StandingApproach 1
#define CASE_CrossBehind 2
#define CASE_Escort_Mode 3
#define CASE_IDLE 8

class MasterNode : public rclcpp::Node {
public:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_cmd_nav_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_cmd_cancel_nav_;
    rclcpp::Publisher<ros2_interface::msg::Personpos>::SharedPtr pub_person_pos_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_fsm_state_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_uwb_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_robot_pose_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_mode_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_battery_;
    rclcpp::Subscription<ros2_interface::msg::Robot>::SharedPtr sub_robot_info_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_status_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data holders
    geometry_msgs::msg::Pose2D uwb_pose_;
    geometry_msgs::msg::Pose2D last_pose_;
    ros2_interface::msg::Robot robot_info_;
    json nav_status_json_;
    
    // State flags
    bool uwb_data_received_ = false;
    bool nav_status_ready_ = false;
    bool fsm_entry_sent_ = false;
    
    // Robot status
    int robot_mode_ = -1;
    int battery_ = 100;
    
    // FSM
    MachineState fsm_robot;
    
    // Nav status vars
    int res = 0;
    int dist = 0;
    
    MasterNode() : Node("master") {
        RCLCPP_INFO(this->get_logger(), "MasterNode initialized.");
        
        // Publishers
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/vel", 1);
        pub_cmd_nav_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/cmd/nav", 1);
        pub_cmd_cancel_nav_ = this->create_publisher<std_msgs::msg::Int8>("/cmd/cancel_nav", 1);
        pub_person_pos_ = this->create_publisher<ros2_interface::msg::Personpos>("/uwb/person_pos", 1);
        pub_fsm_state_ = this->create_publisher<std_msgs::msg::Int8>("/fsm/state", 1);
        
        // Subscribers
        sub_uwb_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/uwb/pose", 1, std::bind(&MasterNode::callbackUWBPose, this, std::placeholders::_1));
        
        sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/reeman/pose", 1, std::bind(&MasterNode::callbackRobotPose, this, std::placeholders::_1));
        
        sub_robot_mode_ = this->create_subscription<std_msgs::msg::Int8>(
            "/reeman/mode", 1, std::bind(&MasterNode::callbackRobotMode, this, std::placeholders::_1));
        
        sub_robot_battery_ = this->create_subscription<std_msgs::msg::Int8>(
            "/reeman/battery", 1, std::bind(&MasterNode::callbackBattery, this, std::placeholders::_1));
        
        sub_robot_info_ = this->create_subscription<ros2_interface::msg::Robot>(
            "/reeman/robot_info", 1, std::bind(&MasterNode::callbackRobotInfo, this, std::placeholders::_1));
        
        sub_nav_status_ = this->create_subscription<std_msgs::msg::String>(
            "/reeman/nav_status", 1, std::bind(&MasterNode::callbackNavStatus, this, std::placeholders::_1));
        
        // Timer
        timer_ = this->create_wall_timer(500ms, std::bind(&MasterNode::timerRoutine, this));
        
        fsm_robot.value = CASE_SittingApproach;  // Start state
        
        RCLCPP_INFO(this->get_logger(), "Master initialized");
    }
    
    // ============================================================
    // CALLBACKS
    // ============================================================
    
    void callbackUWBPose(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        uwb_pose_ = *msg;
        uwb_data_received_ = true;
    }
    
    void callbackRobotPose(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        last_pose_ = *msg;
    }
    
    void callbackRobotMode(const std_msgs::msg::Int8::SharedPtr msg) {
        robot_mode_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Mode: %d", robot_mode_);
    }
    
    void callbackBattery(const std_msgs::msg::Int8::SharedPtr msg) {
        battery_ = msg->data;
        if (battery_ <= 20) {
            RCLCPP_WARN(this->get_logger(), "Battery low (%d%%)", battery_);
        }
    }
    
    void callbackRobotInfo(const ros2_interface::msg::Robot::SharedPtr msg) {
        robot_info_ = *msg;
        battery_ = msg->battery_level;  // Sync battery from robot_info
    }
    
    void callbackNavStatus(const std_msgs::msg::String::SharedPtr msg) {
        nav_status_json_ = json::parse(msg->data);
        res = nav_status_json_["res"];
        dist = nav_status_json_["dist"];
        nav_status_ready_ = true;
    }
    
    // ============================================================
    // HELPER METHODS
    // ============================================================
    
    void Pose_person(float x, float y, float theta, float radius) {
        ros2_interface::msg::Personpos msg;
        msg.uwb_pose.x = x;
        msg.uwb_pose.y = y;
        msg.uwb_pose.theta = theta;
        msg.radius = radius;
        pub_person_pos_->publish(msg);
    }
    
    void sendVelocity(double vx, double wz) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.angular.z = wz;
        pub_cmd_vel_->publish(msg);
    }
    
    void goTo(double x, double y, double th) {
        geometry_msgs::msg::Pose2D msg;
        msg.x = x;
        msg.y = y;
        msg.theta = th;
        pub_cmd_nav_->publish(msg);
    }
    
    void cancelNav() {
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        pub_cmd_cancel_nav_->publish(msg);
    }
    
    // ============================================================
    // TIMER ROUTINE (LOGIKA TIDAK BERUBAH)
    // ============================================================
    
    void timerRoutine()
    {
        // Safety: battery
        if (battery_ > 0 && battery_ < 20)
        {
            RCLCPP_WARN(this->get_logger(), "Battery critical → cancel navigation.");
            std_msgs::msg::Int8 cancel_msg;
            cancel_msg.data = 1;
            pub_cmd_cancel_nav_->publish(cancel_msg);
            return;
        }

        // Check UWB ready
        if (!uwb_data_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Waiting for UWB data...");
            return;
        }

        // ====== PUBLISH FSM STATE SETIAP CYCLE ======
        {
            std_msgs::msg::Int8 fsm_msg;
            fsm_msg.data = fsm_robot.value;
            pub_fsm_state_->publish(fsm_msg);
        }

        // Determine radius for current state
        auto radius_for_state = [&](int st) -> float {
            switch (st)
            {
                case CASE_SittingApproach:   return 1.5f;
                case CASE_StandingApproach:  return 1.1f;
                case CASE_CrossBehind:       return 0.6f;
                default:                     return 0.0f;
            }
        };

        float rad = radius_for_state(fsm_robot.value);

        // ====== ENTRY ACTION ======
        if (!fsm_entry_sent_)
        {
            // Publish UWB dengan radius target untuk entry
            Pose_person(uwb_pose_.x, uwb_pose_.y, uwb_pose_.theta, rad);
            
            RCLCPP_INFO(this->get_logger(), 
                        "FSM state %d ENTRY: publishing UWB (x=%.2f, y=%.2f, rad=%.2f)",
                        fsm_robot.value, uwb_pose_.x, uwb_pose_.y, rad);
            
            fsm_entry_sent_ = true;
            nav_status_ready_ = false;
            return;  // Tunggu io_reeman process callback
        }

        // ====== MONITORING PHASE ======
        // Publish UWB terus-menerus selama navigasi (untuk update virtual wall)
        Pose_person(uwb_pose_.x, uwb_pose_.y, uwb_pose_.theta, rad);

        // Tunggu nav status dari robot
        if (!nav_status_ready_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for nav status...");
            return;
        }

        // ====== EXIT / TRANSITION ======
        // Check jika sudah sampai tujuan
        if (res == 3 && dist < 0.3f)  // res=3 artinya goal tercapai
        {
            RCLCPP_INFO(this->get_logger(),
                        "FSM state %d COMPLETED (res=%d, dist=%.3d) → IDLE",
                        fsm_robot.value, res, dist);
            fsm_robot.value = CASE_IDLE;
            fsm_entry_sent_ = false;
            nav_status_ready_ = false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterNode>();
    RCLCPP_INFO(node->get_logger(), "Master node spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
