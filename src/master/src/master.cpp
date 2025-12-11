#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_interface/msg/robot.hpp"
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <cmath>

// ALIAS
using json = nlohmann::json;

using namespace std::chrono_literals;

#define GOTO 0
#define VEL 1
#define TEST_UWB 2

typedef struct
{
    float pose_x;
    float pose_y;
    float pose_theta;

    float vel_linear;
    float vel_angular;

    int8_t nav_status_res;
    int8_t nav_status_reason;
    std::string nav_status_goal;
    float nav_status_dist;
    float nav_status_mileage;

    int8_t mode;
    int8_t battery_level;
    int8_t charge_flag;
    int8_t emergency_flag;

    int8_t is_moving;

} robot_t;

typedef struct
{
    float pose_x;
    float pose_y;
    float pose_theta;
} pose2d_t;

class MasterNode : public rclcpp::Node
{
public:
    // ============================= ROS2 Utils =============================
    HelpLogger logger;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_cmd_nav_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_cmd_cancel_nav_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_get_pose_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_get_nav_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_robot_pose_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_mode_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_battery_;
    rclcpp::Subscription<ros2_interface::msg::Robot>::SharedPtr sub_robot_info;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_status;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_ui_button_control;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ui_keyboard_control;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_uwb_pose2d;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr keyboard_command_timer_;

    // Internal state tracking
    geometry_msgs::msg::Pose2D last_pose_;
    int robot_mode_ = -1;
    int battery_ = 100;

    MachineState fsm_robot;

    // Temp Val glob
    int count_step = 0;
    json json_msg;
    bool sudahterkitim = 0;

    // VAL NAV STATUS
    int res;

    // UWB Pose
    pose2d_t uwb_pose;
    pose2d_t uwb_pose_offset;
    const float offset_meter = 1.2; // 20 cm

    // ============================= Robot Command =============================
    std::string last_robot_command = "";

    robot_t robot;

    MasterNode() : Node("master")
    {
        RCLCPP_INFO(this->get_logger(), "MasterNode initialized.");

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize HelpLogger.");
        }

        // -----------------------------
        // Options for callback groups
        // -----------------------------
        auto cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions node_options;
        node_options.callback_group = cb_group_;

        // -----------------------------
        // Publishers
        // -----------------------------
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/vel", 1);
        pub_cmd_nav_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/cmd/nav", 1);
        pub_cmd_cancel_nav_ = this->create_publisher<std_msgs::msg::Int8>("/cmd/cancel_nav", 1);
        pub_get_pose_ = this->create_publisher<std_msgs::msg::Int8>("/get/pose", 1);
        pub_get_nav_ = this->create_publisher<std_msgs::msg::Int8>("/get/nav", 1);

        // -----------------------------
        // Subscribers
        // -----------------------------
        // sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        //     "reeman/pose", 1, std::bind(&MasterNode::callbackRobotPose, this, std::placeholders::_1), node_options);
        // sub_robot_mode_ = this->create_subscription<std_msgs::msg::Int8>(
        //     "reeman/mode", 1, std::bind(&MasterNode::callbackRobotMode, this, std::placeholders::_1), node_options);
        // sub_robot_battery_ = this->create_subscription<std_msgs::msg::Int8>(
        //     "reeman/battery", 1, std::bind(&MasterNode::callbackBattery, this, std::placeholders::_1), node_options);
        // sub_nav_status = this->create_subscription<std_msgs::msg::String>(
        //     "reeman/nav_status", 1, std::bind(&MasterNode::statusCallback, this, std::placeholders::_1), node_options);
        sub_robot_info = this->create_subscription<ros2_interface::msg::Robot>(
            "reeman/robot_info", 1, std::bind(&MasterNode::callbackRobotInfo, this, std::placeholders::_1), node_options);
        sub_ui_button_control = this->create_subscription<std_msgs::msg::Int8>(
            "/ui_control", 1, std::bind(&MasterNode::callbackUIButtonControl, this, std::placeholders::_1), node_options);
        sub_ui_keyboard_control = this->create_subscription<std_msgs::msg::String>(
            "/ui_keyboard_control", 1, bind(&MasterNode::callbackUIKeyboardControl, this, std::placeholders::_1), node_options);
        sub_uwb_pose2d = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/uwb_pose2d", 1, std::bind(&MasterNode::callbackUwbPose2D, this, std::placeholders::_1), node_options);

        fsm_robot.value = TEST_UWB;

        // -----------------------------
        // Optional periodic behavior
        // -----------------------------
        timer_ = this->create_wall_timer(100ms, std::bind(&MasterNode::timerRoutine, this));
        // keyboard_command_timer_ = this->create_wall_timer(300ms, std::bind(&MasterNode::keyboardCommandRoutine, this));
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
        robot.pose_x = msg->pose_x;
        robot.pose_y = msg->pose_y;
        robot.pose_theta = msg->pose_theta;
        robot.vel_linear = msg->vel_linear;
        robot.vel_angular = msg->vel_angular;
        robot.nav_status_res = msg->nav_status_res;
        robot.nav_status_reason = msg->nav_status_reason;
        robot.nav_status_goal = msg->nav_status_goal;
        robot.nav_status_dist = msg->nav_status_dist;
        robot.nav_status_mileage = msg->nav_status_mileage;
        robot.mode = msg->mode;
        robot.battery_level = msg->battery_level;
        robot.charge_flag = msg->charge_flag;
        robot.emergency_flag = msg->emergency_flag;
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

    void callbackUIKeyboardControl(const std_msgs::msg::String::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Keyboard command: %s", msg->data.c_str());
        last_robot_command = msg->data;

        if (msg->data == "stop" || msg->data == "x")
        {
            sendVelocity(0.0, 0.0);
        }
    }

    void callbackUIButtonControl(const std_msgs::msg::Int8::SharedPtr msg)
    {
        int button_id = msg->data;

        switch (button_id)
        {
        case 1:
            RCLCPP_INFO(this->get_logger(), "Button 1 pressed.");
            sendVelocity(0.0, 0.5);
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "Button 2 pressed.");
            // Implement button 2 logic here
            sendVelocity(0.0, -0.5);
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "Button 3 pressed.");
            sendVelocity(1.0, 1.0);
            break;
        case 4:
        {
            cancelNav();
            std::this_thread::sleep_for(100ms);

            // Example: Navigate to a predefined point
            RCLCPP_INFO(this->get_logger(), "Button 4 pressed. Navigating to front of robot.");

            float x = robot.pose_x;
            float y = robot.pose_y;
            float yaw = robot.pose_theta;
            float d = 1.5; // jarak ke depan (meter)

            float x_front = x + d * std::cos(yaw);
            float y_front = y + d * std::sin(yaw);

            goTo(x_front, y_front, yaw);

            break;
        }
        case 5:
            RCLCPP_INFO(this->get_logger(), "Button 5 pressed. Canceling navigation.");
            cancelNav();
            std::this_thread::sleep_for(100ms);
            break;
        case 6:
            RCLCPP_INFO(this->get_logger(), "Button 6 pressed.");
            // Implement button 6 logic here
            move_to_uwb_offset();

            break;
        case 7:
            RCLCPP_INFO(this->get_logger(), "Button 7 pressed.");
            // Implement button 7 logic here
            break;
        case 8:
            RCLCPP_INFO(this->get_logger(), "Button 8 pressed.");
            // Implement button 8 logic here
            break;
        case 9:
            RCLCPP_INFO(this->get_logger(), "Button 9 pressed.");
            // Implement button 9 logic here
            sendVelocity(0.0, 0.2);
            std::this_thread::sleep_for(50ms);
            sendVelocity(0.0, 0.0);
            break;
        case 10:
            RCLCPP_INFO(this->get_logger(), "Button 10 pressed.");
            // Implement button 10 logic here
            sendVelocity(0.0, 0.5);

            break;
        case 11:
            RCLCPP_INFO(this->get_logger(), "Button 11 pressed.");
            // Implement button 11 logic here
            sendVelocity(0.0, 0.05);
            break;
        case 12:
            RCLCPP_INFO(this->get_logger(), "Button 12 pressed.");
            // Implement button 12 logic here
            sendVelocity(4.0, 0.0);

            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown button ID: %d", button_id);
            break;
        }

        // Implement button control logic here
    }

    void callbackUwbPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        // Update robot pose based on UWB data
        uwb_pose.pose_x = msg->x;
        uwb_pose.pose_y = msg->y;
        uwb_pose.pose_theta = msg->theta;
        RCLCPP_DEBUG(this->get_logger(), "UWB Pose updated: (%.2f, %.2f, %.2f)", msg->x, msg->y, msg->theta);
    }

    // ============================================================
    // TIMER ROUTINE
    // ============================================================
    void timerRoutine()
    {
        fsm_robot.value = TEST_UWB;

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
        case TEST_UWB:

            get_uwb_pose_offset();

            // move_to_uwb_offset();

            // logger.info("UWB Pose: %.2f, %.2f, %.2f | %.2f %.2f %.2f", uwb_pose_offset.pose_x, uwb_pose_offset.pose_y, uwb_pose_offset.pose_theta, robot.pose_x, robot.pose_y, robot.pose_theta);
            break;

        default:
            break;
        }
    }

    void keyboardCommandRoutine()
    {
        if (last_robot_command != "clear" && last_robot_command != "")
        {
            if (last_robot_command == "forward" || last_robot_command == "w")
            {
                sendVelocity(0.5, 0.0);
            }
            else if (last_robot_command == "backward" || last_robot_command == "s")
            {
                sendVelocity(-0.5, 0.0);
            }
            else if (last_robot_command == "left" || last_robot_command == "a")
            {
                sendVelocity(0.0, 0.5);
            }
            else if (last_robot_command == "right" || last_robot_command == "d")
            {
                sendVelocity(0.0, -0.5);
            }
            else if (last_robot_command == "stop" || last_robot_command == " ")
            {
                sendVelocity(0.0, 0.0);
            }
            logger.info("Last Command: %s", last_robot_command.c_str());
        }

        // ==============================

        // Clear the command after processing
        last_robot_command = "clear";
    }

    // ============================================================
    // MANUAL COMMAND METHODS (optional)
    // ============================================================

    void sendVelocity(float vx, float wz) // m/s, rad/s
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.angular.z = wz;
        pub_cmd_vel_->publish(msg);
    }

    void goTo(float x, float y, float th)
    {
        static float prev_x = x;
        static float prev_y = y;
        static float prev_th = th;
        static int8_t prev_res = -1;

        // logger.info("====== goto x: %.2f", fabs(prev_x - x));
        // logger.info("====== goto y: %.2f", fabs(prev_y - y));
        // logger.info("====== goto th: %.2f", fabs(prev_th - th));
        // logger.info("nav status res: %d | %d", robot.nav_status_res, robot.nav_status_reason);

        if (robot.nav_status_res == 1 || (robot.nav_status_res == 6 && prev_res == 6))
        {
            if (fabs(prev_x - x) < 0.5 && fabs(prev_y - y) < 0.5 && fabs(prev_th - th) < 0.3)
            {
                logger.info("goTo command to (%.2f, %.2f, %.2f) is same as previous. Skipping publish.", x, y, th);

                prev_x = x;
                prev_y = y;
                prev_th = th;
                return;
            }
        }

        prev_res = robot.nav_status_res;
        prev_x = x;
        prev_y = y;
        prev_th = th;

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

    // ============================================================
    // Motion Control Methods (optional)
    // ============================================================
    void get_robot_pose()
    {
        // publish get pose
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        pub_get_pose_->publish(msg);
    }

    void get_robot_nav()
    {
        // publish get nav
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        pub_get_nav_->publish(msg);
    }

    void get_uwb_pose_offset()
    {
        float dx = robot.pose_x - uwb_pose.pose_x;
        float dy = robot.pose_y - uwb_pose.pose_y;

        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 1e-3)
        {
            return;
        }
        else
        {
            float ux = dx / dist;
            float uy = dy / dist;

            uwb_pose_offset.pose_x = uwb_pose.pose_x + ux * offset_meter;
            uwb_pose_offset.pose_y = uwb_pose.pose_y + uy * offset_meter;
        }

        uwb_pose_offset.pose_theta = std::atan2(uwb_pose.pose_y - uwb_pose_offset.pose_y, uwb_pose.pose_x - uwb_pose_offset.pose_x);
    }

    void move_to_uwb_offset()
    {
        get_robot_pose();
        std::this_thread::sleep_for(10ms);
        get_robot_nav();
        std::this_thread::sleep_for(10ms);

        float error_angle = uwb_pose_offset.pose_theta - robot.pose_theta;

        while (error_angle > M_PI)
            error_angle -= 2 * M_PI;
        while (error_angle < -M_PI)
            error_angle += 2 * M_PI;

        if (abs(error_angle) > 0.1)
        {
            // sendVelocity(0.0, 0.5);
            goTo(robot.pose_x, robot.pose_y, uwb_pose_offset.pose_theta);
        }
        else
        {
            if (std::hypot(uwb_pose_offset.pose_x - robot.pose_x, uwb_pose_offset.pose_y - robot.pose_y) > 0.1)
            {
                sendVelocity(0.5, 0.0);
            }
            else
            {
                sendVelocity(0.0, 0.0);
            }
        }
        logger.info("Angle Error: %.2f", abs(error_angle));
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
