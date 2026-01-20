#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_interface/msg/robot.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <vector>
#include <cfloat> // untuk FLT_MAX

// ALIAS
using json = nlohmann::json;

using namespace std::chrono_literals;

#define GOTO 0
#define VEL 1
#define TEST_UWB 2

#define MODE_IDLE 0
#define MODE_TRACK_UWB 1
#define MODE_TRACK_CAMERA 2

typedef struct
{
    float pose_x;
    float pose_y;
    float pose_theta;
    float pose_theta_32_dir;

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

    float target_nav_pose_x;
    float target_nav_pose_y;
    float target_nav_pose_theta;

} robot_t;

struct Pose2D
{
    float x;
    float y;
    float yaw;
};

typedef struct
{
    float pose_x;
    float pose_y;
    float pose_theta;
} pose2d_t;

struct obstacle_checking_t
{
    uint8_t status = 0;
    float distance = 0.0f;
    float angle = 0.0f;
    float pos_x = 0.0f;
    float pos_y = 0.0f;
};

struct point2d_t
{
    float x;
    float y;
};

typedef struct
{
    float pose_x;
    float pose_y;
    float pose_theta;
    float pose_theta_32_dir;

    float prev_pose_x;
    float prev_pose_y;
    float prev_pose_theta;

    float offset_pose_x;
    float offset_pose_y;
    float offset_pose_theta;

    float right_pose_x;
    float right_pose_y;
    float right_pose_theta;

    float sit_pose_x;
    float sit_pose_y;
    float sit_pose_theta;

    float delta_linear;
    float vel_angular;

    int8_t mode;
    int8_t is_sitting;
    int8_t is_detected;
    int8_t is_moving;

    float leg_left;
    float leg_right;

    float distance_to_robot;
    float angle_to_robot;
    float angle_movement;
    float last_angle_movement;
    int8_t orientation;

} human_info_t;

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
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_get_lidar_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_person_global_position_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_person_distance_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_calibrate_uwb_;

    // publisher for ui web
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_ui_robot_pose2d_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_ui_robot_mode_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_ui_human_pose2d_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ui_human_velocity_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_ui_human_mode_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_ui_human_detected_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_ui_human_sitting_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_ui_robot_fsm_mode_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_ui_robot_following_mode_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_ui_target_nav_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_robot_pose_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_mode_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_battery_;
    rclcpp::Subscription<ros2_interface::msg::Robot>::SharedPtr sub_robot_info;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_status;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_ui_button_control;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ui_keyboard_control;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_uwb_pose2d;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vision_person_position;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_vision_pose_detected;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vision_person_orientation;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_dual_leg;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr keyboard_command_timer_;
    rclcpp::TimerBase::SharedPtr timer_mode_;

    MachineState fsm_robot;
    MachineState fsm_mode;

    int timer_counter_ms_ = 3000;

    // Temp Val glob
    int count_step = 0;
    json json_msg;
    bool sudahterkitim = 0;

    int8_t is_used_lidar_uwb = 0;
    int8_t prev_is_used_lidar_uwb = 0;

    // UWB Pose
    pose2d_t uwb_pose;
    pose2d_t uwb_pose_offset;
    const float offset_meter = 1.2; // 20 cm

    // lidar vars
    std::vector<std::pair<float, float>> coords;
    std::vector<std::pair<float, float>> coords_filtered;
    obstacle_checking_t obstacle_most_left;
    obstacle_checking_t obstacle_left;
    obstacle_checking_t obstacle_middle;
    obstacle_checking_t obstacle_right;
    obstacle_checking_t obstacle_most_right;
    obstacle_checking_t obstacle_front_uwb;

    // --------------------------
    std::string vision_person_position_ = "middle"; // most_left, left, middle, right, most_right
    int8_t person_detected_ = 0;
    float person_distance_ = 0.0f;
    pose2d_t person_global_pose_ = {0.0f, 0.0f, 0.0f};

    // Lidar filtering parameters
    const float LIDAR_MIN_RANGE = 0.3f;               // meters
    const float LIDAR_MAX_RANGE = 5.0f;               // meters
    const float LIDAR_FRONT_CONE_ANGLE = M_PI / 2.0f; // ±45° front cone

    int8_t is_move_to_human = 0;
    int8_t prev_is_move_to_human = 0;

    bool case6_nav_active = false;

    // ============================= Robot Command =============================
    std::string last_robot_command = "";

    robot_t robot;
    human_info_t human;

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
        pub_get_lidar_ = this->create_publisher<std_msgs::msg::Int8>("/get/lidar", 1);
        pub_person_global_position_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/person/global_position", 1);
        pub_person_distance_ = this->create_publisher<std_msgs::msg::Float32>("/person/distance", 1);
        pub_calibrate_uwb_ = this->create_publisher<std_msgs::msg::Float32>("/uwb/calibrate", 1);

        // publisher for ui web
        pub_ui_robot_pose2d_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/ui/robot/pose2d", 1);
        pub_ui_robot_mode_ = this->create_publisher<std_msgs::msg::Int8>("/ui/robot/mode", 1);
        pub_ui_human_pose2d_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/ui/human/pose2d", 1);
        pub_ui_human_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("/ui/human/velocity", 1);

        pub_ui_human_mode_ = this->create_publisher<std_msgs::msg::Int8>("/ui/human/mode", 1);
        pub_ui_human_detected_ = this->create_publisher<std_msgs::msg::Int8>("/ui/human/detected", 1);
        pub_ui_human_sitting_ = this->create_publisher<std_msgs::msg::Int8>("/ui/human/sitting", 1);

        pub_ui_robot_fsm_mode_ = this->create_publisher<std_msgs::msg::Int8>("/ui/robot/fsm_mode", 1);
        pub_ui_robot_following_mode_ = this->create_publisher<std_msgs::msg::Int8>("/ui/robot/following_mode", 1);
        pub_ui_target_nav_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/ui/target/nav", 1);

        // -----------------------------
        // Subscribers
        // -----------------------------
        sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/reeman/pose", 1, std::bind(&MasterNode::callbackRobotPose, this, std::placeholders::_1), node_options);
        // sub_robot_mode_ = this->create_subscription<std_msgs::msg::Int8>(
        //     "reeman/mode", 1, std::bind(&MasterNode::callbackRobotMode, this, std::placeholders::_1), node_options);
        // sub_robot_battery_ = this->create_subscription<std_msgs::msg::Int8>(
        //     "reeman/battery", 1, std::bind(&MasterNode::callbackBattery, this, std::placeholders::_1), node_options);
        sub_nav_status = this->create_subscription<std_msgs::msg::String>(
            "/reeman/nav_status", 1, std::bind(&MasterNode::callbackNavStatus, this, std::placeholders::_1), node_options);
        // sub_robot_info = this->create_subscription<ros2_interface::msg::Robot>(
        //     "/reeman/robot_info", 1, std::bind(&MasterNode::callbackRobotInfo, this, std::placeholders::_1), node_options);
        sub_ui_button_control = this->create_subscription<std_msgs::msg::Int8>(
            "/ui_control", 1, std::bind(&MasterNode::callbackUIButtonControl, this, std::placeholders::_1), node_options);
        sub_ui_keyboard_control = this->create_subscription<std_msgs::msg::String>(
            "/ui_keyboard_control", 1, bind(&MasterNode::callbackUIKeyboardControl, this, std::placeholders::_1), node_options);
        sub_uwb_pose2d = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/uwb_pose2d", 1, std::bind(&MasterNode::callbackUwbPose2D, this, std::placeholders::_1), node_options);
        sub_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/reeman/point_cloud", 1, std::bind(&MasterNode::callbackPointCloud, this, std::placeholders::_1), node_options);
        sub_vision_person_position = this->create_subscription<std_msgs::msg::String>(
            "/vision/person_position", 1, std::bind(&MasterNode::callbackVisionPersonPosition, this, std::placeholders::_1), node_options);
        sub_vision_pose_detected = this->create_subscription<std_msgs::msg::Int8>(
            "/vision/pose_detected", 1, std::bind(&MasterNode::callbackVisionPersonDetected, this, std::placeholders::_1), node_options);
        sub_vision_person_orientation = this->create_subscription<std_msgs::msg::String>(
            "/vision/person_orientation", 1, std::bind(&MasterNode::callbackVisionPersonOrientation, this, std::placeholders::_1), node_options);
        sub_dual_leg = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/dual_leg", 1, std::bind(&MasterNode::callbackDualLeg, this, std::placeholders::_1), node_options);

        fsm_robot.value = TEST_UWB;
        fsm_mode.value = MODE_TRACK_UWB;
        // fsm_mode.value = MODE_TRACK_CAMERA;

        // -----------------------------
        // Optional periodic behavior
        // -----------------------------
        timer_ = this->create_wall_timer(500ms, std::bind(&MasterNode::timerRoutine, this));
        // keyboard_command_timer_ = this->create_wall_timer(300ms, std::bind(&MasterNode::keyboardCommandRoutine, this));
        timer_mode_ = this->create_wall_timer(std::chrono::milliseconds(timer_counter_ms_), std::bind(&MasterNode::timerMode, this));
    }

    // ============================================================
    // CALLBACKS
    // ============================================================
    void callbackNavStatus(const std_msgs::msg::String::SharedPtr msg)
    {
        json_msg = json::parse(msg->data);

        robot.nav_status_res = json_msg["res"];
        robot.nav_status_reason = json_msg["reason"];

        // logger.info("Nav Status received: res = %d", robot.nav_status_res);
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
        robot.pose_x = msg->x;
        robot.pose_y = msg->y;
        robot.pose_theta = msg->theta;

        robot.pose_theta_32_dir = angle_to_dir32(msg->theta);
    }

    void callbackRobotMode(const std_msgs::msg::Int8::SharedPtr msg)
    {
        robot.mode = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Mode: %d", robot.mode);
    }

    void callbackBattery(const std_msgs::msg::Int8::SharedPtr msg)
    {
        robot.battery_level = msg->data;
        if (robot.battery_level <= 20)
        {
            RCLCPP_WARN(this->get_logger(), "Battery low (%d%%)", robot.battery_level);
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

        logger.info("Button [ %d ].", button_id);

        switch (button_id)
        {
        case 1:
            fsm_mode.value = MODE_IDLE;
            break;
        case 2:
            fsm_mode.value = MODE_TRACK_UWB;
            break;
        case 3:
            fsm_mode.value = MODE_TRACK_CAMERA;
            break;
        case 4:
            is_move_to_human = !is_move_to_human;
            break;
        case 5:
            goTo(robot.pose_x, robot.pose_y, human.pose_theta);
            break;
        case 6:
        {
            if (human.mode != -1)
            {
                robot.target_nav_pose_x = human.right_pose_x;
                robot.target_nav_pose_y = human.right_pose_y;
                robot.target_nav_pose_theta = human.right_pose_theta;
                goTo(human.right_pose_x, human.right_pose_y, human.right_pose_theta);
            }
            break;
        }
        case 7:
        {
            if (human.mode != -1)
            {
                robot.target_nav_pose_x = human.sit_pose_x;
                robot.target_nav_pose_y = human.sit_pose_y;
                robot.target_nav_pose_theta = human.sit_pose_theta;
                goTo(human.sit_pose_x, human.sit_pose_y, human.sit_pose_theta);
            }
            break;
        }
        case 8:
        {
            if (human.mode != -1)
            {
                robot.target_nav_pose_x = human.offset_pose_x;
                robot.target_nav_pose_y = human.offset_pose_y;
                robot.target_nav_pose_theta = human.offset_pose_theta;

                goTo(human.offset_pose_x, human.offset_pose_y, human.offset_pose_theta);
            }
            // if (human.mode != -1)
            // {

            //     if (human.is_sitting)
            //     {
            //         goTo(human.sit_pose_x, human.sit_pose_y, human.sit_pose_theta);
            //     }
            //     else
            //     {
            //         if (human.mode == 0)
            //         {
            //             goTo(human.right_pose_x, human.right_pose_y, human.right_pose_theta);
            //         }
            //         else
            //         {
            //             goTo(human.offset_pose_x, human.offset_pose_y, human.offset_pose_theta);
            //         }
            //     }
            // }
            break;
        }
        case 9:
            is_move_to_human = !is_move_to_human;
            break;
        case 10:
            break;
        case 11:
            break;
        case 12:
        {
            float delta_robot_uwb_theta = uwb_pose.pose_theta - robot.pose_theta;

            std_msgs::msg::Float32 calibrate_msg;
            calibrate_msg.data = delta_robot_uwb_theta;
            pub_calibrate_uwb_->publish(calibrate_msg);
            logger.info("Calibrate UWB by %.2f radian.", delta_robot_uwb_theta);
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown button ID: %d", button_id);
            break;
        }

        // Implement button control logic here
    }

    void callbackUwbPose2D(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        uwb_pose.pose_x = msg->x;
        uwb_pose.pose_y = msg->y;

        uwb_pose.pose_theta = msg->theta;

        // mapping theta to 8 directions
        // uwb_pose.pose_theta = angle_to_dir32(msg->theta);
    }

    float angle_to_dir32(float theta)
    {
        // change from [-pi, pi] to [0, 2pi]
        theta -= M_PI;
        theta = std::fmod(theta + 2 * M_PI, 2 * M_PI);

        float shifted = theta + M_PI; // [0, 2pi]
        int dir = static_cast<int>(shifted / (2 * M_PI) * 32);
        float res = static_cast<float>(dir % 32) * 11.25f;

        while (res > 180.0f)
            res -= 360.0f;
        while (res < -180.0f)
            res += 360.0f;

        return res; // safety clamp
    }

    void callbackVisionPersonPosition(const std_msgs::msg::String::SharedPtr msg)
    {
        vision_person_position_ = msg->data;
    }

    void callbackVisionPersonDetected(const std_msgs::msg::Int8::SharedPtr msg)
    {
        person_detected_ = msg->data;
    }

    void callbackVisionPersonOrientation(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "turned_away")
        {
            human.orientation = 0;
        }
        else
        {
            human.orientation = 1;
        }
    }

    void callbackDualLeg(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2)
        {
            human.leg_left = msg->data[0];
            human.leg_right = msg->data[1];

            if (human.leg_left > 200.0f)
                human.leg_left = 0.0;
            if (human.leg_right > 200.0f)
                human.leg_right = 0.0f;

            if ((human.leg_left <= 60.0f) && (human.leg_right <= 60.0f))
            {
                human.is_sitting = 0;
            }
            else
            {
                human.is_sitting = 1;
            }
        }
    }

    void callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // get the point cloud and save to coords vector
        coords.clear();
        for (size_t i = 0; i < msg->width * msg->height; ++i)
        {
            float x = *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + 0]);
            float y = *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + 4]);
            float z = *reinterpret_cast<const float *>(&msg->data[i * msg->point_step + 8]);

            // Filter out invalid points (NaN or Inf)
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
            {
                coords.emplace_back(x, y);
            }
        }
    }

    // Helper: Publish person global position
    void publishPersonGlobalPosition()
    {
        geometry_msgs::msg::Pose2D person_msg;
        person_msg.x = person_global_pose_.pose_x;
        person_msg.y = person_global_pose_.pose_y;
        person_msg.theta = person_global_pose_.pose_theta;
        pub_person_global_position_->publish(person_msg);

        std_msgs::msg::Float32 distance_msg;
        distance_msg.data = person_distance_;
        pub_person_distance_->publish(distance_msg);
    }

    void transmit_data_to_web()
    {
        // publish robot info to web ui
        geometry_msgs::msg::Pose2D robot_pose_msg;
        robot_pose_msg.x = robot.pose_x;
        robot_pose_msg.y = robot.pose_y;
        robot_pose_msg.theta = robot.pose_theta;
        pub_ui_robot_pose2d_->publish(robot_pose_msg);

        std_msgs::msg::Int8 robot_mode_msg;
        robot_mode_msg.data = robot.mode;
        pub_ui_robot_mode_->publish(robot_mode_msg);

        geometry_msgs::msg::Pose2D human_pose_msg;
        human_pose_msg.x = human.pose_x;
        human_pose_msg.y = human.pose_y;
        human_pose_msg.theta = human.pose_theta;
        pub_ui_human_pose2d_->publish(human_pose_msg);

        geometry_msgs::msg::Twist human_velocity_msg;
        human_velocity_msg.linear.x = human.delta_linear;
        human_velocity_msg.angular.z = human.vel_angular;
        pub_ui_human_velocity_->publish(human_velocity_msg);

        std_msgs::msg::Int8 human_mode_msg;
        human_mode_msg.data = human.is_moving; //
        pub_ui_human_mode_->publish(human_mode_msg);

        std_msgs::msg::Int8 robot_fsm_mode_msg;
        robot_fsm_mode_msg.data = fsm_mode.value;
        pub_ui_robot_fsm_mode_->publish(robot_fsm_mode_msg);

        geometry_msgs::msg::Pose2D target_nav_msg;
        target_nav_msg.x = robot.target_nav_pose_x;
        target_nav_msg.y = robot.target_nav_pose_y;
        target_nav_msg.theta = robot.target_nav_pose_theta;
        pub_ui_target_nav_->publish(target_nav_msg);

        std_msgs::msg::Int8 human_detected_msg;
        human_detected_msg.data = human.is_detected;
        pub_ui_human_detected_->publish(human_detected_msg);

        std_msgs::msg::Int8 human_sitting_msg;
        human_sitting_msg.data = human.is_sitting;
        pub_ui_human_sitting_->publish(human_sitting_msg);

        logger.info("[Transmit] mode: %d, sitting: %d, detected: %d, leg_l_r: %.2f %.2f", human.mode, human.is_sitting, human.is_detected, human.leg_left, human.leg_right);
    }

    // ============================================================
    // TIMER ROUTINE
    // ============================================================
    void timerMode()
    {
    }

    void timerRoutine()
    {
        get_lidar_data(); //
        std::this_thread::sleep_for(100ms);
        get_robot_pose(); //
        std::this_thread::sleep_for(100ms);

        if (fsm_mode.value == MODE_TRACK_UWB)
        {
            logger.info("[ MODE: TRACK UWB ]");
            get_human_from_uwb();
        }
        else if (fsm_mode.value == MODE_TRACK_CAMERA)
        {
            logger.info("[ MODE: TRACK CAMERA ]");
            get_human_from_camera();
        }

        get_human_pose_offset_back();
        get_human_pose_offset_right();
        get_human_pose_offset_sit();

        //? AUTO FOLLOW HUMAN
        if (is_move_to_human && human.is_detected)
        {
            human_following();
        }

        prev_is_move_to_human = is_move_to_human;

        logger.info("Robot: %.2f %.2f %.2f | Human: %.2f %.2f %.2f",
                    robot.pose_x, robot.pose_y, robot.pose_theta,
                    human.pose_x, human.pose_y, human.pose_theta);

        // if (human.mode == 0)
        // {
        //     logger.info("HUMAN IS STANDING");
        // }
        // else if (human.mode == 1)
        // {
        //     logger.info("HUMAN IS WALKING");
        // }
        // else if (human.mode == 2)
        // {
        //     logger.info("HUMAN IS SITTING");
        // }
        // else if (human.mode == -1)
        // {
        //     logger.info("HUMAN NOT DETECTED, SKIP FOLLOWING");
        // }

        transmit_data_to_web();
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
        logger.info("Navigating to (%.2f, %.2f, %.2f)", x, y, th);

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

    void get_lidar_data()
    {
        // publish get lidar
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        pub_get_lidar_->publish(msg);
    }

    void human_following()
    {
        static float last_x = 0.0f;
        static float last_y = 0.0f;

        static float delta_move_x = 0.0f;
        static float delta_move_y = 0.0f;

        static int8_t prev_mode = 0;
        static int8_t prev_sitting = 0;
        static int8_t is_person_left_area = 0;
        static int16_t cntr_reentry = 0;

        static pose2d_t used_human_pose = {0.0f, 0.0f, 0.0f};

        if (human.is_detected == 0)
        {
            logger.info("HUMAN NOT DETECTED, SKIP FOLLOWING");
            // cancelNav();
            return;
        }

        if (prev_mode == 0)
        {
            if (human.is_sitting)
            {
                logger.info("----------- PREV MODE: SITTING");
            }
            else
            {
                logger.info("----------- PREV MODE: STANDING");
            }
        }
        else if (prev_mode == 1)
            logger.info("----------- PREV MODE: WALKING");
        else if (prev_mode == -1)
            logger.info("----------- PREV MODE: NOT DETECTED");

        if (human.mode == 0)
        {
            logger.info("HUMAN IS STAY");
            used_human_pose.pose_x = human.right_pose_x;
            used_human_pose.pose_y = human.right_pose_y;
            used_human_pose.pose_theta = human.right_pose_theta;
        }
        else if (human.mode == 1)
        {
            logger.info("HUMAN IS WALKING");
            used_human_pose.pose_x = human.offset_pose_x;
            used_human_pose.pose_y = human.offset_pose_y;
            used_human_pose.pose_theta = human.offset_pose_theta;
        }

        if (human.is_sitting)
        {
            logger.info("HUMAN IS SITTING");
            used_human_pose.pose_x = human.sit_pose_x;
            used_human_pose.pose_y = human.sit_pose_y;
            used_human_pose.pose_theta = human.sit_pose_theta;
        }
        else
        {
            logger.info("HUMAN IS NOT SITTING");
        }

        delta_move_x = fabs(used_human_pose.pose_x - last_x);
        delta_move_y = fabs(used_human_pose.pose_y - last_y);

        is_person_left_area = (delta_move_x > 0.5f || delta_move_y > 0.5f);

        if (cntr_reentry > 0)
        {
            cntr_reentry--;
            return;
        }

        int8_t delta_use_lidar_uwb = (prev_is_used_lidar_uwb == 0 && is_used_lidar_uwb == 1) ? 1
                                                                                             : 0;
        int8_t delta_sitting = (prev_sitting != human.is_sitting) ? 1
                                                                  : 0;
        int8_t delta_is_mover_to_human = (prev_is_move_to_human == 0 && is_move_to_human == 1) ? 1
                                                                                               : 0;

        if (is_person_left_area || delta_sitting || delta_use_lidar_uwb || delta_is_mover_to_human)
        {
            if (robot.nav_status_res != 1 || prev_mode != human.mode || delta_sitting || delta_use_lidar_uwb || delta_is_mover_to_human)
            {
                logger.info("==========================");
                logger.info("     MOVING TO HUMAN      ");
                logger.info("==========================");

                robot.target_nav_pose_x = used_human_pose.pose_x;
                robot.target_nav_pose_y = used_human_pose.pose_y;
                robot.target_nav_pose_theta = used_human_pose.pose_theta;

                goTo(robot.target_nav_pose_x, robot.target_nav_pose_y, robot.target_nav_pose_theta);

                last_x = used_human_pose.pose_x;
                last_y = used_human_pose.pose_y;

                cntr_reentry = 5;
            }
            else
            {
                if (fabs(robot.pose_x - last_x) < 1.5f && fabs(robot.pose_y - last_y) < 1.5f)
                {
                    // logger.info("CANCEL NAV TO HUMAN");

                    cancelNav();

                    cntr_reentry = 2;
                }
            }
        }

        prev_mode = human.mode;
        prev_sitting = human.is_sitting;
        prev_is_used_lidar_uwb = is_used_lidar_uwb;

        // logger.info("%d %d | %d %d | %.2f %.2f", is_person_moving, robot.nav_status_res, human.mode, cntr_reentry, delta_move_x, delta_move_y);
    }

    void get_human_pose_offset_back()
    {
        float dx = robot.pose_x - human.pose_x;
        float dy = robot.pose_y - human.pose_y;

        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist > 1.5f)
        {
            if (dist < 1e-3)
            {
                return; //
            }
            else //
            {
                float ux = dx / dist;
                float uy = dy / dist;

                human.offset_pose_x = human.pose_x + ux * 1.5; //
                human.offset_pose_y = human.pose_y + uy * 1.5;
            }

            human.offset_pose_theta = std::atan2(human.pose_y - robot.pose_y, human.pose_x - robot.pose_x);
        }
        else
        {
            human.offset_pose_x = robot.pose_x;
            human.offset_pose_y = robot.pose_y;

            human.offset_pose_theta = std::atan2(human.pose_y - robot.pose_y, human.pose_x - robot.pose_x);
        }
    }

    void get_human_pose_offset_right()
    {
        // float angle_right = human.last_angle_movement - M_PI / 2.0f;
        float angle_right = human.pose_theta - M_PI / 2.0f;

        human.right_pose_x = human.pose_x + 0.9 * std::cos(angle_right);
        human.right_pose_y = human.pose_y + 0.9 * std::sin(angle_right);
        human.right_pose_theta = human.pose_theta;
    }

    void get_human_pose_offset_sit()
    {
        float angle_sit = human.pose_theta - M_PI / 2.0f;

        human.sit_pose_x = human.pose_x + 0.9 * std::cos(angle_sit);
        human.sit_pose_y = human.pose_y + 0.9 * std::sin(angle_sit);
        human.sit_pose_theta = human.pose_theta + (M_PI * 0.25f);
    }

    void proccess_lidar_for_uwb()
    {
        Pose2D pose{robot.pose_x, robot.pose_y, robot.pose_theta};

        coords_filtered.clear();
        for (const auto &p : coords)
        {
            float dist = sqrtf((p.first - pose.x) * (p.first - pose.x) +
                               (p.second - pose.y) * (p.second - pose.y));
            if (dist <= 6.0f)
            {
                coords_filtered.push_back(p);
            }
        }

        std::vector<point2d_t> area_front = {
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {5.0f, 0.4854 * 5.0f},
            {5.0f, -0.4854 * 5.0f}};

        obstacle_front_uwb = checkLidarAreaRect(coords_filtered, pose, area_front);
    }

    void process_lidar()
    {
        // get start time for lidar processing from chrono
        auto start_time = std::chrono::high_resolution_clock::now();
        Pose2D pose{robot.pose_x, robot.pose_y, robot.pose_theta};

        float slope_1 = 0.3854;
        float slope_2 = 0.22916;
        float slope_3 = 0.0770833;
        float slope_4 = -0.0770833;
        float slope_5 = -0.22916;
        float slope_6 = -0.3854;

        // 2. Filter titik LiDAR yang terlalu jauh
        coords_filtered.clear();
        for (const auto &p : coords)
        {
            float dist = sqrtf((p.first - pose.x) * (p.first - pose.x) +
                               (p.second - pose.y) * (p.second - pose.y));
            if (dist <= 6.0f)
            {
                coords_filtered.push_back(p);
            }
        }

        // 3. Definisikan beberapa area deteksi (dalam frame lokal robot)
        std::vector<point2d_t> area_front_most_left = {
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {5.0f, slope_1 * 5.0f},
            {5.0f, slope_2 * 5.0f}};

        std::vector<point2d_t> area_front_left = {
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {5.0f, slope_2 * 5.0f},
            {5.0f, slope_3 * 5.0f}};

        std::vector<point2d_t> area_front_middle = {
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {5.0f, slope_3 * 5.0f},
            {5.0f, slope_4 * 5.0f}};

        std::vector<point2d_t> area_front_right = {
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {5.0f, slope_4 * 5.0f},
            {5.0f, slope_5 * 5.0f}};

        std::vector<point2d_t> area_front_most_right = {
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {5.0f, slope_5 * 5.0f},
            {5.0f, slope_6 * 5.0f}};

        // 4. Jalankan deteksi di tiap area
        obstacle_most_left = checkLidarAreaRect(coords_filtered, pose, area_front_most_left);
        obstacle_left = checkLidarAreaRect(coords_filtered, pose, area_front_left);
        obstacle_middle = checkLidarAreaRect(coords_filtered, pose, area_front_middle);
        obstacle_right = checkLidarAreaRect(coords_filtered, pose, area_front_right);
        obstacle_most_right = checkLidarAreaRect(coords_filtered, pose, area_front_most_right);

        // get end time for lidar processing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> lidar_processing_duration = end_time - start_time;
    }

    obstacle_checking_t checkLidarAreaRect(const std::vector<std::pair<float, float>> &coords,
                                           const Pose2D &robot_pose,
                                           const std::vector<point2d_t> &area_local)
    {
        obstacle_checking_t result;
        float min_dist = FLT_MAX;
        bool found = false;

        auto isLeft = [](const point2d_t &p1, const point2d_t &p2, const point2d_t &p)
        {
            float val = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
            return (val > 0) ? 1 : 0;
        };

        for (const auto &p : coords)
        {
            // Transform global → robot frame
            float x_robot = cosf(-robot_pose.yaw) * (p.first - robot_pose.x) -
                            sinf(-robot_pose.yaw) * (p.second - robot_pose.y);
            float y_robot = sinf(-robot_pose.yaw) * (p.first - robot_pose.x) +
                            cosf(-robot_pose.yaw) * (p.second - robot_pose.y);

            // Cek apakah titik di dalam polygon area_local
            std::vector<uint8_t> left_flags;
            for (size_t i = 0; i < area_local.size(); i++)
            {
                const point2d_t &a = area_local[i];
                const point2d_t &b = area_local[(i + 1) % area_local.size()];
                left_flags.push_back(isLeft(a, b, {x_robot, y_robot}));
            }

            bool inside = std::all_of(left_flags.begin(), left_flags.end(),
                                      [&](uint8_t f)
                                      { return f == left_flags.front(); });

            if (inside)
            {
                float dist = sqrtf(x_robot * x_robot + y_robot * y_robot);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    result.status = 1;
                    result.distance = dist;
                    result.angle = atan2f(y_robot, x_robot);
                    result.pos_x = p.first;
                    result.pos_y = p.second;
                    found = true;
                }
            }
        }

        if (!found)
            result.status = 0;

        return result;
    }

    void get_human_from_camera()
    {
        process_lidar();

        static obstacle_checking_t obs_used;

        if (person_detected_)
        {
            if (obstacle_middle.status && vision_person_position_ == "middle")
            {

                obs_used = obstacle_middle;
            }
            else if (obstacle_left.status && vision_person_position_ == "left")
            {
                obs_used = obstacle_left;
            }
            else if (obstacle_right.status && vision_person_position_ == "right")
            {
                obs_used = obstacle_right;
            }
            else if (obstacle_most_right.status && vision_person_position_ == "most_right")
            {
                obs_used = obstacle_most_right;
            }
            else if (obstacle_most_left.status && vision_person_position_ == "most_left")
            {
                obs_used = obstacle_most_left;
            }
            else
            {
                obs_used.status = 0; // no obstacle
            }
            get_human_from_lidar(obs_used);
        }
        else
        {
            human.is_detected = 0;
            logger.info("[  X  ] No person detected from camera.");
        }
    }

    void get_human_from_lidar(obstacle_checking_t obs_used)
    {

        static int8_t cntr_change_mode = 0;
        static int8_t mode_polling = 0;
        static point2d_t buff_lidar_pos = {0.0f, 0.0f};

        if (obs_used.status)
        {
            human.prev_pose_x = human.pose_x;
            human.prev_pose_y = human.pose_y;
            human.prev_pose_theta = human.pose_theta;

            float dx = obs_used.pos_x - robot.pose_x;
            float dy = obs_used.pos_y - robot.pose_y;

            // human.pose_x = robot.pose_x + dx;
            // human.pose_y = robot.pose_y + dy;

            buff_lidar_pos.x = obs_used.pos_x;
            buff_lidar_pos.y = obs_used.pos_y;

            if (fabs(buff_lidar_pos.x - human.pose_x) + fabs(buff_lidar_pos.y - human.pose_y) < 0.5f)
            {
                human.pose_x = buff_lidar_pos.x;
                human.pose_y = buff_lidar_pos.y;
            }
            else
            {
                human.pose_x = obs_used.pos_x;
                human.pose_y = obs_used.pos_y;
            }

            float angle_to_human = human.orientation == 1 ? std::atan2(dy, dx) + M_PI : std::atan2(dy, dx);

            while (angle_to_human > M_PI)
                angle_to_human -= 2.0f * M_PI;
            while (angle_to_human < -M_PI)
                angle_to_human += 2.0f * M_PI;

            human.pose_theta = angle_to_human;

            human.distance_to_robot = std::sqrt(dx * dx + dy * dy);
            human.angle_to_robot = std::atan2(dy, dx);

            human.is_detected = 1;
            human.is_moving = 0;

            if (fabs(human.pose_x - human.prev_pose_x) > 0.1 || fabs(human.pose_y - human.prev_pose_y) > 0.1)
            {
                human.is_moving = 1;
            }

            if (fabs(human.pose_x - human.prev_pose_x) > 0.4 || fabs(human.pose_y - human.prev_pose_y) > 0.4)
            {
                human.delta_linear = std::sqrt(std::pow(human.pose_x - human.prev_pose_x, 2) + std::pow(human.pose_y - human.prev_pose_y, 2)); // assuming callback every 500ms
                human.angle_movement = (std::atan2(human.pose_y - human.prev_pose_y, human.pose_x - human.prev_pose_x));
                human.last_angle_movement = human.angle_movement;

                mode_polling = 1;
                human.mode = 1;
            }
            else
            {
                human.delta_linear = 0.0f;
                human.angle_movement = 0.0f;

                mode_polling = 0;
                human.mode = 0;
            }
        }
        else
        {
            human.distance_to_robot = 0.0f;
            human.is_detected = 0;
        }
    }

    void get_human_from_uwb()
    {

        static int8_t polling_stay = 0;
        static int16_t polling_move = 0;

        static point2d_t buff_lidar_pos = {0.0f, 0.0f};

        human.prev_pose_x = human.pose_x;
        human.prev_pose_y = human.pose_y;
        human.prev_pose_theta = human.pose_theta;

        human.pose_x = uwb_pose.pose_x;
        human.pose_y = uwb_pose.pose_y;
        human.pose_theta = uwb_pose.pose_theta;

        human.pose_theta_32_dir = angle_to_dir32(human.pose_theta);

        human.distance_to_robot = std::sqrt(std::pow(human.pose_x - robot.pose_x, 2) + std::pow(human.pose_y - robot.pose_y, 2));
        human.angle_to_robot = std::atan2(human.pose_y - robot.pose_y, human.pose_x - robot.pose_x);

        human.is_detected = 1;

        proccess_lidar_for_uwb();

        is_used_lidar_uwb = 0;

        if (obstacle_front_uwb.status)
        {
            // check the obs if the obstacle is closer to human
            if (fabs(obstacle_front_uwb.pos_x - human.pose_x) + fabs(obstacle_front_uwb.pos_y - human.pose_y) < 0.8f)
            {
                buff_lidar_pos.x = obstacle_front_uwb.pos_x;
                buff_lidar_pos.y = obstacle_front_uwb.pos_y;
            }
        }

        if (fabs(buff_lidar_pos.x - human.pose_x) + fabs(buff_lidar_pos.y - human.pose_y) < 1.0f)
        {
            logger.info("============== OVERRIDE UWB POSITION WITH LIDAR DATA ==============");

            human.pose_x = buff_lidar_pos.x;
            human.pose_y = buff_lidar_pos.y;

            is_used_lidar_uwb = 1;
        }

        human.is_moving = 0;
        if (fabs(human.pose_x - human.prev_pose_x) > 0.1 || fabs(human.pose_y - human.prev_pose_y) > 0.1)
        {
            human.is_moving = 1;
        }

        if (fabs(human.pose_x - human.prev_pose_x) > 0.4 || fabs(human.pose_y - human.prev_pose_y) > 0.4)
        {
            human.delta_linear = std::sqrt(std::pow(human.pose_x - human.prev_pose_x, 2) + std::pow(human.pose_y - human.prev_pose_y, 2)); // assuming callback every 500ms
            human.angle_movement = (std::atan2(human.pose_y - human.prev_pose_y, human.pose_x - human.prev_pose_x));
            human.last_angle_movement = human.angle_movement;

            polling_move += 1;

            if (polling_move >= 1)
            {
                human.mode = 1;

                if (polling_move > 10)
                {
                    polling_move = 10;
                }
            }
            polling_stay = 0;
        }
        else
        {
            human.delta_linear = 0.0f;
            human.angle_movement = 0.0f;

            polling_stay += 1; //

            if (polling_stay >= 1)
            {
                human.mode = 0;

                if (polling_stay > 10)
                {
                    polling_stay = 10;
                }
            }
            polling_move = 0;
        }

        // logger.info("poll: %d %d | mode: %d", polling_stay, polling_move, human.mode);

        // logger.info("__UWB: %.2f, %.2f, %.2f | Distance: %.2f m",
        //             human.pose_x, human.pose_y, human.pose_theta, std::sqrt(std::pow(human.pose_x - robot.pose_x, 2) + std::pow(human.pose_y - robot.pose_y, 2)));

        // logger.info("delta linear: %.2f | angle: %.2f -> %.2f", human.delta_linear, human.angle_movement, human.last_angle_movement);
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
