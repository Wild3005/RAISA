#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
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

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr keyboard_command_timer_;
    rclcpp::TimerBase::SharedPtr timer_mode_;

    // Internal state tracking
    geometry_msgs::msg::Pose2D last_pose_;
    int robot_mode_ = -1;
    int battery_ = 100;

    MachineState fsm_robot;
    MachineState fsm_mode;

    int timer_counter_ms_ = 5000;

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

    // lidar vars
    std::vector<std::pair<float, float>> coords;
    std::vector<std::pair<float, float>> coords_filtered;
    obstacle_checking_t obstacle_most_left;
    obstacle_checking_t obstacle_left;
    obstacle_checking_t obstacle_middle;
    obstacle_checking_t obstacle_right;
    obstacle_checking_t obstacle_most_right;

    // --------------------------
    std::string vision_person_position_ = "middle"; // most_left, left, middle, right, most_right
    int8_t person_detected_ = 0;
    float person_distance_ = 0.0f;
    pose2d_t person_global_pose_ = {0.0f, 0.0f, 0.0f};

    // Lidar filtering parameters
    const float LIDAR_MIN_RANGE = 0.3f;               // meters
    const float LIDAR_MAX_RANGE = 5.0f;               // meters
    const float LIDAR_FRONT_CONE_ANGLE = M_PI / 2.0f; // ±45° front cone

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
        pub_get_lidar_ = this->create_publisher<std_msgs::msg::Int8>("/get/lidar", 1);
        pub_person_global_position_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/person/global_position", 1);
        pub_person_distance_ = this->create_publisher<std_msgs::msg::Float32>("/person/distance", 1);

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

        fsm_robot.value = TEST_UWB;
        fsm_mode.value = MODE_IDLE;

        // -----------------------------
        // Optional periodic behavior
        // -----------------------------
        timer_ = this->create_wall_timer(200ms, std::bind(&MasterNode::timerRoutine, this));
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

        logger.info("Nav Status received: res = %d", robot.nav_status_res);
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
            logger.info("Button 5 pressed.");
            // cancelNav();

            get_robot_pose();

            break;
        case 6:
            logger.info("Button 6 pressed.");

            get_lidar_data();
            std::this_thread::sleep_for(200ms);
            get_robot_pose();
            std::this_thread::sleep_for(100ms);

            process_lidar();

            if (obstacle_middle.status && vision_person_position_ == "middle")
            {
                logger.info("Obstacle detected at middle: distance=%.2f m, angle=%.2f rad",
                            obstacle_middle.distance, obstacle_middle.angle);
                logger.info("obs x: %.2f m, obs y: %.2f m",
                            obstacle_middle.pos_x, obstacle_middle.pos_y);
            }
            else if (obstacle_left.status && vision_person_position_ == "left")
            {
                logger.info("Obstacle detected at left: distance=%.2f m, angle=%.2f rad",
                            obstacle_left.distance, obstacle_left.angle);
                logger.info("obs x: %.2f m, obs y: %.2f m",
                            obstacle_left.pos_x, obstacle_left.pos_y);
            }
            else if (obstacle_right.status && vision_person_position_ == "right")
            {
                logger.info("Obstacle detected at right: distance=%.2f m, angle=%.2f rad",
                            obstacle_right.distance, obstacle_right.angle);
                logger.info("obs x: %.2f m, obs y: %.2f m",
                            obstacle_right.pos_x, obstacle_right.pos_y);
            }
            else if (obstacle_most_right.status && vision_person_position_ == "most_right")
            {
                logger.info("Obstacle detected at most right: distance=%.2f m, angle=%.2f rad",
                            obstacle_most_right.distance, obstacle_most_right.angle);
                logger.info("obs x: %.2f m, obs y: %.2f m",
                            obstacle_most_right.pos_x, obstacle_most_right.pos_y);
            }
            else if (obstacle_most_left.status && vision_person_position_ == "most_left")
            {
                logger.info("Obstacle detected at most left: distance=%.2f m, angle=%.2f rad",
                            obstacle_most_left.distance, obstacle_most_left.angle);
                logger.info("obs x: %.2f m, obs y: %.2f m",
                            obstacle_most_left.pos_x, obstacle_most_left.pos_y);
            }

            break;
        case 7:
            logger.info("Button 7 pressed.");
            fsm_mode.value = MODE_IDLE;

            break;
        case 8:
            logger.info("Button 8 pressed.");

            fsm_mode.value = MODE_TRACK_UWB;
            break;
        case 9:
            logger.info("Button 9 pressed.");
            fsm_mode.value = MODE_TRACK_CAMERA;

            break;
        case 10:
            logger.info("Button 10 pressed.");
            {
                static obstacle_checking_t obs_used;

                if (person_detected_)
                {
                    get_lidar_data();
                    std::this_thread::sleep_for(200ms);
                    get_lidar_data();
                    std::this_thread::sleep_for(200ms);
                    get_robot_pose();
                    std::this_thread::sleep_for(200ms);
                    get_robot_pose();
                    std::this_thread::sleep_for(200ms);
                    process_lidar();
                    std::this_thread::sleep_for(200ms);

                    if (obstacle_middle.status && vision_person_position_ == "middle")
                    {
                        logger.info("Using obstacle at middle.");
                        obs_used = obstacle_middle;
                    }
                    else if (obstacle_left.status && vision_person_position_ == "left")
                    {
                        logger.info("Using obstacle at left.");
                        obs_used = obstacle_left;
                    }
                    else if (obstacle_right.status && vision_person_position_ == "right")
                    {
                        logger.info("Using obstacle at right.");
                        obs_used = obstacle_right;
                    }
                    else if (obstacle_most_right.status && vision_person_position_ == "most_right")
                    {
                        logger.info("Using obstacle at most right.");
                        obs_used = obstacle_most_right;
                    }
                    else if (obstacle_most_left.status && vision_person_position_ == "most_left")
                    {
                        logger.info("Using obstacle at most left.");
                        obs_used = obstacle_most_left;
                    }

                    move_to_camera_person(obs_used);
                }
            }

            break;
        case 11:
            logger.info("Button 11 pressed.");
            break;
        case 12:
            logger.info("Button 12 pressed.");

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
        // RCLCPP_DEBUG(this->get_logger(), "UWB Pose updated: (%.2f, %.2f, %.2f)", msg->x, msg->y, msg->theta);
    }

    void callbackVisionPersonPosition(const std_msgs::msg::String::SharedPtr msg)
    {
        vision_person_position_ = msg->data;
    }

    void callbackVisionPersonDetected(const std_msgs::msg::Int8::SharedPtr msg)
    {
        person_detected_ = msg->data;
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

    void callbackPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        get_robot_pose();

        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %u points", msg->width * msg->height);

        // log all data x,y from points
        std::vector<std::pair<float, float>> points = extractPointsFromCloud(msg);
        if (points.empty())
        {
            return;
        }
        for (const auto &[x, y] : points)
        {
            logger.info("Point: (%.2f, %.2f)", x, y);
        }

        if (!person_detected_)
        {
            RCLCPP_INFO(this->get_logger(), "No person detected by vision, skipping point cloud processing");
            return; // No person detected by camera, skip processing
        }

        // Parse point cloud data

        logger.info("Extracted %zu points from point cloud", points.size());

        // Get angle range based on camera detection zone
        auto [angle_min, angle_max] = getAngleRangeForVisionZone(vision_person_position_);

        // Filter points: front cone + camera direction + distance range
        float closest_distance = std::numeric_limits<float>::max();
        float closest_angle = 0.0f;

        for (const auto &[x, y] : points)
        {
            // Calculate distance and angle in robot frame
            float distance = std::sqrt(x * x + y * y);
            float angle = std::atan2(y, x);

            // Filter by distance range
            if (distance < LIDAR_MIN_RANGE || distance > LIDAR_MAX_RANGE)
                continue;

            // Filter by front cone
            if (std::abs(angle) > LIDAR_FRONT_CONE_ANGLE)
                continue;

            // Filter by camera vision zone
            if (angle < angle_min || angle > angle_max)
                continue;

            // Find closest point in the zone
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_angle = angle;
            }
        }

        logger.info("robot pose: (%.2f, %.2f, %.2f)", robot.pose_x, robot.pose_y, robot.pose_theta);

        // If we found a valid point, transform to global coordinates
        if (closest_distance < std::numeric_limits<float>::max())
        {
            person_distance_ = closest_distance;

            // Transform from robot-local to global coordinates
            float global_angle = robot.pose_theta + closest_angle;
            person_global_pose_.pose_x = robot.pose_x + closest_distance * std::cos(global_angle);
            person_global_pose_.pose_y = robot.pose_y + closest_distance * std::sin(global_angle);
            person_global_pose_.pose_theta = 0.0f; // Not estimating person orientation

            // Publish results
            publishPersonGlobalPosition();

            logger.info("Person detected at distance %.2fm, global position (%.2f, %.2f)",
                        person_distance_, person_global_pose_.pose_x, person_global_pose_.pose_y);
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

    // ============================================================
    // TIMER ROUTINE
    // ============================================================
    void timerMode()
    {

        switch (fsm_mode.value)
        {
        case MODE_TRACK_UWB:
        {
            // Example periodic UWB handling
            get_robot_pose();

            get_uwb_pose_offset();

            move_to_uwb_offset();

            logger.info("UWB Pose: %.2f, %.2f, %.2f | %.2f %.2f %.2f", uwb_pose_offset.pose_x, uwb_pose_offset.pose_y, uwb_pose_offset.pose_theta, robot.pose_x, robot.pose_y, robot.pose_theta);

            break;
        }
        case MODE_TRACK_CAMERA:
        {
            static obstacle_checking_t obs_used;

            if (person_detected_)
            {
                get_lidar_data();
                std::this_thread::sleep_for(200ms);
                get_robot_pose();
                std::this_thread::sleep_for(100ms);
                process_lidar();
                std::this_thread::sleep_for(100ms);

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

                move_to_camera_person(obs_used);
            }
            break;
        }
        default:
            break;
        }
    }

    void timerRoutine()
    {
        get_lidar_data();
        std::this_thread::sleep_for(200ms);
        get_robot_pose();
        std::this_thread::sleep_for(200ms);
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
        std::this_thread::sleep_for(50ms);

        static float prev_x = uwb_pose_offset.pose_x;
        static float prev_y = uwb_pose_offset.pose_y;
        static float prev_theta = uwb_pose.pose_theta;
        float deg_theshold = 30.0f; // degrees

        // Only send goTo command if the target position has changed significantly
        if (std::fabs(uwb_pose_offset.pose_x - prev_x) < 1.0f &&
            std::fabs(uwb_pose_offset.pose_y - prev_y) < 1.0f
            // && std::fabs(uwb_pose.pose_theta - prev_theta) < (deg_theshold * 0.017453293f)g
        )
        {
            logger.info("SKIP move");
        }
        else
        {
            goTo(uwb_pose_offset.pose_x, uwb_pose_offset.pose_y, uwb_pose.pose_theta);
        }

        prev_x = uwb_pose_offset.pose_x;
        prev_y = uwb_pose_offset.pose_y;
        prev_theta = uwb_pose.pose_theta;
    }

    void move_to_camera_person(obstacle_checking_t obs_used)
    {
        pose2d_t target_offset;
        static float prev_x = target_offset.pose_x;
        static float prev_y = target_offset.pose_y;
        static float prev_theta = target_offset.pose_theta;

        target_offset = get_near_obstacle_point(obs_used);

        if (std::fabs(target_offset.pose_x - prev_x) < 1.0f &&
            std::fabs(target_offset.pose_y - prev_y) < 1.0f
            // && std::fabs(target_offset.pose_theta - prev_theta) < (deg_theshold * 0.017453293f)
        )
        {
            logger.info("SKIP move");
        }
        else
        {

            goTo(target_offset.pose_x, target_offset.pose_y, target_offset.pose_theta);
        }

        prev_x = target_offset.pose_x;
        prev_y = target_offset.pose_y;
        prev_theta = target_offset.pose_theta;
    }

    // Helper: Extract (x, y) points from PointCloud2 message
    std::vector<std::pair<float, float>> extractPointsFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
    {
        std::vector<std::pair<float, float>> points;

        // PointCloud2 data is stored as binary blob
        // Assuming format: x (float32), y (float32), z (float32) = 12 bytes per point
        const size_t point_step = msg->point_step; // Should be 12 for xyz
        const size_t num_points = msg->width * msg->height;

        points.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i)
        {
            size_t offset = i * point_step;

            if (offset + 8 > msg->data.size())
                break;

            float x, y;
            std::memcpy(&x, &msg->data[offset], sizeof(float));
            std::memcpy(&y, &msg->data[offset + 4], sizeof(float));

            points.emplace_back(x, y);
        }

        return points;
    }

    // Helper: Get angle range for camera vision zone
    std::pair<float, float> getAngleRangeForVisionZone(const std::string &zone)
    {
        // Convert camera zones to lidar angle ranges (in radians)
        // Robot forward = 0°, left = positive, right = negative

        if (zone == "most_left")
            return {M_PI / 3.0f, M_PI / 2.0f}; // 60° to 90°
        else if (zone == "left")
            return {M_PI / 10.0f, M_PI / 3.0f}; // 18° to 60°
        else if (zone == "middle")
            return {-M_PI / 10.0f, M_PI / 10.0f}; // -18° to +18°
        else if (zone == "right")
            return {-M_PI / 3.0f, -M_PI / 10.0f}; // -60° to -18°
        else if (zone == "most_right")
            return {-M_PI / 2.0f, -M_PI / 3.0f}; // -90° to -60°
        else
            return {-M_PI / 10.0f, M_PI / 10.0f}; // Default to middle
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

    pose2d_t get_near_obstacle_point(obstacle_checking_t obs_used)
    {

        pose2d_t target_point;

        float d_stop = 1.5f;
        float dx = obs_used.pos_x - robot.pose_x;
        float dy = obs_used.pos_y - robot.pose_y;
        float distance_to_obstacle = sqrtf(dx * dx + dy * dy);

        float scale = (distance_to_obstacle - d_stop) / distance_to_obstacle;
        target_point.pose_x = robot.pose_x + (dx * scale);
        target_point.pose_y = robot.pose_y + (dy * scale);

        target_point.pose_theta = atan2f(target_point.pose_y - robot.pose_y, target_point.pose_x - robot.pose_x);

        logger.info("Target Point to near obstacle: (%.2f, %.2f, %.2f)", target_point.pose_x, target_point.pose_y, target_point.pose_theta);

        return target_point;
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
