#include <rclcpp/rclcpp.hpp>

#include "ReemanClient.hpp" // header-only HTTP client

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros2_interface/msg/robot.hpp"
#include "ros2_utils/simple_fsm.hpp"

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

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

class IOReeman : public rclcpp::Node
{
public:
    robot_t robot;

    // -------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------
    std::string reeman_ros_ip_;
    int min_request_period_speed_ms_ = 100;
    int polling_period_ms_ = 300;

    // Reeman client
    std::shared_ptr<ReemanClient> reeman_;

    // -------------------------------------------------------------
    // ROS Subscribers
    // -------------------------------------------------------------
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_cmd_nav_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_cmd_cancel_nav_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_nav_name_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_cmd_reloc_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_cmd_set_mode_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_get_pose_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_get_lidar_;

    // Move / turn
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_move_turn_;

    // -------------------------------------------------------------
    // ROS Publishers
    // -------------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_robot_pose_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_robot_battery_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_robot_mode_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_last_post_status_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;
    rclcpp::Publisher<ros2_interface::msg::Robot>::SharedPtr pub_robot_info_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_map_name_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_nav_status_;

    // Timer for polling robot state
    rclcpp::TimerBase::SharedPtr polling_timer_;

    // -------------------------------------------------------------
    // Worker thread for /cmd/vel → POST /cmd/speed
    // -------------------------------------------------------------
    std::mutex cmd_mutex_;
    std::queue<geometry_msgs::msg::Twist> cmd_queue_;
    std::thread worker_thread_;
    std::atomic<bool> worker_running_{false};
    std::chrono::steady_clock::time_point last_speed_post_;

    // -------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------
    IOReeman() : Node("io_reeman")
    {
        // ----------------------------
        // Load parameters
        // ----------------------------
        this->declare_parameter<std::string>("reeman_ros_ip", "192.168.1.100");
        this->get_parameter("reeman_ros_ip", reeman_ros_ip_);

        this->declare_parameter<int>("min_request_period_speed_ms", min_request_period_speed_ms_);
        this->get_parameter("min_request_period_speed_ms", min_request_period_speed_ms_);

        this->declare_parameter<int>("polling_period_ms", polling_period_ms_);
        this->get_parameter("polling_period_ms", polling_period_ms_);

        RCLCPP_INFO(this->get_logger(), "Connecting ReemanClient to %s", reeman_ros_ip_.c_str());

        reeman_ = std::make_shared<ReemanClient>(reeman_ros_ip_);

        // ----------------------------
        // Publishers
        // ----------------------------
        pub_robot_pose_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/reeman/pose", 1);
        pub_robot_battery_ = this->create_publisher<std_msgs::msg::Int8>("/reeman/battery", 1);
        pub_robot_mode_ = this->create_publisher<std_msgs::msg::Int8>("/reeman/mode", 1);
        pub_last_post_status_ = this->create_publisher<std_msgs::msg::Int8>("/reeman/last_post_status", 1);
        pub_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reeman/point_cloud", 1);
        //=====================
        pub_robot_info_ = this->create_publisher<ros2_interface::msg::Robot>("/reeman/robot_info", 1);
        //=====================
        pub_laser_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/reeman/laser_scan", 1);
        pub_map_name_ = this->create_publisher<std_msgs::msg::String>("/reeman/map_name", 1);
        pub_nav_status_ = this->create_publisher<std_msgs::msg::String>("/reeman/nav_status", 1);

        // ----------------------------
        // Subscribers
        // ----------------------------
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd/vel", 1, std::bind(&IOReeman::callbackCmdVel, this, std::placeholders::_1));

        sub_cmd_nav_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/cmd/nav", 1, std::bind(&IOReeman::callbackCmdNav, this, std::placeholders::_1));

        sub_cmd_nav_name_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/nav_name", 1, std::bind(&IOReeman::callbackCmdNavName, this, std::placeholders::_1));

        sub_cmd_reloc_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/cmd/reloc", 1, std::bind(&IOReeman::callbackCmdReloc, this, std::placeholders::_1));

        sub_cmd_cancel_nav_ = this->create_subscription<std_msgs::msg::Int8>(
            "/cmd/cancel_nav", 1, std::bind(&IOReeman::callbackCmdCancelNav, this, std::placeholders::_1));

        sub_cmd_set_mode_ = this->create_subscription<std_msgs::msg::Int8>(
            "/cmd/set_mode", 1, std::bind(&IOReeman::callbackCmdSetMode, this, std::placeholders::_1));

        sub_get_pose_ = this->create_subscription<std_msgs::msg::Int8>(
            "/get/pose", 1, std::bind(&IOReeman::callbackGetPose, this, std::placeholders::_1));

        sub_get_lidar_ = this->create_subscription<std_msgs::msg::Int8>(
            "/get/lidar", 1, std::bind(&IOReeman::callbackGetLidar, this, std::placeholders::_1));

        // ----------------------------
        // Worker thread to send speed
        // ----------------------------
        worker_running_ = true;
        last_speed_post_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(min_request_period_speed_ms_);
        worker_thread_ = std::thread(&IOReeman::speedCommandWorker, this);

        // ----------------------------
        // Polling timer
        // ----------------------------
        polling_timer_ = this->create_wall_timer(std::chrono::milliseconds(polling_period_ms_), std::bind(&IOReeman::pollingTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "IOReeman initialized.");
    }

    ~IOReeman()
    {
        worker_running_ = false;
        if (worker_thread_.joinable())
            worker_thread_.join();
        RCLCPP_INFO(this->get_logger(), "IOReeman stopped.");
    }

    // ---------------------------------------------------------------------
    // CALLBACKS
    // ---------------------------------------------------------------------
    void callbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_queue_.push(*msg);
    }

    void callbackCmdNav(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        bool ok = reeman_->sendNav(msg->x, msg->y, msg->theta);
        if (!ok)
            RCLCPP_WARN(this->get_logger(), "NAV failed");
    }

    void callbackCmdNavName(const std_msgs::msg::String::SharedPtr msg)
    {
        bool ok = reeman_->sendNavByName(msg->data);
        if (!ok)
            RCLCPP_WARN(this->get_logger(), "NAV by name failed");
    }

    void callbackCmdReloc(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        bool ok = reeman_->relocateAbsolute(msg->x, msg->y, msg->theta);
        if (!ok)
            RCLCPP_WARN(this->get_logger(), "Reloc failed");
    }

    void callbackCmdSetMode(const std_msgs::msg::Int8::SharedPtr msg)
    {
        bool ok = reeman_->setMode(msg->data);
        if (!ok)
            RCLCPP_WARN(this->get_logger(), "SetMode failed");
    }

    void callbackCmdCancelNav(const std_msgs::msg::Int8::SharedPtr)
    {
        bool ok = reeman_->cancelNav();
        if (!ok)
            RCLCPP_WARN(this->get_logger(), "Cancel NAV failed");
    }

    // ---------------------------------------------------------------------
    // SPEED WORKER
    // ---------------------------------------------------------------------
    void speedCommandWorker()
    {
        auto min_interval = std::chrono::milliseconds(min_request_period_speed_ms_);

        while (worker_running_ && rclcpp::ok())
        {
            geometry_msgs::msg::Twist cmd;
            bool has_cmd = false;

            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                if (!cmd_queue_.empty())
                {
                    while (cmd_queue_.size() > 1)
                        cmd_queue_.pop();

                    cmd = cmd_queue_.front();
                    std::queue<geometry_msgs::msg::Twist> empty;
                    std::swap(cmd_queue_, empty);
                    has_cmd = true;
                }
            }

            if (has_cmd)
            {
                auto now = std::chrono::steady_clock::now();
                if (now - last_speed_post_ >= min_interval)
                {
                    // bool ok = reeman_->sendSlowSpeed(cmd.linear.x, cmd.angular.z);
                    bool ok = reeman_->sendSpeed(cmd.linear.x, cmd.angular.z);
                    last_speed_post_ = now;

                    std_msgs::msg::Int8 status;
                    status.data = ok ? 1 : 0;
                    pub_last_post_status_->publish(status);
                }
            }

            std::this_thread::sleep_for(10ms);
        }
    }

    // ---------------------------------------------------------------------
    // POLLING LOOP
    // ---------------------------------------------------------------------
    void pollingTimerCallback()
    {
        // publishRobotInfo();
        // publishNavStatus();
        // publishLaserScan();
        // publishMapName();
    }

    void getPose()
    {
        if (auto pose = reeman_->getPose())
        {
            robot.pose_x = (*pose)["x"];
            robot.pose_y = (*pose)["y"];
            robot.pose_theta = (*pose)["theta"];
        }
    }

    void getBattery()
    {
        if (auto pwr = reeman_->getPower())
        {
            robot.battery_level = (*pwr)["battery"];
            robot.charge_flag = (*pwr)["chargeFlag"];
            robot.emergency_flag = (*pwr)["emergencyButton"];
        }
    }

    void getMode()
    {
        if (auto mode = reeman_->getMode())
        {
            robot.mode = (*mode)["mode"];
        }
    }

    void getVelocity()
    {
        // Velocities
        if (auto vel = reeman_->getSpeedState())
        {
            robot.vel_linear = (*vel)["vx"];
            robot.vel_angular = (*vel)["vth"];
        }
    }

    void getNavStatus()
    {
        if (auto nav_status = reeman_->getNavStatus())
        {
            robot.nav_status_res = (*nav_status)["res"];
            robot.nav_status_reason = (*nav_status)["reason"];
            robot.nav_status_goal = (*nav_status)["goal"];
            robot.nav_status_dist = (*nav_status)["dist"];
            robot.nav_status_mileage = (*nav_status)["mileage"];
        }
    }

    void publishRobotInfo()
    {
        // Update robot state
        getPose();
        std::this_thread::sleep_for(10ms);
        getBattery();
        std::this_thread::sleep_for(10ms);
        getMode();
        std::this_thread::sleep_for(10ms);
        getVelocity();
        std::this_thread::sleep_for(10ms);
        getNavStatus();
        std::this_thread::sleep_for(10ms);

        // Publish robot state
        ros2_interface::msg::Robot robot_msg;

        robot_msg.pose_x = robot.pose_x;
        robot_msg.pose_y = robot.pose_y;
        robot_msg.pose_theta = robot.pose_theta;
        robot_msg.vel_linear = robot.vel_linear;
        robot_msg.vel_angular = robot.vel_angular;
        robot_msg.mode = robot.mode;
        robot_msg.battery_level = robot.battery_level;
        robot_msg.charge_flag = robot.charge_flag;
        robot_msg.emergency_flag = robot.emergency_flag;
        robot_msg.nav_status_res = robot.nav_status_res;
        robot_msg.nav_status_reason = robot.nav_status_reason;
        robot_msg.nav_status_goal = robot.nav_status_goal;
        robot_msg.nav_status_dist = robot.nav_status_dist;
        robot_msg.nav_status_mileage = robot.nav_status_mileage;

        pub_robot_info_->publish(robot_msg);
    }

    void publishNavStatus()
    {
        if (auto nav_json = reeman_->getNavStatus())
        {
            std_msgs::msg::String msg;
            msg.data = nav_json->dump();
            pub_nav_status_->publish(msg);
        }
    }

    void publishMapName()
    {
        if (auto name = reeman_->getCurrentMapName())
        {
            std_msgs::msg::String msg;
            msg.data = *name;
            pub_map_name_->publish(msg);
        }
    }

    void publishRobotPose()
    {
        if (auto pose = reeman_->getPose())
        {
            geometry_msgs::msg::Pose2D msg;
            msg.x = (*pose)["x"];
            msg.y = (*pose)["y"];
            msg.theta = (*pose)["theta"];
            pub_robot_pose_->publish(msg);
        }
    }

    void publishLaserScan()
    {
        if (auto laser_json = reeman_->getLaser())
        {
            if (!laser_json->contains("coordinates"))
            {
                RCLCPP_WARN(this->get_logger(), "Laser JSON missing 'coordinates'");
                return;
            }

            const auto &coords = (*laser_json)["coordinates"];

            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_cloud->clear();
            pcl_cloud->reserve(coords.size());

            for (const auto &pt : coords)
            {
                if (pt.size() < 2)
                    continue;

                float x = pt[0].get<float>();
                float y = pt[1].get<float>();
                float z = 0.0f;

                pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
            }

            pcl_cloud->width = pcl_cloud->size();
            pcl_cloud->height = 1;
            pcl_cloud->is_dense = true;

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*pcl_cloud, output);
            output.header.frame_id = "laser_frame";
            output.header.stamp = this->now();
            pub_point_cloud_->publish(output);
        }
    }

    // ---------------------------------------------------------------------
    // Get and Publish
    // ---------------------------------------------------------------------
    void callbackGetLidar(const std_msgs::msg::Int8::SharedPtr)
    {
        publishLaserScan();
    }

    void callbackGetPose(const std_msgs::msg::Int8::SharedPtr)
    {
        publishRobotPose();
        std::this_thread::sleep_for(50ms);
        publishNavStatus();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IOReeman>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
