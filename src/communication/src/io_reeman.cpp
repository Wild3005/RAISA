#include <rclcpp/rclcpp.hpp>
#include "ReemanClient.hpp"
#include "ros2_interface/msg/robot.hpp"
#include "ros2_interface/msg/personpos.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

typedef struct {
    float pose_x, pose_y, pose_theta;
    float vel_linear, vel_angular;
    int8_t mode, battery_level, charge_flag, emergency_flag;
} robot_t;

struct Point { float x, y; };

class IOReeman : public rclcpp::Node {
public:

IOReeman() : Node("io_reeman") {
    this->declare_parameter<std::string>("reeman_ros_ip", "192.168.1.100");
    this->get_parameter("reeman_ros_ip", reeman_ros_ip_);
    this->declare_parameter<int>("polling_period_ms", 300);
    this->get_parameter("polling_period_ms", polling_period_ms_);
    
    RCLCPP_INFO(this->get_logger(), "Connecting to %s", reeman_ros_ip_.c_str());
    reeman_ = std::make_shared<ReemanClient>(reeman_ros_ip_);
    
    // Publishers
    // pub_robot_pose_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/reeman/pose", 1);
    // pub_robot_battery_ = this->create_publisher<std_msgs::msg::Int8>("/reeman/battery", 1);
    // pub_robot_mode_ = this->create_publisher<std_msgs::msg::Int8>("/reeman/mode", 1);
    // pub_last_post_status_ = this->create_publisher<std_msgs::msg::Int8>("/reeman/last_post_status", 1);
    // pub_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/reeman/point_cloud", 1);
    pub_robot_info_ = this->create_publisher<ros2_interface::msg::Robot>("/reeman/robot_info", 1);
    // pub_laser_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/reeman/laser_scan", 1);
    // pub_map_name_ = this->create_publisher<std_msgs::msg::String>("/reeman/map_name", 1);
    pub_nav_status_ = this->create_publisher<std_msgs::msg::String>("/reeman/nav_status", 1);
    
    // Subscribers
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
    
    // Virtual wall subscriber (LANGSUNG PROSES)
    sub_cmd_virtual_wall_ = this->create_subscription<ros2_interface::msg::Personpos>(
        "/uwb/person_pos", 1, std::bind(&IOReeman::callbackCmdVirtualWall, this, std::placeholders::_1));
    
    sub_fsm_state_ = this->create_subscription<std_msgs::msg::Int8>(
        "/fsm/state", 1, [this](const std_msgs::msg::Int8::SharedPtr msg) {
            current_fsm_state_ = msg->data;
        });
    
    // Speed worker thread
    worker_running_ = true;
    last_speed_post_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(100);
    worker_thread_ = std::thread(&IOReeman::speedCommandWorker, this);
    
    // Virtual wall worker thread (POST async agar tidak block)
    vwall_thread_ = std::thread(&IOReeman::virtualWallWorker, this);
    
    // Polling timer (hanya untuk robot info, SKIP GET jika timeout)
    polling_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(polling_period_ms_),
        std::bind(&IOReeman::pollingTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "IOReeman initialized");
}

~IOReeman() {
    worker_running_ = false;
    if (worker_thread_.joinable()) worker_thread_.join();
    if (vwall_thread_.joinable()) vwall_thread_.join();
    RCLCPP_INFO(this->get_logger(), "IOReeman stopped.");
}

    robot_t robot = {};
    std::string reeman_ros_ip_;
    int polling_period_ms_ = 300;
    std::shared_ptr<ReemanClient> reeman_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_cmd_nav_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_cmd_cancel_nav_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_nav_name_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_cmd_reloc_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_cmd_set_mode_;
    rclcpp::Subscription<ros2_interface::msg::Personpos>::SharedPtr sub_cmd_virtual_wall_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_fsm_state_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_robot_pose_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_robot_battery_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_robot_mode_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_last_post_status_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;
    rclcpp::Publisher<ros2_interface::msg::Robot>::SharedPtr pub_robot_info_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_map_name_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_nav_status_;
    
    rclcpp::TimerBase::SharedPtr polling_timer_;
    std::mutex cmd_mutex_;
    std::queue<geometry_msgs::msg::Twist> cmd_queue_;
    std::thread worker_thread_;
    std::atomic<bool> worker_running_{false};
    std::chrono::steady_clock::time_point last_speed_post_;
    
    // Virtual wall worker thread
    std::mutex vwall_mutex_;
    std::vector<VirtualWallSegment> pending_segments_;
    bool pending_vwall_{false};
    std::thread vwall_thread_;
    
    // Nav tracking
    int current_fsm_state_ = -1;
    geometry_msgs::msg::Pose2D current_goal_{};
    bool goal_active_ = false;
    
    
    void callbackCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_queue_.push(*msg);
    }
    
    void callbackCmdNav(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        bool ok = reeman_->sendNav(msg->x, msg->y, msg->theta);
        if (!ok) RCLCPP_WARN(this->get_logger(), "NAV failed");
    }
    
    void callbackCmdNavName(const std_msgs::msg::String::SharedPtr msg) {
        bool ok = reeman_->sendNavByName(msg->data);
        if (!ok) RCLCPP_WARN(this->get_logger(), "NAV by name failed");
    }
    
    void callbackCmdCancelNav(const std_msgs::msg::Int8::SharedPtr) {
        bool ok = reeman_->cancelNav();
        if (!ok) RCLCPP_WARN(this->get_logger(), "Cancel NAV failed");
    }
    
    void callbackCmdSetMode(const std_msgs::msg::Int8::SharedPtr msg) {
        bool ok = reeman_->setMode(msg->data);
        if (!ok) RCLCPP_WARN(this->get_logger(), "SetMode failed");
    }
    
    void callbackCmdReloc(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        bool ok = reeman_->relocateAbsolute(msg->x, msg->y, msg->theta);
        if (!ok) RCLCPP_WARN(this->get_logger(), "Reloc failed");
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
    
    // Callback Personpos: LANGSUNG queue virtual wall + kirim NAV
    void callbackCmdVirtualWall(const ros2_interface::msg::Personpos::SharedPtr msg) {
        float x = msg->uwb_pose.x;
        float y = msg->uwb_pose.y;
        float yaw = msg->uwb_pose.theta;
        float radius = msg->radius;

        // 1) Queue virtual wall untuk async POST

        if(current_fsm_state_ == 2){
            {
                std::lock_guard<std::mutex> lock(vwall_mutex_);
                pending_segments_ = calculateLineSegments(x, y, yaw, radius, 2.00);
                pending_vwall_ = true;
            }
        }else if(current_fsm_state_ == 3 || current_fsm_state_ == 4 || current_fsm_state_ == 5 || current_fsm_state_ == 6 || current_fsm_state_ == 7){
            {
                std::lock_guard<std::mutex> lock(vwall_mutex_);
                pending_segments_ = calculateCircleSegmentsWithDiagonals(x, y, yaw, radius);
                pending_vwall_ = true;
            }
        }
        else{
            {
                std::lock_guard<std::mutex> lock(vwall_mutex_);
                pending_segments_ = calculateCircleSegments(x, y, yaw, radius);
                pending_vwall_ = true;
            }
        }

        // 2) Goal calc + NAV (synchronous, langsung)
        Point person{x, y};
        Point robot_xy{ robot.pose_x, robot.pose_y };

        // float approach_dist = (current_fsm_state_ == 0) ? 1.6f : 
        //                      (current_fsm_state_ == 1) ? 1.2f : 0.0f;

        float approach_dist = 0.0f;
        if(current_fsm_state_ == 0) approach_dist = 1.6f;
        else if(current_fsm_state_ == 1) approach_dist = 1.2f;
        else if(current_fsm_state_ == 2) approach_dist = 0.7f;
        
        if (approach_dist > 0.0f) {
            if(current_fsm_state_ == 0 || current_fsm_state_ == 1){
                
                geometry_msgs::msg::Pose2D goal =
                    calculateBestGoalInteractionPoint(person, yaw, robot_xy,
                                                    approach_dist, 30.0f, 45.0f);

                float d_goal = std::hypot(goal.x - current_goal_.x, goal.y - current_goal_.y);
                if (!goal_active_ || d_goal > 0.10f) {
                    bool ok_nav = reeman_->sendNav(goal.x, goal.y, goal.theta);
                    if (ok_nav) {
                        current_goal_ = goal;
                        goal_active_ = true;
                        RCLCPP_DEBUG(this->get_logger(), "NAV sent to (%.2f, %.2f)", goal.x, goal.y);
                    }
                }

            }else if(current_fsm_state_ == 2) {
                
                geometry_msgs::msg::Pose2D goal;
                goal.x = 0.0;
                goal.y = 0.0;
                goal.theta = 1.6;

                // Kirim NAV hanya jika belum aktif atau jarak ke goal berubah signifikan
                float d_goal = std::hypot(goal.x - current_goal_.x, goal.y - current_goal_.y);
                if (!goal_active_ || d_goal > 0.10f) {
                    bool ok_nav = reeman_->sendNav(goal.x, goal.y, goal.theta);
                    if (ok_nav) {
                        current_goal_ = goal;
                        goal_active_ = true;
                        RCLCPP_DEBUG(this->get_logger(), "Case2: NAV sent to (%.2f, %.2f)", goal.x, goal.y);
                    }
                }

                RCLCPP_INFO(this->get_logger(), "STATE CASE_CrossBehind");
            }else if(current_fsm_state_ == 3) {
                // // Escort mode: selalu di sisi kanan person
                // geometry_msgs::msg::Pose2D goal =
                //     calculateEscortGoalRight(person, yaw, robot_xy, 0.8f, 1.0f);
                
                // // Update goal setiap saat (karena person bergerak)
                // float d_goal = std::hypot(goal.x - current_goal_.x, goal.y - current_goal_.y);
                // if (!goal_active_ || d_goal > 0.05f) {  // Threshold lebih kecil untuk escort (lebih responsif)
                //     bool ok_nav = reeman_->sendNav(goal.x, goal.y, goal.theta);
                //     if (ok_nav) {
                //         current_goal_ = goal;
                //         goal_active_ = true;
                //         RCLCPP_DEBUG(this->get_logger(), "Escort: NAV sent to right (%.2f, %.2f, th=%.2f)", 
                //                     goal.x, goal.y, goal.theta);
                //     }
                // }

                RCLCPP_INFO(this->get_logger(), "STATE CASE_EscortMode");
            }
        } else {
            goal_active_ = false;
        }
    }
    
    // Worker thread untuk POST virtual wall (async, tidak block callback)
    void virtualWallWorker() {
        while (worker_running_ && rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(vwall_mutex_);
                if (pending_vwall_) {
                    bool ok = reeman_->postVirtualWall(pending_segments_);
                    if (ok) {
                        RCLCPP_DEBUG(this->get_logger(), "VirtualWall POST ok (%zu seg)", pending_segments_.size());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "VirtualWall POST failed");
                    }
                    pending_vwall_ = false;
                }
            }
            std::this_thread::sleep_for(50ms);
        }
    }
    
    void speedCommandWorker() {
        auto min_interval = std::chrono::milliseconds(100);
        while (worker_running_ && rclcpp::ok()) {
            geometry_msgs::msg::Twist cmd;
            bool has_cmd = false;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                if (!cmd_queue_.empty()) {
                    while (cmd_queue_.size() > 1) cmd_queue_.pop();
                    cmd = cmd_queue_.front();
                    std::queue<geometry_msgs::msg::Twist> empty;
                    std::swap(cmd_queue_, empty);
                    has_cmd = true;
                }
            }
            if (has_cmd) {
                auto now = std::chrono::steady_clock::now();
                if (now - last_speed_post_ >= min_interval) {
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
    
    // Polling: SKIP GET jika sering timeout
    void pollingTimerCallback() {

        publishNavStatus();

        // Fetch latest robot state dari Reeman API
        if (auto pose = reeman_->getPose()) {
            robot.pose_x = (*pose)["x"];
            robot.pose_y = (*pose)["y"];
            robot.pose_theta = (*pose)["theta"];
        }
        
        // if (auto pwr = reeman_->getPower()) {
        //     robot.battery_level = (*pwr)["battery"];
        //     robot.charge_flag = (*pwr)["chargeFlag"];
        //     robot.emergency_flag = (*pwr)["emergencyButton"];
        // }
        
        // if (auto mode = reeman_->getMode()) {
        //     robot.mode = (*mode)["mode"];
        // }
        
        // if (auto vel = reeman_->getSpeedState()) {
        //     robot.vel_linear = (*vel)["vx"];
        //     robot.vel_angular = (*vel)["vth"];
        // }

        // Publish robot info
        ros2_interface::msg::Robot msg;
        msg.pose_x = robot.pose_x;
        msg.pose_y = robot.pose_y;
        msg.pose_theta = robot.pose_theta;
        msg.vel_linear = robot.vel_linear;
        msg.vel_angular = robot.vel_angular;
        msg.mode = robot.mode;
        msg.battery_level = robot.battery_level;
        msg.charge_flag = robot.charge_flag;
        msg.emergency_flag = robot.emergency_flag;
        pub_robot_info_->publish(msg);

        // Check jarak robot ke goal
        if (goal_active_) {
            float dx = robot.pose_x - current_goal_.x;
            float dy = robot.pose_y - current_goal_.y;
            float dist = std::hypot(dx, dy);

            // if (dist < 0.30f) {
            //     RCLCPP_INFO(this->get_logger(), "Goal reached (dist=%.2f). Stop.", dist);
            //     reeman_->sendSpeed(0.0f, 0.0f);
            //     goal_active_ = false;
            // }
        }
    }

    // Helper: hitung circle segments
    std::vector<VirtualWallSegment> calculateCircleSegments(float cx, float cy, float yaw, float radius) {
        // Balik arah yaw jika sistem robot menggunakan konvensi berlawanan
        float yaw_fixed = -yaw;

        std::vector<VirtualWallSegment> segments;
        float c = std::cos(yaw_fixed);
        float s = std::sin(yaw_fixed);

        auto rot = [&](float xl, float yl) -> Point {
            return { cx + c * xl - s * yl,
                    cy + s * xl + c * yl };
        };

        Point p_front = rot( radius, 0.0f);
        Point p_back  = rot(-radius, 0.0f);
        Point p_right = rot(0.0f,  radius);
        Point p_left  = rot(0.0f, -radius);

        segments.push_back(VirtualWallSegment{ p_back.x, p_back.y, p_front.x, p_front.y });
        segments.push_back(VirtualWallSegment{ p_left.x, p_left.y, p_right.x, p_right.y });

        return segments;
    }

    std::vector<VirtualWallSegment> calculateLineSegments(float cx, float cy, float yaw, float radius, float front_radius) {

                // Balik arah yaw jika sistem robot menggunakan konvensi berlawanan
        float yaw_fixed = -yaw;

        std::vector<VirtualWallSegment> segments;
        float c = std::cos(yaw_fixed);
        float s = std::sin(yaw_fixed);

        auto rot = [&](float xl, float yl) -> Point {
            return { cx + c * xl - s * yl,
                    cy + s * xl + c * yl };
        };

        Point p_front = rot( radius + front_radius, 0.0f);
        Point p_back  = rot(-radius, 0.0f);
        Point p_right = rot(0.0f,  radius);
        Point p_left  = rot(0.0f, -radius);

        segments.push_back(VirtualWallSegment{ p_back.x, p_back.y, p_front.x, p_front.y });
        segments.push_back(VirtualWallSegment{ p_left.x, p_left.y, p_right.x, p_right.y });

        return segments;
    }

    // Helper: hitung goal interaksi
    geometry_msgs::msg::Pose2D calculateBestGoalInteractionPoint(
        const Point& person, float person_yaw, const Point& robot,
        float dist, float deg_min, float deg_max)
    {
        // Balik arah yaw agar sisi kiri/kanan sesuai harapan
        float yaw_fixed = -person_yaw;

        geometry_msgs::msg::Pose2D result;
        float theta = (deg_min + deg_max) * 0.5f * M_PI / 180.0f;

        Point left{
            person.x + dist * std::cos(yaw_fixed + theta),
            person.y + dist * std::sin(yaw_fixed + theta)
        };
        Point right{
            person.x + dist * std::cos(yaw_fixed - theta),
            person.y + dist * std::sin(yaw_fixed - theta)
        };

        auto dist2 = [&](const Point& p) {
            float dx = p.x - robot.x;
            float dy = p.y - robot.y;
            return dx * dx + dy * dy;
        };

        Point best = (dist2(left) < dist2(right)) ? left : right;
        float yaw_to_person = std::atan2(person.y - best.y, person.x - best.x);

        result.x = best.x;
        result.y = best.y;
        result.theta = yaw_to_person;
        return result;
    }

    // Helper: hitung goal escort (selalu di sisi kanan person)
    geometry_msgs::msg::Pose2D calculateEscortGoalRight(
        const Point& person, 
        float person_yaw, 
        const Point& robot,
        float lateral_min = 0.8f,   // jarak lateral minimum (m)
        float lateral_max = 1.0f)   // jarak lateral maksimum (m)
    {
        // Balik arah yaw agar sisi kiri/kanan sesuai harapan
        float yaw_fixed = -person_yaw;
        
        geometry_msgs::msg::Pose2D result;
        
        // Pilih lateral distance berdasarkan jarak robot saat ini
        // Jika robot terlalu jauh, gunakan lateral_min (lebih dekat)
        // Jika robot sudah dekat, gunakan lateral_max (lebih jauh, lebih aman)
        float dx_robot = robot.x - person.x;
        float dy_robot = robot.y - person.y;
        float dist_robot = std::hypot(dx_robot, dy_robot);
        
        // Linear interpolation antara lateral_min dan lateral_max
        float lateral_dist = lateral_min;
        if (dist_robot > lateral_min && dist_robot < lateral_max * 1.5f) {
            lateral_dist = lateral_min + (lateral_max - lateral_min) * 
                        ((dist_robot - lateral_min) / (lateral_max * 0.5f));
        } else if (dist_robot >= lateral_max * 1.5f) {
            lateral_dist = lateral_max;
        }
        
        // Clamp ke range yang diinginkan
        lateral_dist = std::max(lateral_min, std::min(lateral_max, lateral_dist));
        
        // Basis kanan person (rotasi -90° dari heading person)
        // Heading person: yaw_fixed
        // Kanan person: yaw_fixed - 90° = yaw_fixed - M_PI/2
        float right_angle = yaw_fixed - M_PI_2;
        
        // Titik goal: di sisi kanan person dengan offset lateral
        Point goal_right{
            person.x + lateral_dist * std::cos(right_angle),
            person.y + lateral_dist * std::sin(right_angle)
        };
        
        // Heading goal: menghadap arah yang sama dengan person (paralel)
        // Agar robot berjalan sejajar dengan person
        result.x = goal_right.x;
        result.y = goal_right.y;
        result.theta = yaw_fixed;  // Paralel dengan person
        
        return result;
    }

    std::vector<VirtualWallSegment> calculateCircleSegmentsWithDiagonals(float cx, float cy, float yaw, float radius) {
        // Balik arah yaw jika sistem robot menggunakan konvensi berlawanan
        float yaw_fixed = -yaw;

        std::vector<VirtualWallSegment> segments;
        float c = std::cos(yaw_fixed);
        float s = std::sin(yaw_fixed);

        auto rot = [&](float xl, float yl) -> Point {
            return { cx + c * xl - s * yl,
                     cy + s * xl + c * yl };
        };

        // Garis utama (tegak lurus)
        Point p_front = rot( radius, 0.0f);
        Point p_back  = rot(-radius, 0.0f);
        Point p_right = rot(0.0f,  radius);
        Point p_left  = rot(0.0f, -radius);

        segments.push_back(VirtualWallSegment{ p_back.x,  p_back.y,  p_front.x, p_front.y }); // depan-belakang
        segments.push_back(VirtualWallSegment{ p_left.x,  p_left.y,  p_right.x, p_right.y }); // kiri-kanan

        // Garis diagonal (dua garis)
        float d = radius / std::sqrt(2.0f);
        Point p_d1a = rot( d,  d);
        Point p_d1b = rot(-d, -d);
        Point p_d2a = rot( d, -d);
        Point p_d2b = rot(-d,  d);

        segments.push_back(VirtualWallSegment{ p_d1a.x, p_d1a.y, p_d1b.x, p_d1b.y }); // diagonal 1
        segments.push_back(VirtualWallSegment{ p_d2a.x, p_d2a.y, p_d2b.x, p_d2b.y }); // diagonal 2

        return segments;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IOReeman>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
