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
#define CASE_IDLE 8

#define MODE_INTERACTION 10
#define MODE_NAVIGATION 11

struct velocity{
    double vx;
    double vy;
};

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
    // rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_button_mode_;
    
    // ===== Tambahkan Timer =====
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr t_pose_now, t_pose_prev;

    // ======================================================================
    // MODE state
    // int MODE = MODE_NAVIGATION;
    int MODE = MODE_INTERACTION;
    // ======================================================================


    // Patrol NAV points (A ↔ B)
    struct Pose2 { double x, y, th; };
    Pose2 patrol_A_{0.0, 0.0, 0.0};
    Pose2 patrol_B_{4.5, 7.0, -1.5};
    bool going_to_A_{true}; // toggle A/B
    bool nav_in_progress_{false};


    // FSM object (dari ros2_utils/simple_fsm.hpp)
    MachineState fsm_robot;

    // Flags/status
    bool fsm_entry_sent_{false};
    bool nav_status_ready_{false};
    bool uwb_data_received_{false};
    bool robot_pose_received_{false};
    bool robot_info_received_{false};

    // State data
    geometry_msgs::msg::Pose2D uwb_pose_{};
    geometry_msgs::msg::Pose2D last_pose_{};
    geometry_msgs::msg::Pose2D uwb_pose_prev_{};  // ← tracking pose UWB sebelumnya
    int robot_mode_{-1};
    int battery_{-1};
    ros2_interface::msg::Robot robot_info_{};

    // Nav status (JSON dari /reeman/nav_status)
    json nav_status_json_;
    int res{0};
    int prev_res = res;

    // Thresholds (m) untuk arrival detection
    double pos_threshold_sit_{0.5};
    double pos_threshold_stand_{0.5};

    // Tracking waktu untuk velocity person
    rclcpp::Time last_uwb_time_;
    bool first_uwb_{true};

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
        
        // Subscribe MODE dari Web UI
        // sub_button_mode_ = this->create_subscription<std_msgs::msg::Int8>(
        //     "/button/mode", 10,
        //     [this](const std_msgs::msg::Int8::SharedPtr msg) {
        //         int new_mode = msg->data;
        //         if (new_mode == MODE_INTERACTION || new_mode == MODE_NAVIGATION) {
        //             MODE = new_mode;
        //             fsm_entry_sent_ = false;
        //             nav_status_ready_ = false;
        //             nav_in_progress_ = false;
        //             RCLCPP_INFO(this->get_logger(), "Switch MODE → %s",
        //                 MODE == MODE_INTERACTION ? "INTERACTION" : "NAVIGATION");
        //             // Set starting case sesuai mode
        //             fsm_robot.value = (MODE == MODE_INTERACTION) ? CASE_SittingApproach : CASE_CrossBehind;
        //         } else {
        //             RCLCPP_WARN(this->get_logger(), "Unknown MODE: %d", new_mode);
        //         }
        //     }
        // );

        // ==========================================================================
        // Inisialisasi start state default
        // fsm_robot.value = CASE_CrossBehind;
        fsm_robot.value = CASE_StandingApproach;
        // ==========================================================================

        // Timer
        timer_ = this->create_wall_timer(500ms, std::bind(&MasterNode::timerRoutine, this));
        
        RCLCPP_INFO(this->get_logger(), "Master initialized");
    }
    
    // ============================================================
    // CALLBACKS
    // ============================================================
    
    void callbackUWBPose(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        // Simpan pose sebelumnya sebelum update
        if (!first_uwb_) {
            uwb_pose_prev_ = uwb_pose_;
        }
        
        uwb_pose_ = *msg;
        last_uwb_time_ = this->now();
        uwb_data_received_ = true;
        
        if (first_uwb_) {
            uwb_pose_prev_ = uwb_pose_;  // inisialisasi
            first_uwb_ = false;
        }
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
        battery_ = msg->battery_level;
        // salin pose robot dari robot_info
        last_pose_.x = msg->pose_x;
        last_pose_.y = msg->pose_y;
        last_pose_.theta = msg->pose_theta;
        robot_info_received_ = true;
        robot_pose_received_ = true;
    }
    
    void callbackNavStatus(const std_msgs::msg::String::SharedPtr msg) {
        nav_status_json_ = json::parse(msg->data);
        res = nav_status_json_["res"];
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
        if (battery_ > 0 && battery_ < 20) {
            RCLCPP_WARN(this->get_logger(), "Battery critical → cancel navigation.");
            std_msgs::msg::Int8 cancel_msg;
            cancel_msg.data = 1;
            pub_cmd_cancel_nav_->publish(cancel_msg);
            nav_in_progress_ = false;
            return;
        }

        // Check UWB ready
        if (!uwb_data_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for UWB data...");
            return;
        }

        // Publish FSM state setiap cycle
        {
            std_msgs::msg::Int8 fsm_msg;
            fsm_msg.data = fsm_robot.value;
            pub_fsm_state_->publish(fsm_msg);
        }

        // Evaluasi yaw person vs robot
        evaluateRelativeYawPersonVsRobot();

        // Hitung velocity person dari UWB
        velocity v = calculateVelocityPerson();
        RCLCPP_INFO(this->get_logger(),"Person VEL X %.2f Y %.2f m/s", v.vx, v.vy);

        // RCLCPP_INFO(this->get_logger(),"MODE %d",MODE);
        RCLCPP_INFO(this->get_logger(),"RES %d || RES PREV %d",res, prev_res);


        // ====== MODE SWITCH (BERSARANG) ======
        if (MODE == MODE_INTERACTION) {
            // pastikan pose robot sudah ada
            if (!robot_info_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Waiting for robot pose (robot_info)...");
                return;
            }

            auto radius_for_state = [&](int st) -> float {
                switch (st) {
                    case CASE_SittingApproach:   return 1.5f;
                    case CASE_StandingApproach:  return 1.1f;
                    default:                     return 0.0f;
                }
            };
            auto threshold_for_state = [&](int st) -> double {
                switch (st) {
                    case CASE_SittingApproach:   return pos_threshold_sit_;
                    case CASE_StandingApproach:  return pos_threshold_stand_;
                    default:                     return 0.0;
                }
            };

            float rad = radius_for_state(fsm_robot.value);
            double pos_th = threshold_for_state(fsm_robot.value);

            // ENTRY
            if (!fsm_entry_sent_) {
                Pose_person(uwb_pose_.x, uwb_pose_.y, uwb_pose_.theta, rad);
                RCLCPP_INFO(this->get_logger(),
                    "INTERACTION ENTRY: state=%d UWB(x=%.2f,y=%.2f,rad=%.2f)",
                    fsm_robot.value, uwb_pose_.x, uwb_pose_.y, rad);
                fsm_entry_sent_ = true;
                nav_status_ready_ = false;  // res diabaikan, tapi biarkan flag
                nav_in_progress_ = true;
                return;
            }

            // MONITOR: kirim UWB untuk virtual wall update
            Pose_person(uwb_pose_.x, uwb_pose_.y, uwb_pose_.theta, rad);

            // Arrival check berbasis pose robot vs goal (UWB)
            double d = dist2d(last_pose_.x, last_pose_.y, uwb_pose_.x, uwb_pose_.y);
            if (d <= pos_th) {
                RCLCPP_INFO(this->get_logger(),
                    "INTERACTION ARRIVED: state=%d dist=%.2f (th=%.2f)",
                    fsm_robot.value, d, pos_th);
                nav_in_progress_ = false;

                // Toggle Sitting ↔ Standing
                if (fsm_robot.value == CASE_SittingApproach) {
                    fsm_robot.value = CASE_StandingApproach;
                } else {
                    fsm_robot.value = CASE_SittingApproach;
                }
                fsm_entry_sent_ = false;
                nav_status_ready_ = false; // meski res diabaikan
            }
        }
        else if (MODE == MODE_NAVIGATION) {
            // Mode NAVIGATION → gunakan CASE 2 (CrossBehind) + patrol A↔B
            auto radius_for_state = [&](int st) -> float {
                switch (st) {
                    case CASE_CrossBehind:       return 0.6f;
                    default:                     return 0.0f;
                }
            };

            float rad = radius_for_state(fsm_robot.value);

            // ENTRY untuk CrossBehind (virtual wall + NAV target A/B)
            if (!fsm_entry_sent_) {
                Pose_person(uwb_pose_.x, uwb_pose_.y, uwb_pose_.theta, rad);
                // Tentukan target patrol
                Pose2 target = going_to_A_ ? patrol_A_ : patrol_B_;
                goTo(target.x, target.y, target.th);
                nav_in_progress_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "NAV ENTRY: CrossBehind → goTo (%.2f, %.2f, th=%.2f) target=%s",
                    target.x, target.y, target.th, going_to_A_ ? "A" : "B");
                fsm_entry_sent_ = true;
                nav_status_ready_ = false;
                return;
            }

            // MONITOR: terus update virtual wall dari UWB
            Pose_person(uwb_pose_.x, uwb_pose_.y, uwb_pose_.theta, rad);

            if (!nav_status_ready_) {
                RCLCPP_DEBUG(this->get_logger(), "NAV waiting status...");
                return;
            }

            // EXIT: jika goal tercapai, toggle A/B dan kirim NAV lagi
            if (res == 3 && prev_res != res) {
                RCLCPP_INFO(this->get_logger(), 
                    "NAV reached %s → switching to %s", 
                    going_to_A_ ? "A" : "B", 
                    going_to_A_ ? "B" : "A");
                
                // Toggle ke target berikutnya
                going_to_A_ = !going_to_A_;
                Pose2 next_target = going_to_A_ ? patrol_A_ : patrol_B_;
                
                // Kirim NAV berikutnya
                goTo(next_target.x, next_target.y, next_target.th);
                
                RCLCPP_INFO(this->get_logger(),
                    "NAV NEXT: goTo (%.2f, %.2f, th=%.2f) target=%s",
                    next_target.x, next_target.y, next_target.th, 
                    going_to_A_ ? "A" : "B");
                
                // Reset flags untuk cycle berikutnya
                nav_in_progress_ = true;
                nav_status_ready_ = false;  // ← PENTING: reset ini
                fsm_entry_sent_ = true;      // tetap di CrossBehind
            }
        }

        prev_res = res;
    }

    inline double rad2deg(double r) { return r * 180.0 / M_PI; }
    inline double normAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
    
    double dist2d(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx*dx + dy*dy);
    }
    
    // ======================================================================================
    //LANJUTKAN TTL
    void evaluateRelativeYawPersonVsRobot() {
        if (!robot_info_received_ || !uwb_data_received_) return;

        double robot_yaw = last_pose_.theta;     // dari /reeman/robot_info
        double person_yaw = uwb_pose_.theta;     // dari UWB
        double delta_rad = normAngle(person_yaw - robot_yaw);
        double delta_deg = rad2deg(delta_rad);
        double abs_deg   = std::fabs(delta_deg);

        const char* category = nullptr;
        if (abs_deg <= 30.0) {
            category = "BERLAWANAN ARAH";
        } else if (abs_deg > 30.0 && abs_deg <= 160.0) {
            category = "MEMOTONG JALAN ORANG";
        } else { // 160–180
            category = "SEARAH";
        }

        RCLCPP_INFO(this->get_logger(),
            "Yaw robot=%.1f°, person=%.1f°, delta=%.1f° → %s",
            rad2deg(robot_yaw), rad2deg(person_yaw), delta_deg, category);
    }

    velocity calculateVelocityPerson()
    {
        velocity v;
        v.vx = 0.0;
        v.vy = 0.0;

        if (first_uwb_ || !uwb_data_received_) {
            return v;  // belum ada data cukup
        }

        // Hitung delta waktu (dt)
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_uwb_time_).seconds();

        if (dt <= 1e-6) {  // hindari division by zero
            return v;
        }

        // Hitung delta posisi
        double dx = uwb_pose_.x - uwb_pose_prev_.x;
        double dy = uwb_pose_.y - uwb_pose_prev_.y;

        // Velocity dalam frame global
        v.vx = dx / dt;
        v.vy = dy / dt;

        return v;
    }




    // ======================================================================================

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterNode>();
    RCLCPP_INFO(node->get_logger(), "Master node spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
