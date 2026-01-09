#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <ros2_interface/msg/personpos.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <mutex>

class ConvertCsvNode : public rclcpp::Node {
public:
  ConvertCsvNode() : rclcpp::Node("convert_csv_node") {
    // Konfigurasi folder tujuan (default: CASE_Sitting)
    subfolder_ = this->declare_parameter<std::string>("subfolder", "CASE_Sitting");
    // Jika user ingin override path full, isi output_path. Jika kosong → pakai share/data/<subfolder>/person_robot_<ts>.csv
    output_path_ = this->declare_parameter<std::string>("output_path", std::string{});
    bool append = this->declare_parameter<bool>("append", true);

    if (output_path_.empty()) {
      const std::string share = ament_index_cpp::get_package_share_directory("master");
      const std::string folder = share + "/data/" + subfolder_;
      std::error_code ec;
      std::filesystem::create_directories(folder, ec);
      // timestamped filename
      auto now = std::chrono::system_clock::now();
      std::time_t tt = std::chrono::system_clock::to_time_t(now);
      std::tm tm{};
      localtime_r(&tt, &tm);
      char buf[32];
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
      output_path_ = folder + "/person_robot_" + std::string(buf) + ".csv";
      // Saat timestamped, append tidak relevan → pakai trunc agar tulis header baru
      append = false;
    }

    // Buka file
    std::ios_base::openmode mode = std::ios::out | std::ios::binary;
    mode |= append ? std::ios::app : std::ios::trunc;
    ofs_.open(output_path_, mode);
    if (!ofs_.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot open CSV file: %s", output_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    if (!append || isFileEmpty()) {
      ofs_ << "stamp_sec,robot_x,robot_y,robot_theta,person_x,person_y,person_theta,person_radius\n";
      ofs_.flush();
    }
    RCLCPP_INFO(this->get_logger(), "CSV logging to: %s", output_path_.c_str());

    // Subs
    sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/reeman/pose", 10,
      [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        last_robot_ = *msg;
        has_robot_ = true;
        maybeWriteRow();
      }
    );

    sub_person_pos_ = this->create_subscription<ros2_interface::msg::Personpos>(
      "/uwb/person_pos", 10,
      [this](const ros2_interface::msg::Personpos::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        last_person_ = msg->uwb_pose;
        last_radius_ = msg->radius;
        has_person_ = true;
        maybeWriteRow();
      }
    );
  }

  ~ConvertCsvNode() override {
    if (ofs_.is_open()) ofs_.close();
  }

private:
  bool isFileEmpty() {
    ofs_.seekp(0, std::ios::end);
    return ofs_.tellp() == 0;
  }

  void maybeWriteRow() {
    if (!has_robot_ || !has_person_) return;

    const rclcpp::Time now = this->now();
    double stamp = now.seconds();

    ofs_ << std::fixed << std::setprecision(6)
         << stamp << ","
         << last_robot_.x << "," << last_robot_.y << "," << last_robot_.theta << ","
         << last_person_.x << "," << last_person_.y << "," << last_person_.theta << ","
         << last_radius_ << "\n";
    ofs_.flush();
  }

  // State
  std::ofstream ofs_;
  std::string output_path_;
  std::string subfolder_;
  std::mutex mtx_;
  bool has_robot_{false}, has_person_{false};
  geometry_msgs::msg::Pose2D last_robot_{}, last_person_{};
  double last_radius_{0.0};

  // Subs
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_robot_pose_;
  rclcpp::Subscription<ros2_interface::msg::Personpos>::SharedPtr sub_person_pos_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertCsvNode>());
  rclcpp::shutdown();
  return 0;
}