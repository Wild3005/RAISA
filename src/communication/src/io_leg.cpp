#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

// UDP ports
constexpr int PORT_LEFT  = 12345;
constexpr int PORT_RIGHT = 12346;

// Shared data
std::atomic<float> left_angle(0.0f);
std::atomic<float> right_angle(0.0f);

std::atomic<bool> left_connected(false);
std::atomic<bool> right_connected(false);

std::atomic<uint64_t> left_packet_count(0);
std::atomic<uint64_t> right_packet_count(0);

// Timestamp
auto start_time = std::chrono::steady_clock::now();


// ---------------------------------------------
// UDP receive thread
// ---------------------------------------------
void udp_receiver(int port, std::atomic<float>& angle_ref,
                  std::atomic<bool>& connected_ref,
                  std::atomic<uint64_t>& counter_ref,
                  const char* tag)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[" << tag << "] Failed to create socket\n";
        return;
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[" << tag << "] Bind failed\n";
        close(sockfd);
        return;
    }

    std::cout << "[" << tag << "] Listening on UDP port " << port << "\n";

    while (true) {
        uint8_t buffer[1024];
        sockaddr_in sender {};
        socklen_t sender_len = sizeof(sender);

        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                             (sockaddr*)&sender, &sender_len);

        if (n < 0) {
            std::cerr << "[" << tag << "] recvfrom() error\n";
            continue;
        }

        if (!connected_ref.load()) {
            connected_ref.store(true);
            char ip[32];
            inet_ntop(AF_INET, &sender.sin_addr, ip, sizeof(ip));
            std::cout << "\n✓ [" << tag << "] Connected from "
                      << ip << ":" << ntohs(sender.sin_port) << "\n\n";
        }

        counter_ref++;

        if (n == 4) {
            float value;
            memcpy(&value, buffer, sizeof(float));
            angle_ref.store(value);
        }
    }

    close(sockfd);
}


// -------------------------------------------------
// ROS2 publisher node
// -------------------------------------------------
class DualLegPublisher : public rclcpp::Node
{
public:
    DualLegPublisher()
    : Node("dual_leg_publisher_cpp")
    {
        pub_left_  = create_publisher<std_msgs::msg::Float64>("dual_leg_l", 10);
        pub_right_ = create_publisher<std_msgs::msg::Float64>("dual_leg_r", 10);

        timer_ = create_wall_timer(500ms, std::bind(&DualLegPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        auto msg_l = std_msgs::msg::Float64();
        auto msg_r = std_msgs::msg::Float64();

        msg_l.data = left_angle.load();
        msg_r.data = right_angle.load();

        pub_left_->publish(msg_l);
        pub_right_->publish(msg_r);

        RCLCPP_INFO(this->get_logger(), "Publishing  L: %.2f  |  R: %.2f",
                    msg_l.data, msg_r.data);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// -------------------------------------------------
// MAIN
// -------------------------------------------------
int main(int argc, char* argv[])
{
    std::cout << "=====================================================\n";
    std::cout << " Dual UDP Receiver for AS5600 (ROS2 C++)\n";
    std::cout << "=====================================================\n";

    // Spawn UDP threads
    std::thread t_left(udp_receiver, PORT_LEFT,
                       std::ref(left_angle),
                       std::ref(left_connected),
                       std::ref(left_packet_count),
                       "LEFT LEG");

    std::thread t_right(udp_receiver, PORT_RIGHT,
                        std::ref(right_angle),
                        std::ref(right_connected),
                        std::ref(right_packet_count),
                        "RIGHT LEG");

    t_left.detach();
    t_right.detach();

    // ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualLegPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
