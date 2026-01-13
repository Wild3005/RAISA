#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"
#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class Capture : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;

    cv::VideoCapture cap_;
    std::string camera_path;

    // ===Publisher===
    image_transport::Publisher pub_image_raw;
    image_transport::Publisher pub_image_display;

    Capture() : Node("Capture")
    {
        this->declare_parameter("camera_path", camera_path);
        this->get_parameter("camera_path", camera_path);

        cap_.open(camera_path);

        int width = 640;
        int height = 480;

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        // optionally set FPS
        cap_.set(cv::CAP_PROP_FPS, 30);

        // // verify what actually got applied
        // double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        // double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT); //
        // double actual_fps = cap_.get(cv::CAP_PROP_FPS);

        if (!cap_.isOpened())
        {
            RCLCPP_FATAL(get_logger(), "Failed to open camera");
            rclcpp::shutdown();
            return;
        }

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        pub_image_raw = image_transport::create_publisher(this, "/vision/image_raw");
        pub_image_display = image_transport::create_publisher(this, "/vision/image_display");

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&Capture::callback_routine, this));

        logger.info("Capture node initialized");
    }

    ~Capture()
    {
    }

    void callback_routine()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
            return;

        // flip horizontally
        cv::flip(frame, frame, 1);

        auto msg = cv_bridge::CvImage(
                       std_msgs::msg::Header(),
                       "bgr8",
                       frame)
                       .toImageMsg();

        msg->header.stamp = this->get_clock()->now();
        pub_image_display.publish(*msg);

        static int count = 0;
        count++;
        if (count % 8 != 0)
            return; // publish every 10th frame

        count = 0;
        msg->header.stamp = this->get_clock()->now();
        pub_image_raw.publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_Capture = std::make_shared<Capture>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_Capture);
    executor.spin();

    return 0;
}