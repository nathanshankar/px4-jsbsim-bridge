#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

// TF2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class FGFSHttpStereoPublisher : public rclcpp::Node
{
public:
    FGFSHttpStereoPublisher() : Node("fgfs_stereo_publisher")
    {
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/fgfs/left/image_raw", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/fgfs/right/image_raw", 10);

        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/fgfs/left/camera_info", 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/fgfs/right/camera_info", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FGFSHttpStereoPublisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    // Removed disparity_pub_
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    static size_t write_data(void* ptr, size_t size, size_t nmemb, void* userdata)
    {
        auto* stream = static_cast<std::vector<uchar>*>(userdata);
        size_t count = size * nmemb;
        stream->insert(stream->end(), (uchar*)ptr, (uchar*)ptr + count);
        return count;
    }

    sensor_msgs::msg::CameraInfo create_camera_info(int width, int height, const std::string& frame_id, bool is_left)
    {
        sensor_msgs::msg::CameraInfo info;
        info.header.frame_id = frame_id;
        info.width = width;
        info.height = height;

        double fx = 500.0;
        double fy = 500.0;
        double cx = width / 2.0;
        double cy = height / 2.0;
        double baseline = 1.0; // meters

        info.k = {fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1};
        info.r = {1, 0, 0,
                  0, 1, 0,
                  0, 0, 1};
        info.d = {0, 0, 0, 0, 0};
        info.distortion_model = "plumb_bob";

        if (is_left) {
            info.p = {fx, 0, cx, 0,
                      0, fy, cy, 0,
                      0, 0, 1, 0};
        } else {
            info.p = {fx, 0, cx, -fx * baseline,
                      0, fy, cy, 0,
                      0, 0, 1, 0};
        }

        return info;
    }

    void timer_callback()
    {
        CURL* curl = curl_easy_init();
        if (!curl)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize CURL");
            return;
        }

        std::string url = "http://localhost:5500/screenshot";
        std::vector<uchar> buffer;

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 1000L);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK)
        {
            RCLCPP_WARN(this->get_logger(), "CURL request failed: %s", curl_easy_strerror(res));
            return;
        }

        if (buffer.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty buffer from FGFS");
            return;
        }

        cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if (img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Image decode failed");
            return;
        }

        int top_crop = 50;
        if (img.rows <= top_crop)
        {
            RCLCPP_WARN(this->get_logger(), "Image too small to crop top bar");
            return;
        }
        img = img(cv::Rect(0, top_crop, img.cols, img.rows - top_crop));

        int half_width = img.cols / 2;
        int height = img.rows;

        cv::Mat left_img = img(cv::Rect(0, 0, half_width, height));
        cv::Mat right_img = img(cv::Rect(half_width, 0, half_width, height));

        // Removed disparity calculation and conversion

        // Create ROS headers and messages
        auto timestamp = this->now();
        std::string frame_id = "camera_frame";

        std_msgs::msg::Header header;
        header.stamp = timestamp;
        header.frame_id = frame_id;

        auto left_msg = cv_bridge::CvImage(header, "bgr8", left_img).toImageMsg();
        auto right_msg = cv_bridge::CvImage(header, "bgr8", right_img).toImageMsg();
        // Removed disparity message

        sensor_msgs::msg::CameraInfo left_info = create_camera_info(half_width, height, frame_id, true);
        sensor_msgs::msg::CameraInfo right_info = create_camera_info(half_width, height, frame_id, false);

        left_info.header.stamp = timestamp;
        right_info.header.stamp = timestamp;

        // Publish
        left_image_pub_->publish(*left_msg);
        right_image_pub_->publish(*right_msg);
        // Removed disparity publishing
        left_info_pub_->publish(left_info);
        right_info_pub_->publish(right_info);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = timestamp;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "camera_frame";

        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(-M_PI_2, 0, -M_PI_2);
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FGFSHttpStereoPublisher>());
    rclcpp::shutdown();
    return 0;
}
